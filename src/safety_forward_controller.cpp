#include "safety_forward_controller/safety_forward_controller.hpp"

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace safety_forward_controller
{
SafetyForwardController::SafetyForwardController()
    : controller_interface::ControllerInterface(), rt_command_ptr_( nullptr ),
      joints_command_subscriber_( nullptr )
{
}

void SafetyForwardController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>( get_node() );
}

controller_interface::CallbackReturn SafetyForwardController::read_parameters()
{
  if ( !param_listener_ ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Error encountered during init" );
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if ( params_.joints.empty() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "'joints' parameter was empty" );
    return controller_interface::CallbackReturn::ERROR;
  }

  std::string interface_prefix = "";
  if ( !params_.passthrough_controller.empty() )
    interface_prefix = params_.passthrough_controller + "/";

  if ( !params_.interface_type.empty() ) {
    if ( params_.interface_type == "velocity" || params_.interface_type == "effort" ) {
      interface_type_ = params_.interface_type;
    } else {
      RCLCPP_ERROR( get_node()->get_logger(),
                    "Only 'velocity' or 'effort' interfaces are supported" );
      return controller_interface::CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_ERROR( get_node()->get_logger(), "'interface' parameter was empty" );
    return controller_interface::CallbackReturn::ERROR;
  }

  safty_timer_period_ms_ = (int)params_.safety_timer_duration;

  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF( this->get_robot_description() );
  for ( const auto &joint : params_.joints ) {
    command_interface_types_.push_back( interface_prefix + joint + "/" + interface_type_ );
    state_interface_types_.push_back( joint + "/" + interface_type_ );
    joint_limits_.push_back( urdf->getJoint( joint )->limits );
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SafetyForwardController::on_init()
{
  try {
    declare_parameters();
  } catch ( const std::exception &e ) {
    fprintf( stderr, "Exception thrown during init stage with message: %s \n", e.what() );
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SafetyForwardController::on_configure( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  auto ret = this->read_parameters();
  if ( ret != controller_interface::CallbackReturn::SUCCESS ) {
    return ret;
  }

  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
      "~/commands", rclcpp::SystemDefaultsQoS(), [this]( const CmdType::SharedPtr msg ) {
        rt_command_ptr_.writeFromNonRT( msg );
        safety_engaged_ = false;
        safety_timer_->reset();
      } );

  safety_timer_ =
      get_node()->create_wall_timer( std::chrono::milliseconds( safty_timer_period_ms_ ), [this]() {
        if ( !safety_engaged_ ) {
          safety_engaged_ = true;
          RCLCPP_WARN( get_node()->get_logger(), "Safety engaged, stopping all commands" );
        }
      } );

  RCLCPP_INFO( get_node()->get_logger(), "configure successful" );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SafetyForwardController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration SafetyForwardController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interface_config;
  state_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interface_config.names = state_interface_types_;

  return state_interface_config;
}

controller_interface::CallbackReturn
SafetyForwardController::on_activate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
  if ( !controller_interface::get_ordered_interfaces( command_interfaces_, command_interface_types_,
                                                      std::string( "" ), ordered_interfaces ) ||
       command_interface_types_.size() != ordered_interfaces.size() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
                  command_interface_types_.size(), ordered_interfaces.size() );
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>( nullptr );

  safety_timer_->reset();

  RCLCPP_INFO( get_node()->get_logger(), "activate successful" );
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SafetyForwardController::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  // reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>( nullptr );
  safety_timer_->cancel();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
SafetyForwardController::update( const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/ )
{
  auto joint_commands = rt_command_ptr_.readFromRT();
  // no command received yet
  if ( !joint_commands || !( *joint_commands ) ) {
    return controller_interface::return_type::OK;
  }

  bool successful = true;
  // Set commands for joints
  for ( auto index = 0ul; index < command_interfaces_.size(); ++index ) {
    if ( safety_engaged_ ) {
      successful = command_interfaces_[index].set_value( 0.0 ) && successful;
      continue;
    }

    double command = ( *joint_commands )->data[index];

    auto limits = joint_limits_[index];
    if ( limits ) {
      if ( interface_type_ == "velocity" ) {
        if ( limits->velocity )
          command = std::clamp( command, -limits->velocity, limits->velocity );
      }
      if ( interface_type_ == "effort" ) {
        if ( limits->effort )
          command = std::clamp( command, -limits->effort, limits->effort );
      }
    }
    successful = command_interfaces_[index].set_value( command ) && successful;
  }

  if ( !successful )
    return controller_interface::return_type::ERROR;

  return controller_interface::return_type::OK;
}

} // namespace safety_forward_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS( safety_forward_controller::SafetyForwardController,
                        controller_interface::ControllerInterface )