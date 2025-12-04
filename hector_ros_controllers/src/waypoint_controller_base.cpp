#include "waypoint_controller_base/waypoint_controller_base.hpp"

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace waypoint_controller_base
{
WaypointControllerBase::WaypointControllerBase()
    : controller_interface::ControllerInterface(), trajectory_( nullptr )
{
}

void WaypointControllerBase::declare_parameters()
{
  // param_listener_ = std::make_shared<ParamListener>( get_node() );
}

controller_interface::CallbackReturn WaypointControllerBase::read_parameters()
{
  /*if ( !param_listener_ ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Error encountered during init" );
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

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
  }*/

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WaypointControllerBase::on_init()
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
WaypointControllerBase::on_configure( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  auto ret = this->read_parameters();
  if ( ret != controller_interface::CallbackReturn::SUCCESS ) {
    return ret;
  }

  navigation_server_ =
      rclcpp_action::create_server<hector_controller_msgs::action::WaypointNavigation>(
          get_node() "~/waypoint_navigation", rclcpp::SystemDefaultsQoS(),
          [this]( const CmdType::SharedPtr msg ) {
            rt_command_ptr_.writeFromNonRT( msg );
            safety_engaged_ = false;
            safety_timer_->reset();
          } );

  RCLCPP_INFO( get_node()->get_logger(), "configure successful" );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
WaypointControllerBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration WaypointControllerBase::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interface_config;
  state_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interface_config.names = state_interface_types_;

  return state_interface_config;
}

controller_interface::CallbackReturn
WaypointControllerBase::on_activate( const rclcpp_lifecycle::State & /*previous_state*/ )
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
  trajectory_ = realtime_tools::RealtimeBuffer<std::shared_ptr<std::vector<Goal>>>( nullptr );

  RCLCPP_INFO( get_node()->get_logger(), "activate successful" );
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
WaypointControllerBase::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  // reset command buffer
  trajectory_ = realtime_tools::RealtimeBuffer<std::shared_ptr<std::vector<Goal>>>( nullptr );

  return controller_interface::CallbackReturn::SUCCESS;
}

bool check_goal_completion( const Goal &goal, const Pose &current_pose ) { return false; }

controller_interface::return_type
WaypointControllerBase::update( const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/ )
{
  auto trajectory = trajectory_.readFromRT();

  if ( !trajectory ) {
    // Reset trajectory waypoint
    current_goal_idx_ = 0;
    return controller_interface::return_type::OK;
  }

  if ( check_goal_completion( current_goal_, current_pose_ ) )

    current_goal_idx_ += 1;
  if ( current_goal_idx_ == trajectory->get()->size() ) {
    // finished trajectory

  } else {
    // proceed to next goal
    current_goal_idx_++;
    current_goal_ = ( *trajectory )[current_goal_idx_];
  }

  bool successful = true;

  if ( !successful )
    return controller_interface::return_type::ERROR;

  return controller_interface::return_type::OK;
}

} // namespace waypoint_controller_base

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS( waypoint_controller_base::WaypointControllerBase,
                        controller_interface::ControllerInterface )
