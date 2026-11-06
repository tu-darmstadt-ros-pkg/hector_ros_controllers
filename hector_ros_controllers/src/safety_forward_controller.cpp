#include "safety_forward_controller/safety_forward_controller.hpp"
#include "hector_ros_controllers/reference_handles.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace safety_forward_controller
{

SafetyForwardController::SafetyForwardController()
    : controller_interface::ChainableControllerInterface(), safety_engaged_( false ),
      safety_timer_period_ms_( 0 ), rt_command_ptr_( nullptr ), joints_command_subscriber_( nullptr )
{
}

void SafetyForwardController::declare_parameters()
{ param_listener_ = std::make_shared<ParamListener>( get_node() ); }

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

  std::string interface_prefix;
  if ( !params_.passthrough_controller.empty() ) {
    interface_prefix = params_.passthrough_controller + "/";
  }

  if ( !params_.interface_type.empty() ) {
    if ( params_.interface_type == "velocity" || params_.interface_type == "effort" ||
         params_.interface_type == "position" ) {
      interface_type_ = params_.interface_type;
    } else {
      RCLCPP_ERROR( get_node()->get_logger(),
                    "Only 'position', 'velocity' or 'effort' interfaces are supported" );
      return controller_interface::CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_ERROR( get_node()->get_logger(), "'interface' parameter was empty" );
    return controller_interface::CallbackReturn::ERROR;
  }

  safety_timer_period_ms_ = static_cast<int>( params_.safety_timer_duration );

  const urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF( this->get_robot_description() );

  command_interface_types_.clear();
  joints_.clear();
  joint_limits_.clear();

  for ( const auto &joint : params_.joints ) {
    joints_.push_back( joint );

    command_interface_types_.push_back( interface_prefix + joint + "/" + interface_type_ );

    if ( urdf && urdf->getJoint( joint ) ) {
      joint_limits_.push_back( urdf->getJoint( joint )->limits );
    } else {
      joint_limits_.push_back( nullptr );
      RCLCPP_WARN( get_node()->get_logger(), "No URDF limits found for joint '%s'", joint.c_str() );
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SafetyForwardController::on_init()
{
  try {
    declare_parameters();
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Exception thrown during init: %s", e.what() );
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

  // Subscriber for external commands (used when NOT in chained mode)
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
      "~/commands", rclcpp::SystemDefaultsQoS(), [this]( const CmdType::SharedPtr msg ) {
        rt_command_ptr_.writeFromNonRT( msg );
        const bool was_engaged = safety_engaged_.exchange( false );
        if ( safety_timer_ ) {
          safety_timer_->reset();
        }
        if ( was_engaged ) {
          publish_status();
        }
      } );

  // Safety timer: if no commands for some time, engage safety (zero commands)
  if ( params_.interface_type != "position" ) // Position controllers don't need a safety timer
  {
    safety_timer_ = get_node()->create_wall_timer(
        std::chrono::milliseconds( safety_timer_period_ms_ ), [this]() {
          if ( !safety_engaged_.load() ) {
            safety_engaged_.store( true );
            RCLCPP_WARN( get_node()->get_logger(),
                         "Safety engaged, stopping all commands (timeout)" );
            publish_status();
          }
        } );
  }

  // Status publisher (latched)
  auto qos_latched = rclcpp::QoS( 1 ).transient_local().reliable();
  status_pub_ =
      get_node()->create_publisher<hector_ros_controllers_msgs::msg::SafetyForwardControllerStatus>(
          "~/status", qos_latched );

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
{ return { controller_interface::interface_configuration_type::NONE, {} }; }

controller_interface::CallbackReturn
SafetyForwardController::on_activate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  // Check that we have the expected command interfaces
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
  ordered_interfaces.reserve( command_interface_types_.size() );
  if ( !controller_interface::get_ordered_interfaces( command_interfaces_, command_interface_types_,
                                                      std::string( "" ), ordered_interfaces ) ||
       command_interface_types_.size() != ordered_interfaces.size() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
                  command_interface_types_.size(), ordered_interfaces.size() );
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>( nullptr );
  // No target until a new command arrives, so the hardware holds instead of resuming an old one
  if ( !hector_ros_controllers::invalidate_references( ordered_exported_reference_interfaces_ ) ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Could not reset the reference interfaces" );
    return controller_interface::CallbackReturn::ERROR;
  }

  safety_engaged_.store( false );
  if ( safety_timer_ ) {
    safety_timer_->reset();
  }

  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
      "~/commands", rclcpp::SystemDefaultsQoS(), [this]( const CmdType::SharedPtr msg ) {
        rt_command_ptr_.writeFromNonRT( msg );
        const bool was_engaged = safety_engaged_.exchange( false );
        if ( safety_timer_ ) {
          safety_timer_->reset();
        }
        if ( was_engaged ) {
          publish_status();
        }
      } );

  publish_status();

  RCLCPP_INFO( get_node()->get_logger(), "activate successful" );
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SafetyForwardController::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  // Reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>( nullptr );
  if ( safety_timer_ ) {
    safety_timer_->cancel();
  }

  joints_command_subscriber_.reset();

  safety_engaged_.store( false );

  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
SafetyForwardController::on_export_reference_interfaces_list()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> refs;
  refs.reserve( joints_.size() );

  const std::string controller_name = get_node()->get_name(); // e.g. "flipper_velocity_controller"

  for ( std::size_t i = 0; i < joints_.size(); ++i ) {
    // Resource name must start with controller name
    const std::string resource_name = controller_name + "/" + joints_[i];
    // Interface type is the same as the hardware command interface ("position"/"velocity"/"effort")
    refs.push_back(
        std::make_shared<hardware_interface::CommandInterface>( resource_name, interface_type_ ) );
  }
  reference_values_.assign( refs.size(), std::numeric_limits<double>::quiet_NaN() );

  return refs;
}

controller_interface::return_type
SafetyForwardController::update_reference_from_subscribers( const rclcpp::Time & /*time*/,
                                                            const rclcpp::Duration & /*period*/ )
{
  // In chained mode, references come from preceding controller,
  // so we DON'T touch the references here.
  if ( is_in_chained_mode() ) {
    return controller_interface::return_type::OK;
  }

  const auto joint_commands = rt_command_ptr_.readFromRT();
  if ( !joint_commands || !( *joint_commands ) ) {
    // No new command message → keep previous references
    return controller_interface::return_type::OK;
  }

  const auto &data = ( *joint_commands )->data;

  const auto &references = ordered_exported_reference_interfaces_;
  if ( data.size() < references.size() ) {
    RCLCPP_ERROR_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 5000,
                           "Received command of size %zu, but controller expects %zu", data.size(),
                           references.size() );
    // Use only the available part
  }

  if ( const auto busy = hector_ros_controllers::write_references( references, data ); busy > 0 ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 5000,
                          "%zu reference interfaces were busy; they keep their previous value",
                          busy );
  }
  return controller_interface::return_type::OK;
}

bool SafetyForwardController::on_set_chained_mode( const bool chained_mode )
{
  if ( chained_mode ) {
    joints_command_subscriber_.reset();
    safety_timer_.reset();
    RCLCPP_INFO( get_node()->get_logger(), "Switched to CHAINED mode" );
  } else {
    RCLCPP_INFO( get_node()->get_logger(), "Switched to UNCHAINED mode" );
    rt_command_ptr_.reset();
  }
  // The previous source's last reference must not outlive the switch
  return hector_ros_controllers::invalidate_references( ordered_exported_reference_interfaces_ );
}

controller_interface::return_type
SafetyForwardController::update_and_write_commands( const rclcpp::Time & /*time*/,
                                                    const rclcpp::Duration & /*period*/ )
{
  if ( const auto busy = hector_ros_controllers::read_references(
           ordered_exported_reference_interfaces_, reference_values_ );
       busy > 0 ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 5000,
                          "%zu reference interfaces were busy; using their last value", busy );
  }

  bool successful = true;

  // Set commands for joints
  for ( std::size_t index = 0; index < command_interfaces_.size(); ++index ) {
    double command_value = 0.0;

    if ( safety_engaged_.load() ) {
      // --- SAFETY TIMER BEHAVIOR ---
      command_value = 0.0;
    } else {
      // --- NORMAL OPERATION ---
      if ( index >= reference_values_.size() ) {
        RCLCPP_ERROR_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 5000,
                               "too few reference interfaces (index %zu, size %zu)", index,
                               reference_values_.size() );
        successful = false;
        continue;
      }

      command_value = reference_values_[index];

      const auto &limits = joint_limits_[index];
      if ( limits ) {
        if ( interface_type_ == "velocity" ) {
          if ( limits->velocity != 0.0 ) {
            command_value = std::clamp( command_value, -limits->velocity, limits->velocity );
          }
        } else if ( interface_type_ == "effort" ) {
          if ( limits->effort != 0.0 ) {
            command_value = std::clamp( command_value, -limits->effort, limits->effort );
          }
        } else if ( interface_type_ == "position" ) {
          if ( limits->lower != 0.0 || limits->upper != 0.0 ) {
            command_value = std::clamp( command_value, limits->lower, limits->upper );
          }
        }
      }
    }

    successful &= command_interfaces_[index].set_value( command_value );
  }

  if ( !successful ) {
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

void SafetyForwardController::publish_status()
{
  if ( !status_pub_ ) {
    return;
  }
  hector_ros_controllers_msgs::msg::SafetyForwardControllerStatus msg;
  msg.header.stamp = get_node()->now();
  msg.safety_timer_engaged = safety_engaged_.load();
  status_pub_->publish( msg );
}

} // namespace safety_forward_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS( safety_forward_controller::SafetyForwardController,
                        controller_interface::ChainableControllerInterface )
