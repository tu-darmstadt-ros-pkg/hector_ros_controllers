#include "safety_forward_controller/safety_forward_controller.hpp"

#include <algorithm>
#include <chrono>
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
      safety_timer_period_ms_( 0 ), estop_active_( false ), estop_engaged_( false ),
      rt_command_ptr_( nullptr ), joints_command_subscriber_( nullptr )
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
  state_interface_types_.clear();
  joints_.clear();
  joint_limits_.clear();

  for ( const auto &joint : params_.joints ) {
    joints_.push_back( joint );

    command_interface_types_.push_back( interface_prefix + joint + "/" + interface_type_ );
    // For now, we use the same interface type for states
    state_interface_types_.push_back( joint + "/" + interface_type_ );

    if ( urdf && urdf->getJoint( joint ) ) {
      joint_limits_.push_back( urdf->getJoint( joint )->limits );
    } else {
      joint_limits_.push_back( nullptr );
      RCLCPP_WARN( get_node()->get_logger(), "No URDF limits found for joint '%s'", joint.c_str() );
    }
  }

  hold_positions_.assign( joints_.size(), 0.0 );

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

  // Subscriber for external commands (used when NOT in chained mode)
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
      "~/commands", rclcpp::SystemDefaultsQoS(), [this]( const CmdType::SharedPtr msg ) {
        rt_command_ptr_.writeFromNonRT( msg );
        safety_engaged_.store( false );
        if ( safety_timer_ ) {
          safety_timer_->reset();
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
          }
        } );
  }

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
  // Check that we have the expected command interfaces
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
  if ( !controller_interface::get_ordered_interfaces( command_interfaces_, command_interface_types_,
                                                      std::string( "" ), ordered_interfaces ) ||
       command_interface_types_.size() != ordered_interfaces.size() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
                  command_interface_types_.size(), ordered_interfaces.size() );
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>( nullptr );

  safety_engaged_.store( false );
  if ( safety_timer_ ) {
    safety_timer_->reset();
  }

  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
      "~/commands", rclcpp::SystemDefaultsQoS(), [this]( const CmdType::SharedPtr msg ) {
        rt_command_ptr_.writeFromNonRT( msg );
        safety_engaged_.store( false );
        if ( safety_timer_ ) {
          safety_timer_->reset();
        }
      } );

  // E-stop subscription
  safety_estop_subscriber_ = get_node()->create_subscription<std_msgs::msg::Bool>(
      "~/safety_estop", rclcpp::SystemDefaultsQoS(),
      [this]( const std_msgs::msg::Bool::SharedPtr msg ) {
        if ( msg->data != estop_active_.load() ) {
          estop_active_.store( msg->data );
          RCLCPP_WARN( get_node()->get_logger(), "E-STOP %s",
                       estop_active_ ? "ENGAGED" : "DISENGAGED" );
        }
      } );

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

  safety_estop_subscriber_.reset();
  joints_command_subscriber_.reset();

  estop_active_.store( false );
  estop_engaged_.store( false );
  safety_engaged_.store( false );

  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
SafetyForwardController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> refs;
  refs.reserve( joints_.size() );

  // One reference per joint.
  reference_interfaces_.assign( joints_.size(), 0.0 );

  const std::string controller_name = get_node()->get_name(); // e.g. "flipper_velocity_controller"

  for ( std::size_t i = 0; i < joints_.size(); ++i ) {
    // Resource name must start with controller name
    const std::string resource_name = controller_name + "/" + joints_[i];
    // Interface type is the same as the hardware command interface ("position"/"velocity"/"effort")
    refs.emplace_back( resource_name, interface_type_, &reference_interfaces_[i] );
  }

  return refs;
}

std::vector<hardware_interface::StateInterface> SafetyForwardController::on_export_state_interfaces()
{
  return {};
}

controller_interface::return_type
SafetyForwardController::update_reference_from_subscribers( const rclcpp::Time & /*time*/,
                                                            const rclcpp::Duration & /*period*/ )
{
  // In chained mode, references come from preceding controller,
  // so we DON'T touch reference_interfaces_ here.
  if ( is_in_chained_mode() ) {
    return controller_interface::return_type::OK;
  }

  const auto joint_commands = rt_command_ptr_.readFromRT();
  if ( !joint_commands || !( *joint_commands ) ) {
    // No new command message → keep previous reference_interfaces_ values
    return controller_interface::return_type::OK;
  }

  const auto &data = ( *joint_commands )->data;

  if ( data.size() < reference_interfaces_.size() ) {
    RCLCPP_ERROR_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 5000,
                           "Received command of size %zu, but controller expects %zu", data.size(),
                           reference_interfaces_.size() );
    // Use only the available part
  }

  const std::size_t n = std::min( reference_interfaces_.size(), data.size() );
  for ( std::size_t i = 0; i < n; ++i ) { reference_interfaces_[i] = data[i]; }

  return controller_interface::return_type::OK;
}

static std::size_t find_joint_index( const std::vector<std::string> &joints, const std::string &name )
{
  for ( std::size_t i = 0; i < joints.size(); ++i ) {
    if ( joints[i] == name ) {
      return i;
    }
  }
  return static_cast<std::size_t>( -1 );
}

void SafetyForwardController::record_hold_positions()
{
  hold_positions_.assign( joints_.size(), 0.0 );

  for ( const auto &state_iface : state_interfaces_ ) {
    const auto &joint_name = state_iface.get_name();
    const auto &iface_name = state_iface.get_interface_name();

    if ( iface_name == "position" ) {
      const std::size_t idx = find_joint_index( joints_, joint_name );
      if ( idx < joints_.size() ) {
        const auto opt = state_iface.get_optional();
        if ( opt.has_value() )
          hold_positions_[idx] = opt.value();
      }
    }
  }
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
  return true;
}

controller_interface::return_type
SafetyForwardController::update_and_write_commands( const rclcpp::Time & /*time*/,
                                                    const rclcpp::Duration & /*period*/ )
{
  const bool estop_active = estop_active_.load();
  bool estop_engaged = estop_engaged_.load();

  // Detect edge: requested state vs. actually engaged state
  if ( estop_active != estop_engaged ) {
    if ( estop_active ) {
      if ( params_.interface_type == "position" ) {
        // -------- E-STOP ENGAGE --------
        // Latch current joint positions (if available from state interfaces)
        record_hold_positions();
      }

      estop_engaged_.store( true );
      estop_engaged = true;

      RCLCPP_WARN( get_node()->get_logger(), "E-STOP engaged: latching positions for %zu joints",
                   hold_positions_.size() );
    } else {
      // -------- E-STOP RELEASE --------
      estop_engaged_.store( false );
      estop_engaged = false;

      RCLCPP_WARN( get_node()->get_logger(), "E-STOP released: resuming command output" );
      // on e-stop release, invalidate commands once
      for ( auto &ref : reference_interfaces_ ) ref = std::numeric_limits<double>::quiet_NaN();
      return controller_interface::return_type::OK;
    }
  }

  bool successful = true;

  // Set commands for joints
  for ( std::size_t index = 0; index < command_interfaces_.size(); ++index ) {
    double command_value = 0.0;

    if ( estop_engaged ) {
      // --- E-STOP BEHAVIOR ---
      if ( interface_type_ == "position" ) {
        // Hold current position
        command_value = hold_positions_[index];
      } else {
        // velocity / effort → stop (zero)
        command_value = 0.0;
      }
    } else if ( safety_engaged_.load() ) {
      // --- SAFETY TIMER BEHAVIOR (no e-stop) ---
      command_value = 0.0;
    } else {
      // --- NORMAL OPERATION ---
      if ( index >= reference_interfaces_.size() ) {
        RCLCPP_ERROR_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 5000,
                               "reference_interfaces_ too small (index %zu, size %zu)", index,
                               reference_interfaces_.size() );
        successful = false;
        continue;
      }

      command_value = reference_interfaces_[index];

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

} // namespace safety_forward_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS( safety_forward_controller::SafetyForwardController,
                        controller_interface::ChainableControllerInterface )
