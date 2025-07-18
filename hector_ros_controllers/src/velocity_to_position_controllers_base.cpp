#include "velocity_to_position_command_controller/velocity_to_position_controllers_base.hpp"

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace velocity_to_position_command_controller
{
VelocityToPositionControllersBase::VelocityToPositionControllersBase()
    : controller_interface::ChainableControllerInterface(), e_stop_active_( false ),
      rt_buffer_ptr_( nullptr )
{
}

controller_interface::CallbackReturn VelocityToPositionControllersBase::on_init()
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
VelocityToPositionControllersBase::on_configure( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  auto ret = this->read_parameters();
  if ( ret != controller_interface::CallbackReturn::SUCCESS ) {
    return ret;
  }

  /*joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
      "~/commands", rclcpp::SystemDefaultsQoS(),
      [this]( const CmdType::SharedPtr msg ) { rt_command_ptr_.writeFromNonRT( msg ); } );*/

  RCLCPP_INFO( get_node()->get_logger(), "configure successful" );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
VelocityToPositionControllersBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
VelocityToPositionControllersBase::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interface_config;
  state_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interface_config.names = state_interface_types_;

  return state_interface_config;
}

std::vector<hardware_interface::CommandInterface>
VelocityToPositionControllersBase::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  RCLCPP_INFO( get_node()->get_logger(), "Exporting reference interfaces" );
  for ( size_t i = 0; i < reference_interface_names_.size(); ++i ) {
    RCLCPP_INFO( get_node()->get_logger(), "Exporting reference interface %s",
                 reference_interface_names_[i].c_str() );
    reference_interfaces.emplace_back( get_node()->get_name(), reference_interface_names_[i],
                                       &reference_interfaces_[i] );
  }

  return reference_interfaces;
}

controller_interface::return_type VelocityToPositionControllersBase::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/ )
{
  auto joint_commands = rt_buffer_ptr_.readFromRT();
  // message is valid
  if ( !( !joint_commands || !( *joint_commands ) ) ) {
    if ( reference_interfaces_.size() != ( *joint_commands )->data.size() ) {
      RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *( get_node()->get_clock() ), 1000,
          "command size (%zu) does not match number of reference interfaces (%zu)",
          ( *joint_commands )->data.size(), reference_interfaces_.size() );
      return controller_interface::return_type::ERROR;
    }
    reference_interfaces_ = ( *joint_commands )->data;
  }

  return controller_interface::return_type::OK;
}

bool VelocityToPositionControllersBase::on_set_chained_mode( bool /*chained_mode*/ )
{
  return true;
}

controller_interface::CallbackReturn
VelocityToPositionControllersBase::on_activate( const rclcpp_lifecycle::State & /*previous_state*/ )
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
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>( nullptr );

  RCLCPP_INFO( get_node()->get_logger(), "activate successful" );
  for ( auto index = 0ul; index < command_interfaces_.size(); ++index ) {
    const auto &state = state_interfaces_[index].get_optional();
    if ( state.has_value() && !std::isnan( state.value() ) ) {
      // If we have a valid state, we set the last position to the current state
      last_positions_[index] = state.value();
    } else {
      // If we don't have a valid state, we set it to NaN
      last_positions_[index] = std::numeric_limits<double>::quiet_NaN();
    }
    stopping_[index] = false;
  }

  auto qos = rclcpp::QoS( rclcpp::KeepLast( 1 ) );
  qos.transient_local();
  hard_estop_sub_ = this->get_node()->create_subscription<std_msgs::msg::Bool>(
      e_stop_topic_, qos, [this]( const std_msgs::msg::Bool::SharedPtr msg ) {
        if ( msg->data ) {
          RCLCPP_WARN(
              get_node()->get_logger(),
              "Hard E-Stop activated, stopping all joints && enable continous target pos update" );
          e_stop_active_ = true;
          // invalidate last positions
          for ( auto &position : last_positions_ )
            position = std::numeric_limits<double>::quiet_NaN();
        } else {
          e_stop_active_ = false;
        }
      } );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
VelocityToPositionControllersBase::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  // reset command buffer
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>( nullptr );

  hard_estop_sub_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
VelocityToPositionControllersBase::update_and_write_commands( const rclcpp::Time & /*time*/,
                                                              const rclcpp::Duration &p /*period*/ )
{
  bool successful = true;

  if ( e_stop_active_ ) {
    for ( auto index = 0ul; index < command_interfaces_.size(); index++ ) {
      const auto &state = state_interfaces_[index].get_optional();
      if ( state.has_value() && !std::isnan( state.value() ) ) {
        last_positions_[index] = state.value();
        successful = command_interfaces_[index].set_value( last_positions_[index] );
      }
    }
  } else {
    // Set commands for joints
    for ( auto index = 0ul; index < command_interfaces_.size(); index++ ) {

      // skip if no command received from high level controller
      if ( std::isnan( reference_interfaces_[index] ) )
        continue;

      double vel_command = reference_interfaces_[index];

      double new_position = last_positions_[index] + vel_command * p.seconds();
      if ( stopping_[index] ) {
        // If we were stopped, but now we have a velocity command, we set the new position
        if ( vel_command != 0.0 ) {
          stopping_[index] = false;
          successful = command_interfaces_[index].set_value( new_position );
          last_positions_[index] = new_position;
          // Stopping without velocity input, holding initial position
        } else {
          successful = command_interfaces_[index].set_value( last_positions_[index] );
        }
      } else {
        // Going from movement to stop at current position
        if ( vel_command == 0.0 ) {
          stopping_[index] = true;
          const auto &state = state_interfaces_[index].get_optional();
          if ( state.has_value() && !std::isnan( state.value() ) ) {
            last_positions_[index] = state.value();
            successful = command_interfaces_[index].set_value( last_positions_[index] );
          }
        }
        // Continuous movement
        else {
          successful = command_interfaces_[index].set_value( new_position );
          last_positions_[index] = new_position;
        }
      }
    }
  }

  if ( !successful )
    return controller_interface::return_type::ERROR;

  return controller_interface::return_type::OK;
}

} // namespace velocity_to_position_command_controller
