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

bool is_valid( const std::optional<double> &v_opt )
{
  return v_opt.has_value() && !std::isnan( v_opt.value() );
}

void VelocityToPositionControllersBase::update_joint_states_if_valid()
{
  bool all_joints_valid = true;
  for ( size_t i = 0ul; i < joints_.size(); ++i ) {
    const auto &pos_state = state_interfaces_[2 * i].get_optional();
    const auto &vel_state = state_interfaces_[2 * i + 1].get_optional();

    bool states_valid = is_valid( pos_state ) && is_valid( vel_state );
    all_joints_valid &= states_valid;
    if ( states_valid ) {
      // If we have a valid state, we set the last position to the current state
      joint_position_states_[i] = pos_state.value();
      joint_prev_vel_states_[i] =
          std::isnan( joint_velocity_states_[i] ) ? vel_state.value() : joint_velocity_states_[i];
      joint_velocity_states_[i] = vel_state.value();

    } else {
      // If we don't have a valid state, we set it to NaN
      joint_position_states_[i] = std::numeric_limits<double>::quiet_NaN();
      joint_velocity_states_[i] = std::numeric_limits<double>::quiet_NaN();
      joint_prev_vel_states_[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  interfaces_valid_ = all_joints_valid;
}

void VelocityToPositionControllersBase::update_move_state( const double &vel_command,
                                                           const size_t &joint_idx )
{
  switch ( move_states_[joint_idx] ) {
  case MOVING:
    if ( vel_command == 0.0 )
      move_states_[joint_idx] = STOPPING;
    break;

  case STOPPING:
    if ( vel_command != 0.0 )
      move_states_[joint_idx] = MOVING;
    else {
      if ( std::abs( joint_velocity_states_[joint_idx] ) <= 0.005 )
        move_states_[joint_idx] = STOPPED;
    }
    break;

  case STOPPED:
    if ( vel_command != 0.0 )
      move_states_[joint_idx] = MOVING;
    break;
  }
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
          for ( auto &position : joint_position_states_ )
            position = std::numeric_limits<double>::quiet_NaN();
        } else {
          e_stop_active_ = false;
        }
      } );

  update_joint_states_if_valid();

  for ( size_t i = 0; i < joints_.size(); i++ ) {
    move_states_[i] = STOPPED;

    if ( interfaces_valid_ )
      hold_positions_[i] = joint_position_states_[i];
    else {
      hold_positions_[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }

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
  update_joint_states_if_valid();
  if ( !interfaces_valid_ )
    return controller_interface::return_type::ERROR;

  bool successful = true;
  if ( e_stop_active_ ) {
    for ( auto index = 0ul; index < command_interfaces_.size(); index++ ) {
      if ( !std::isnan( joint_position_states_[index] ) )
        successful = command_interfaces_[index].set_value( joint_position_states_[index] );
    }
  } else {
    // Set commands for joints
    for ( size_t index = 0ul; index < command_interfaces_.size(); index++ ) {

      // skip if no command received from high level controller
      if ( std::isnan( reference_interfaces_[index] ) )
        continue;

      const double &vel_command = reference_interfaces_[index];

      update_move_state( vel_command, index );

      double new_position = std::numeric_limits<double>::quiet_NaN();
      std::string move_state;
      switch ( move_states_[index] ) {

      case MOVING:
        new_position = joint_position_states_[index] + vel_command * p.seconds() +
                       p_gain_ * ( vel_command - joint_velocity_states_[index] ) * p.seconds() -
                       d_gain_ * ( joint_velocity_states_[index] - joint_prev_vel_states_[index] ) *
                           ( p.seconds() * p.seconds() );
        move_state = "MOVING";
        break;
      case STOPPING:
        new_position = joint_position_states_[index] + vel_command * p.seconds() +
                       p_gain_ * ( vel_command - joint_velocity_states_[index] ) * p.seconds() -
                       d_gain_ * ( joint_velocity_states_[index] - joint_prev_vel_states_[index] ) *
                           ( p.seconds() * p.seconds() );
        hold_positions_[index] = joint_position_states_[index];
        move_state = "STOPPING";
        break;
      case STOPPED:
        new_position = hold_positions_[index];
        move_state = "STOPPED";
        break;
      default:
        continue;
      }

      if ( index == 0 )
        RCLCPP_INFO(
            get_node()->get_logger(), "Joint {%s}: Pos {%f}, Vel {%f}. New pos {%f}. Command vel {%f}, Hold position {%f}. Move state %s",
            joints_[index].c_str(), joint_position_states_[index], joint_velocity_states_[index],
            new_position, vel_command, hold_positions_[index], move_state.c_str() );

      successful &= command_interfaces_[index].set_value( new_position );

      /*double new_position = last_positions_[index] + vel_command * p.seconds();
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
      }*/
    }
  }

  if ( !successful )
    return controller_interface::return_type::ERROR;

  return controller_interface::return_type::OK;
}

} // namespace velocity_to_position_command_controller
