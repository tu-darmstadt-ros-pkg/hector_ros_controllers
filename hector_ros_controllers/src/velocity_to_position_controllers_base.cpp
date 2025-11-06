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

rcl_interfaces::msg::SetParametersResult
VelocityToPositionControllersBase::setPIDGains( const rclcpp::Parameter &p )
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  const double val = p.as_double();

  if ( val <= 0 )
    result.successful = false;
  else {
    result.successful = true;
    if ( p.get_name() == "kp" ) {
      kp_ = val;
    }
    if ( p.get_name() == "kd" ) {
      kd_ = val;
    }
    if ( p.get_name() == "kp_sync" ) {
      kp_sync_ = val;
    }

    RCLCPP_INFO( get_node()->get_logger(), "Reconfigured %s to %f", p.get_name().c_str(), val );
  }

  return result;
}

controller_interface::CallbackReturn VelocityToPositionControllersBase::on_init()
{
  try {
    declare_parameters();

    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>( get_node() );
    cb_handle_kp_ = param_subscriber_->add_parameter_callback(
        "kp",
        std::bind( &VelocityToPositionControllersBase::setPIDGains, this, std::placeholders::_1 ),
        get_node()->get_name() );

    cb_handle_kd_ = param_subscriber_->add_parameter_callback(
        "kd",
        std::bind( &VelocityToPositionControllersBase::setPIDGains, this, std::placeholders::_1 ),
        get_node()->get_name() );

    cb_handle_sync_kp_ = param_subscriber_->add_parameter_callback(
        "kp_sync",
        std::bind( &VelocityToPositionControllersBase::setPIDGains, this, std::placeholders::_1 ),
        get_node()->get_name() );

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

void VelocityToPositionControllersBase::update_move_states( const double &vel_command,
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
      if ( std::abs( joint_velocity_states_[joint_idx] ) <= stopping_vel_threshold_ )
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

  auto qos = rclcpp::QoS( rclcpp::KeepLast( 1 ) );
  qos.transient_local();
  hard_estop_sub_ = this->get_node()->create_subscription<std_msgs::msg::Bool>(
      e_stop_topic_, qos, [this]( const std_msgs::msg::Bool::SharedPtr msg ) {
        if ( msg->data ) {
          RCLCPP_WARN(
              get_node()->get_logger(),
              "Hard E-Stop activated, stopping all joints && enable continuous target pos update" );
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

    // Set synchronization states to false to reset target offsets to current positions
    sync_states_[i] = false;
  }
  update_sync_offsets();

  RCLCPP_INFO( get_node()->get_logger(), "activate successful" );
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

void VelocityToPositionControllersBase::update_sync_states( const std::vector<double> &vel_commands )
{
  for ( auto &group : groups_ ) {
    const std::vector<size_t> &group_indices = group.second;

    if ( group_indices.size() == 1 ) {
      sync_states_[group_indices[0]] = false;
      continue;
    }

    bool group_is_synchronized = true;
    const double &common_vel_command = vel_commands[group_indices[0]];
    for ( size_t i = 1; i < group_indices.size(); i++ ) {
      group_is_synchronized &= common_vel_command == vel_commands[group_indices[i]];
    }

    for ( size_t i = 0; i < group_indices.size(); i++ ) {
      sync_states_[group_indices[i]] = group_is_synchronized;
    }
  }
}

void VelocityToPositionControllersBase::update_sync_offsets()
{
  for ( size_t joint_idx = 0; joint_idx < joints_.size(); joint_idx++ ) {
    if ( !sync_states_[joint_idx] ) {
      for ( size_t i = 0; i < synced_joints_[joint_idx].size(); i++ ) {
        sync_offsets_[joint_idx][i] =
            joint_position_states_[synced_joints_[joint_idx][i]] - joint_position_states_[joint_idx];
      }
    }
  }
}

double VelocityToPositionControllersBase::sync_p_control( const size_t &joint_idx )
{
  double sync_pos_command = 0.0;
  for ( size_t i = 0; i < synced_joints_[joint_idx].size(); i++ ) {
    sync_pos_command += ( joint_position_states_[synced_joints_[joint_idx][i]] -
                          joint_position_states_[joint_idx] - sync_offsets_[joint_idx][i] ) *
                        kp_sync_;
  }
  return sync_pos_command / (double)synced_joints_[joint_idx].size();
}

double VelocityToPositionControllersBase::pos_pd_control( const size_t &joint_idx,
                                                          const double &vel_command,
                                                          const rclcpp::Duration &p )
{
  // FF + P + D
  return joint_position_states_[joint_idx] + vel_command * p.seconds() +
         kp_ * ( vel_command - joint_velocity_states_[joint_idx] ) * p.seconds() -
         kd_ * ( joint_velocity_states_[joint_idx] - joint_prev_vel_states_[joint_idx] ) *
             ( p.seconds() * p.seconds() );
}

double VelocityToPositionControllersBase::position_control( const size_t &joint_idx,
                                                            const double &vel_command,
                                                            const rclcpp::Duration &p )
{
  double next_position = pos_pd_control( joint_idx, vel_command, p );
  if ( sync_states_[joint_idx] )
    next_position += sync_p_control( joint_idx );

  return next_position;
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
      hold_positions_[index] = joint_position_states_[index];
    }
  } else {

    update_sync_states( reference_interfaces_ );
    update_sync_offsets();

    // Set commands for joints
    for ( size_t joint_idx = 0ul; joint_idx < command_interfaces_.size(); joint_idx++ ) {

      // skip if no command received from high level controller
      if ( std::isnan( reference_interfaces_[joint_idx] ) )
        continue;

      const double &vel_command = reference_interfaces_[joint_idx];

      update_move_states( vel_command, joint_idx );

      double pos_command = std::numeric_limits<double>::quiet_NaN();
      std::string move_state = "MOVING";
      switch ( move_states_[joint_idx] ) {
      case STOPPED:
        pos_command = hold_positions_[joint_idx];
        move_state = "STOPPED";
        break;

      // Position command calculation is the same for MOVING and STOPPING states
      default:
        pos_command = position_control( joint_idx, vel_command, p );
        // Set to most recent position during movement to avoid drift when stopped
        hold_positions_[joint_idx] = joint_position_states_[joint_idx];
      }

      if ( joint_idx == 0 )
        /*RCLCPP_INFO(
            get_node()->get_logger(), "Joint {%s}: Pos {%f}, Vel {%f}. New pos {%f}. Command vel
           {%f}, Hold position {%f}. Move state %s", joints_[joint_idx].c_str(),
           joint_position_states_[joint_idx], joint_velocity_states_[joint_idx], pos_command,
           vel_command, hold_positions_[joint_idx], move_state.c_str() );*/

        if ( std::isnan( pos_command ) )
          continue;

      successful &= command_interfaces_[joint_idx].set_value( pos_command );
    }
  }

  if ( !successful )
    return controller_interface::return_type::ERROR;

  return controller_interface::return_type::OK;
}

} // namespace velocity_to_position_command_controller
