#include "velocity_to_position_command_controller/velocity_to_position_command_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace velocity_to_position_command_controller
{

VelocityToPositionCommandController::VelocityToPositionCommandController()
    : controller_interface::ChainableControllerInterface(), stopping_vel_threshold_( 0.005 ),
      e_stop_active_( false ), interfaces_valid_( false ), kp_( 0.0 ), kd_( 0.0 ), kp_sync_( 0.0 ),
      rt_buffer_ptr_( nullptr )
{
}

// ---------------------------------------------------------------------------
// Parameter utilities
// ---------------------------------------------------------------------------

void VelocityToPositionCommandController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>( get_node() );
}

controller_interface::CallbackReturn VelocityToPositionCommandController::read_parameters()
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
  if ( !params_.passthrough_controller.empty() )
    interface_prefix = params_.passthrough_controller + "/";

  for ( const auto &joint : params_.joints ) {
    joints_.push_back( joint );
    command_interface_types_.push_back( interface_prefix + joint + "/position" );
    state_interface_types_.push_back( joint + "/position" );
    state_interface_types_.push_back( joint + "/velocity" );
    reference_interface_names_.push_back( joint + "/velocity" );

    joint_position_states_.push_back( std::numeric_limits<double>::quiet_NaN() );
    joint_velocity_states_.push_back( std::numeric_limits<double>::quiet_NaN() );
  }

  if ( !params_.synchronous_groups.empty() && params_.synchronous_groups.size() != joints_.size() ) {
    RCLCPP_ERROR( get_node()->get_logger(),
                  "Need to either specify a sync group for each joint or none at all" );
    return controller_interface::CallbackReturn::ERROR;
  }

  synced_joints_ = std::vector<std::vector<size_t>>( joints_.size() );
  sync_states_ = std::vector<bool>( joints_.size(), false );
  sync_offsets_ = std::vector<std::vector<double>>( joints_.size() );

  for ( size_t i = 0; i < params_.synchronous_groups.size(); i++ ) {
    joint_groups_.push_back( params_.synchronous_groups[i] );
    groups_[params_.synchronous_groups[i]].push_back( i );
  }

  for ( const auto &group : groups_ ) {
    for ( size_t i = 0; i < group.second.size(); i++ ) {
      size_t joint_idx = group.second[i];

      for ( size_t j = 0; j < group.second.size() - 1; j++ ) {
        size_t synced_joint_idx = group.second[( i + j + 1 ) % group.second.size()];

        synced_joints_[joint_idx].push_back( synced_joint_idx );
        sync_offsets_[joint_idx].push_back( std::numeric_limits<double>::quiet_NaN() );
        RCLCPP_INFO( get_node()->get_logger(),
                     "Adding synced joint %s for joint %s for vel to pos controller",
                     joints_[group.second[i]].c_str(),
                     joints_[group.second[( i + j + 1 ) % group.second.size()]].c_str() );
      }
    }
  }

  const size_t num_joints = reference_interface_names_.size();
  reference_interfaces_.resize( num_joints );
  hold_positions_.resize( num_joints );
  desired_positions_.resize( num_joints );
  move_states_.resize( num_joints );

  // Initialize limits to NaN (= no limit) by default
  joint_lower_limits_.assign( joints_.size(), std::numeric_limits<double>::quiet_NaN() );
  joint_upper_limits_.assign( joints_.size(), std::numeric_limits<double>::quiet_NaN() );

  e_stop_topic_ = params_.e_stop_topic;
  kp_ = params_.kp;
  kd_ = params_.kd;
  kp_sync_ = params_.kp_sync;
  stopping_vel_threshold_ = params_.stopping_velocity_threshold;

  return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// URDF joint limit parsing
// ---------------------------------------------------------------------------

void VelocityToPositionCommandController::parse_joint_limits_from_urdf()
{
  const std::string &urdf_string = this->get_robot_description();
  if ( urdf_string.empty() ) {
    RCLCPP_WARN( get_node()->get_logger(),
                 "Robot description is empty, position limits will not be enforced" );
    return;
  }

  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF( urdf_string );
  if ( !urdf_model ) {
    RCLCPP_WARN( get_node()->get_logger(),
                 "Failed to parse URDF, position limits will not be enforced" );
    return;
  }

  for ( size_t i = 0; i < joints_.size(); ++i ) {
    auto urdf_joint = urdf_model->getJoint( joints_[i] );
    if ( !urdf_joint ) {
      RCLCPP_WARN( get_node()->get_logger(), "Joint '%s' not found in URDF, no limits applied",
                   joints_[i].c_str() );
      continue;
    }

    if ( urdf_joint->type == urdf::Joint::CONTINUOUS ) {
      RCLCPP_DEBUG( get_node()->get_logger(), "Joint '%s' is continuous, no position limits",
                    joints_[i].c_str() );
      continue;
    }

    if ( ( urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::PRISMATIC ) &&
         urdf_joint->limits ) {
      joint_lower_limits_[i] = urdf_joint->limits->lower;
      joint_upper_limits_[i] = urdf_joint->limits->upper;
      RCLCPP_INFO( get_node()->get_logger(), "Joint '%s': position limits [%f, %f]",
                   joints_[i].c_str(), joint_lower_limits_[i], joint_upper_limits_[i] );
    }
  }
}

// ---------------------------------------------------------------------------
// Dynamic parameter reconfiguration
// ---------------------------------------------------------------------------

rcl_interfaces::msg::SetParametersResult
VelocityToPositionCommandController::set_pid_gains( const rclcpp::Parameter &p )
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  const double val = p.as_double();

  if ( val < 0 ) {
    result.successful = false;
  } else {
    result.successful = true;
    if ( p.get_name() == "kp" ) {
      kp_ = val;
    } else if ( p.get_name() == "kd" ) {
      kd_ = val;
    } else if ( p.get_name() == "kp_sync" ) {
      kp_sync_ = val;
    }
    RCLCPP_INFO( get_node()->get_logger(), "Reconfigured %s to %f", p.get_name().c_str(), val );
  }

  return result;
}

// ---------------------------------------------------------------------------
// Lifecycle callbacks
// ---------------------------------------------------------------------------

controller_interface::CallbackReturn VelocityToPositionCommandController::on_init()
{
  try {
    declare_parameters();

    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>( get_node() );
    cb_handle_kp_ = param_subscriber_->add_parameter_callback(
        "kp",
        std::bind( &VelocityToPositionCommandController::set_pid_gains, this, std::placeholders::_1 ),
        get_node()->get_name() );

    cb_handle_kd_ = param_subscriber_->add_parameter_callback(
        "kd",
        std::bind( &VelocityToPositionCommandController::set_pid_gains, this, std::placeholders::_1 ),
        get_node()->get_name() );

    cb_handle_sync_kp_ = param_subscriber_->add_parameter_callback(
        "kp_sync",
        std::bind( &VelocityToPositionCommandController::set_pid_gains, this, std::placeholders::_1 ),
        get_node()->get_name() );

    // Debug joint state publishers (dynamically reconfigurable)
    update_debug_publishers( param_listener_->get_params().publish_debug_joint_states );
    cb_handle_debug_pubs_ = param_subscriber_->add_parameter_callback(
        "publish_debug_joint_states",
        [this]( const rclcpp::Parameter &p ) { update_debug_publishers( p.as_bool() ); },
        get_node()->get_name() );

  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Exception thrown during init: %s", e.what() );
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
VelocityToPositionCommandController::on_configure( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  auto ret = read_parameters();
  if ( ret != controller_interface::CallbackReturn::SUCCESS ) {
    return ret;
  }

  parse_joint_limits_from_urdf();

  RCLCPP_INFO( get_node()->get_logger(), "configure successful" );
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
VelocityToPositionCommandController::on_activate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
  if ( !controller_interface::get_ordered_interfaces( command_interfaces_, command_interface_types_,
                                                      std::string( "" ), ordered_interfaces ) ||
       command_interface_types_.size() != ordered_interfaces.size() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
                  command_interface_types_.size(), ordered_interfaces.size() );
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reset command buffer
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>( nullptr );
  e_stop_active_.writeFromNonRT( false );

  auto qos = rclcpp::QoS( rclcpp::KeepLast( 1 ) );
  hard_estop_sub_ = this->get_node()->create_subscription<std_msgs::msg::Bool>(
      e_stop_topic_, qos, [this]( const std_msgs::msg::Bool::SharedPtr msg ) {
        if ( msg->data ) {
          RCLCPP_WARN(
              get_node()->get_logger(),
              "Hard E-Stop activated, stopping all joints && enable continuous target pos update" );
          e_stop_active_.writeFromNonRT( true );
        } else {
          e_stop_active_.writeFromNonRT( false );
        }
      } );

  update_joint_states_if_valid();

  for ( size_t i = 0; i < joints_.size(); i++ ) {
    move_states_[i] = STOPPED;

    if ( interfaces_valid_ ) {
      hold_positions_[i] = joint_position_states_[i];
      desired_positions_[i] = joint_position_states_[i];
    } else {
      hold_positions_[i] = std::numeric_limits<double>::quiet_NaN();
      desired_positions_[i] = std::numeric_limits<double>::quiet_NaN();
    }

    sync_states_[i] = false;
  }
  update_sync_offsets();

  RCLCPP_INFO( get_node()->get_logger(), "activate successful" );
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
VelocityToPositionCommandController::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>( nullptr );
  hard_estop_sub_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Chainable controller interface
// ---------------------------------------------------------------------------

controller_interface::InterfaceConfiguration
VelocityToPositionCommandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = command_interface_types_;
  return config;
}

controller_interface::InterfaceConfiguration
VelocityToPositionCommandController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = state_interface_types_;
  return config;
}

std::vector<hardware_interface::CommandInterface>
VelocityToPositionCommandController::on_export_reference_interfaces()
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

controller_interface::return_type VelocityToPositionCommandController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/ )
{
  auto joint_commands = rt_buffer_ptr_.readFromRT();
  if ( joint_commands && *joint_commands ) {
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

bool VelocityToPositionCommandController::on_set_chained_mode( bool /*chained_mode*/ )
{
  return true;
}

// ---------------------------------------------------------------------------
// State reading
// ---------------------------------------------------------------------------

static bool is_valid( const std::optional<double> &v_opt )
{
  return v_opt.has_value() && !std::isnan( v_opt.value() );
}

void VelocityToPositionCommandController::update_joint_states_if_valid()
{
  bool all_joints_valid = true;
  for ( size_t i = 0; i < joints_.size(); ++i ) {
    const auto &pos_state = state_interfaces_[2 * i].get_optional();
    const auto &vel_state = state_interfaces_[2 * i + 1].get_optional();

    bool states_valid = is_valid( pos_state ) && is_valid( vel_state );
    all_joints_valid &= states_valid;
    if ( states_valid ) {
      joint_position_states_[i] = pos_state.value();
      joint_velocity_states_[i] = vel_state.value();
    } else {
      joint_position_states_[i] = std::numeric_limits<double>::quiet_NaN();
      joint_velocity_states_[i] = std::numeric_limits<double>::quiet_NaN();
      RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *( get_node()->get_clock() ), 2000,
                            "Joint '%s' has invalid state interfaces (pos=%s, vel=%s)",
                            joints_[i].c_str(), is_valid( pos_state ) ? "ok" : "NaN/missing",
                            is_valid( vel_state ) ? "ok" : "NaN/missing" );
    }
  }

  interfaces_valid_ = all_joints_valid;
}

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------

void VelocityToPositionCommandController::update_move_states( double vel_command, size_t joint_idx )
{
  switch ( move_states_[joint_idx] ) {
  case MOVING:
    if ( vel_command == 0.0 )
      move_states_[joint_idx] = STOPPING;
    break;

  case STOPPING:
    if ( vel_command != 0.0 ) {
      move_states_[joint_idx] = MOVING;
      desired_positions_[joint_idx] = joint_position_states_[joint_idx];
    } else {
      if ( std::abs( joint_velocity_states_[joint_idx] ) <= stopping_vel_threshold_ )
        move_states_[joint_idx] = STOPPED;
    }
    break;

  case STOPPED:
    if ( vel_command != 0.0 ) {
      move_states_[joint_idx] = MOVING;
      desired_positions_[joint_idx] = joint_position_states_[joint_idx];
    }
    break;
  }
}

// ---------------------------------------------------------------------------
// Synchronization
// ---------------------------------------------------------------------------

void VelocityToPositionCommandController::update_sync_states( const std::vector<double> &vel_commands )
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

void VelocityToPositionCommandController::update_sync_offsets()
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

double VelocityToPositionCommandController::sync_p_control( size_t joint_idx )
{
  double sync_pos_command = 0.0;
  for ( size_t i = 0; i < synced_joints_[joint_idx].size(); i++ ) {
    sync_pos_command += ( joint_position_states_[synced_joints_[joint_idx][i]] -
                          joint_position_states_[joint_idx] - sync_offsets_[joint_idx][i] ) *
                        kp_sync_;
  }
  return sync_pos_command / static_cast<double>( synced_joints_[joint_idx].size() );
}

// ---------------------------------------------------------------------------
// Control law
// ---------------------------------------------------------------------------

double VelocityToPositionCommandController::pos_pd_control( size_t joint_idx, double vel_command,
                                                            const rclcpp::Duration &period )
{
  const double dt = period.seconds();

  // Integrate desired position along velocity command trajectory
  desired_positions_[joint_idx] += vel_command * dt;

  // P-term: velocity tracking correction
  const double vel_p = kp_ * ( vel_command - joint_velocity_states_[joint_idx] ) * dt;

  // D-term: damp position changes proportional to measured velocity.
  // This acts as a classical derivative-of-position damper: when the joint
  // moves fast the command is pulled back, preventing overshoot and oscillation.
  const double vel_d = kd_ * joint_velocity_states_[joint_idx] * dt;

  return desired_positions_[joint_idx] + vel_p - vel_d;
}

double VelocityToPositionCommandController::position_control( size_t joint_idx, double vel_command,
                                                              const rclcpp::Duration &period )
{
  double next_position = pos_pd_control( joint_idx, vel_command, period );
  if ( sync_states_[joint_idx] )
    next_position += sync_p_control( joint_idx );

  return next_position;
}

// ---------------------------------------------------------------------------
// Debug publishers
// ---------------------------------------------------------------------------

void VelocityToPositionCommandController::update_debug_publishers( bool enable )
{
  if ( enable ) {
    if ( !debug_in_js_pub_ ) {
      debug_in_js_pub_ =
          get_node()->create_publisher<sensor_msgs::msg::JointState>( "~/debug_in_joint_states", 10 );
    }
    if ( !debug_out_js_pub_ ) {
      debug_out_js_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
          "~/debug_out_joint_states", 10 );
    }
    RCLCPP_INFO( get_node()->get_logger(), "Debug joint state publishers enabled" );
  } else {
    debug_in_js_pub_.reset();
    debug_out_js_pub_.reset();
  }
}

void VelocityToPositionCommandController::publish_debug_joint_state_in()
{
  if ( !debug_in_js_pub_ )
    return;

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = get_node()->now();
  msg.name = joints_;
  msg.velocity = reference_interfaces_;
  debug_in_js_pub_->publish( msg );
}

void VelocityToPositionCommandController::publish_debug_joint_state_out(
    const std::vector<double> &positions )
{
  if ( !debug_out_js_pub_ )
    return;

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = get_node()->now();
  msg.name = joints_;
  msg.position = positions;
  debug_out_js_pub_->publish( msg );
}

// ---------------------------------------------------------------------------
// Main update
// ---------------------------------------------------------------------------

controller_interface::return_type
VelocityToPositionCommandController::update_and_write_commands( const rclcpp::Time & /*time*/,
                                                                const rclcpp::Duration &period )
{
  update_joint_states_if_valid();

  publish_debug_joint_state_in();

  bool successful = true;
  // TODO: e-stop
  // if ( *( e_stop_active_.readFromRT() ) ) {
  //   for ( size_t index = 0; index < command_interfaces_.size(); index++ ) {
  //     if ( !std::isnan( joint_position_states_[index] ) ) {
  //       hold_positions_[index] = joint_position_states_[index];
  //       desired_positions_[index] = joint_position_states_[index];
  //     }
  //     move_states_[index] = STOPPED;
  //   }
  //   RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *( get_node()->get_clock() ), 2000,
  //                         "E-Stop active, holding current joint positions" );
  //   return controller_interface::return_type::OK;
  // }

  update_sync_states( reference_interfaces_ );
  update_sync_offsets();

  for ( size_t joint_idx = 0; joint_idx < command_interfaces_.size(); joint_idx++ ) {

    // Skip if no command received from high level controller
    if ( std::isnan( reference_interfaces_[joint_idx] ) ) {
      RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *( get_node()->get_clock() ), 2000,
                            "No velocity command received for joint '%s'",
                            joints_[joint_idx].c_str() );
      continue;
    }
    // Skip joints with invalid state interfaces
    if ( std::isnan( joint_position_states_[joint_idx] ) ||
         std::isnan( joint_velocity_states_[joint_idx] ) ) {
      RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *( get_node()->get_clock() ), 2000,
                            "Joint '%s' has invalid state interfaces (pos=%s, vel=%s)",
                            joints_[joint_idx].c_str(),
                            std::isnan( joint_position_states_[joint_idx] ) ? "NaN" : "ok",
                            std::isnan( joint_velocity_states_[joint_idx] ) ? "NaN" : "ok" );
      continue;
    }

    const double vel_command = reference_interfaces_[joint_idx];

    update_move_states( vel_command, joint_idx );

    double pos_command = std::numeric_limits<double>::quiet_NaN();
    switch ( move_states_[joint_idx] ) {
    case STOPPED:
      pos_command = hold_positions_[joint_idx];
      break;

    // Position command calculation is the same for MOVING and STOPPING states
    default:
      pos_command = position_control( joint_idx, vel_command, period );
      // Update hold position to desired position (where joint *should* be)
      hold_positions_[joint_idx] = desired_positions_[joint_idx];
    }

    if ( std::isnan( pos_command ) ) {
      RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *( get_node()->get_clock() ), 2000,
                            "Position command is NaN for joint '%s'", joints_[joint_idx].c_str() );
      continue;
    }

    // Clamp to URDF position limits for revolute/prismatic joints
    if ( !std::isnan( joint_lower_limits_[joint_idx] ) ) {
      pos_command =
          std::clamp( pos_command, joint_lower_limits_[joint_idx], joint_upper_limits_[joint_idx] );
      desired_positions_[joint_idx] =
          std::clamp( desired_positions_[joint_idx], joint_lower_limits_[joint_idx],
                      joint_upper_limits_[joint_idx] );
      hold_positions_[joint_idx] =
          std::clamp( hold_positions_[joint_idx], joint_lower_limits_[joint_idx],
                      joint_upper_limits_[joint_idx] );
    }

    successful &= command_interfaces_[joint_idx].set_value( pos_command );
  }

  publish_debug_joint_state_out( desired_positions_ );

  if ( !successful )
    return controller_interface::return_type::ERROR;

  return controller_interface::return_type::OK;
}

} // namespace velocity_to_position_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS( velocity_to_position_command_controller::VelocityToPositionCommandController,
                        controller_interface::ChainableControllerInterface )
