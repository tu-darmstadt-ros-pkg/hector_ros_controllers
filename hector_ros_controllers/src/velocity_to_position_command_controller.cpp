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
      max_velocity_( 1.0 ), max_acceleration_( 2.0 ), max_deceleration_( 4.0 ),
      e_stop_active_( false ), interfaces_valid_( false ), kp_( 0.0 ), kd_( 0.0 ), kp_sync_( 0.0 ),
      velocity_command_timeout_( 0.0 ), rt_buffer_ptr_( nullptr )
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

  // Clear vectors to prevent duplicates if on_configure() is called multiple times
  joints_.clear();
  command_interface_types_.clear();
  state_interface_types_.clear();
  reference_interface_names_.clear();
  joint_position_states_.clear();
  joint_velocity_states_.clear();
  joint_groups_.clear();
  groups_.clear();

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

  sync_states_ = std::vector<bool>( joints_.size(), false );

  for ( size_t i = 0; i < params_.synchronous_groups.size(); i++ ) {
    joint_groups_.push_back( params_.synchronous_groups[i] );
    groups_[params_.synchronous_groups[i]].push_back( i );
  }

  sync_pairs_.init( joints_.size(), groups_ );

  for ( size_t i = 0; i < joints_.size(); ++i ) {
    if ( sync_pairs_.has_partner( i ) ) {
      RCLCPP_INFO( get_node()->get_logger(),
                   "Sync pair: joint %s <-> joint %s for vel to pos controller", joints_[i].c_str(),
                   joints_[sync_pairs_.partner( i )].c_str() );
    }
  }

  const size_t num_joints = reference_interface_names_.size();
  reference_interfaces_.resize( num_joints );
  hold_positions_.resize( num_joints );
  desired_positions_.resize( num_joints );
  move_states_.resize( num_joints );
  braking_profiles_.resize( num_joints );
  braking_start_times_.resize( num_joints );

  // Initialize limits to NaN (= no limit) by default
  joint_lower_limits_.assign( joints_.size(), std::numeric_limits<double>::quiet_NaN() );
  joint_upper_limits_.assign( joints_.size(), std::numeric_limits<double>::quiet_NaN() );

  e_stop_topic_ = params_.e_stop_topic;
  kp_ = params_.kp;
  kd_ = params_.kd;
  kp_sync_ = params_.kp_sync;
  kd_sync_ = params_.kd_sync;
  sync_velocity_factor_ = params_.sync_velocity_factor;
  sync_velocity_min_threshold_ = params_.sync_velocity_min_threshold;
  stopping_vel_threshold_ = params_.stopping_velocity_threshold;
  max_velocity_ = params_.max_velocity;
  max_acceleration_ = params_.max_acceleration;
  max_deceleration_ = params_.max_deceleration;
  velocity_command_timeout_ = params_.velocity_command_timeout;

  // Build group index map and RT buffers for group actions
  group_index_map_.clear();
  group_names_.clear();
  for ( const auto &group : groups_ ) {
    group_index_map_[group.first] = group_names_.size();
    group_names_.push_back( group.first );
  }
  rt_group_action_cmds_.resize( group_names_.size() );
  group_action_states_ = std::vector<std::atomic<GroupActionState>>( group_names_.size() );
  for ( auto &state : group_action_states_ ) { state.store( GroupActionState::IDLE ); }

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
    } else if ( p.get_name() == "kd_sync" ) {
      kd_sync_ = val;
    } else if ( p.get_name() == "sync_velocity_factor" ) {
      sync_velocity_factor_ = val;
    } else if ( p.get_name() == "sync_velocity_min_threshold" ) {
      sync_velocity_min_threshold_ = val;
    } else if ( p.get_name() == "max_velocity" ) {
      max_velocity_ = val;
    } else if ( p.get_name() == "max_acceleration" ) {
      max_acceleration_ = val;
    } else if ( p.get_name() == "max_deceleration" ) {
      max_deceleration_ = val;
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

    cb_handle_sync_kd_ = param_subscriber_->add_parameter_callback(
        "kd_sync",
        std::bind( &VelocityToPositionCommandController::set_pid_gains, this, std::placeholders::_1 ),
        get_node()->get_name() );

    cb_handle_sync_velocity_factor_ = param_subscriber_->add_parameter_callback(
        "sync_velocity_factor",
        std::bind( &VelocityToPositionCommandController::set_pid_gains, this, std::placeholders::_1 ),
        get_node()->get_name() );

    cb_handle_sync_velocity_min_threshold_ = param_subscriber_->add_parameter_callback(
        "sync_velocity_min_threshold",
        std::bind( &VelocityToPositionCommandController::set_pid_gains, this, std::placeholders::_1 ),
        get_node()->get_name() );

    cb_handle_max_velocity_ = param_subscriber_->add_parameter_callback(
        "max_velocity",
        std::bind( &VelocityToPositionCommandController::set_pid_gains, this, std::placeholders::_1 ),
        get_node()->get_name() );

    cb_handle_max_acceleration_ = param_subscriber_->add_parameter_callback(
        "max_acceleration",
        std::bind( &VelocityToPositionCommandController::set_pid_gains, this, std::placeholders::_1 ),
        get_node()->get_name() );

    cb_handle_max_deceleration_ = param_subscriber_->add_parameter_callback(
        "max_deceleration",
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

  // Reset command buffer and reference interfaces to prevent stale velocity commands
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>( nullptr );
  std::fill( reference_interfaces_.begin(), reference_interfaces_.end(),
             std::numeric_limits<double>::quiet_NaN() );
  e_stop_active_.writeFromNonRT( false );

  // Topic subscriber for non-chained mode
  auto cmd_qos = rclcpp::QoS( rclcpp::KeepLast( 1 ) );
  cmd_sub_ = get_node()->create_subscription<CmdType>(
      "~/commands", cmd_qos,
      [this]( const CmdType::SharedPtr msg ) { rt_buffer_ptr_.writeFromNonRT( msg ); } );

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
  for ( size_t i = 0; i < joints_.size(); i++ ) { reset_sync_offsets( i ); }

  // Initialize velocity command timeout (use Time(0) as default; update loop will set it)
  last_command_time_ = rclcpp::Time( 0, 0, RCL_ROS_TIME );

  // Reset group action states
  for ( size_t i = 0; i < group_names_.size(); i++ ) {
    GroupActionCommand idle_cmd;
    idle_cmd.active = false;
    rt_group_action_cmds_[i].writeFromNonRT( idle_cmd );
    group_action_states_[i].store( GroupActionState::IDLE );
  }

  // Seed snapshot so non-RT readers see a correctly sized vector before the first update tick.
  rt_joint_position_snapshot_.set( joint_position_states_ );

  // Create action servers
  using namespace std::placeholders;
  drive_flipper_action_server_ = rclcpp_action::create_server<DriveFlipperGroupAction>(
      get_node(), "~/drive_flipper_group",
      std::bind( &VelocityToPositionCommandController::handle_drive_goal, this, _1, _2 ),
      std::bind( &VelocityToPositionCommandController::handle_drive_cancel, this, _1 ),
      std::bind( &VelocityToPositionCommandController::handle_drive_accepted, this, _1 ) );

  sync_flipper_action_server_ = rclcpp_action::create_server<SyncFlipperGroupAction>(
      get_node(), "~/sync_flipper_group",
      std::bind( &VelocityToPositionCommandController::handle_sync_goal, this, _1, _2 ),
      std::bind( &VelocityToPositionCommandController::handle_sync_cancel, this, _1 ),
      std::bind( &VelocityToPositionCommandController::handle_sync_accepted, this, _1 ) );

  RCLCPP_INFO( get_node()->get_logger(), "activate successful" );
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
VelocityToPositionCommandController::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  // Cancel all active group actions
  for ( size_t i = 0; i < group_names_.size(); i++ ) {
    group_action_states_[i].store( GroupActionState::CANCELLED );
  }

  cleanup_monitor_threads();

  drive_flipper_action_server_.reset();
  sync_flipper_action_server_.reset();
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>( nullptr );
  cmd_sub_.reset();
  hard_estop_sub_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

void VelocityToPositionCommandController::cleanup_monitor_threads()
{
  std::lock_guard<std::mutex> lock( action_monitor_threads_mutex_ );
  for ( auto &mt : action_monitor_threads_ ) {
    if ( mt.thread.joinable() ) {
      mt.thread.join();
    }
  }
  action_monitor_threads_.clear();
}

void VelocityToPositionCommandController::reap_finished_monitor_threads()
{
  std::lock_guard<std::mutex> lock( action_monitor_threads_mutex_ );
  auto it = action_monitor_threads_.begin();
  while ( it != action_monitor_threads_.end() ) {
    if ( it->done && it->done->load() ) {
      if ( it->thread.joinable() ) {
        it->thread.join();
      }
      it = action_monitor_threads_.erase( it );
    } else {
      ++it;
    }
  }
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
    std::copy( ( *joint_commands )->data.begin(), ( *joint_commands )->data.end(),
               reference_interfaces_.begin() );
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

void VelocityToPositionCommandController::update_move_states( double vel_command, size_t joint_idx,
                                                              const rclcpp::Time &time )
{
  switch ( move_states_[joint_idx] ) {
  case MOVING:
    if ( vel_command == 0.0 ) {
      const double current_pos = joint_position_states_[joint_idx];
      const double current_vel = joint_velocity_states_[joint_idx];

      braking_profiles_[joint_idx] =
          TrapezoidalProfile::compute_braking( current_pos, current_vel, max_deceleration_ );

      // Clamp target to URDF limits
      if ( !std::isnan( joint_lower_limits_[joint_idx] ) ) {
        braking_profiles_[joint_idx].target_position =
            std::clamp( braking_profiles_[joint_idx].target_position,
                        joint_lower_limits_[joint_idx], joint_upper_limits_[joint_idx] );
      }
      braking_start_times_[joint_idx] = time;
      move_states_[joint_idx] = STOPPING;

      RCLCPP_INFO( get_node()->get_logger(),
                   "[BRAKE] %s: MOVING->STOPPING  vel=%.4f  pos=%.4f  target=%.4f  duration=%.4f",
                   joints_[joint_idx].c_str(), current_vel, current_pos,
                   braking_profiles_[joint_idx].target_position,
                   braking_profiles_[joint_idx].total_time );
    }
    break;

  case STOPPING:
    if ( vel_command != 0.0 ) {
      move_states_[joint_idx] = MOVING;
      desired_positions_[joint_idx] = joint_position_states_[joint_idx];
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

void VelocityToPositionCommandController::reset_sync_offsets( size_t joint_idx )
{
  if ( !sync_pairs_.has_partner( joint_idx ) || std::isnan( joint_position_states_[joint_idx] ) )
    return;
  const size_t p = sync_pairs_.partner( joint_idx );
  if ( !std::isnan( joint_position_states_[p] ) ) {
    sync_pairs_.set_offset( joint_idx, joint_position_states_[p] - joint_position_states_[joint_idx] );
  }
}

void VelocityToPositionCommandController::update_sync_offsets()
{
  for ( size_t joint_idx = 0; joint_idx < joints_.size(); joint_idx++ ) {
    // Only update offsets for MOVING joints that are NOT synced (moving independently).
    if ( move_states_[joint_idx] != MOVING )
      continue;
    if ( !sync_pairs_.has_partner( joint_idx ) )
      continue;
    if ( sync_states_[joint_idx] || std::isnan( joint_position_states_[joint_idx] ) )
      continue;
    const size_t p = sync_pairs_.partner( joint_idx );
    if ( !std::isnan( joint_position_states_[p] ) ) {
      sync_pairs_.set_offset( joint_idx,
                              joint_position_states_[p] - joint_position_states_[joint_idx] );
    }
  }
}

double VelocityToPositionCommandController::sync_pd_control( size_t joint_idx, double vel_command )
{
  if ( !sync_pairs_.has_partner( joint_idx ) || sync_pairs_.is_offset_nan( joint_idx ) )
    return 0.0;

  const size_t p = sync_pairs_.partner( joint_idx );
  if ( std::isnan( joint_position_states_[p] ) )
    return 0.0;

  // P-term: proportional to position offset error
  const double current_diff = joint_position_states_[p] - joint_position_states_[joint_idx];
  const double offset_error = current_diff - sync_pairs_.get_offset( joint_idx );
  const double p_term = kp_sync_ * offset_error;

  // D-term: damps oscillation using velocity difference between partners
  const double vel_diff = joint_velocity_states_[p] - joint_velocity_states_[joint_idx];
  const double d_term = kd_sync_ * vel_diff;

  double correction = p_term + d_term;

  // Clamp correction to prevent excessive corrections at high speeds
  double max_correction =
      std::max( std::abs( vel_command ) * sync_velocity_factor_, sync_velocity_min_threshold_ );
  return std::clamp( correction, -max_correction, max_correction );
}

// ---------------------------------------------------------------------------
// Control law
// ---------------------------------------------------------------------------

double VelocityToPositionCommandController::pos_pd_control( size_t joint_idx, double vel_command,
                                                            const rclcpp::Duration &period )
{
  const double dt = period.seconds();
  // feedforward position from velocity command
  desired_positions_[joint_idx] += vel_command * dt;
  // PD control to achieve desired velocity
  const double vel_p = kp_ * ( vel_command - joint_velocity_states_[joint_idx] ) * dt;
  const double vel_d = kd_ * joint_velocity_states_[joint_idx] * dt;

  return desired_positions_[joint_idx] + vel_p - vel_d;
}

double VelocityToPositionCommandController::position_control( size_t joint_idx, double vel_command,
                                                              const rclcpp::Duration &period )
{
  double next_position = pos_pd_control( joint_idx, vel_command, period );
  if ( sync_states_[joint_idx] ) {
    const double sync_correction = sync_pd_control( joint_idx, vel_command );
    const double corrected_position = next_position + sync_correction;

    // Prevent sync correction from causing movement opposite to the commanded direction
    if ( vel_command > 0.0 ) {
      next_position = std::max( corrected_position, joint_position_states_[joint_idx] );
    } else if ( vel_command < 0.0 ) {
      next_position = std::min( corrected_position, joint_position_states_[joint_idx] );
    } else {
      next_position = corrected_position;
    }
  }

  return next_position;
}

// ---------------------------------------------------------------------------
// Debug publishers
// ---------------------------------------------------------------------------

void VelocityToPositionCommandController::update_debug_publishers( bool enable )
{
  if ( enable ) {
    if ( !rt_debug_in_js_pub_ ) {
      rt_debug_in_js_pub_ =
          std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
              get_node()->create_publisher<sensor_msgs::msg::JointState>( "~/debug_in_joint_states",
                                                                          10 ) );
      // Pre-allocate message fields once
      rt_debug_in_js_pub_->msg_.name = joints_;
      rt_debug_in_js_pub_->msg_.velocity.resize( joints_.size(), 0.0 );
    }
    if ( !rt_debug_out_js_pub_ ) {
      rt_debug_out_js_pub_ =
          std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
              get_node()->create_publisher<sensor_msgs::msg::JointState>(
                  "~/debug_out_joint_states", 10 ) );
      // Pre-allocate message fields once
      rt_debug_out_js_pub_->msg_.name = joints_;
      rt_debug_out_js_pub_->msg_.position.resize( joints_.size(), 0.0 );
    }
    if ( !rt_sync_status_pub_ ) {
      rt_sync_status_pub_ =
          std::make_shared<realtime_tools::RealtimePublisher<hector_ros_controllers_msgs::msg::SyncStatus>>(
              get_node()->create_publisher<hector_ros_controllers_msgs::msg::SyncStatus>(
                  "~/sync_status", 10 ) );
    }
    // Set flag last -- publishers are fully initialized before RT loop sees them
    debug_pubs_enabled_.store( true, std::memory_order_release );
    RCLCPP_INFO( get_node()->get_logger(), "Debug publishers enabled" );
  } else {
    // Clear flag first -- RT loop stops accessing publishers before we destroy them
    debug_pubs_enabled_.store( false, std::memory_order_release );
    rt_debug_in_js_pub_.reset();
    rt_debug_out_js_pub_.reset();
    rt_sync_status_pub_.reset();
  }
}

void VelocityToPositionCommandController::publish_debug_joint_state_in()
{
  if ( !debug_pubs_enabled_.load( std::memory_order_acquire ) )
    return;
  if ( !rt_debug_in_js_pub_->trylock() )
    return;

  rt_debug_in_js_pub_->msg_.header.stamp = get_node()->now();
  for ( size_t i = 0; i < joints_.size(); i++ ) {
    rt_debug_in_js_pub_->msg_.velocity[i] = reference_interfaces_[i];
  }
  rt_debug_in_js_pub_->unlockAndPublish();
}

void VelocityToPositionCommandController::publish_debug_joint_state_out(
    const std::vector<double> &positions )
{
  if ( !debug_pubs_enabled_.load( std::memory_order_acquire ) )
    return;
  if ( !rt_debug_out_js_pub_->trylock() )
    return;

  rt_debug_out_js_pub_->msg_.header.stamp = get_node()->now();
  for ( size_t i = 0; i < positions.size(); i++ ) {
    rt_debug_out_js_pub_->msg_.position[i] = positions[i];
  }
  rt_debug_out_js_pub_->unlockAndPublish();
}

void VelocityToPositionCommandController::publish_sync_status( const std::vector<double> &vel_commands_out )
{
  if ( !rt_sync_status_pub_ || !rt_sync_status_pub_->trylock() )
    return;

  auto &msg = rt_sync_status_pub_->msg_;
  msg.header.stamp = get_node()->now();
  msg.joint_names = joints_;
  msg.vel_command_in.resize( joints_.size() );
  msg.vel_command_out = vel_commands_out;
  msg.desired_sync_offset.resize( joints_.size() );
  msg.current_sync_offset.resize( joints_.size() );

  for ( size_t i = 0; i < joints_.size(); ++i ) {
    msg.vel_command_in[i] = reference_interfaces_[i];

    if ( sync_pairs_.has_partner( i ) ) {
      const size_t p = sync_pairs_.partner( i );
      msg.desired_sync_offset[i] = sync_pairs_.get_offset( i );
      msg.current_sync_offset[i] = joint_position_states_[p] - joint_position_states_[i];
    } else {
      msg.desired_sync_offset[i] = std::numeric_limits<double>::quiet_NaN();
      msg.current_sync_offset[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  rt_sync_status_pub_->unlockAndPublish();
}

// ---------------------------------------------------------------------------
// Main update
// ---------------------------------------------------------------------------

controller_interface::return_type
VelocityToPositionCommandController::update_and_write_commands( const rclcpp::Time &time,
                                                                const rclcpp::Duration &period )
{
  update_joint_states_if_valid();

  // Snapshot positions for non-RT consumers (action feedback). try_set is non-blocking.
  rt_joint_position_snapshot_.try_set( joint_position_states_ );

  publish_debug_joint_state_in();

  // Velocity command timeout: zero references if no non-zero command received within timeout
  if ( velocity_command_timeout_ > 0.0 ) {
    bool has_nonzero_command = false;
    for ( size_t i = 0; i < reference_interfaces_.size(); i++ ) {
      if ( !std::isnan( reference_interfaces_[i] ) && reference_interfaces_[i] != 0.0 ) {
        has_nonzero_command = true;
        break;
      }
    }
    if ( has_nonzero_command ) {
      last_command_time_ = time;
    } else if ( ( time - last_command_time_ ).seconds() > velocity_command_timeout_ ) {
      for ( size_t i = 0; i < reference_interfaces_.size(); i++ ) {
        if ( !std::isnan( reference_interfaces_[i] ) ) {
          reference_interfaces_[i] = 0.0;
        }
      }
    }
  }

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

  // Process active group actions (drive/sync). This may write position commands directly
  // and set reference_interfaces_ to NaN for controlled joints to skip normal control.
  successful &= process_group_actions( time );

  update_sync_states( reference_interfaces_ );
  update_sync_offsets();

  const double dt = period.seconds();
  std::vector<double> vel_commands_out( joints_.size(), std::numeric_limits<double>::quiet_NaN() );

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

    // Clamp velocity reference to max_velocity (use abs to be safe if param is set negative)
    const double velocity_limit = std::abs( max_velocity_ );
    const double vel_command =
        std::clamp( reference_interfaces_[joint_idx], -velocity_limit, velocity_limit );

    update_move_states( vel_command, joint_idx, time );

    double pos_command = std::numeric_limits<double>::quiet_NaN();
    switch ( move_states_[joint_idx] ) {
    case STOPPED:
      pos_command = hold_positions_[joint_idx];
      break;

    case STOPPING: {
      const double elapsed = ( time - braking_start_times_[joint_idx] ).seconds();
      const auto [pos, vel] = braking_profiles_[joint_idx].evaluate( elapsed );

      if ( elapsed >= braking_profiles_[joint_idx].total_time ) {
        // Profile complete -- hold at target
        const double target = braking_profiles_[joint_idx].target_position;
        hold_positions_[joint_idx] = target;
        desired_positions_[joint_idx] = target;
        pos_command = target;
        move_states_[joint_idx] = STOPPED;

        RCLCPP_INFO( get_node()->get_logger(), "[BRAKE] %s: stopped at pos=%.4f",
                     joints_[joint_idx].c_str(), target );
      } else {
        desired_positions_[joint_idx] = pos;
        pos_command = pos;
        hold_positions_[joint_idx] = pos;
      }
      break;
    }

    case MOVING:
      pos_command = position_control( joint_idx, vel_command, period );
      // Update hold position to desired position (where joint *should* be)
      hold_positions_[joint_idx] = desired_positions_[joint_idx];
      break;
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

    if ( dt > 0.0 ) {
      vel_commands_out[joint_idx] = ( pos_command - joint_position_states_[joint_idx] ) / dt;
    }

    successful &= command_interfaces_[joint_idx].set_value( pos_command );
  }

  publish_debug_joint_state_out( desired_positions_ );
  publish_sync_status( vel_commands_out );

  if ( !successful )
    return controller_interface::return_type::ERROR;

  return controller_interface::return_type::OK;
}

} // namespace velocity_to_position_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS( velocity_to_position_command_controller::VelocityToPositionCommandController,
                        controller_interface::ChainableControllerInterface )
