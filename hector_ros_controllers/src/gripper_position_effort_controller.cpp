#include "gripper_position_effort_controller/gripper_position_effort_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "urdf_parser/urdf_parser.h"

namespace gripper_position_effort_controller
{

namespace
{
double clamp_to_limits( double value, double lower, double upper )
{
  if ( !std::isnan( lower ) && value < lower )
    return lower;
  if ( !std::isnan( upper ) && value > upper )
    return upper;
  return value;
}
} // namespace

GripperPositionEffortController::GripperPositionEffortController()
    : controller_interface::ControllerInterface(),
      joint_lower_limit_( std::numeric_limits<double>::quiet_NaN() ),
      joint_upper_limit_( std::numeric_limits<double>::quiet_NaN() ), target_{ 0.0, 0.0 },
      action_monitor_period_( rclcpp::Duration::from_seconds( 0 ) ),
      last_movement_time_( 0, 0, RCL_ROS_TIME ), input_seq_counter_( 0 ), action_cmd_seq_( 0 ),
      last_consumed_action_seq_( 0 ), position_cmd_seq_( 0 ), velocity_cmd_seq_( 0 ),
      last_consumed_position_seq_( 0 ), last_consumed_velocity_seq_( 0 ),
      last_velocity_msg_time_( 0, 0, RCL_ROS_TIME ), velocity_cached_valid_( false ),
      last_is_grasped_publish_time_( 0, 0, RCL_ROS_TIME ), is_grasped_( false ),
      is_grasped_dwell_counter_( 0 )
{
}

controller_interface::CallbackReturn GripperPositionEffortController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>( get_node() );
    params_ = param_listener_->get_params();
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Exception thrown during init: %s", e.what() );
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GripperPositionEffortController::on_configure( const rclcpp_lifecycle::State & )
{
  params_ = param_listener_->get_params();

  if ( params_.joint.empty() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "'joint' parameter must not be empty" );
    return controller_interface::CallbackReturn::ERROR;
  }

  action_monitor_period_ = rclcpp::Duration::from_seconds( 1.0 / params_.action_monitor_rate );

  parse_joint_limits_from_urdf();

  if ( params_.max_effort_limit > 0.0 && params_.default_max_effort > params_.max_effort_limit ) {
    RCLCPP_WARN( get_node()->get_logger(),
                 "default_max_effort (%.3f) exceeds max_effort_limit (%.3f); will be clamped",
                 params_.default_max_effort, params_.max_effort_limit );
  }

  RCLCPP_INFO( get_node()->get_logger(),
               "Configured for joint '%s', effort_command_interface='%s', default_max_effort=%.3f, "
               "max_effort_limit=%.3f, action_monitor_rate=%.1f Hz",
               params_.joint.c_str(), params_.effort_command_interface.c_str(),
               params_.default_max_effort, params_.max_effort_limit, params_.action_monitor_rate );

  return controller_interface::CallbackReturn::SUCCESS;
}

void GripperPositionEffortController::parse_joint_limits_from_urdf()
{
  joint_lower_limit_ = std::numeric_limits<double>::quiet_NaN();
  joint_upper_limit_ = std::numeric_limits<double>::quiet_NaN();

  const std::string &urdf_string = this->get_robot_description();
  if ( urdf_string.empty() ) {
    RCLCPP_WARN( get_node()->get_logger(),
                 "Robot description is empty; integrated velocity will not be clamped" );
    return;
  }

  auto urdf_model = urdf::parseURDF( urdf_string );
  if ( !urdf_model ) {
    RCLCPP_WARN( get_node()->get_logger(),
                 "Failed to parse URDF; integrated velocity will not be clamped" );
    return;
  }

  auto urdf_joint = urdf_model->getJoint( params_.joint );
  if ( !urdf_joint ) {
    RCLCPP_WARN( get_node()->get_logger(), "Joint '%s' not found in URDF; no limits applied",
                 params_.joint.c_str() );
    return;
  }

  if ( urdf_joint->type == urdf::Joint::CONTINUOUS )
    return;

  if ( ( urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::PRISMATIC ) &&
       urdf_joint->limits ) {
    joint_lower_limit_ = urdf_joint->limits->lower;
    joint_upper_limit_ = urdf_joint->limits->upper;
    RCLCPP_INFO( get_node()->get_logger(), "Joint '%s' position limits: [%f, %f]",
                 params_.joint.c_str(), joint_lower_limit_, joint_upper_limit_ );
  }
}

controller_interface::InterfaceConfiguration
GripperPositionEffortController::command_interface_configuration() const
{
  std::vector<std::string> names;
  names.reserve( 2 );
  names.push_back( params_.joint + "/" + hardware_interface::HW_IF_POSITION );
  if ( params_.effort_command_interface == "required" ) {
    names.push_back( params_.joint + "/" + hardware_interface::HW_IF_EFFORT );
  }
  return { controller_interface::interface_configuration_type::INDIVIDUAL, std::move( names ) };
}

controller_interface::InterfaceConfiguration
GripperPositionEffortController::state_interface_configuration() const
{
  return { controller_interface::interface_configuration_type::INDIVIDUAL,
           { params_.joint + "/" + hardware_interface::HW_IF_POSITION,
             params_.joint + "/" + hardware_interface::HW_IF_VELOCITY,
             params_.joint + "/" + hardware_interface::HW_IF_EFFORT } };
}

controller_interface::CallbackReturn
GripperPositionEffortController::on_activate( const rclcpp_lifecycle::State & )
{
  const bool effort_required = ( params_.effort_command_interface == "required" );

  // Locate command interfaces
  auto pos_cmd_it =
      std::find_if( command_interfaces_.begin(), command_interfaces_.end(),
                    [this]( const hardware_interface::LoanedCommandInterface &ci ) {
                      return ci.get_prefix_name() == params_.joint &&
                             ci.get_interface_name() == hardware_interface::HW_IF_POSITION;
                    } );
  if ( pos_cmd_it == command_interfaces_.end() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Expected position command interface for joint '%s'",
                  params_.joint.c_str() );
    return controller_interface::CallbackReturn::ERROR;
  }

  auto eff_cmd_it = command_interfaces_.end();
  if ( effort_required ) {
    eff_cmd_it = std::find_if( command_interfaces_.begin(), command_interfaces_.end(),
                               [this]( const hardware_interface::LoanedCommandInterface &ci ) {
                                 return ci.get_prefix_name() == params_.joint &&
                                        ci.get_interface_name() == hardware_interface::HW_IF_EFFORT;
                               } );
    if ( eff_cmd_it == command_interfaces_.end() ) {
      RCLCPP_ERROR( get_node()->get_logger(),
                    "effort_command_interface='required' but no effort command interface was "
                    "claimed for joint '%s'",
                    params_.joint.c_str() );
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  auto pos_state_it =
      std::find_if( state_interfaces_.begin(), state_interfaces_.end(),
                    [this]( const hardware_interface::LoanedStateInterface &si ) {
                      return si.get_prefix_name() == params_.joint &&
                             si.get_interface_name() == hardware_interface::HW_IF_POSITION;
                    } );
  auto vel_state_it =
      std::find_if( state_interfaces_.begin(), state_interfaces_.end(),
                    [this]( const hardware_interface::LoanedStateInterface &si ) {
                      return si.get_prefix_name() == params_.joint &&
                             si.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                    } );
  auto eff_state_it =
      std::find_if( state_interfaces_.begin(), state_interfaces_.end(),
                    [this]( const hardware_interface::LoanedStateInterface &si ) {
                      return si.get_prefix_name() == params_.joint &&
                             si.get_interface_name() == hardware_interface::HW_IF_EFFORT;
                    } );
  if ( pos_state_it == state_interfaces_.end() || vel_state_it == state_interfaces_.end() ||
       eff_state_it == state_interfaces_.end() ) {
    RCLCPP_ERROR( get_node()->get_logger(),
                  "Expected position, velocity and effort state interfaces for joint '%s'",
                  params_.joint.c_str() );
    return controller_interface::CallbackReturn::ERROR;
  }

  position_command_interface_ = *pos_cmd_it;
  if ( effort_required ) {
    effort_command_interface_ = *eff_cmd_it;
  } else {
    effort_command_interface_ = std::nullopt;
  }
  position_state_interface_ = *pos_state_it;
  velocity_state_interface_ = *vel_state_it;
  effort_state_interface_ = *eff_state_it;

  // Initialise target to current position so the actuator does not jump on activation
  const double current_position = position_state_interface_->get().get_optional().value_or( 0.0 );
  target_.position = current_position;
  target_.max_effort = params_.default_max_effort;

  // Reset action / topic state
  store_goal_slot( rt_active_goal_, nullptr );
  store_goal_slot( previous_rt_goal_, nullptr );
  rt_position_cmd_.writeFromNonRT( nullptr );
  rt_velocity_cmd_.writeFromNonRT( nullptr );
  input_seq_counter_.store( 0 );
  action_cmd_seq_.store( 0 );
  position_cmd_seq_.store( 0 );
  velocity_cmd_seq_.store( 0 );
  last_consumed_action_seq_ = 0;
  last_consumed_position_seq_ = 0;
  last_consumed_velocity_seq_ = 0;
  last_velocity_msg_time_ = rclcpp::Time( 0, 0, RCL_ROS_TIME );
  velocity_cached_valid_ = false;
  is_grasped_ = false;
  is_grasped_dwell_counter_ = 0;
  last_is_grasped_publish_time_ = rclcpp::Time( 0, 0, RCL_ROS_TIME );

  pre_alloc_result_ = std::make_shared<GripperCommandAction::Result>();

  // Action server
  action_server_ = rclcpp_action::create_server<GripperCommandAction>(
      get_node(), "~/gripper_cmd",
      std::bind( &GripperPositionEffortController::goal_callback, this, std::placeholders::_1,
                 std::placeholders::_2 ),
      std::bind( &GripperPositionEffortController::cancel_callback, this, std::placeholders::_1 ),
      std::bind( &GripperPositionEffortController::accepted_callback, this, std::placeholders::_1 ) );

  // Topic subscribers
  auto qos = rclcpp::QoS( rclcpp::KeepLast( 1 ) );
  position_cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
      "~/position_command", qos, [this]( std_msgs::msg::Float64::SharedPtr msg ) {
        rt_position_cmd_.writeFromNonRT( msg );
        position_cmd_seq_.store( input_seq_counter_.fetch_add( 1 ) + 1 );
      } );
  velocity_cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
      "~/velocity_command", qos, [this]( std_msgs::msg::Float64::SharedPtr msg ) {
        rt_velocity_cmd_.writeFromNonRT( msg );
        velocity_cmd_seq_.store( input_seq_counter_.fetch_add( 1 ) + 1 );
      } );

  // is_grasped publisher
  rt_is_grasped_pub_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(
      get_node()->create_publisher<std_msgs::msg::Bool>( "~/is_grasped", qos ) );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GripperPositionEffortController::on_deactivate( const rclcpp_lifecycle::State & )
{
  preempt_active_goal( "controller deactivated" );
  // Flush before tearing down the timer so the canceled status reaches the client.
  flush_previous_goal_if_any();
  position_command_interface_ = std::nullopt;
  effort_command_interface_ = std::nullopt;
  position_state_interface_ = std::nullopt;
  velocity_state_interface_ = std::nullopt;
  effort_state_interface_ = std::nullopt;
  release_interfaces();
  action_server_.reset();
  position_cmd_sub_.reset();
  velocity_cmd_sub_.reset();
  rt_is_grasped_pub_.reset();
  goal_handle_timer_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse GripperPositionEffortController::goal_callback(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperCommandAction::Goal> goal )
{
  if ( !std::isfinite( goal->command.position ) || !std::isfinite( goal->command.max_effort ) ) {
    RCLCPP_WARN( get_node()->get_logger(),
                 "Rejecting GripperCommand goal with non-finite position (%f) or max_effort (%f)",
                 goal->command.position, goal->command.max_effort );
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO( get_node()->get_logger(), "Received & accepted new GripperCommand goal" );
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
GripperPositionEffortController::cancel_callback( std::shared_ptr<GoalHandle> goal_handle )
{
  const auto active_goal = load_goal_slot( rt_active_goal_ );
  if ( !active_goal || active_goal->gh_ != goal_handle ) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  RCLCPP_INFO( get_node()->get_logger(), "Cancelling active GripperCommand goal" );
  set_hold_position();
  // Discard any queued action command so update() can't apply it after the cancel.
  clear_pending_action_command();
  auto res = std::make_shared<GripperCommandAction::Result>();
  active_goal->setCanceled( res );
  // Non-RT executor thread: flush synchronously so canceled status reaches the client.
  active_goal->runNonRealtime();
  clear_active_goal();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperPositionEffortController::accepted_callback( std::shared_ptr<GoalHandle> goal_handle )
{
  // goal_callback already validated finiteness.
  preempt_active_goal( "superseded by new action goal" );

  const auto goal = goal_handle->get_goal();
  Command cmd;
  cmd.position = goal->command.position;
  cmd.max_effort =
      ( goal->command.max_effort > 0.0 ) ? goal->command.max_effort : params_.default_max_effort;

  // Flush the prior goal's deferred terminal state BEFORE making the new goal active —
  // otherwise a racing RT clear could stash the new goal into previous_rt_goal_ and we'd
  // lose the old terminal flush.
  flush_previous_goal_if_any();

  rt_action_command_.writeFromNonRT( cmd );
  action_cmd_seq_.store( input_seq_counter_.fetch_add( 1 ) + 1 );

  // last_movement_time_ and pre_alloc_result_ flags are owned by the RT update thread.

  auto rt_goal = std::make_shared<RealtimeGoalHandle>( goal_handle );
  rt_goal->execute();
  store_goal_slot( rt_active_goal_, rt_goal );

  goal_handle_timer_.reset();
  goal_handle_timer_ =
      get_node()->create_wall_timer( action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
                                     std::bind( &RealtimeGoalHandle::runNonRealtime, rt_goal ) );
}

void GripperPositionEffortController::preempt_active_goal( const std::string &reason )
{
  // Callable from both non-RT and RT (topic-driven preemption in update()). Goal-slot
  // access is lock-free; the formatting/allocation here are not hard-RT but match the
  // throttled-log pattern used elsewhere in the controller. runNonRealtime() is NOT
  // called — it would take the wrapper's mutex; the wall_timer flushes the terminal
  // flag on its next tick, and non-RT callers explicitly call flush_previous_goal_if_any.
  const auto active_goal = load_goal_slot( rt_active_goal_ );
  if ( active_goal ) {
    RCLCPP_INFO( get_node()->get_logger(), "Preempting active GripperCommand goal: %s",
                 reason.c_str() );
    clear_pending_action_command();
    auto res = std::make_shared<GripperCommandAction::Result>();
    active_goal->setAborted( res );
    clear_active_goal();
  }
}

void GripperPositionEffortController::clear_active_goal()
{
  // Park the active goal in previous_rt_goal_ so its bound wall_timer can still flush
  // the terminal flag. The timer is reset only in accepted_callback / on_deactivate.
  store_goal_slot( previous_rt_goal_, exchange_goal_slot( rt_active_goal_, nullptr ) );
}

void GripperPositionEffortController::flush_previous_goal_if_any()
{
  auto handle = exchange_goal_slot( previous_rt_goal_, nullptr );
  if ( handle ) {
    handle->runNonRealtime();
  }
}

void GripperPositionEffortController::store_goal_slot( RealtimeGoalHandlePtr &slot,
                                                       RealtimeGoalHandlePtr handle )
{
  std::atomic_store( &slot, std::move( handle ) );
}

GripperPositionEffortController::RealtimeGoalHandlePtr
GripperPositionEffortController::load_goal_slot( const RealtimeGoalHandlePtr &slot )
{
  return std::atomic_load( &slot );
}

GripperPositionEffortController::RealtimeGoalHandlePtr
GripperPositionEffortController::exchange_goal_slot( RealtimeGoalHandlePtr &slot,
                                                     RealtimeGoalHandlePtr handle )
{
  return std::atomic_exchange( &slot, std::move( handle ) );
}

void GripperPositionEffortController::clear_pending_action_command()
{
  // One-cycle race: if update() already snapshotted action_cmd_seq_, it may apply the
  // command once before observing the zero. Accepted; cancel still completes terminally.
  action_cmd_seq_.store( 0 );
}

void GripperPositionEffortController::set_hold_position()
{
  if ( position_state_interface_ ) {
    target_.position = position_state_interface_->get().get_optional().value_or( target_.position );
  }
  target_.max_effort = params_.default_max_effort;
}

controller_interface::return_type
GripperPositionEffortController::update( const rclcpp::Time &time, const rclcpp::Duration &period )
{
  if ( !position_command_interface_ || !position_state_interface_ || !velocity_state_interface_ ||
       !effort_state_interface_ ) {
    return controller_interface::return_type::ERROR;
  }

  const double current_position =
      position_state_interface_->get().get_optional().value_or( target_.position );
  const double current_velocity = velocity_state_interface_->get().get_optional().value_or( 0.0 );
  const double current_effort = effort_state_interface_->get().get_optional().value_or( 0.0 );

  // ----- Input arbitration: true last-writer-wins by sequence number -----
  // Snapshot per-source sequences. Pre-validate so malformed inputs can't win or preempt
  // an active action goal; all snapshotted sources are consumed regardless of validity.
  const uint64_t action_seq = action_cmd_seq_.load();
  const uint64_t pos_seq = position_cmd_seq_.load();
  const uint64_t vel_seq = velocity_cmd_seq_.load();
  const bool action_unconsumed = ( action_seq != 0 && action_seq != last_consumed_action_seq_ );
  const bool pos_unconsumed = ( pos_seq != last_consumed_position_seq_ );
  const bool vel_unconsumed = ( vel_seq != last_consumed_velocity_seq_ );

  Command pending_action{};
  bool action_valid = false;
  if ( action_unconsumed ) {
    pending_action = *rt_action_command_.readFromRT();
    action_valid =
        std::isfinite( pending_action.position ) && std::isfinite( pending_action.max_effort );
  }
  std::shared_ptr<std_msgs::msg::Float64> pending_pos;
  bool pos_valid = false;
  if ( pos_unconsumed ) {
    pending_pos = *rt_position_cmd_.readFromRT();
    pos_valid = pending_pos && std::isfinite( pending_pos->data );
  }
  std::shared_ptr<std_msgs::msg::Float64> pending_vel;
  bool vel_valid = false;
  if ( vel_unconsumed ) {
    pending_vel = *rt_velocity_cmd_.readFromRT();
    vel_valid = pending_vel && std::isfinite( pending_vel->data );
  }

  InputSource winner = InputSource::None;
  uint64_t winning_seq = 0;
  if ( action_valid && action_seq > winning_seq ) {
    winner = InputSource::Action;
    winning_seq = action_seq;
  }
  if ( pos_valid && pos_seq > winning_seq ) {
    winner = InputSource::PositionTopic;
    winning_seq = pos_seq;
  }
  if ( vel_valid && vel_seq > winning_seq ) {
    winner = InputSource::VelocityTopic;
    winning_seq = vel_seq;
  }

  // Mark every snapshotted source consumed — including invalid ones, so a stuck NaN
  // publisher doesn't keep "winning".
  if ( action_unconsumed )
    last_consumed_action_seq_ = action_seq;
  if ( pos_unconsumed )
    last_consumed_position_seq_ = pos_seq;
  if ( vel_unconsumed )
    last_consumed_velocity_seq_ = vel_seq;

  if ( ( action_unconsumed && !action_valid ) || ( pos_unconsumed && !pos_valid ) ||
       ( vel_unconsumed && !vel_valid ) ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Dropping malformed input (action=%d pos=%d vel=%d)",
                          action_unconsumed && !action_valid, pos_unconsumed && !pos_valid,
                          vel_unconsumed && !vel_valid );
  }

  switch ( winner ) {
  case InputSource::Action:
    target_.position =
        clamp_to_limits( pending_action.position, joint_lower_limit_, joint_upper_limit_ );
    target_.max_effort = pending_action.max_effort;
    last_movement_time_ = time;
    // Non-velocity source won — invalidate the cached velocity so None cycles don't
    // resume integrating it. Flag-only; calling writeFromNonRT here would be RT-unsafe.
    velocity_cached_valid_ = false;
    break;
  case InputSource::PositionTopic:
    preempt_active_goal( "preempted by position_command topic" );
    target_.position = clamp_to_limits( pending_pos->data, joint_lower_limit_, joint_upper_limit_ );
    target_.max_effort = params_.default_max_effort;
    velocity_cached_valid_ = false;
    break;
  case InputSource::VelocityTopic:
    preempt_active_goal( "preempted by velocity_command topic" );
    last_velocity_msg_time_ = time;
    velocity_cached_valid_ = true;
    target_.position = clamp_to_limits( target_.position + pending_vel->data * period.seconds(),
                                        joint_lower_limit_, joint_upper_limit_ );
    target_.max_effort = params_.default_max_effort;
    break;
  case InputSource::None:
    // No fresh winner — keep integrating any cached velocity within the watchdog window.
    continue_velocity_integration_if_within_watchdog( time, period );
    break;
  }

  // ----- Write commands -----
  if ( !std::isfinite( target_.position ) || !std::isfinite( target_.max_effort ) ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Refusing to write non-finite command (pos=%f, eff=%f)", target_.position,
                          target_.max_effort );
  } else {
    // Clamp the effort to max_effort_limit (0 = no limit)
    double effort_to_write = std::fabs( target_.max_effort );
    if ( params_.max_effort_limit > 0.0 && effort_to_write > params_.max_effort_limit ) {
      RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 1000,
                            "Clamping commanded max_effort %.3f to max_effort_limit %.3f",
                            effort_to_write, params_.max_effort_limit );
      effort_to_write = params_.max_effort_limit;
    }
    std::ignore = position_command_interface_->get().set_value( target_.position );
    if ( effort_command_interface_ ) {
      std::ignore = effort_command_interface_->get().set_value( effort_to_write );
    }
  }

  // ----- Action goal monitoring -----
  // Read AFTER arbitration so a topic-preempted goal isn't treated as active here.
  auto active_goal = load_goal_slot( rt_active_goal_ );
  if ( active_goal ) {
    const double error_position = target_.position - current_position;
    check_for_success( time, error_position, current_position, current_velocity, current_effort );

    // check_for_success may have terminated the goal — re-check before publishing feedback.
    active_goal = load_goal_slot( rt_active_goal_ );
    if ( active_goal ) {
      auto feedback = std::make_shared<GripperCommandAction::Feedback>();
      feedback->position = current_position;
      feedback->effort = current_effort;
      feedback->reached_goal = false;
      feedback->stalled = false;
      active_goal->setFeedback( feedback );
    }
  }

  // ----- is_grasped detection from measured velocity and effort state -----
  if ( std::fabs( current_velocity ) < params_.is_grasped_velocity_threshold &&
       std::fabs( current_effort ) > params_.is_grasped_effort_threshold ) {
    if ( is_grasped_dwell_counter_ < params_.is_grasped_dwell_cycles ) {
      ++is_grasped_dwell_counter_;
    }
  } else {
    is_grasped_dwell_counter_ = 0;
  }
  is_grasped_ = ( is_grasped_dwell_counter_ >= params_.is_grasped_dwell_cycles );

  if ( rt_is_grasped_pub_ ) {
    if ( last_is_grasped_publish_time_.nanoseconds() == 0 ||
         ( time - last_is_grasped_publish_time_ ) >= action_monitor_period_ ) {
      if ( rt_is_grasped_pub_->trylock() ) {
        rt_is_grasped_pub_->msg_.data = is_grasped_;
        rt_is_grasped_pub_->unlockAndPublish();
        last_is_grasped_publish_time_ = time;
      }
    }
  }

  return controller_interface::return_type::OK;
}

bool GripperPositionEffortController::continue_velocity_integration_if_within_watchdog(
    const rclcpp::Time &time, const rclcpp::Duration &period )
{
  if ( !velocity_cached_valid_ )
    return false;

  const double age = ( time - last_velocity_msg_time_ ).seconds();
  if ( age > params_.velocity_command_timeout ) {
    velocity_cached_valid_ = false;
    return false;
  }

  auto msg = *rt_velocity_cmd_.readFromRT();
  if ( !msg ) {
    velocity_cached_valid_ = false;
    return false;
  }
  if ( !std::isfinite( msg->data ) ) {
    velocity_cached_valid_ = false;
    return false;
  }

  target_.position = clamp_to_limits( target_.position + msg->data * period.seconds(),
                                      joint_lower_limit_, joint_upper_limit_ );
  target_.max_effort = params_.default_max_effort;
  return true;
}

void GripperPositionEffortController::check_for_success( const rclcpp::Time &time,
                                                         double error_position,
                                                         double current_position,
                                                         double current_velocity,
                                                         double current_effort )
{
  const auto active_goal = load_goal_slot( rt_active_goal_ );
  if ( !active_goal )
    return;

  if ( std::fabs( error_position ) < params_.goal_tolerance ) {
    pre_alloc_result_->position = current_position;
    pre_alloc_result_->effort = current_effort;
    pre_alloc_result_->reached_goal = true;
    pre_alloc_result_->stalled = false;
    active_goal->setSucceeded( pre_alloc_result_ );
    clear_active_goal();
    return;
  }

  if ( std::fabs( current_velocity ) > params_.stall_velocity_threshold ) {
    last_movement_time_ = time;
    return;
  }

  if ( ( time - last_movement_time_ ).seconds() > params_.stall_timeout ) {
    pre_alloc_result_->position = current_position;
    pre_alloc_result_->effort = current_effort;
    pre_alloc_result_->reached_goal = false;
    pre_alloc_result_->stalled = true;
    if ( params_.allow_stalling ) {
      active_goal->setSucceeded( pre_alloc_result_ );
    } else {
      active_goal->setAborted( pre_alloc_result_ );
    }
    clear_active_goal();
  }
}

} // namespace gripper_position_effort_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS( gripper_position_effort_controller::GripperPositionEffortController,
                        controller_interface::ControllerInterface )
