#include "max_effort_gripper_action_controller/max_effort_gripper_action_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "urdf_parser/urdf_parser.h"

namespace max_effort_gripper_action_controller
{

namespace
{
constexpr const char *kPositionInterface = hardware_interface::HW_IF_POSITION;
constexpr const char *kEffortInterface = hardware_interface::HW_IF_EFFORT;
constexpr const char *kVelocityInterface = hardware_interface::HW_IF_VELOCITY;

double clamp_to_limits( double value, double lower, double upper )
{
  // Caller is responsible for rejecting NaN before this point; assert in debug builds.
  if ( !std::isnan( lower ) && value < lower )
    return lower;
  if ( !std::isnan( upper ) && value > upper )
    return upper;
  return value;
}
} // namespace

MaxEffortGripperActionController::MaxEffortGripperActionController()
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

controller_interface::CallbackReturn MaxEffortGripperActionController::on_init()
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
MaxEffortGripperActionController::on_configure( const rclcpp_lifecycle::State & )
{
  params_ = param_listener_->get_params();

  if ( params_.joint.empty() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "'joint' parameter must not be empty" );
    return controller_interface::CallbackReturn::ERROR;
  }

  action_monitor_period_ = rclcpp::Duration::from_seconds( 1.0 / params_.action_monitor_rate );

  parse_joint_limits_from_urdf();

  RCLCPP_INFO( get_node()->get_logger(),
               "Configured for joint '%s', default_max_effort=%.3f, action_monitor_rate=%.1f Hz",
               params_.joint.c_str(), params_.default_max_effort, params_.action_monitor_rate );

  return controller_interface::CallbackReturn::SUCCESS;
}

void MaxEffortGripperActionController::parse_joint_limits_from_urdf()
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
MaxEffortGripperActionController::command_interface_configuration() const
{
  return { controller_interface::interface_configuration_type::INDIVIDUAL,
           { params_.joint + "/" + kPositionInterface, params_.joint + "/" + kEffortInterface } };
}

controller_interface::InterfaceConfiguration
MaxEffortGripperActionController::state_interface_configuration() const
{
  return { controller_interface::interface_configuration_type::INDIVIDUAL,
           { params_.joint + "/" + kPositionInterface, params_.joint + "/" + kVelocityInterface,
             params_.joint + "/" + kEffortInterface } };
}

controller_interface::CallbackReturn
MaxEffortGripperActionController::on_activate( const rclcpp_lifecycle::State & )
{
  // Locate command interfaces
  auto pos_cmd_it = std::find_if( command_interfaces_.begin(), command_interfaces_.end(),
                                  [this]( const hardware_interface::LoanedCommandInterface &ci ) {
                                    return ci.get_prefix_name() == params_.joint &&
                                           ci.get_interface_name() == kPositionInterface;
                                  } );
  auto eff_cmd_it = std::find_if( command_interfaces_.begin(), command_interfaces_.end(),
                                  [this]( const hardware_interface::LoanedCommandInterface &ci ) {
                                    return ci.get_prefix_name() == params_.joint &&
                                           ci.get_interface_name() == kEffortInterface;
                                  } );
  if ( pos_cmd_it == command_interfaces_.end() || eff_cmd_it == command_interfaces_.end() ) {
    RCLCPP_ERROR( get_node()->get_logger(),
                  "Expected position and effort command interfaces for joint '%s'",
                  params_.joint.c_str() );
    return controller_interface::CallbackReturn::ERROR;
  }

  auto pos_state_it = std::find_if( state_interfaces_.begin(), state_interfaces_.end(),
                                    [this]( const hardware_interface::LoanedStateInterface &si ) {
                                      return si.get_prefix_name() == params_.joint &&
                                             si.get_interface_name() == kPositionInterface;
                                    } );
  auto vel_state_it = std::find_if( state_interfaces_.begin(), state_interfaces_.end(),
                                    [this]( const hardware_interface::LoanedStateInterface &si ) {
                                      return si.get_prefix_name() == params_.joint &&
                                             si.get_interface_name() == kVelocityInterface;
                                    } );
  auto eff_state_it = std::find_if( state_interfaces_.begin(), state_interfaces_.end(),
                                    [this]( const hardware_interface::LoanedStateInterface &si ) {
                                      return si.get_prefix_name() == params_.joint &&
                                             si.get_interface_name() == kEffortInterface;
                                    } );
  if ( pos_state_it == state_interfaces_.end() || vel_state_it == state_interfaces_.end() ||
       eff_state_it == state_interfaces_.end() ) {
    RCLCPP_ERROR( get_node()->get_logger(),
                  "Expected position, velocity and effort state interfaces for joint '%s'",
                  params_.joint.c_str() );
    return controller_interface::CallbackReturn::ERROR;
  }

  position_command_interface_ = *pos_cmd_it;
  effort_command_interface_ = *eff_cmd_it;
  position_state_interface_ = *pos_state_it;
  velocity_state_interface_ = *vel_state_it;
  effort_state_interface_ = *eff_state_it;

  // Initialise target to current position so the actuator does not jump on activation
  const double current_position = position_state_interface_->get().get_optional().value_or( 0.0 );
  target_.position = current_position;
  target_.max_effort = params_.default_max_effort;

  // Reset action / topic state
  rt_active_goal_.writeFromNonRT( RealtimeGoalHandlePtr() );
  store_previous_rt_goal( nullptr );
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
  pre_alloc_result_->position = current_position;
  pre_alloc_result_->effort = 0.0;
  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;

  // Action server
  action_server_ = rclcpp_action::create_server<GripperCommandAction>(
      get_node(), "~/gripper_cmd",
      std::bind( &MaxEffortGripperActionController::goal_callback, this, std::placeholders::_1,
                 std::placeholders::_2 ),
      std::bind( &MaxEffortGripperActionController::cancel_callback, this, std::placeholders::_1 ),
      std::bind( &MaxEffortGripperActionController::accepted_callback, this, std::placeholders::_1 ) );

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
MaxEffortGripperActionController::on_deactivate( const rclcpp_lifecycle::State & )
{
  preempt_active_goal( "controller deactivated" );
  // Flush the just-aborted goal synchronously before tearing down the timer so the
  // notification reaches the action client.
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

rclcpp_action::GoalResponse
MaxEffortGripperActionController::goal_callback( const rclcpp_action::GoalUUID &,
                                                 std::shared_ptr<const GripperCommandAction::Goal> )
{
  RCLCPP_INFO( get_node()->get_logger(), "Received & accepted new GripperCommand goal" );
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
MaxEffortGripperActionController::cancel_callback( std::shared_ptr<GoalHandle> goal_handle )
{
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if ( active_goal && active_goal->gh_ == goal_handle ) {
    RCLCPP_INFO( get_node()->get_logger(), "Cancelling active GripperCommand goal" );
    set_hold_position();
    // Drop any pending action command queued by accepted_callback that update() has not
    // consumed yet — otherwise a cancel arriving before the next update cycle would still
    // execute the goal once.
    clear_pending_action_command();
    auto res = std::make_shared<GripperCommandAction::Result>();
    active_goal->setCanceled( res );
    // We're on a non-RT executor thread; flush the realtime wrapper synchronously so the
    // canceled status reaches the action client even if the wall timer is reset before its
    // next tick.
    active_goal->runNonRealtime();
    clear_active_goal();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MaxEffortGripperActionController::accepted_callback( std::shared_ptr<GoalHandle> goal_handle )
{
  preempt_active_goal( "superseded by new action goal" );

  const auto goal = goal_handle->get_goal();
  if ( !std::isfinite( goal->command.position ) || !std::isfinite( goal->command.max_effort ) ) {
    RCLCPP_WARN( get_node()->get_logger(),
                 "Rejecting action goal with non-finite position (%f) or max_effort (%f)",
                 goal->command.position, goal->command.max_effort );
    auto rt_goal = std::make_shared<RealtimeGoalHandle>( goal_handle );
    rt_goal->execute();
    auto res = std::make_shared<GripperCommandAction::Result>();
    rt_goal->setAborted( res );
    rt_goal->runNonRealtime();
    return;
  }

  Command cmd;
  cmd.position = goal->command.position;
  cmd.max_effort =
      ( goal->command.max_effort > 0.0 ) ? goal->command.max_effort : params_.default_max_effort;

  // Flush any prior goal whose terminal-state flag was set in RT (check_for_success or
  // a topic-driven preempt) but never delivered. This MUST happen before we make the new
  // goal active or queue its command: if update() ran between making the new goal active
  // and flushing, check_for_success could overwrite previous_rt_goal_ with the new goal
  // and lose the old goal's terminal state.
  flush_previous_goal_if_any();

  rt_action_command_.writeFromNonRT( cmd );
  action_cmd_seq_.store( input_seq_counter_.fetch_add( 1 ) + 1 );

  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;
  // last_movement_time_ is written exclusively from the RT update thread (Winner::ACTION
  // and check_for_success). The next update() cycle will set it when it consumes this
  // command. Writing here from non-RT would be a data race on rclcpp::Time.

  auto rt_goal = std::make_shared<RealtimeGoalHandle>( goal_handle );
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT( rt_goal );

  goal_handle_timer_.reset();
  goal_handle_timer_ =
      get_node()->create_wall_timer( action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
                                     std::bind( &RealtimeGoalHandle::runNonRealtime, rt_goal ) );
}

void MaxEffortGripperActionController::preempt_active_goal( const std::string &reason )
{
  // Note: callable from both non-RT (accepted_callback / on_deactivate) and RT (update()
  // topic-driven preemption). We do NOT call runNonRealtime() here because that would block
  // on the wrapper's mutex; safe in non-RT but unsafe in RT. The terminal flag is set; the
  // existing wall_timer will flush it on its next tick (within action_monitor_period_), and
  // accepted_callback / on_deactivate explicitly flush via flush_previous_goal_if_any().
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if ( active_goal ) {
    RCLCPP_INFO( get_node()->get_logger(), "Preempting active GripperCommand goal: %s",
                 reason.c_str() );
    clear_pending_action_command();
    auto res = std::make_shared<GripperCommandAction::Result>();
    res->reached_goal = false;
    res->stalled = false;
    active_goal->setAborted( res );
    clear_active_goal();
  }
}

void MaxEffortGripperActionController::clear_active_goal()
{
  // Move the active goal wrapper into previous_rt_goal_ so the wall_timer that's bound to
  // it can keep flushing pending terminal flags. Do NOT reset goal_handle_timer_ here;
  // doing so would tear down the timer before the deferred succeed/abort/canceled call had
  // a chance to fire, dropping the notification to the action client. The timer is reset
  // in accepted_callback (after a synchronous flush) or in on_deactivate.
  // Use the atomic helper because clear_active_goal() may be called from RT
  // (check_for_success) AND non-RT (cancel_callback / preempt_active_goal).
  store_previous_rt_goal( *rt_active_goal_.readFromNonRT() );
  rt_active_goal_.writeFromNonRT( RealtimeGoalHandlePtr() );
}

void MaxEffortGripperActionController::flush_previous_goal_if_any()
{
  // Atomically yank the slot so a concurrent clear_active_goal cannot stash a fresh
  // wrapper into it after we've decided to flush.
  auto handle = exchange_previous_rt_goal( nullptr );
  if ( handle ) {
    handle->runNonRealtime();
  }
}

void MaxEffortGripperActionController::store_previous_rt_goal( RealtimeGoalHandlePtr handle )
{
  std::atomic_store( &previous_rt_goal_, std::move( handle ) );
}

MaxEffortGripperActionController::RealtimeGoalHandlePtr
MaxEffortGripperActionController::exchange_previous_rt_goal( RealtimeGoalHandlePtr handle )
{
  return std::atomic_exchange( &previous_rt_goal_, std::move( handle ) );
}

void MaxEffortGripperActionController::clear_pending_action_command()
{
  // Mark the currently-pending action command as already consumed so update() ignores it.
  // Race window: if update() is mid-cycle and has already read action_cmd_seq_ above the
  // dispatch fence, it may still apply the command once. That window is one control cycle
  // and the result remains terminal (cancel still runs). We accept that as a known minor
  // race; eliminating it requires adding a real lock around the dispatch.
  action_cmd_seq_.store( 0 );
}

void MaxEffortGripperActionController::set_hold_position()
{
  if ( position_state_interface_ ) {
    target_.position = position_state_interface_->get().get_optional().value_or( target_.position );
  }
  target_.max_effort = params_.default_max_effort;
}

controller_interface::return_type
MaxEffortGripperActionController::update( const rclcpp::Time &time, const rclcpp::Duration &period )
{
  if ( !position_command_interface_ || !effort_command_interface_ || !position_state_interface_ ||
       !velocity_state_interface_ || !effort_state_interface_ ) {
    return controller_interface::return_type::ERROR;
  }

  const double current_position =
      position_state_interface_->get().get_optional().value_or( target_.position );
  const double current_velocity = velocity_state_interface_->get().get_optional().value_or( 0.0 );
  const double current_effort = effort_state_interface_->get().get_optional().value_or( 0.0 );

  // ----- Input arbitration: true last-writer-wins by sequence number -----
  // Snapshot all per-source sequence numbers once. Inputs are validated (NaN/Inf rejected)
  // BEFORE they can win arbitration so a malformed topic message cannot preempt a valid
  // active action goal. All snapshotted sources are marked consumed at the end regardless
  // of whether they won — discarding malformed inputs prevents them from competing forever.
  const uint64_t action_seq = action_cmd_seq_.load();
  const uint64_t pos_seq = position_cmd_seq_.load();
  const uint64_t vel_seq = velocity_cmd_seq_.load();
  const bool action_unconsumed = ( action_seq != 0 && action_seq != last_consumed_action_seq_ );
  const bool pos_unconsumed = ( pos_seq != last_consumed_position_seq_ );
  const bool vel_unconsumed = ( vel_seq != last_consumed_velocity_seq_ );

  // Pre-validate each unconsumed source. Only valid messages are eligible to win.
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

  enum class Winner { NONE, ACTION, POS, VEL } winner = Winner::NONE;
  uint64_t winning_seq = 0;
  if ( action_valid && action_seq > winning_seq ) {
    winner = Winner::ACTION;
    winning_seq = action_seq;
  }
  if ( pos_valid && pos_seq > winning_seq ) {
    winner = Winner::POS;
    winning_seq = pos_seq;
  }
  if ( vel_valid && vel_seq > winning_seq ) {
    winner = Winner::VEL;
    winning_seq = vel_seq;
  }

  // Mark ALL snapshotted unconsumed sources as consumed (including invalid ones — we don't
  // want a single NaN message to keep "winning" forever).
  if ( action_unconsumed )
    last_consumed_action_seq_ = action_seq;
  if ( pos_unconsumed )
    last_consumed_position_seq_ = pos_seq;
  if ( vel_unconsumed )
    last_consumed_velocity_seq_ = vel_seq;

  // Log dropped malformed inputs (throttled to avoid log floods on a stuck NaN publisher).
  if ( ( action_unconsumed && !action_valid ) || ( pos_unconsumed && !pos_valid ) ||
       ( vel_unconsumed && !vel_valid ) ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Dropping malformed input (action=%d pos=%d vel=%d)",
                          action_unconsumed && !action_valid, pos_unconsumed && !pos_valid,
                          vel_unconsumed && !vel_valid );
  }

  switch ( winner ) {
  case Winner::ACTION:
    target_.position =
        clamp_to_limits( pending_action.position, joint_lower_limit_, joint_upper_limit_ );
    target_.max_effort = pending_action.max_effort;
    last_movement_time_ = time;
    // A non-velocity source took over — invalidate the cached velocity so subsequent
    // NONE cycles don't keep replaying it. We deliberately do NOT call
    // rt_velocity_cmd_.writeFromNonRT(nullptr) here: that takes a mutex and is
    // RT-unsafe. The flag is sufficient — apply_velocity_topic_command checks it.
    velocity_cached_valid_ = false;
    break;
  case Winner::POS:
    preempt_active_goal( "preempted by position_command topic" );
    target_.position = clamp_to_limits( pending_pos->data, joint_lower_limit_, joint_upper_limit_ );
    target_.max_effort = params_.default_max_effort;
    velocity_cached_valid_ = false;
    break;
  case Winner::VEL:
    preempt_active_goal( "preempted by velocity_command topic" );
    last_velocity_msg_time_ = time;
    velocity_cached_valid_ = true;
    target_.position = clamp_to_limits( target_.position + pending_vel->data * period.seconds(),
                                        joint_lower_limit_, joint_upper_limit_ );
    target_.max_effort = params_.default_max_effort;
    break;
  case Winner::NONE:
    // No fresh winner this cycle — but if a recent velocity command is still within its
    // watchdog window, keep integrating it.
    continue_velocity_integration_if_within_watchdog( time, period );
    break;
  }

  // ----- Write commands -----
  if ( !std::isfinite( target_.position ) || !std::isfinite( target_.max_effort ) ) {
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Refusing to write non-finite command (pos=%f, eff=%f)", target_.position,
                          target_.max_effort );
  } else {
    std::ignore = position_command_interface_->get().set_value( target_.position );
    std::ignore = effort_command_interface_->get().set_value( std::fabs( target_.max_effort ) );
  }

  // ----- Action goal monitoring -----
  // Read active_goal AFTER arbitration so a goal that was just preempted by a topic input
  // is not still treated as active here.
  auto active_goal = *rt_active_goal_.readFromRT();
  if ( active_goal ) {
    const double error_position = target_.position - current_position;
    check_for_success( time, error_position, current_position, current_velocity, current_effort );

    // check_for_success may have transitioned the goal to a terminal state and cleared
    // rt_active_goal_; only publish feedback if the goal is still active.
    active_goal = *rt_active_goal_.readFromRT();
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

bool MaxEffortGripperActionController::continue_velocity_integration_if_within_watchdog(
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

void MaxEffortGripperActionController::check_for_success( const rclcpp::Time &time,
                                                          double error_position,
                                                          double current_position,
                                                          double current_velocity,
                                                          double current_effort )
{
  const auto active_goal = *rt_active_goal_.readFromRT();
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

} // namespace max_effort_gripper_action_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS( max_effort_gripper_action_controller::MaxEffortGripperActionController,
                        controller_interface::ControllerInterface )
