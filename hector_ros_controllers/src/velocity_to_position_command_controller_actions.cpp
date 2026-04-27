#include "velocity_to_position_command_controller/velocity_to_position_command_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"

namespace velocity_to_position_command_controller
{

// ---------------------------------------------------------------------------
// Action helpers
// ---------------------------------------------------------------------------

std::string VelocityToPositionCommandController::start_group_action(
    const std::string &group_name, const std::vector<double> &target_positions_per_joint,
    double max_vel, double max_accel )
{
  const size_t group_idx = group_index_map_[group_name];
  const auto &group_joint_indices = groups_[group_name];

  if ( target_positions_per_joint.size() != group_joint_indices.size() ) {
    return "Target positions size mismatch for group '" + group_name + "'";
  }

  GroupActionCommand cmd;
  cmd.active = true;
  cmd.start_time = get_node()->now();
  cmd.target_position = target_positions_per_joint.front(); // for feedback

  for ( size_t i = 0; i < group_joint_indices.size(); i++ ) {
    const size_t idx = group_joint_indices[i];
    const double start = joint_position_states_[idx];
    if ( std::isnan( start ) ) {
      return "Joint " + joints_[idx] + " has invalid position state";
    }
    cmd.joint_profiles.push_back(
        TrapezoidalProfile::compute( start, target_positions_per_joint[i], max_vel, max_accel ) );
  }

  group_action_states_[group_idx].store( GroupActionState::EXECUTING );
  rt_group_action_cmds_[group_idx].writeFromNonRT( cmd );
  return {};
}

void VelocityToPositionCommandController::cancel_group_actions( const std::vector<size_t> &group_indices )
{
  for ( size_t gi : group_indices ) {
    if ( group_action_states_[gi].load() == GroupActionState::EXECUTING ) {
      GroupActionCommand cancel_cmd;
      cancel_cmd.active = false;
      rt_group_action_cmds_[gi].writeFromNonRT( cancel_cmd );
    }
    group_action_states_[gi].store( GroupActionState::IDLE );
  }
}

void VelocityToPositionCommandController::monitor_group_actions(
    const std::vector<size_t> &group_indices, std::function<bool()> is_canceling,
    MonitorCompleteFn on_complete, MonitorAbortFn on_abort, MonitorFeedbackFn on_feedback )
{
  rclcpp::Rate rate( 20.0 );

  while ( rclcpp::ok() ) {
    if ( is_canceling() ) {
      cancel_group_actions( group_indices );
      on_abort( "Cancelled by client" );
      return;
    }

    bool any_cancelled = false;
    bool all_done = true;
    double min_progress = 1.0;

    for ( size_t gi : group_indices ) {
      const auto state = group_action_states_[gi].load();
      if ( state == GroupActionState::CANCELLED ) {
        any_cancelled = true;
      }
      if ( state != GroupActionState::COMPLETED && state != GroupActionState::IDLE ) {
        all_done = false;
      }

      // Compute progress
      const auto *cmd_ptr = rt_group_action_cmds_[gi].readFromRT();
      if ( cmd_ptr && cmd_ptr->active && !cmd_ptr->joint_profiles.empty() ) {
        const double elapsed = ( get_node()->now() - cmd_ptr->start_time ).seconds();
        double max_total = 0.0;
        for ( const auto &prof : cmd_ptr->joint_profiles ) {
          max_total = std::max( max_total, prof.total_time );
        }
        const double progress = ( max_total > 0.0 ) ? std::min( elapsed / max_total, 1.0 ) : 1.0;
        min_progress = std::min( min_progress, progress );
      }
    }

    if ( any_cancelled ) {
      cancel_group_actions( group_indices );
      on_abort( "Cancelled by velocity command" );
      return;
    }

    if ( all_done ) {
      for ( size_t gi : group_indices ) {
        group_action_states_[gi].store( GroupActionState::IDLE );
      }
      on_complete();
      return;
    }

    on_feedback( min_progress );
    rate.sleep();
  }
}

// ---------------------------------------------------------------------------
// Action server callbacks
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse VelocityToPositionCommandController::handle_drive_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const DriveFlipperGroupAction::Goal> goal )
{
  if ( group_index_map_.find( goal->group_name ) == group_index_map_.end() ) {
    RCLCPP_WARN( get_node()->get_logger(), "DriveFlipperGroup: unknown group '%s'",
                 goal->group_name.c_str() );
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse VelocityToPositionCommandController::handle_drive_cancel(
    std::shared_ptr<DriveFlipperGroupGoalHandle> /*goal_handle*/ )
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void VelocityToPositionCommandController::handle_drive_accepted(
    std::shared_ptr<DriveFlipperGroupGoalHandle> goal_handle )
{
  const auto goal = goal_handle->get_goal();
  const auto &group_joint_indices = groups_[goal->group_name];
  const size_t group_idx = group_index_map_[goal->group_name];

  const double target =
      ( goal->target_position == 0.0 ) ? params_.upright_position : goal->target_position;
  const double max_vel = ( goal->max_velocity <= 0.0 ) ? max_velocity_ : goal->max_velocity;
  const double max_accel =
      ( goal->max_acceleration <= 0.0 ) ? max_acceleration_ : goal->max_acceleration;

  // All joints in the group drive to the same target
  std::vector<double> targets( group_joint_indices.size(), target );

  const auto error = start_group_action( goal->group_name, targets, max_vel, max_accel );
  if ( !error.empty() ) {
    auto result = std::make_shared<DriveFlipperGroupAction::Result>();
    result->success = false;
    result->message = error;
    goal_handle->abort( result );
    return;
  }

  RCLCPP_INFO( get_node()->get_logger(), "DriveFlipperGroup: driving group '%s' to %.4f",
               goal->group_name.c_str(), target );

  reap_finished_monitor_threads();
  auto done_flag = std::make_shared<std::atomic<bool>>( false );
  std::thread t( [this, goal_handle, group_idx, group_joint_indices, done_flag]() {
    monitor_group_actions(
        { group_idx }, [&]() { return goal_handle->is_canceling(); },
        /*on_complete*/
        [&]() {
          auto result = std::make_shared<DriveFlipperGroupAction::Result>();
          result->success = true;
          result->message = "Target reached";
          goal_handle->succeed( result );
        },
        /*on_abort*/
        [&]( const std::string &reason ) {
          auto result = std::make_shared<DriveFlipperGroupAction::Result>();
          result->success = false;
          result->message = reason;
          if ( reason.find( "client" ) != std::string::npos ) {
            goal_handle->canceled( result );
          } else {
            goal_handle->abort( result );
          }
        },
        /*on_feedback*/
        [&]( double progress ) {
          auto feedback = std::make_shared<DriveFlipperGroupAction::Feedback>();
          feedback->progress = progress;
          const auto positions = rt_joint_position_snapshot_.get();
          for ( size_t idx : group_joint_indices ) {
            if ( idx < positions.size() ) {
              feedback->current_positions.push_back( positions[idx] );
            }
          }
          goal_handle->publish_feedback( feedback );
        } );
    done_flag->store( true );
  } );
  {
    std::lock_guard<std::mutex> lock( action_monitor_threads_mutex_ );
    action_monitor_threads_.push_back( { std::move( t ), done_flag } );
  }
}

rclcpp_action::GoalResponse VelocityToPositionCommandController::handle_sync_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const SyncFlipperGroupAction::Goal> goal )
{
  const auto &names = goal->group_names;
  if ( names.empty() ) {
    if ( group_names_.empty() ) {
      RCLCPP_WARN( get_node()->get_logger(), "SyncFlipperGroup: no synchronous groups configured" );
      return rclcpp_action::GoalResponse::REJECT;
    }
  } else {
    for ( const auto &name : names ) {
      if ( group_index_map_.find( name ) == group_index_map_.end() ) {
        RCLCPP_WARN( get_node()->get_logger(), "SyncFlipperGroup: unknown group '%s'", name.c_str() );
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse VelocityToPositionCommandController::handle_sync_cancel(
    std::shared_ptr<SyncFlipperGroupGoalHandle> /*goal_handle*/ )
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void VelocityToPositionCommandController::handle_sync_accepted(
    std::shared_ptr<SyncFlipperGroupGoalHandle> goal_handle )
{
  const auto goal = goal_handle->get_goal();
  const double max_vel = ( goal->max_velocity <= 0.0 ) ? max_velocity_ : goal->max_velocity;
  const double max_accel =
      ( goal->max_acceleration <= 0.0 ) ? max_acceleration_ : goal->max_acceleration;

  const std::vector<std::string> target_groups =
      goal->group_names.empty() ? group_names_ : goal->group_names;

  std::vector<size_t> target_group_indices;
  std::vector<double> group_avg_positions;

  for ( const auto &group_name : target_groups ) {
    const auto &group_joint_indices = groups_[group_name];
    target_group_indices.push_back( group_index_map_[group_name] );

    // Compute average position
    double sum = 0.0;
    size_t valid_count = 0;
    for ( size_t idx : group_joint_indices ) {
      if ( !std::isnan( joint_position_states_[idx] ) ) {
        sum += joint_position_states_[idx];
        valid_count++;
      }
    }

    if ( valid_count == 0 ) {
      auto result = std::make_shared<SyncFlipperGroupAction::Result>();
      result->success = false;
      result->message = "Group '" + group_name + "' has no valid joint positions";
      goal_handle->abort( result );
      return;
    }

    const double avg = sum / static_cast<double>( valid_count );
    group_avg_positions.push_back( avg );

    // Each joint in this group drives to the group average
    std::vector<double> targets( group_joint_indices.size(), avg );
    const auto error = start_group_action( group_name, targets, max_vel, max_accel );
    if ( !error.empty() ) {
      auto result = std::make_shared<SyncFlipperGroupAction::Result>();
      result->success = false;
      result->message = error;
      goal_handle->abort( result );
      return;
    }

    RCLCPP_INFO( get_node()->get_logger(), "SyncFlipperGroup: syncing group '%s' to avg=%.4f",
                 group_name.c_str(), avg );
  }

  reap_finished_monitor_threads();
  auto done_flag = std::make_shared<std::atomic<bool>>( false );
  std::thread t(
      [this, goal_handle, target_groups, target_group_indices, group_avg_positions, done_flag]() {
        monitor_group_actions(
            target_group_indices, [&]() { return goal_handle->is_canceling(); },
            /*on_complete*/
            [&]() {
              auto result = std::make_shared<SyncFlipperGroupAction::Result>();
              result->success = true;
              result->message = "All groups synced";
              result->synced_positions = group_avg_positions;
              goal_handle->succeed( result );
            },
            /*on_abort*/
            [&]( const std::string &reason ) {
              auto result = std::make_shared<SyncFlipperGroupAction::Result>();
              result->success = false;
              result->message = reason;
              if ( reason.find( "client" ) != std::string::npos ) {
                goal_handle->canceled( result );
              } else {
                goal_handle->abort( result );
              }
            },
            /*on_feedback*/
            [&]( double progress ) {
              auto feedback = std::make_shared<SyncFlipperGroupAction::Feedback>();
              feedback->progress = progress;
              // Compute max position error across all groups using an RT snapshot.
              const auto positions = rt_joint_position_snapshot_.get();
              double max_error = 0.0;
              for ( size_t g = 0; g < target_groups.size(); g++ ) {
                for ( size_t idx : groups_[target_groups[g]] ) {
                  if ( idx < positions.size() && !std::isnan( positions[idx] ) ) {
                    max_error =
                        std::max( max_error, std::abs( positions[idx] - group_avg_positions[g] ) );
                  }
                }
              }
              feedback->max_position_error = max_error;
              goal_handle->publish_feedback( feedback );
            } );
        done_flag->store( true );
      } );
  {
    std::lock_guard<std::mutex> lock( action_monitor_threads_mutex_ );
    action_monitor_threads_.push_back( { std::move( t ), done_flag } );
  }
}

// ---------------------------------------------------------------------------
// Group action processing (called from update loop)
// ---------------------------------------------------------------------------

bool VelocityToPositionCommandController::process_group_actions( const rclcpp::Time &time )
{
  bool all_successful = true;
  for ( size_t g = 0; g < group_names_.size(); g++ ) {
    const auto *cmd_ptr = rt_group_action_cmds_[g].readFromRT();
    if ( !cmd_ptr || !cmd_ptr->active ) {
      continue;
    }

    const auto &group_joint_indices = groups_[group_names_[g]];

    // Check if any joint in this group has a non-zero velocity command -> cancel action
    bool velocity_override = false;
    for ( size_t idx : group_joint_indices ) {
      if ( !std::isnan( reference_interfaces_[idx] ) && reference_interfaces_[idx] != 0.0 ) {
        velocity_override = true;
        break;
      }
    }

    if ( velocity_override ) {
      // Deactivate the action command via a new write
      GroupActionCommand cancel_cmd;
      cancel_cmd.active = false;
      rt_group_action_cmds_[g].writeFromNonRT( cancel_cmd );
      group_action_states_[g].store( GroupActionState::CANCELLED );
      RCLCPP_INFO( get_node()->get_logger(), "Group action for '%s' cancelled by velocity command",
                   group_names_[g].c_str() );
      continue;
    }

    const double elapsed = ( time - cmd_ptr->start_time ).seconds();

    // Check if all profiles are complete
    bool all_complete = true;
    for ( const auto &prof : cmd_ptr->joint_profiles ) {
      if ( elapsed < prof.total_time ) {
        all_complete = false;
        break;
      }
    }

    if ( all_complete ) {
      // Hold at target
      for ( size_t i = 0; i < group_joint_indices.size(); i++ ) {
        const size_t idx = group_joint_indices[i];
        const double target = cmd_ptr->joint_profiles[i].target_position;
        desired_positions_[idx] = target;
        hold_positions_[idx] = target;
        move_states_[idx] = STOPPED;
        all_successful &= command_interfaces_[idx].set_value( target );
      }

      // Deactivate and mark completed
      GroupActionCommand done_cmd;
      done_cmd.active = false;
      rt_group_action_cmds_[g].writeFromNonRT( done_cmd );
      group_action_states_[g].store( GroupActionState::COMPLETED );
      RCLCPP_INFO( get_node()->get_logger(), "Group action for '%s' completed",
                   group_names_[g].c_str() );
    } else {
      // Follow profile
      for ( size_t i = 0; i < group_joint_indices.size(); i++ ) {
        const size_t idx = group_joint_indices[i];
        const auto [pos, vel] = cmd_ptr->joint_profiles[i].evaluate( elapsed );

        // Clamp to URDF limits
        double clamped_pos = pos;
        if ( !std::isnan( joint_lower_limits_[idx] ) ) {
          clamped_pos = std::clamp( pos, joint_lower_limits_[idx], joint_upper_limits_[idx] );
        }

        desired_positions_[idx] = clamped_pos;
        hold_positions_[idx] = clamped_pos;
        move_states_[idx] = MOVING;
        all_successful &= command_interfaces_[idx].set_value( clamped_pos );
      }
    }

    // Set reference interfaces to NaN for joints in this group so they're skipped by normal control
    for ( size_t idx : group_joint_indices ) {
      reference_interfaces_[idx] = std::numeric_limits<double>::quiet_NaN();
    }
  }
  return all_successful;
}

} // namespace velocity_to_position_command_controller
