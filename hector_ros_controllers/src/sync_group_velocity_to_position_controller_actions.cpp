#include "sync_group_velocity_to_position_controller/sync_group_velocity_to_position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"

namespace sync_group_velocity_to_position_controller
{

// ---------------------------------------------------------------------------
// Action helpers
// ---------------------------------------------------------------------------

std::string SyncGroupVelocityToPositionController::start_group_action(
    const std::string &group_name, const std::vector<double> &target_positions_per_joint,
    double max_vel, double max_accel )
{
  const auto group_index_it = group_index_map_.find( group_name );
  const auto group_it = groups_.find( group_name );
  if ( group_index_it == group_index_map_.end() || group_it == groups_.end() ) {
    return "Unknown group '" + group_name + "'";
  }
  const size_t group_idx = group_index_it->second;
  const auto &group_joint_indices = group_it->second;

  // Defense in depth: even if the goal callback accepted, refuse to overwrite
  // an in-flight goal's RT command/state. Without this, a second client could
  // silently hijack the first goal and the first monitor thread would report
  // success for the wrong motion.
  if ( group_action_states_[group_idx].load() == GroupActionState::EXECUTING ) {
    return "Group '" + group_name + "' is already executing";
  }

  if ( target_positions_per_joint.size() != group_joint_indices.size() ) {
    return "Target positions size mismatch for group '" + group_name + "'";
  }

  // Read the joint positions through the RT-safe snapshot — joint_position_states_
  // is concurrently written by the RT update loop.
  const auto positions = rt_joint_position_snapshot_.get();

  GroupActionCommand cmd;
  cmd.active = true;
  cmd.start_time = get_node()->now();
  cmd.target_position = target_positions_per_joint.front(); // for feedback

  for ( size_t i = 0; i < group_joint_indices.size(); i++ ) {
    const size_t idx = group_joint_indices[i];
    if ( idx >= positions.size() ) {
      return "Joint index " + std::to_string( idx ) + " out of range in position snapshot";
    }
    const double start = positions[idx];
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

void SyncGroupVelocityToPositionController::cancel_group_actions( const std::vector<size_t> &group_indices )
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

void SyncGroupVelocityToPositionController::monitor_group_actions(
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

  // rclcpp::ok() became false (shutdown). Make sure we don't leave the RT
  // command active and the goal hanging on the client side.
  cancel_group_actions( group_indices );
  on_abort( "Aborted due to shutdown" );
}

// ---------------------------------------------------------------------------
// Action server callbacks
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse SyncGroupVelocityToPositionController::handle_drive_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const DriveFlipperGroupAction::Goal> goal )
{
  const auto group_index_it = group_index_map_.find( goal->group_name );
  if ( group_index_it == group_index_map_.end() ) {
    RCLCPP_WARN( get_node()->get_logger(), "DriveFlipperGroup: unknown group '%s'",
                 goal->group_name.c_str() );
    return rclcpp_action::GoalResponse::REJECT;
  }
  // Refuse to overlap an in-flight goal. The client must cancel the current
  // goal first; otherwise we'd silently hijack it and the first goal handle
  // would later receive a misleading result.
  if ( group_action_states_[group_index_it->second].load() == GroupActionState::EXECUTING ) {
    RCLCPP_WARN( get_node()->get_logger(),
                 "DriveFlipperGroup: group '%s' already executing; rejecting overlapping goal",
                 goal->group_name.c_str() );
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SyncGroupVelocityToPositionController::handle_drive_cancel(
    std::shared_ptr<DriveFlipperGroupGoalHandle> /*goal_handle*/ )
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SyncGroupVelocityToPositionController::handle_drive_accepted(
    std::shared_ptr<DriveFlipperGroupGoalHandle> goal_handle )
{
  const auto goal = goal_handle->get_goal();
  // Goal callback validated the name and rejected overlap, so find() should
  // succeed; abort cleanly if it doesn't (e.g. groups_ changed concurrently).
  const auto group_it = groups_.find( goal->group_name );
  const auto group_index_it = group_index_map_.find( goal->group_name );
  if ( group_it == groups_.end() || group_index_it == group_index_map_.end() ) {
    auto result = std::make_shared<DriveFlipperGroupAction::Result>();
    result->success = false;
    result->message = "Unknown group '" + goal->group_name + "'";
    goal_handle->abort( result );
    return;
  }
  const auto &group_joint_indices = group_it->second;
  const size_t group_idx = group_index_it->second;

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

rclcpp_action::GoalResponse SyncGroupVelocityToPositionController::handle_sync_goal(
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
  // Refuse to overlap an in-flight goal on any target group. With empty
  // group_names we sync everything, so check all groups; otherwise just the
  // requested ones.
  const std::vector<std::string> &check_names = names.empty() ? group_names_ : names;
  for ( const auto &name : check_names ) {
    const auto it = group_index_map_.find( name );
    if ( it != group_index_map_.end() &&
         group_action_states_[it->second].load() == GroupActionState::EXECUTING ) {
      RCLCPP_WARN( get_node()->get_logger(),
                   "SyncFlipperGroup: group '%s' already executing; rejecting overlapping goal",
                   name.c_str() );
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SyncGroupVelocityToPositionController::handle_sync_cancel(
    std::shared_ptr<SyncFlipperGroupGoalHandle> /*goal_handle*/ )
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SyncGroupVelocityToPositionController::handle_sync_accepted(
    std::shared_ptr<SyncFlipperGroupGoalHandle> goal_handle )
{
  const auto goal = goal_handle->get_goal();
  const double max_vel = ( goal->max_velocity <= 0.0 ) ? max_velocity_ : goal->max_velocity;
  const double max_accel =
      ( goal->max_acceleration <= 0.0 ) ? max_acceleration_ : goal->max_acceleration;

  const std::vector<std::string> target_groups =
      goal->group_names.empty() ? group_names_ : goal->group_names;

  // Read the joint positions through the RT-safe snapshot — joint_position_states_
  // is concurrently written by the RT update loop.
  const auto positions = rt_joint_position_snapshot_.get();

  std::vector<size_t> target_group_indices;
  std::vector<double> group_avg_positions;
  // Track which groups we've successfully started so we can roll them back if
  // a later group fails — otherwise the operator gets a "failed" goal result
  // while earlier groups keep moving.
  std::vector<size_t> started_group_indices;

  auto abort_with = [&]( const std::string &message ) {
    if ( !started_group_indices.empty() ) {
      cancel_group_actions( started_group_indices );
    }
    auto result = std::make_shared<SyncFlipperGroupAction::Result>();
    result->success = false;
    result->message = message;
    goal_handle->abort( result );
  };

  for ( const auto &group_name : target_groups ) {
    const auto group_index_it = group_index_map_.find( group_name );
    const auto group_it = groups_.find( group_name );
    if ( group_index_it == group_index_map_.end() || group_it == groups_.end() ) {
      abort_with( "Unknown group '" + group_name + "'" );
      return;
    }
    const auto &group_joint_indices = group_it->second;
    target_group_indices.push_back( group_index_it->second );

    // Compute average position
    double sum = 0.0;
    size_t valid_count = 0;
    for ( size_t idx : group_joint_indices ) {
      if ( idx < positions.size() && !std::isnan( positions[idx] ) ) {
        sum += positions[idx];
        valid_count++;
      }
    }

    if ( valid_count == 0 ) {
      abort_with( "Group '" + group_name + "' has no valid joint positions" );
      return;
    }

    const double avg = sum / static_cast<double>( valid_count );
    group_avg_positions.push_back( avg );

    // Each joint in this group drives to the group average
    std::vector<double> targets( group_joint_indices.size(), avg );
    const auto error = start_group_action( group_name, targets, max_vel, max_accel );
    if ( !error.empty() ) {
      abort_with( error );
      return;
    }
    started_group_indices.push_back( group_index_it->second );

    RCLCPP_INFO( get_node()->get_logger(), "SyncFlipperGroup: syncing group '%s' to avg=%.4f",
                 group_name.c_str(), avg );
  }

  reap_finished_monitor_threads();
  auto done_flag = std::make_shared<std::atomic<bool>>( false );
  std::thread t( [this, goal_handle, target_groups, target_group_indices, group_avg_positions,
                  done_flag]() {
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
          const auto fb_positions = rt_joint_position_snapshot_.get();
          double max_error = 0.0;
          for ( size_t g = 0; g < target_groups.size(); g++ ) {
            for ( size_t idx : groups_[target_groups[g]] ) {
              if ( idx < fb_positions.size() && !std::isnan( fb_positions[idx] ) ) {
                max_error =
                    std::max( max_error, std::abs( fb_positions[idx] - group_avg_positions[g] ) );
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

bool SyncGroupVelocityToPositionController::process_group_actions( const rclcpp::Time &time )
{
  bool all_successful = true;
  for ( size_t g = 0; g < group_names_.size(); g++ ) {
    const GroupActionState prev_rt_state = group_last_rt_state_[g];
    const GroupActionState cur_state = group_action_states_[g].load();

    const auto group_it = groups_.find( group_names_[g] );
    if ( group_it == groups_.end() ) {
      group_last_rt_state_[g] = cur_state;
      continue;
    }
    const std::vector<size_t> &group_joint_indices = group_it->second;

    // EXECUTING -> IDLE here is exclusively a non-RT abort/cancel (client cancel
    // or rollback); RT stamps COMPLETED/CANCELLED inline below. Re-seed the offset
    // from measured state so the pair holds its current physical pose.
    if ( prev_rt_state == GroupActionState::EXECUTING && cur_state == GroupActionState::IDLE ) {
      for ( size_t idx : group_joint_indices ) { reset_sync_offsets( idx ); }
      group_pending_recapture_[g] = false;
    }

    // Gate on the atomic state — RT must NOT call writeFromNonRT on the buffer.
    // The buffer's `active` flag stays as-is and is overwritten on the next goal start.
    if ( cur_state != GroupActionState::EXECUTING ) {
      group_last_rt_state_[g] = cur_state;
      continue;
    }
    const auto *cmd_ptr = rt_group_action_cmds_[g].readFromRT();
    if ( !cmd_ptr || !cmd_ptr->active ) {
      group_last_rt_state_[g] = cur_state;
      continue;
    }

    // Check if any joint in this group has a non-zero velocity command -> cancel action
    bool velocity_override = false;
    for ( size_t idx : group_joint_indices ) {
      if ( !std::isnan( reference_interfaces_[idx] ) && reference_interfaces_[idx] != 0.0 ) {
        velocity_override = true;
        break;
      }
    }

    // Post-tick RT state, stamped into the latch so a later COMPLETED/CANCELLED
    // -> IDLE collapse is not misread as an EXECUTING -> IDLE cancel.
    GroupActionState next_rt_state = GroupActionState::EXECUTING;

    if ( velocity_override ) {
      // Velocity command overrides the action. Re-seed the offset from the
      // current measured pose so the pair holds its physical relative position.
      for ( size_t idx : group_joint_indices ) { reset_sync_offsets( idx ); }
      group_pending_recapture_[g] = false;
      group_action_states_[g].store( GroupActionState::CANCELLED );
      next_rt_state = GroupActionState::CANCELLED;
      RCLCPP_INFO( get_node()->get_logger(), "Group action for '%s' cancelled by velocity command",
                   group_names_[g].c_str() );
      // Fall through: leave reference_interfaces_ as-is so normal velocity
      // control resumes for these joints this tick.
      group_last_rt_state_[g] = next_rt_state;
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
      // Hold at target — clamp to URDF limits before writing, matching the
      // in-flight branch. Without this, a target outside joint limits would
      // produce an out-of-range command on the completion tick.
      std::vector<double> clamped_targets( group_joint_indices.size() );
      for ( size_t i = 0; i < group_joint_indices.size(); i++ ) {
        const size_t idx = group_joint_indices[i];
        double target = cmd_ptr->joint_profiles[i].target_position;
        if ( !std::isnan( joint_lower_limits_[idx] ) ) {
          target = std::clamp( target, joint_lower_limits_[idx], joint_upper_limits_[idx] );
        }
        clamped_targets[i] = target;
        desired_positions_[idx] = target;
        hold_positions_[idx] = target;
        move_states_[idx] = STOPPED;
        all_successful &= command_interfaces_[idx].set_value( target );
      }

      // Record the offset from the final commanded targets (not measured state,
      // which may not have settled). Normally 0, but non-zero if a partner
      // clamped against a different URDF limit.
      for ( size_t i = 0; i < group_joint_indices.size(); i++ ) {
        const size_t idx = group_joint_indices[i];
        if ( !sync_pairs_.has_partner( idx ) )
          continue;
        const size_t p = sync_pairs_.partner( idx );
        // Find the partner's position within this group's index list.
        for ( size_t j = 0; j < group_joint_indices.size(); j++ ) {
          if ( group_joint_indices[j] == p ) {
            sync_pairs_.set_offset( idx, clamped_targets[j] - clamped_targets[i] );
            break;
          }
        }
      }
      group_pending_recapture_[g] = false;

      group_action_states_[g].store( GroupActionState::COMPLETED );
      next_rt_state = GroupActionState::COMPLETED;
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

    group_last_rt_state_[g] = next_rt_state;
  }
  return all_successful;
}

} // namespace sync_group_velocity_to_position_controller
