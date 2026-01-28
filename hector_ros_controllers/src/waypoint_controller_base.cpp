#include "waypoint_controller_base/waypoint_controller_base.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace waypoint_controller_base
{
WaypointControllerBase::WaypointControllerBase() : controller_interface::ControllerInterface() { }

controller_interface::CallbackReturn WaypointControllerBase::read_parameters()
{
  if ( !base_param_listener_ ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Error encountered during init" );
    return controller_interface::CallbackReturn::ERROR;
  }
  base_params_ = base_param_listener_->get_params();

  if ( base_params_.base_link_frame_name.empty() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Base link frame name cannot be empty" );
    return controller_interface::CallbackReturn::ERROR;
  }
  base_link_frame_ = base_params_.base_link_frame_name;

  try {
    action_monitor_period_ = std::chrono::milliseconds( base_params_.action_monitior_period );
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Specified invalid action monitor period: %s", e.what() );
  }

  velocity_interfaces_prefix_ = base_params_.velocity_interfaces_prefix;
  use_cmd_vel_ = base_params_.use_cmd_vel;

  if ( !use_cmd_vel_ ) {
    // Define the command and state interface types based on the specified prefix
    command_interface_names_.push_back( velocity_interfaces_prefix_ + "/linear/velocity" );
    command_interface_names_.push_back( velocity_interfaces_prefix_ + "/angular/velocity" );

    // state_interface_names_.push_back( velocity_interfaces_prefix_ + "/linear/velocity" );
    // state_interface_names_.push_back( velocity_interfaces_prefix_ + "/angular/velocity" );
  }

  if ( base_params_.goal_completion_tolerance < 0 ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Goal completion tolerance must be non-negative" );
    return controller_interface::CallbackReturn::ERROR;
  }
  goal_completion_tolerance_ = base_params_.goal_completion_tolerance;

  return controller_interface::CallbackReturn::SUCCESS;
}

void WaypointControllerBase::declare_parameters()
{
  base_param_listener_ =
      std::make_shared<waypoint_controller_base_parameters::ParamListener>( get_node() );
}

controller_interface::CallbackReturn WaypointControllerBase::on_init()
{
  try {
    declare_parameters();

  } catch ( const std::exception &e ) {
    fprintf( stderr, "Exception thrown during init stage with message: %s \n", e.what() );
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void WaypointControllerBase::update_pose_cb()
{
  geometry_msgs::msg::TransformStamped t;

  try {
    t = tf_buffer_->lookupTransform( "map", base_link_frame_, tf2::TimePointZero );
  } catch ( const tf2::TransformException &ex ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Could not transform the base link frame to map: %s",
                  ex.what() );
  }
  const auto pose = Pose( t.transform.translation.x, t.transform.translation.y,
                          tf2::getYaw( t.transform.rotation ) );
  current_pose_.writeFromNonRT( pose );
}

controller_interface::CallbackReturn
WaypointControllerBase::on_configure( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  auto ret = this->read_parameters();
  if ( ret != controller_interface::CallbackReturn::SUCCESS ) {
    return ret;
  }

  if ( use_cmd_vel_ ) {
    vel_pub_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
        std::string( get_node()->get_namespace() ) + "/cmd_vel", rclcpp::SystemDefaultsQoS() );
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>( get_node()->get_clock() );
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_ );

  pose_update_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds( 50 ), std::bind( &WaypointControllerBase::update_pose_cb, this ) );

  pose_update_timer_->cancel();

  navigation_server_ = rclcpp_action::create_server<WaypointNav>(
      get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
      get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
      std::string( get_node()->get_name() ) + "/waypoint_navigation",
      std::bind( &WaypointControllerBase::goal_received_callback, this, std::placeholders::_1,
                 std::placeholders::_2 ),
      std::bind( &WaypointControllerBase::goal_cancelled_callback, this, std::placeholders::_1 ),
      std::bind( &WaypointControllerBase::goal_accepted_callback, this, std::placeholders::_1 ) );

  RCLCPP_INFO( get_node()->get_logger(), "configure successful" );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
WaypointControllerBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration WaypointControllerBase::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interface_config;
  state_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interface_config.names = state_interface_names_;

  return state_interface_config;
}

controller_interface::CallbackReturn
WaypointControllerBase::on_activate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
  if ( !controller_interface::get_ordered_interfaces( command_interfaces_, command_interface_names_,
                                                      std::string( "" ), ordered_interfaces ) ||
       command_interface_names_.size() != ordered_interfaces.size() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
                  command_interface_names_.size(), ordered_interfaces.size() );
    return controller_interface::CallbackReturn::ERROR;
  }

  pose_update_timer_->reset();

  // reset command buffer if a command came through callback when controller was inactive
  trajectory_buffer_.writeFromNonRT( nullptr );

  RCLCPP_INFO( get_node()->get_logger(), "activate successful" );
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
WaypointControllerBase::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  canceled_.store( true );
  active_ = false;

  pose_update_timer_->cancel();

  action_monitor_timer_->cancel();
  action_monitor_timer_ = nullptr;

  const auto active_trajectory = *trajectory_gh_buffer_.readFromNonRT();
  // Abort any currently active goal
  if ( active_trajectory ) {
    const auto action_res = get_result_msg( false, "Goal aborted due to controller deactivation" );
    active_trajectory->setAborted( std::make_shared<WaypointNav::Result>( action_res ) );
  }

  trajectory_buffer_.writeFromNonRT( nullptr );
  trajectory_gh_buffer_.writeFromNonRT( std::shared_ptr<RtGhWayNav>() );

  // Stop the robot
  set_base_velocities( MoveCommand{ 0.0, 0.0 } );

  return controller_interface::CallbackReturn::SUCCESS;
}

bool WaypointControllerBase::check_goal_completion( const Waypoint &goal, const Pose &current_pose )
{
  // Use euclidean distance to check if goal is reached
  return std::sqrt( std::pow( goal.x - current_pose.x, 2 ) +
                    std::pow( goal.y - current_pose.y, 2 ) ) <= goal_completion_tolerance_;
}

// Simple placeholder implementation for testing. Proper controllers should override this method.
MoveCommand WaypointControllerBase::computeCommand( const Waypoint &goal, const Pose &pose,
                                                    const double &curr_linear_vel,
                                                    const double &curr_angular_vel )
{
  MoveCommand cmd;

  // Gains (tune these)
  const double k_rho = 0.8;
  const double k_alpha = 2.0;

  // Velocity limits
  const double v_max = 0.5;     // m/s
  const double omega_max = 1.5; // rad/s

  // Position error
  double dx = goal.x - pose.x;
  double dy = goal.y - pose.y;

  // Distance to goal
  double rho = std::sqrt( dx * dx + dy * dy );

  // Desired heading
  double theta_des = std::atan2( dy, dx );

  // Heading error
  double alpha = ( theta_des - pose.heading );
  alpha = std::atan2( std::sin( alpha ), std::cos( alpha ) );

  // Control law
  cmd.linear_vel_cmd = std::clamp( k_rho * rho, -v_max, v_max );
  cmd.angual_vel_cmd = std::clamp( k_alpha * alpha, -omega_max, omega_max );

  return cmd;
}

rclcpp_action::CancelResponse WaypointControllerBase::goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<WaypointNav>> goal_handle )
{
  auto active_trajectory = *trajectory_gh_buffer_.readFromNonRT();

  // Check that cancel request refers to currently active goal (if any)
  if ( !canceled_.load() && active_trajectory->gh_ == goal_handle ) {
    // Mark the current goal as canceled
    canceled_.store( true );

    const auto res_msg = get_result_msg( false, "Goal canceled by user callback" );
    active_trajectory->setCanceled( std::make_shared<WaypointNav::Result>( res_msg ) );

    trajectory_gh_buffer_.writeFromNonRT( std::shared_ptr<RtGhWayNav>() );
    trajectory_buffer_.writeFromNonRT( nullptr );

    // action_monitor_timer_->cancel();
    // action_monitor_timer_ = nullptr;

    RCLCPP_INFO( get_node()->get_logger(),
                 "Canceling active trajectory goal because cancel callback was received." );
  } else {
    RCLCPP_INFO( get_node()->get_logger(), "Received cancel request for inactive trajectory goal" );
    return rclcpp_action::CancelResponse::REJECT;
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::GoalResponse
WaypointControllerBase::goal_received_callback( const rclcpp_action::GoalUUID &,
                                                std::shared_ptr<const WaypointNav::Goal> goal )
{
  RCLCPP_INFO( get_node()->get_logger(), "Received new trajectory goal" );

  // Precondition: Running controller
  if ( !( get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ) ) {
    RCLCPP_ERROR( get_node()->get_logger(),
                  "Can't accept new action goals. Controller is not running." );
    return rclcpp_action::GoalResponse::REJECT;
  }

  if ( !validate_trajectory( goal->waypoint_trajectory ) ) {

    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

bool WaypointControllerBase::validate_trajectory( const std::vector<geometry_msgs::msg::Point> &trajectory )
{
  if ( trajectory.empty() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Received empty trajectory, rejecting goal" );
    return false;
  }

  const auto current_pose = current_pose_.readFromNonRT();
  // compute distance from current position to first waypoint
  double total_distance = std::sqrt( std::pow( trajectory[0].x - current_pose->x, 2 ) +
                                     std::pow( trajectory[0].y - current_pose->y, 2 ) );
  for ( size_t i = 1; i < trajectory.size(); ++i ) {
    total_distance += std::sqrt( std::pow( trajectory[i].x - trajectory[i - 1].x, 2 ) +
                                 std::pow( trajectory[i].y - trajectory[i - 1].y, 2 ) );
  }
  if ( total_distance <= 0.0 ) {
    RCLCPP_ERROR( get_node()->get_logger(),
                  "Received trajectory with zero total distance, rejecting goal" );
    return false;
  }

  return true;
}

void WaypointControllerBase::preempt_active_goal(
    std::shared_ptr<waypoint_controller_base::RtGhWayNav> active_trajectory )
{
  const auto res_msg = get_result_msg( false, "Goal preempted by a new goal" );
  active_trajectory->setCanceled( std::make_shared<WaypointNav::Result>( res_msg ) );

  // action_monitor_timer_->cancel();
  // action_monitor_timer_ = nullptr;

  trajectory_gh_buffer_.writeFromNonRT( std::shared_ptr<RtGhWayNav>() );
  trajectory_buffer_.writeFromNonRT( nullptr );
}

void WaypointControllerBase::goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<WaypointNav>> goal_handle )
{
  canceled_.store( true );

  const auto active_trajectory = *trajectory_gh_buffer_.readFromNonRT();
  // Cancel any currently active goal
  if ( active_trajectory ) {
    preempt_active_goal( active_trajectory );
  }

  // Update the active goal handle
  std::shared_ptr<RtGhWayNav> rt_goal =
      std::make_shared<RtGhWayNav>( goal_handle, get_node()->get_logger() );
  rt_goal->execute();
  trajectory_gh_buffer_.writeFromNonRT( rt_goal );

  // Update ttrajectory
  trajectory_buffer_.writeFromNonRT( goal_handle->get_goal() );

  // Delete previous timer
  // Timer is kept to keep previous gh in scope until cancelation/success/aborting is processed
  if ( action_monitor_timer_ )
    action_monitor_timer_->reset();

  action_monitor_timer_ = get_node()->create_wall_timer(
      action_monitor_period_, std::bind( &RtGhWayNav::runNonRealtime, rt_goal ) );

  RCLCPP_INFO( get_node()->get_logger(), "Accepted new trajectory goal. Start executing." );

  reset_trajectory_.store( true );
  canceled_.store( false );
}

bool WaypointControllerBase::stop_base() { return set_base_velocities( MoveCommand{ 0.0, 0.0 } ); }

bool WaypointControllerBase::set_base_velocities( const MoveCommand &cmd )
{
  bool success = true;
  if ( use_cmd_vel_ ) {
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->get_node()->now();
    twist_msg.twist.linear.x = cmd.linear_vel_cmd;
    twist_msg.twist.angular.z = cmd.angual_vel_cmd;
    vel_pub_->publish( twist_msg );
  } else {
    success = success && command_interfaces_[0].set_value( cmd.linear_vel_cmd ); // linear velocity
    success = success && command_interfaces_[1].set_value( cmd.angual_vel_cmd ); // angular velocity
  }

  // Implement command setting logic here
  return success;
}

controller_interface::return_type
WaypointControllerBase::update( const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/ )
{
  // Only overwrite trajectory & goal handle if reset flag was set by non-RT thread
  if ( reset_trajectory_.exchange( false ) ) {
    active_gh_ = *trajectory_gh_buffer_.readFromRT();
    active_trajectory_ = *trajectory_buffer_.readFromRT();

    current_goal_idx_ = 0;
    current_goal_ = active_trajectory_->waypoint_trajectory[0];
    active_ = true;
  }

  if ( canceled_.load() || !active_ ) {
    // Stop the robot
    if ( stop_base() ) {
      return controller_interface::return_type::OK;
    } else {
      return controller_interface::return_type::ERROR;
    }
  }

  update_feedback();

  if ( check_goal_completion( current_goal_, *current_pose_.readFromRT() ) ) {
    current_goal_idx_ += 1;

    if ( current_goal_idx_ == active_trajectory_->waypoint_trajectory.size() ) {
      // finished trajectory
      const auto res_msg = get_result_msg( true, "" );
      active_gh_->setSucceeded( std::make_shared<WaypointNav::Result>( res_msg ) );

      active_ = false;

      if ( stop_base() ) {
        return controller_interface::return_type::OK;
      } else {
        return controller_interface::return_type::ERROR;
      }
    } else {
      // proceed to next goal
      current_goal_ = active_trajectory_->waypoint_trajectory[current_goal_idx_];
    }
  }

  const auto velCmd = computeCommand( current_goal_, *current_pose_.readFromRT(), 0.0, 0.0 );

  bool success = set_base_velocities( velCmd );
  if ( !success ) {
    const auto res_msg = get_result_msg( false, "Failed to set base velocities" );
    active_gh_->setAborted( std::make_shared<WaypointNav::Result>( res_msg ) );

    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

void WaypointControllerBase::update_feedback()
{
  WaypointNav::Feedback feedback;
  const auto pose = current_pose_.readFromRT();
  feedback.current_position.x = pose->x;
  feedback.current_position.y = pose->y;
  feedback.current_heading = pose->heading;
  feedback.current_goal.x = current_goal_.x;
  feedback.current_goal.y = current_goal_.y;

  feedback_buffer_.writeFromRT( feedback );
  active_gh_->setFeedback( std::make_shared<WaypointNav::Feedback>( feedback ) );
}

WaypointNav::Result WaypointControllerBase::get_result_msg( bool success,
                                                            const std::string &failure_report )
{
  WaypointNav::Result result;
  const auto pose = current_pose_.readFromRT();
  result.success = success;
  result.final_position.x = pose->x;
  result.final_position.y = pose->y;
  result.failure_report = failure_report;

  return result;
}
} // namespace waypoint_controller_base

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS( waypoint_controller_base::WaypointControllerBase,
                        controller_interface::ControllerInterface )
