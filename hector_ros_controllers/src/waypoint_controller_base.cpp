#include "waypoint_controller_base/waypoint_controller_base.hpp"

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

void WaypointControllerBase::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>( get_node() );
}

controller_interface::CallbackReturn WaypointControllerBase::read_parameters()
{
  if ( !param_listener_ ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Error encountered during init" );
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  std::string interface_prefix = "";
  if ( !params_.passthrough_controller.empty() )
    interface_prefix = params_.passthrough_controller + "/";

  return controller_interface::CallbackReturn::SUCCESS;
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

controller_interface::CallbackReturn
WaypointControllerBase::on_configure( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  auto ret = this->read_parameters();
  if ( ret != controller_interface::CallbackReturn::SUCCESS ) {
    return ret;
  }

  RCLCPP_INFO( get_node()->get_logger(), "configure successful" );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
WaypointControllerBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration WaypointControllerBase::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interface_config;
  state_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interface_config.names = state_interface_types_;

  return state_interface_config;
}

controller_interface::CallbackReturn
WaypointControllerBase::on_activate( const rclcpp_lifecycle::State & /*previous_state*/ )
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
  current_trajectory_.writeFromNonRT( nullptr );

  RCLCPP_INFO( get_node()->get_logger(), "activate successful" );
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
WaypointControllerBase::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/ )
{
  // reset command buffer
  current_trajectory_.writeFromNonRT( nullptr );
  is_active_.store( false );

  // Stop the robot
  set_base_velocities( MoveCommand{ 0.0, 0.0 } );

  return controller_interface::CallbackReturn::SUCCESS;
}

bool check_goal_completion( const Waypoint &goal, const Pose &current_pose ) { return false; }

rclcpp_action::CancelResponse WaypointControllerBase::goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<WaypointNav>> goal_handle )
{
  auto active_trajectory = *active_trajectory_rt_gh_.readFromNonRT();

  // Check that cancel request refers to currently active goal (if any)
  if ( is_active_ && active_trajectory->gh_ == goal_handle ) {
    // Mark the current goal as canceled
    is_active_.store( false );
    auto action_res = std::make_shared<WaypointNav::Result>();
    active_trajectory->setCanceled( action_res );
    active_trajectory_rt_gh_.writeFromNonRT( std::shared_ptr<RtGhWayNav>() );
    RCLCPP_INFO( get_node()->get_logger(),
                 "Canceling active trajectory goal because cancel callback was received." );
  }
  {
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

  if ( !validate_trajectory( goal->waypoint_trajectory ) ) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  const auto active_trajectory = *active_trajectory_rt_gh_.readFromNonRT();
  // Cancel any currently active goal
  if ( active_trajectory ) {
    is_active_.store( false );
    preempt_active_goal( active_trajectory );
  }

  current_trajectory_.writeFromNonRT( goal );
  current_goal_idx_.store( 0 );
  is_active_.store( true );

  RCLCPP_INFO( get_node()->get_logger(), "Accepted new trajectory goal" );
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

bool WaypointControllerBase::validate_trajectory( const std::vector<geometry_msgs::msg::Point> &trajectory )
{
  if ( trajectory.empty() ) {
    RCLCPP_ERROR( get_node()->get_logger(), "Received empty trajectory, rejecting goal" );
    return false;
  }

  double total_distance = 0.0;
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
  auto action_res = std::make_shared<WaypointNav::Result>();
  action_res->success = false;
  action_res->failure_report = "Goal preempted by a new goal";

  active_trajectory->setCanceled( action_res );
  active_trajectory_rt_gh_.writeFromNonRT( std::shared_ptr<RtGhWayNav>() );
}

controller_interface::return_type stop_base()
{
  return set_base_velocities( MoveCommand{ 0.0, 0.0 } );
}

controller_interface::return_type set_base_velocities( const MoveCommand &cmd )
{
  // Implement command setting logic here
  return controller_interface::return_type::OK;
}

controller_interface::return_type
WaypointControllerBase::update( const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/ )
{
  auto active_trajectory_gh_ = *active_trajectory_rt_gh_.readFromNonRT();
  auto trajectory = *current_trajectory_.readFromRT();

  if ( !is_active_ ) {
    // Stop the robot
    return stop_base();
  }

  if ( check_goal_completion( current_goal_, current_pose_ ) )
    current_goal_idx_.fetch_add( 1 );

  if ( current_goal_idx_ == trajectory->waypoint_trajectory.size() ) {
    // finished trajectory
    active_trajectory_gh_->setSucceeded();
    is_active_.store( false );

    return stop_base();
  }

  // proceed to next goal
  current_goal_ = trajectory->waypoint_trajectory[current_goal_idx_];
  return set_base_velocities( computeCommand( current_goal_, current_pose_, 0.0, 0.0 ) );
}

} // namespace waypoint_controller_base

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS( waypoint_controller_base::WaypointControllerBase,
                        controller_interface::ControllerInterface )
