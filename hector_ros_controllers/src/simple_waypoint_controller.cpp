#include "waypoint_controller/simple_waypoint_controller.hpp"
namespace waypoint_controller
{
MoveCommand SimpleWaypointController::computeCommand( const Waypoint &goal, const Pose &pose,
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
} // namespace waypoint_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS( waypoint_controller::SimpleWaypointController,
                        controller_interface::ControllerInterface )
