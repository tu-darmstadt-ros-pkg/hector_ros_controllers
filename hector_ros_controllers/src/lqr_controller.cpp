#include "waypoint_controller/lqr_controller.hpp"

namespace waypoint_controller
{

rcl_interfaces::msg::SetParametersResult LQRController::set_weights( const rclcpp::Parameter &p )
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  const double val = p.as_double();

  if ( val < 0 ) {
    result.successful = false;
  } else {
    if ( p.get_name() == "q11" ) {
      q11_ = val;
    }
    if ( p.get_name() == "q22" ) {
      q22_ = val;
    }
    if ( p.get_name() == "r" ) {
      r_ = val;
    }
    RCLCPP_INFO( get_node()->get_logger(), "Reconfigured %s to %f", p.get_name().c_str(), val );
  }

  return result;
}

controller_interface::CallbackReturn LQRController::on_init() { }

MoveCommand LQRController::computeCommand( const Waypoint &goal, const Pose &pose,
                                           const double &curr_linear_vel,

                                           const double &curr_angular_vel )
{

  (void) curr_angular_vel; 

  MoveCommand cmd;

  // 1. Calculate Lateral and Heading Errors
  // Vector from robot to waypoint
  double dx = goal.x - pose.x;
  double dy = goal.y - pose.y;

  // Distance to goal
  double dist = std::hypot( dx, dy );

  // Angle to waypoint in global frame
  double angle_to_waypoint = std::atan2( dy, dx );

  // Heading error: Difference between robot heading and goal heading (or path angle)
  // Here we use the difference between robot heading and the vector to the goal
  double angle_diff = angle_to_waypoint - pose.heading;

  // Normalize angle to [-PI, PI]
  auto normalize_angle = []( double angle ) {
    while ( angle > M_PI ) angle -= 2.0 * M_PI;
    while ( angle < -M_PI ) angle += 2.0 * M_PI;
    return angle;
  };

  double heading_error = normalize_angle( angle_diff );

  // Lateral error: Perpendicular distance to the path vector
  // sin(heading_error) * distance gives the cross-track error
  double lateral_error = std::sin( heading_error ) * dist;

  // 2. Solve LQR for optimal gains
  // We use the absolute linear velocity to ensure the system remains stable
  double v = std::max( std::abs( curr_linear_vel ), 0.1 ); // Avoid division by zero
  LQRGains gains = calc_lqr( v );

  // 3. Compute Control Law
  // u = -Kx -> omega = - (k1 * lateral_error + k2 * heading_error)
  // Note: Implementation B uses a feedforward term (v/R), but if the goal is a
  // point-waypoint, we rely on the feedback logic.
  double omega_fb = -gains.k1 * lateral_error - gains.k2 * heading_error;

  cmd.linear_vel_cmd = curr_linear_vel; // Maintain commanded velocity
  cmd.angual_vel_cmd = omega_fb;

  return cmd;
}

LQRController::LQRGains LQRController::calc_lqr( double v )
{
  LQRGains gains;

  // Analytical solution to the Continuous Algebraic Riccati Equation
  // for the kinematic unicycle model linearized for path following.
  // P is the solution to: A^T P + P A - P B R^-1 B^T P + Q = 0

  double p12 = std::sqrt( q11_ * r_ );
  double p11 = std::sqrt( q11_ * ( 2.0 * p12 * v + q22_ ) / ( v * v ) );
  double p22 = std::sqrt( r_ ) * v * p11 / std::sqrt( q11_ );

  gains.k1 = ( 1.0 / r_ ) * p12;
  gains.k2 = ( 1.0 / r_ ) * p22;

  return gains;
}

bool LQRController::check_goal_completion( const Waypoint &goal, const Pose &pose, bool is_final_goal )
{
  double dist = std::hypot(goal.x - pose.x, goal.y - pose.y);
  
  // Retrieve tolerances (ideally from base_params_ via WaypointControllerBase)
  double dist_tolerance = 0.2; 
  
  if (is_final_goal) {
    return dist < dist_tolerance;
  }
  
  // For intermediate waypoints, we might accept a larger tolerance
  return dist < (dist_tolerance * 2.0);
}

} // namespace waypoint_controller
