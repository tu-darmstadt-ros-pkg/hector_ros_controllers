#ifndef LQR_CONTROLLER__LQR_CONTROLLER_HPP_
#define LQR_CONTROLLER__LQR_CONTROLLER_HPP_

#include "waypoint_controller/waypoint_controller_base.hpp"

#include <hector_ros_controllers/lqr_controller_parameters.hpp>

namespace waypoint_controller
{

class LQRController : public waypoint_controller::WaypointControllerBase
{
public:
  controller_interface::CallbackReturn on_init() override;

  MoveCommand computeCommand( const Waypoint &goal, const Pose &pose, const double &curr_linear_vel,
                              const double &curr_angular_vel ) override;

  bool check_goal_completion( const Waypoint &goal, const Pose &pose, bool is_final_goal ) override;

private:
  struct LQRGains {
    double k1;
    double k2;
  };

  LQRGains calc_lqr(double v);

  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_q11_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_q22_;

  rcl_interfaces::msg::SetParametersResult set_weights( const rclcpp::Parameter &p );

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;

  std::shared_ptr<lqr_controller::ParamListener> param_listener_;

  double q11_{ 1000.0 }; // Lateral error weight
  double q22_{ 1.0 };    // Heading error weight
  double r_{ 1.0 };      // Control effort weight

  // Internal state
  double last_lateral_error_{ 0.0 };
};

} // namespace waypoint_controller

#endif