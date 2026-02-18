#ifndef LQR_CONTROLLER__LQR_CONTROLLER_HPP_
#define LQR_CONTROLLER__LQR_CONTROLLER_HPP_

#include "waypoint_controller/waypoint_controller_base.hpp"

namespace waypoint_controller
{

class LQRController : public waypoint_controller::WaypointControllerBase
{
  public: 
    MoveCommand computeCommand( const Waypoint &goal, const Pose &pose, const double &curr_linear_vel,
                              const double &curr_angular_vel ) override;
    
    bool check_goal_completion( const Waypoint &goal, const Pose &pose, bool is_final_goal ) override; 

  private: 

  void calc_lqr(); 

};
} // namespace waypoint_controller

#endif