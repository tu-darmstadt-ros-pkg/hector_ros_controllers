#include "waypoint_controller/lqr_controller.hpp"

namespace waypoint_controller
{

MoveCommand LQRController::computeCommand( const Waypoint &goal, const Pose &pose,
                                           const double &curr_linear_vel,

                                           const double &curr_angular_vel )
{
    //TODO: define method 
}

bool LQRController::check_goal_completion( const Waypoint &goal, const Pose &pose, bool is_final_goal )
{
    //TODO: define method
}

void LQRController::calc_lqr(){ 
    //TODO: calc lqr params
}

} // namespace waypoint_controller
