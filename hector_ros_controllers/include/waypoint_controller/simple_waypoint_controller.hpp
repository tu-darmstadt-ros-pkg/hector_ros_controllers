#ifndef SIMPLE_WAYPOINT_CONTROLLER__SIMPLE_WAYPOINT_CONTROLLER_HPP_
#define SIMPLE_WAYPOINT_CONTROLLER__SIMPLE_WAYPOINT_CONTROLLER_HPP_

#include "waypoint_controller/waypoint_controller_base.hpp"

namespace waypoint_controller
{
class SimpleWaypointController : public waypoint_controller::WaypointControllerBase
{
  MoveCommand computeCommand( const Waypoint &goal, const Pose &pose, const double &curr_linear_vel,
                              const double &curr_angular_vel ) override;
};

} // namespace waypoint_controller

#endif // HPP
