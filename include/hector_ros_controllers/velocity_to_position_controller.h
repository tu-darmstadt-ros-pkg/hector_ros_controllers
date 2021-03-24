#ifndef HECTOR_ROS_CONTROLLERS_VELOCITY_TO_POSITION_CONTROLLER_H
#define HECTOR_ROS_CONTROLLERS_VELOCITY_TO_POSITION_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <std_msgs/Float64.h>

namespace hector_ros_controllers {
class VelocityToPositionController : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
public:
  VelocityToPositionController();

  bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n) override;

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void starting(const ros::Time& time) override;

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  /**
 * \brief Callback from /command subscriber for setpoint
 */
  void setCommandCB(const std_msgs::Float64ConstPtr& msg);

  hardware_interface::JointHandle joint_;
  double vel_command_;                                /**< Last commanded velocity. */
  double pos_command_;
  ros::Subscriber sub_command_;
};
}

#endif