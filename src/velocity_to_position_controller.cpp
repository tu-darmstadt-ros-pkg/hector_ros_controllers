#include <hector_ros_controllers/velocity_to_position_controller.h>
#include <pluginlib/class_list_macros.hpp>


namespace hector_ros_controllers {

VelocityToPositionController::VelocityToPositionController()
: vel_command_(0.0),
  pos_command_(0.0) {
}

bool VelocityToPositionController::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n) {
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  // Start command subscriber
  sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &VelocityToPositionController::setCommandCB, this);

  return true;
}

void VelocityToPositionController::starting(const ros::Time &time) {
  vel_command_ = 0.0;
  pos_command_ = joint_.getPosition();
  ROS_INFO_STREAM("Started VelocityToPositionController, initialized position to " << pos_command_);
}

void VelocityToPositionController::update(const ros::Time &time, const ros::Duration &period) {
  // Integrate velocity
  pos_command_ += vel_command_ * period.toSec();
  ROS_INFO_STREAM("command: " << pos_command_);
  joint_.setCommand(pos_command_);
}

void VelocityToPositionController::setCommandCB(const std_msgs::Float64ConstPtr &msg) {
  vel_command_ = msg->data;
}

} // namespace hector_ros_controllers

PLUGINLIB_EXPORT_CLASS(hector_ros_controllers::VelocityToPositionController, controller_interface::ControllerBase);
