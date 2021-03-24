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

  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", n))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf_ = urdf.getJoint(joint_name);
  if (!joint_urdf_)
  {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  // Start command subscriber
  sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &VelocityToPositionController::setCommandCB, this);

  return true;
}

void VelocityToPositionController::starting(const ros::Time &time) {
  vel_command_ = 0.0;
  pos_command_ = joint_.getPosition();
}

void VelocityToPositionController::update(const ros::Time &time, const ros::Duration &period) {
  // Integrate velocity
  pos_command_ += vel_command_ * period.toSec();
  enforceJointLimits(pos_command_);
  joint_.setCommand(pos_command_);
}

void VelocityToPositionController::setCommandCB(const std_msgs::Float64ConstPtr &msg) {
  vel_command_ = msg->data;
}

void VelocityToPositionController::enforceJointLimits(double &command) {
  // Check that this joint has applicable limits
  if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC) {
    if (command > joint_urdf_->limits->upper) { // above upper limit
      command = joint_urdf_->limits->upper;
    } else if (command < joint_urdf_->limits->lower) { // below lower limit
      command = joint_urdf_->limits->lower;
    }
  }
}

} // namespace hector_ros_controllers

PLUGINLIB_EXPORT_CLASS(hector_ros_controllers::VelocityToPositionController, controller_interface::ControllerBase);
