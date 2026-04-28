cmake_minimum_required(VERSION 3.16)
project(hector_ros_controllers_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/SafetyPositionControllerStatus.msg"
  "msg/SafetyForwardControllerStatus.msg"
  "msg/SyncStatus.msg"
  "action/DriveFlipperGroup.action"
  "action/SyncFlipperGroup.action"
  DEPENDENCIES
  std_msgs
  action_msgs)

ament_package()
