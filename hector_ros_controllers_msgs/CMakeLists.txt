cmake_minimum_required(VERSION 3.8)
project(hector_ros_controllers_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} action/WaypointNavigation.action
                           DEPENDENCIES geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
