cmake_minimum_required(VERSION 3.16)
project(hector_ros_controllers LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Werror=conversion
    -Werror=unused-but-set-variable
    -Werror=return-type
    -Werror=shadow
    -Werror=format
    -Werror=range-loop-construct
    -Werror=missing-braces)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    backward_ros
    controller_interface
    srdfdom
    visualization_msgs
    generate_parameter_library
    hardware_interface
    pluginlib
    hpp-fcl
    pinocchio
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_msgs
    std_srvs
    fcl
    tf2_ros
    tf2_geometry_msgs
    hector_ros_controllers_msgs
    lifecycle_msgs
    ad_kinematics)

foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

set(DEFS PINOCCHIO_WITH_HPP_FCL PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
         PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR)
# Add parameter_libraries here

generate_parameter_library(
  velocity_to_position_command_controller_parameters
  params/velocity_to_position_command_controller_parameters.yaml)

generate_parameter_library(
  self_collision_avoidance_controller_parameters
  params/self_collision_avoidance_controller_parameters.yaml)

generate_parameter_library(safety_forward_controller_parameters
                           params/safety_forward_controller_parameters.yaml)

generate_parameter_library(waypoint_controller_base_parameters
                           params/waypoint_controller_base_parameters.yaml)

generate_parameter_library(safety_position_controller_parameters
                           params/safety_position_controller_parameters.yaml)

# Add controllers here Define Controllers as Shared Libraries
add_library(
  controllers SHARED
  src/lqr_controller.cpp
  src/simple_waypoint_controller.cpp
  src/waypoint_controller_base.cpp
  src/velocity_to_position_controllers_base.cpp
  src/safety_forward_controller.cpp
  src/velocity_to_position_command_controller.cpp
  src/self_collision_avoidance_controller.cpp
  src/safety_position_controller.cpp
  src/collision_checker.cpp)

target_compile_features(controllers PUBLIC cxx_std_17)

target_include_directories(
  controllers PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include/controllers>)

target_link_libraries(
  controllers
  PUBLIC fcl
         hpp-fcl::hpp-fcl
         pinocchio::pinocchio
         velocity_to_position_command_controller_parameters
         self_collision_avoidance_controller_parameters
         safety_forward_controller_parameters
         safety_position_controller_parameters
         waypoint_controller_base_parameters)

target_link_libraries(controllers PUBLIC ad_kinematics::ad_kinematics)
ament_target_dependencies(controllers PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_compile_definitions(controllers PUBLIC ${DEFS})

pluginlib_export_plugin_description_file(controller_interface
                                         hector_ros_controller_plugin.xml)

install(DIRECTORY include/ DESTINATION include/)
install(
  TARGETS controllers
          velocity_to_position_command_controller_parameters
          self_collision_avoidance_controller_parameters
          safety_forward_controller_parameters
          safety_position_controller_parameters
          waypoint_controller_base_parameters
  EXPORT export_hector_ros_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES
  DESTINATION include)

ament_export_targets(export_hector_ros_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
