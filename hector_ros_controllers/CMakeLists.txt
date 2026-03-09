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
    hector_ros_controllers_msgs
    pluginlib
    hpp-fcl
    pinocchio
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    sensor_msgs
    std_msgs
    std_srvs
    fcl
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

generate_parameter_library(safety_position_controller_parameters
                           params/safety_position_controller_parameters.yaml)
# Add controllers here Define Controllers as Shared Libraries

add_library(
  controllers SHARED
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
         safety_position_controller_parameters)

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
  EXPORT export_hector_ros_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES
  DESTINATION include)

ament_export_targets(export_hector_ros_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(GTest CONFIG REQUIRED COMPONENTS GTest GMock)
  find_package(rtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  install(DIRECTORY test/config DESTINATION share/${PROJECT_NAME}/test)

  test_tools_add_doubles(
    controllers
    AMENT_DEPENDENCIES
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${hector_ros_controllers_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    controller_interface::controller_interface
    hardware_interface::hardware_interface
    realtime_tools::realtime_tools
    pluginlib::pluginlib
    ${srdfdom_TARGETS}
    fcl
    hpp-fcl::hpp-fcl
    pinocchio::pinocchio
    ad_kinematics::ad_kinematics
    velocity_to_position_command_controller_parameters
    self_collision_avoidance_controller_parameters
    safety_forward_controller_parameters
    safety_position_controller_parameters)

  ament_add_gmock(test_collision_checker test/test_collision_checker.cpp)
  target_include_directories(
    test_collision_checker PRIVATE ${PROJECT_SOURCE_DIR}/include
                                   ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(
    test_collision_checker ${PROJECT_NAME}::controllers_test_doubles
    ament_index_cpp::ament_index_cpp)
  target_compile_definitions(test_collision_checker PRIVATE ${DEFS})

  ament_add_gmock(test_safety_forward_controller
                  test/test_safety_forward_controller.cpp)
  target_include_directories(
    test_safety_forward_controller PRIVATE ${PROJECT_SOURCE_DIR}/include
                                           ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(
    test_safety_forward_controller ${PROJECT_NAME}::controllers_test_doubles
    ament_index_cpp::ament_index_cpp)
  target_compile_definitions(test_safety_forward_controller PRIVATE ${DEFS})

  ament_add_gmock(test_safety_position_controller
                  test/test_safety_position_controller.cpp)
  target_include_directories(
    test_safety_position_controller PRIVATE ${PROJECT_SOURCE_DIR}/include
                                            ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(
    test_safety_position_controller ${PROJECT_NAME}::controllers_test_doubles
    ament_index_cpp::ament_index_cpp)
  target_compile_definitions(test_safety_position_controller PRIVATE ${DEFS})

  find_package(ament_cmake_google_benchmark REQUIRED)
  ament_add_google_benchmark(benchmark_collision_checker
                             test/benchmark_collision_checker.cpp)
  if(TARGET benchmark_collision_checker)
    target_include_directories(
      benchmark_collision_checker PRIVATE ${PROJECT_SOURCE_DIR}/include
                                          ${PROJECT_SOURCE_DIR}/test)
    target_link_libraries(
      benchmark_collision_checker ${PROJECT_NAME}::controllers_test_doubles
      ament_index_cpp::ament_index_cpp)
    target_compile_definitions(benchmark_collision_checker PRIVATE ${DEFS})
  endif()

  ament_add_gmock(test_velocity_to_position_command_controller
                  test/test_velocity_to_position_command_controller.cpp)
  target_include_directories(
    test_velocity_to_position_command_controller
    PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(
    test_velocity_to_position_command_controller
    ${PROJECT_NAME}::controllers_test_doubles ament_index_cpp::ament_index_cpp)
  target_compile_definitions(test_velocity_to_position_command_controller
                             PRIVATE ${DEFS})
endif()

ament_package()
