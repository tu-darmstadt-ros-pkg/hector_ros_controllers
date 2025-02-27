cmake_minimum_required(VERSION 3.16)
project(hector_ros_controllers LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
                      -Werror=missing-braces)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  speed_to_position_command_controller_parameters
  src/speed_to_position_command_controller_parameters.yaml
)

add_library(speed_to_position_command_controller SHARED
  src/speed_to_position_controllers_base.cpp
  src/speed_to_position_command_controller.cpp
)
target_compile_features(speed_to_position_command_controller PUBLIC cxx_std_17)
target_include_directories(speed_to_position_command_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/speed_to_position_command_controller>
)
target_link_libraries(speed_to_position_command_controller PUBLIC
  speed_to_position_command_controller_parameters
)
ament_target_dependencies(speed_to_position_command_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(speed_to_position_command_controller PRIVATE "SPEED_TO_POSITION_COMMAND_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface hector_ros_controller_plugin.xml)


if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_speed_to_position_command_controller
    test/test_load_speed_to_position_command_controller.cpp
  )
  target_link_libraries(test_load_speed_to_position_command_controller
    speed_to_position_command_controller
  )
  ament_target_dependencies(test_load_speed_to_position_command_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(test_speed_to_position_command_controller
    test/test_speed_to_position_command_controller.cpp
  )
  target_link_libraries(test_speed_to_position_command_controller
    speed_to_position_command_controller
  )

endif()

install(
  DIRECTORY include/
  DESTINATION include/speed_to_position_command_controller
)
install(
  TARGETS
    speed_to_position_command_controller
    speed_to_position_command_controller_parameters
  EXPORT export_speed_to_position_command_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_speed_to_position_command_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
