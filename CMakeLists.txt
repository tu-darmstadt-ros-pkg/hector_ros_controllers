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

# Add parameter_libaries here

generate_parameter_library(
  velocity_to_position_command_controller_parameters
  src/velocity_to_position_command_controller_parameters.yaml
)

generate_parameter_library(
  passthrough_controller_parameters
  src/passthrough_controller_parameters.yaml
)

# Add controllers here
# Define Controllers as Shared Libraries

add_library(controllers SHARED
  src/velocity_to_position_controllers_base.cpp
  src/velocity_to_position_command_controller.cpp
  src/passthrough_controller.cpp
)
target_compile_features(controllers PUBLIC cxx_std_17)
target_include_directories(controllers PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/controllers>
)
target_link_libraries(controllers PUBLIC
  velocity_to_position_command_controller_parameters
  passthrough_controller_parameters
)
ament_target_dependencies(controllers PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(controllers PRIVATE "VELOCITY_TO_POSITION_COMMAND_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface hector_ros_controller_plugin.xml)


if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_velocity_to_position_command_controller
    test/test_load_velocity_to_position_command_controller.cpp
  )
  target_link_libraries(test_load_velocity_to_position_command_controller
    velocity_to_position_command_controller
  )
  ament_target_dependencies(test_load_velocity_to_position_command_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(test_velocity_to_position_command_controller
    test/test_velocity_to_position_command_controller.cpp
  )
  target_link_libraries(test_velocity_to_position_command_controller
    velocity_to_position_controller
  )

endif()

install(
  DIRECTORY include/
  DESTINATION include/
)
install(
  TARGETS
    controllers
    velocity_to_position_command_controller_parameters
    passthrough_controller_parameters
  EXPORT export_hector_ros_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_hector_ros_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
