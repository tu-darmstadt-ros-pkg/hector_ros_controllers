cmake_minimum_required(VERSION 3.16)
project(dynamics_mock_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS hardware_interface pluginlib pinocchio rclcpp
                                 rclcpp_lifecycle)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED src/dynamics_mock_hardware.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_compile_definitions(
  ${PROJECT_NAME} PUBLIC PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
                         PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR)

pluginlib_export_plugin_description_file(hardware_interface
                                         dynamics_mock_hardware_plugin.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES
  DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_dynamics_mock_hardware
                  test/test_dynamics_mock_hardware.cpp)
  target_link_libraries(test_dynamics_mock_hardware ${PROJECT_NAME})
  ament_target_dependencies(test_dynamics_mock_hardware hardware_interface
                            lifecycle_msgs rclcpp rclcpp_lifecycle)
  target_compile_definitions(
    test_dynamics_mock_hardware PRIVATE PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
                                        PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR)
endif()

ament_package()
