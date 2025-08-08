cmake_minimum_required(VERSION 3.8)
project(hector_controller_spawner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

add_executable(hector_controller_spawner src/hector_controller_spawner.cpp)

target_include_directories(
  hector_controller_spawner
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(hector_controller_spawner rclcpp std_msgs
                          controller_manager_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # for ROS 2 launch‐testing
  find_package(launch_testing_ament_cmake REQUIRED)
  # This will install and run your test/controller_spawner_launch.py
  add_launch_test(test/test_spawner_basic.test.py TIMEOUT 360)
  add_launch_test(test/test_spawner_twice.test.py TIMEOUT 360)
  add_launch_test(test/test_spawner_estop.test.py TIMEOUT 360)
  add_launch_test(test/test_spawner_chain_detection.test.py TIMEOUT 360)
endif()

install(TARGETS hector_controller_spawner DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch test
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
