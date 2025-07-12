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

install(TARGETS hector_controller_spawner DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
