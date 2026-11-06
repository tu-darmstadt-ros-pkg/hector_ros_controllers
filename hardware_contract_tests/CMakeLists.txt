cmake_minimum_required(VERSION 3.16)
project(hardware_contract_tests LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(
  ${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(
  ${PROJECT_NAME} INTERFACE hardware_interface::hardware_interface
                            rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(hardware_interface rclcpp rclcpp_lifecycle)
ament_package()
