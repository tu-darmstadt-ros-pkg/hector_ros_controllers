cmake_minimum_required(VERSION 3.16)
project(hector_self_collision_checker LANGUAGES CXX)

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

set(THIS_PACKAGE_INCLUDE_DEPENDS coal pinocchio rclcpp rclcpp_lifecycle)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

set(DEFS PINOCCHIO_WITH_HPP_FCL PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
         PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR)

add_library(${PROJECT_NAME} SHARED src/collision_checker.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(
  ${PROJECT_NAME} PUBLIC coal::coal pinocchio::pinocchio rclcpp::rclcpp
                         rclcpp_lifecycle::rclcpp_lifecycle)
target_compile_definitions(${PROJECT_NAME} PUBLIC ${DEFS})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  install(DIRECTORY test/config DESTINATION share/${PROJECT_NAME}/test)

  ament_add_gmock(test_collision_checker test/test_collision_checker.cpp)
  target_link_libraries(test_collision_checker ${PROJECT_NAME}
                        ament_index_cpp::ament_index_cpp)

  find_package(ament_cmake_google_benchmark QUIET)
  if(ament_cmake_google_benchmark_FOUND)
    ament_add_google_benchmark(benchmark_collision_checker
                               test/benchmark_collision_checker.cpp)
    if(TARGET benchmark_collision_checker)
      target_link_libraries(benchmark_collision_checker ${PROJECT_NAME}
                            ament_index_cpp::ament_index_cpp)
    endif()
  endif()
endif()

ament_package()
