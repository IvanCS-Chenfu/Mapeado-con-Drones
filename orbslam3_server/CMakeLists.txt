cmake_minimum_required(VERSION 3.8)
project(orbslam3_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orbslam3_msgs REQUIRED)
find_package(orbslam3_multi REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(global_map_server src/global_map_server.cpp)
target_include_directories(global_map_server PRIVATE include)
ament_target_dependencies(
  global_map_server geometry_msgs orbslam3_msgs orbslam3_multi rclcpp sensor_msgs
  std_msgs visualization_msgs)
target_link_libraries(global_map_server Eigen3::Eigen)

install(
  TARGETS global_map_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_primary_queue test/test_primary_queue.cpp)
  target_include_directories(test_primary_queue PRIVATE include)
  ament_target_dependencies(test_primary_queue orbslam3_msgs)

  ament_add_gtest(test_ground_truth_buffer test/test_ground_truth_buffer.cpp)
  target_include_directories(test_ground_truth_buffer PRIVATE include)
  ament_target_dependencies(test_ground_truth_buffer geometry_msgs)

  ament_add_gtest(test_secondary_queue test/test_secondary_queue.cpp)
  target_include_directories(test_secondary_queue PRIVATE include)
  ament_target_dependencies(test_secondary_queue orbslam3_multi)

  ament_add_gtest(test_submap_color test/test_submap_color.cpp)
  target_include_directories(test_submap_color PRIVATE include)
  ament_target_dependencies(test_submap_color std_msgs)
endif()

ament_package()
