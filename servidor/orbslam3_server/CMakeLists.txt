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
find_package(yaml-cpp REQUIRED)

add_library(fiducial_object_interpreter src/fiducial_object_interpreter.cpp)
target_include_directories(fiducial_object_interpreter PUBLIC include)
ament_target_dependencies(
  fiducial_object_interpreter geometry_msgs orbslam3_msgs orbslam3_multi)
target_link_libraries(fiducial_object_interpreter Eigen3::Eigen yaml-cpp)

add_executable(global_map_server src/global_map_server.cpp)
target_include_directories(global_map_server PRIVATE include)
ament_target_dependencies(
  global_map_server geometry_msgs orbslam3_msgs orbslam3_multi rclcpp sensor_msgs
  std_msgs visualization_msgs)
target_link_libraries(global_map_server Eigen3::Eigen fiducial_object_interpreter)

install(
  TARGETS global_map_server fiducial_object_interpreter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/fiducial_config_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES scripts/fiducial_config.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_primary_queue test/test_primary_queue.cpp)
  target_include_directories(test_primary_queue PRIVATE include)
  ament_target_dependencies(test_primary_queue orbslam3_msgs)

  ament_add_gtest(test_secondary_queue test/test_secondary_queue.cpp)
  target_include_directories(test_secondary_queue PRIVATE include)
  ament_target_dependencies(test_secondary_queue orbslam3_multi)

  ament_add_gtest(test_submap_color test/test_submap_color.cpp)
  target_include_directories(test_submap_color PRIVATE include)
  ament_target_dependencies(test_submap_color std_msgs)

  ament_add_gtest(
    test_fiducial_object_interpreter test/test_fiducial_object_interpreter.cpp)
  target_include_directories(test_fiducial_object_interpreter PRIVATE include)
  target_compile_definitions(
    test_fiducial_object_interpreter PRIVATE
    FIDUCIAL_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/fiducial_objects.yaml")
  ament_target_dependencies(
    test_fiducial_object_interpreter geometry_msgs orbslam3_msgs orbslam3_multi)
  target_link_libraries(
    test_fiducial_object_interpreter fiducial_object_interpreter Eigen3::Eigen)

  ament_add_pytest_test(
    test_fiducial_config_server
    test/test_fiducial_config_server.py
    TIMEOUT 30
  )

  ament_add_pytest_test(
    test_backpressure_contract
    test/test_backpressure_contract.py
    TIMEOUT 30
  )
endif()

ament_package()
