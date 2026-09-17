cmake_minimum_required(VERSION 3.8)
project(task_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(mission_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(orbslam3_msgs REQUIRED)
find_package(task_lib REQUIRED)

add_library(drone_registry SHARED src/drone_registry.cpp)
target_include_directories(drone_registry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(drone_registry mission_msgs)

add_library(workflow_scheduler SHARED src/workflow_scheduler.cpp)
target_include_directories(workflow_scheduler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(evidence_pipeline SHARED src/evidence_pipeline.cpp)
target_include_directories(evidence_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(evidence_pipeline mission_msgs task_lib)

add_executable(task_server_node src/task_server_node.cpp)
ament_target_dependencies(task_server_node
  ament_index_cpp mission_msgs orbslam3_msgs rclcpp rclcpp_action sensor_msgs std_msgs std_srvs task_lib)
target_link_libraries(task_server_node drone_registry evidence_pipeline workflow_scheduler)

install(TARGETS drone_registry evidence_pipeline workflow_scheduler task_server_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_drone_registry test/test_drone_registry.cpp)
  target_link_libraries(test_drone_registry drone_registry)
  ament_target_dependencies(test_drone_registry mission_msgs)
  ament_add_gtest(test_workflow_scheduler test/test_workflow_scheduler.cpp)
  target_link_libraries(test_workflow_scheduler workflow_scheduler)
  ament_add_gtest(test_evidence_pipeline test/test_evidence_pipeline.cpp)
  target_link_libraries(test_evidence_pipeline evidence_pipeline)
  ament_target_dependencies(test_evidence_pipeline mission_msgs task_lib)
endif()

ament_package()
