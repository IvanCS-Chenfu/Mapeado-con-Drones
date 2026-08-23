cmake_minimum_required(VERSION 3.8)
project(orbslam3_multi)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orbslam3_msgs REQUIRED)
find_package(rclcpp REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(raw_map_database SHARED
  src/covisibility_database.cpp
  src/fiducial_anchor_manager.cpp
  src/fused_landmark_manager.cpp
  src/global_map_builder.cpp
  src/global_pose_store.cpp
  src/landmark_score_manager.cpp
  src/loop_pipeline.cpp
  src/optimization_manager.cpp
  src/optimization_validator.cpp
  src/pose_graph_builder.cpp
  src/raw_map_database.cpp
  src/sparse_global_backend.cpp
)
target_include_directories(raw_map_database PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(raw_map_database geometry_msgs orbslam3_msgs rclcpp)
target_link_libraries(raw_map_database Eigen3::Eigen)

install(
  TARGETS raw_map_database
  EXPORT export_orbslam3_multi
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_orbslam3_multi HAS_LIBRARY_TARGET)
ament_export_libraries(raw_map_database)
ament_export_dependencies(Eigen3 geometry_msgs orbslam3_msgs rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_raw_map_database test/test_raw_map_database.cpp)
  target_link_libraries(test_raw_map_database raw_map_database)
  ament_target_dependencies(test_raw_map_database orbslam3_msgs rclcpp)

  ament_add_gtest(test_sparse_global_backend test/test_sparse_global_backend.cpp)
  target_link_libraries(test_sparse_global_backend raw_map_database)
  ament_target_dependencies(test_sparse_global_backend geometry_msgs orbslam3_msgs rclcpp)

  ament_add_gtest(test_fiducial_anchor_manager test/test_fiducial_anchor_manager.cpp)
  target_link_libraries(test_fiducial_anchor_manager raw_map_database)
  ament_target_dependencies(test_fiducial_anchor_manager geometry_msgs)

  ament_add_gtest(test_landmark_score_manager test/test_landmark_score_manager.cpp)
  target_link_libraries(test_landmark_score_manager raw_map_database)
  ament_target_dependencies(test_landmark_score_manager orbslam3_msgs)

  ament_add_gtest(test_global_map_builder test/test_global_map_builder.cpp)
  target_link_libraries(test_global_map_builder raw_map_database)
  ament_target_dependencies(test_global_map_builder geometry_msgs orbslam3_msgs)

  ament_add_gtest(test_scalability_3g test/test_scalability_3g.cpp)
  target_link_libraries(test_scalability_3g raw_map_database)
  ament_target_dependencies(test_scalability_3g geometry_msgs orbslam3_msgs)

  ament_add_gtest(
    test_fiducial_optimization test/test_fiducial_optimization.cpp)
  target_link_libraries(test_fiducial_optimization raw_map_database)
  ament_target_dependencies(test_fiducial_optimization geometry_msgs)

  ament_add_gtest(test_loop_pipeline test/test_loop_pipeline.cpp)
  target_link_libraries(test_loop_pipeline raw_map_database)
  ament_target_dependencies(test_loop_pipeline geometry_msgs orbslam3_msgs rclcpp)

  ament_add_gtest(
    test_fused_landmark_manager test/test_fused_landmark_manager.cpp)
  target_link_libraries(test_fused_landmark_manager raw_map_database)
  ament_target_dependencies(
    test_fused_landmark_manager geometry_msgs orbslam3_msgs rclcpp)
endif()

ament_package()
