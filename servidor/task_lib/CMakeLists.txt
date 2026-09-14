cmake_minimum_required(VERSION 3.8)
project(task_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED src/mission_config.cpp src/voxel_map.cpp
  src/dstar_lite.cpp src/facade_coverage.cpp src/reservation_overlay.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_mission_config test/test_mission_config.cpp)
  target_link_libraries(test_mission_config ${PROJECT_NAME})
  ament_add_gtest(test_voxel_map test/test_voxel_map.cpp)
  target_link_libraries(test_voxel_map ${PROJECT_NAME})
  ament_add_gtest(test_dstar_lite test/test_dstar_lite.cpp)
  target_link_libraries(test_dstar_lite ${PROJECT_NAME})
  ament_add_gtest(test_facade_coverage test/test_facade_coverage.cpp)
  target_link_libraries(test_facade_coverage ${PROJECT_NAME})
  ament_add_gtest(test_reservation_overlay test/test_reservation_overlay.cpp)
  target_link_libraries(test_reservation_overlay ${PROJECT_NAME})
endif()

ament_package()
