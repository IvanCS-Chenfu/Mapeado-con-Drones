cmake_minimum_required(VERSION 3.5)
project(orbslam3)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/iron/lib/python3.10/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc features2d calib3d highgui aruco)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
# AÑADIDO #
find_package(orbslam3_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# AÑADIDO #

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
)

include_directories(${OpenCV_INCLUDE_DIRS})

install(FILES
  ${ORB_SLAM3_ROOT_DIR}/lib/libORB_SLAM3.so
  DESTINATION lib
)

link_directories(
  include
)

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)
target_link_libraries(mono ${OpenCV_LIBS})

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)
target_link_libraries(rgbd ${OpenCV_LIBS})

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
  src/stereo/navigation-state-estimator.cpp
  src/stereo/visual-evidence-metrics.cpp
  src/stereo/fiducial-detector.cpp
)
# AÑADIDO #
# ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin orbslam3_msgs geometry_msgs std_msgs tf2_ros tf2_geometry_msgs)
# AÑADIDO #
target_link_libraries(stereo ${OpenCV_LIBS})

add_executable(fiducial_visualizer
  src/stereo/fiducial-visualizer-node.cpp
)
ament_target_dependencies(fiducial_visualizer rclcpp sensor_msgs cv_bridge)
target_link_libraries(fiducial_visualizer ${OpenCV_LIBS})

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)
target_link_libraries(stereo-inertial ${OpenCV_LIBS})

install(TARGETS mono rgbd stereo stereo-inertial fiducial_visualizer
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fiducial_detector
    test/test_fiducial_detector.cpp
    src/stereo/fiducial-detector.cpp
  )
  target_include_directories(test_fiducial_detector PRIVATE src/stereo)
  target_link_libraries(test_fiducial_detector ${OpenCV_LIBS})

  ament_add_gtest(test_navigation_state_estimator
    test/test_navigation_state_estimator.cpp
    src/stereo/navigation-state-estimator.cpp
  )
  target_include_directories(test_navigation_state_estimator PRIVATE src/stereo)
  target_link_libraries(test_navigation_state_estimator Sophus::Sophus)

  ament_add_gtest(test_visual_evidence_metrics
    test/test_visual_evidence_metrics.cpp
    src/stereo/visual-evidence-metrics.cpp
  )
  target_include_directories(test_visual_evidence_metrics PRIVATE src/stereo)
  target_link_libraries(test_visual_evidence_metrics ${OpenCV_LIBS})
endif()

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()
