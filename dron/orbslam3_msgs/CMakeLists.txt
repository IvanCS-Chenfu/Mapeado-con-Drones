cmake_minimum_required(VERSION 3.8)
project(orbslam3_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OrbDescriptor.msg"
  "msg/OrbKeyPoint.msg"
  "msg/OrbObservation.msg"
  "msg/OrbMapPoint.msg"
  "msg/OrbKeyFrame.msg"
  "msg/OrbMap.msg"
  "msg/MapCorrection.msg"
  "msg/CorrectedKeyFrame.msg"
  "msg/CorrectedKeyFrameArray.msg"
  "msg/FiducialLockStatus.msg"
  "msg/FiducialTagConfig.msg"
  "msg/FiducialTagObservation.msg"
  "msg/FiducialKeyFrameObservations.msg"
  "msg/NavigationState.msg"
  "msg/GlobalKeyFramePose.msg"

  "srv/GetOrbMap.srv"
  "srv/GetFiducialConfig.srv"
  "srv/GetGlobalKeyFramePose.srv"

  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
