cmake_minimum_required(VERSION 3.8)
project(mission_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AxisAlignedBox.msg"
  "msg/MappingLevel.msg"
  "msg/BaseSubRoi.msg"
  "msg/MissionGeometry.msg"
  "msg/DroneRegistration.msg"
  "msg/DroneRegistry.msg"
  "msg/TaskState.msg"
  "msg/TaskStateArray.msg"
  "msg/TrajectoryWaypoint.msg"
  "msg/TrajectoryPlan.msg"
  "msg/SafetyEvent.msg"
  "srv/RegisterDrone.srv"
  "action/ExecuteTrajectory.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
