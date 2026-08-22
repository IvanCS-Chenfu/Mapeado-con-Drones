cmake_minimum_required(VERSION 3.8)
project(simulacion_dron)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dron_individual REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    pipeline_flow_contract
    test/test_pipeline_flow_contract.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    global_map_config_contract
    test/test_global_map_config.py
    TIMEOUT 30
  )
endif()

install(DIRECTORY
  launch
  urdf
  config
  rviz
  web
  worlds

  DESTINATION share/${PROJECT_NAME}/
)




add_library(plugin_actuar_motores SHARED src/plugins/plugin_actuar_motores.cpp)
ament_target_dependencies(plugin_actuar_motores gazebo_ros rclcpp std_msgs)
target_include_directories(plugin_actuar_motores PRIVATE include ${gazebo_ros_INCLUDE_DIRS})
target_link_libraries(plugin_actuar_motores ${gazebo_ros_LIBRARIES})

add_library(plugin_sensor_groundtrurh SHARED src/plugins/plugin_sensor_groundtrurh.cpp)
ament_target_dependencies(plugin_sensor_groundtrurh gazebo_ros rclcpp geometry_msgs)
target_include_directories(plugin_sensor_groundtrurh PRIVATE include ${gazebo_ros_INCLUDE_DIRS})
target_link_libraries(plugin_sensor_groundtrurh ${gazebo_ros_LIBRARIES})


install(TARGETS
  plugin_actuar_motores
  plugin_sensor_groundtrurh

  LIBRARY DESTINATION lib
)

add_executable(clock src/otros/clock.cpp)
ament_target_dependencies(clock rclcpp)

add_executable(generador_URDF src/generar_URDF/generador_URDF.cpp)
ament_target_dependencies(generador_URDF rclcpp gazebo_msgs ament_index_cpp)




add_executable(graficar_GT src/graficar/graficar_GT.cpp)
ament_target_dependencies(graficar_GT rclcpp geometry_msgs ament_index_cpp std_msgs)

add_executable(graficar_tray src/graficar/graficar_tray.cpp)
ament_target_dependencies(graficar_tray rclcpp std_msgs dron_individual)

add_executable(graficar_GTvsTray src/graficar/graficar_GTvsTray.cpp)
ament_target_dependencies(graficar_GTvsTray rclcpp std_msgs dron_individual)




add_executable(scenario_runner_node src/control_tray/scenario_runner_node.cpp)
ament_target_dependencies(scenario_runner_node rclcpp rclcpp_action geometry_msgs std_msgs dron_individual ament_index_cpp)
target_link_libraries(scenario_runner_node yaml-cpp)


install(TARGETS
  clock
  generador_URDF
  graficar_GT
  graficar_tray
  graficar_GTvsTray

  scenario_runner_node

  DESTINATION lib/${PROJECT_NAME}
)


install(PROGRAMS
  src/graficar/graficar.py
  src/control_tray/gui_tray_multi.py
  src/visualizer/pipeline_flow_bridge.py
  src/visualizer/pipeline_flow_browser.py

  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
