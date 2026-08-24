cmake_minimum_required(VERSION 3.8)
project(dron_individual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lib_tray REQUIRED)
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



install(DIRECTORY
  launch
  config

  DESTINATION share/${PROJECT_NAME}/
)

set(ORB_VOCABULARY_FILE "" CACHE FILEPATH
  "Ruta al ORBvoc.txt completo preparado fuera de src")
if(NOT ORB_VOCABULARY_FILE OR NOT EXISTS "${ORB_VOCABULARY_FILE}")
  message(FATAL_ERROR
    "ORB_VOCABULARY_FILE debe apuntar al ORBvoc.txt completo; use build_selected_packages.sh")
endif()
file(SIZE "${ORB_VOCABULARY_FILE}" ORB_VOCABULARY_SIZE)
if(ORB_VOCABULARY_SIZE LESS 100000000)
  message(FATAL_ERROR "ORB_VOCABULARY_FILE no contiene el vocabulario completo")
endif()
install(FILES "${ORB_VOCABULARY_FILE}"
  DESTINATION share/${PROJECT_NAME}/config/orbslam/vocabulary
  RENAME ORBvoc.txt)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/TrayAction.action"

  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rclcpp_action)


add_executable(clock src/otros/clock.cpp)
ament_target_dependencies(clock rclcpp)



add_executable(gen_tray src/control_tray/gen_tray.cpp)
ament_target_dependencies(gen_tray rclcpp geometry_msgs std_msgs rclcpp_action lib_tray)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gen_tray "${cpp_typesupport_target}")



add_executable(control_calcular_fuerzas src/control_tray/control_calcular_fuerzas.cpp)
ament_target_dependencies(control_calcular_fuerzas rclcpp geometry_msgs std_msgs rclcpp_action Eigen3)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(control_calcular_fuerzas "${cpp_typesupport_target}")

add_executable(aplicar_fuerzas_dron src/control_tray/aplicar_fuerzas_dron.cpp)
ament_target_dependencies(aplicar_fuerzas_dron rclcpp geometry_msgs std_msgs Eigen3)




add_executable(control_dron src/vision/control_dron.cpp)
ament_target_dependencies(control_dron rclcpp std_msgs geometry_msgs rclcpp_action)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(control_dron "${cpp_typesupport_target}")




install(TARGETS
  clock
  gen_tray
  control_calcular_fuerzas
  aplicar_fuerzas_dron
  control_dron

  DESTINATION lib/${PROJECT_NAME}
)


install(PROGRAMS

  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
