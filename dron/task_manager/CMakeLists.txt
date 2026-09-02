cmake_minimum_required(VERSION 3.8)
project(task_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(mission_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(task_manager_lib REQUIRED)

add_executable(task_manager_node src/task_manager_node.cpp)
ament_target_dependencies(task_manager_node
  mission_msgs rclcpp std_msgs task_manager_lib)
install(TARGETS task_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
