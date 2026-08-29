cmake_minimum_required(VERSION 3.8)
project(multidron_gui_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orbslam3_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Compatible con Qt5 (objetivo típico ROS 2 Humble/Ubuntu 22.04) y Qt6.
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Gui OpenGL)
if(QT_VERSION_MAJOR EQUAL 6)
  find_package(Qt6 REQUIRED COMPONENTS Widgets Gui OpenGL OpenGLWidgets)
  set(MULTIDRON_QT_LIBS Qt6::Widgets Qt6::Gui Qt6::OpenGL Qt6::OpenGLWidgets)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets Gui OpenGL)
  set(MULTIDRON_QT_LIBS Qt5::Widgets Qt5::Gui Qt5::OpenGL)
endif()

add_library(${PROJECT_NAME} SHARED
  include/multidron_gui_lib/types.hpp
  include/multidron_gui_lib/gui_data_model.hpp
  include/multidron_gui_lib/fiducial_config_loader.hpp
  include/multidron_gui_lib/ros_data_bridge.hpp
  include/multidron_gui_lib/scene3d_widget.hpp
  include/multidron_gui_lib/main_window.hpp
  src/gui_data_model.cpp
  src/fiducial_config_loader.cpp
  src/ros_data_bridge.cpp
  src/scene3d_widget.cpp
  src/main_window.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  orbslam3_msgs
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

# ament_target_dependencies() usa internamente la firma plana de
# target_link_libraries(). CMake no permite mezclarla con la firma keyword
# (PRIVATE/PUBLIC) para el mismo target, por lo que mantenemos aquí también
# la firma plana. Los headers públicos exponen tipos Qt, así que la
# transitividad de Qt es intencionada.
target_link_libraries(${PROJECT_NAME}
  yaml-cpp
  ${MULTIDRON_QT_LIBS}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  geometry_msgs orbslam3_msgs rclcpp sensor_msgs std_msgs visualization_msgs yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # El repositorio vigente usa estas exclusiones para fuentes sin cabecera legal
  # por fichero; la licencia de paquete sigue declarada como GPL-3.0-only.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_gui_data_model test/test_gui_data_model.cpp)
  target_link_libraries(test_gui_data_model ${PROJECT_NAME})

  ament_add_gtest(test_fiducial_config_loader test/test_fiducial_config_loader.cpp)
  target_link_libraries(test_fiducial_config_loader ${PROJECT_NAME})
endif()

ament_package()
