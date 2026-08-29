cmake_minimum_required(VERSION 3.8)
project(multidron_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# La librería exporta tipos Qt en sus headers; se localiza Qt antes de importar
# el target para que sus dependencias transitivas estén definidas.
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Gui OpenGL)
if(QT_VERSION_MAJOR EQUAL 6)
  find_package(Qt6 REQUIRED COMPONENTS Widgets Gui OpenGL OpenGLWidgets)
  set(MULTIDRON_QT_LIBS Qt6::Widgets Qt6::Gui Qt6::OpenGL Qt6::OpenGLWidgets)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets Gui OpenGL)
  set(MULTIDRON_QT_LIBS Qt5::Widgets Qt5::Gui Qt5::OpenGL)
endif()

find_package(multidron_gui_lib REQUIRED)

add_executable(multidron_gui src/main.cpp)
ament_target_dependencies(multidron_gui rclcpp multidron_gui_lib)
target_link_libraries(multidron_gui ${MULTIDRON_QT_LIBS})

install(
  TARGETS multidron_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
