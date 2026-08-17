cmake_minimum_required(VERSION 3.8)
project(lib_tray)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
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


# Librerías
add_library(gen_tray_pol3 src/generacion/gen_tray_pol3.cpp)
add_library(gen_tray_veltrap src/generacion/gen_tray_veltrap.cpp)
add_library(gen_tray_elipse src/generacion/gen_tray_elipse.cpp)




target_include_directories(gen_tray_pol3 PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_include_directories(gen_tray_veltrap PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_include_directories(gen_tray_elipse PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

target_compile_features(gen_tray_pol3 PUBLIC cxx_std_17)
target_compile_features(gen_tray_veltrap PUBLIC cxx_std_17)
target_compile_features(gen_tray_elipse PUBLIC cxx_std_17)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS gen_tray_pol3 gen_tray_veltrap gen_tray_elipse
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(gen_tray_pol3 gen_tray_veltrap gen_tray_elipse)




# Ejecutable

add_executable(test_gen_tray_pol3 test/test_gen_tray_pol3.cpp)
target_link_libraries(test_gen_tray_pol3 gen_tray_pol3)
target_compile_features(test_gen_tray_pol3 PUBLIC cxx_std_17)
install(TARGETS test_gen_tray_pol3 DESTINATION lib/${PROJECT_NAME})

add_executable(test_gen_tray_veltrap test/test_gen_tray_veltrap.cpp)
target_link_libraries(test_gen_tray_veltrap gen_tray_veltrap)
target_compile_features(test_gen_tray_veltrap PUBLIC cxx_std_17)
install(TARGETS test_gen_tray_veltrap DESTINATION lib/${PROJECT_NAME})

ament_package()
