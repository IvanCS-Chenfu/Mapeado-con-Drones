#include "multidron_gui_lib/gui_data_model.hpp"
#include "multidron_gui_lib/main_window.hpp"
#include "multidron_gui_lib/ros_data_bridge.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include <QApplication>
#include <QCoreApplication>
#include <QSurfaceFormat>
#include <QTimer>
#include <QtGlobal>

#include <atomic>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{

struct QtArguments
{
  std::vector<std::vector<char>> storage;
  std::vector<char *> argv;
  int argc = 0;
};

QtArguments BuildQtArguments(int argc, char ** argv)
{
  QtArguments result;
  const auto non_ros = rclcpp::remove_ros_arguments(argc, argv);
  result.storage.reserve(non_ros.size());
  for (const auto & argument : non_ros) {
    result.storage.emplace_back(argument.begin(), argument.end());
    result.storage.back().push_back('\0');
  }
  result.argv.reserve(result.storage.size() + 1U);
  for (auto & argument : result.storage) {
    result.argv.push_back(argument.data());
  }
  result.argv.push_back(nullptr);
  result.argc = static_cast<int>(result.storage.size());
  return result;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Se fuerza un perfil compatible OpenGL 2.1 para mantener el mismo shader en
  // Qt5 y Qt6. Debe fijarse antes de crear QApplication.
  QSurfaceFormat format;
  format.setRenderableType(QSurfaceFormat::OpenGL);
  format.setVersion(2, 1);
  format.setProfile(QSurfaceFormat::CompatibilityProfile);
  format.setDepthBufferSize(24);
  format.setStencilBufferSize(8);
  format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
  QSurfaceFormat::setDefaultFormat(format);

  QtArguments qt_arguments = BuildQtArguments(argc, argv);
  QApplication app(qt_arguments.argc, qt_arguments.argv.data());
  QCoreApplication::setApplicationName("multidron_gui");
  QCoreApplication::setOrganizationName("Mapeado-con-Drones");

  auto model = std::make_shared<multidron_gui_lib::GuiDataModel>();
  auto bridge = std::make_shared<multidron_gui_lib::RosDataBridge>(model);

  RCLCPP_INFO(
    bridge->get_logger(), "[GUI-BOOT] Qt=%s process=independent world_frame=world",
    qVersion());

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(bridge);
  std::thread ros_thread([&executor]() {executor.spin();});

  multidron_gui_lib::MainWindow window(model);
  window.show();

  QTimer ros_liveness_timer;
  ros_liveness_timer.setInterval(100);
  QObject::connect(&ros_liveness_timer, &QTimer::timeout, &app, [&app]() {
    if (!rclcpp::ok()) {
      app.quit();
    }
  });
  ros_liveness_timer.start();

  const int result = app.exec();

  RCLCPP_INFO(bridge->get_logger(), "[GUI-SHUTDOWN] qt_event_loop_finished=true");
  executor.cancel();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  if (ros_thread.joinable()) {
    ros_thread.join();
  }
  bridge.reset();
  model.reset();
  return result;
}
