#include "multidron_gui_lib/gui_data_model.hpp"
#include "multidron_gui_lib/main_window.hpp"

#include <gtest/gtest.h>

#include <QApplication>
#include <QCoreApplication>
#include <QDockWidget>
#include <QGroupBox>
#include <QScrollArea>
#include <QToolButton>

TEST(MainWindow, BuildsMissionRegionMenuOutsideDroneDock)
{
  int argc = 1;
  char app_name[] = "test_main_window";
  char * argv[] = {app_name, nullptr};
  std::unique_ptr<QApplication> owned_app;
  if (QApplication::instance() == nullptr) {
    owned_app = std::make_unique<QApplication>(argc, argv);
  }

  auto model = std::make_shared<multidron_gui_lib::GuiDataModel>();
  model->SetMissionRegions(
    {multidron_gui_lib::MissionRegionVisual{
        "level_0_AB", 0U, "AB", QVector3D(-1.0F, -1.0F, 0.0F),
        QVector3D(1.0F, 0.0F, 2.0F)}});
  for (std::uint32_t drone_id = 1; drone_id <= 20; ++drone_id) {
    multidron_gui_lib::DroneState drone;
    drone.drone_id = drone_id;
    drone.sample_sequence = 1U;
    drone.received_steady_ns = 1;
    ASSERT_TRUE(model->UpdateDrone(drone));
  }

  multidron_gui_lib::MainWindow window(model);
  ASSERT_TRUE(QMetaObject::invokeMethod(&window, "RefreshFromModel", Qt::DirectConnection));
  auto * scroll = window.findChild<QScrollArea *>("droneCardsScroll");
  auto * regions = window.findChild<QToolButton *>("missionRegionsMenuButton");
  auto * region = window.findChild<QAction *>("missionRegionToggle_level_0_AB");
  ASSERT_NE(scroll, nullptr);
  ASSERT_NE(regions, nullptr);
  ASSERT_NE(region, nullptr);
  EXPECT_TRUE(regions->isEnabled());
  EXPECT_TRUE(region->isCheckable());
  region->trigger();
  EXPECT_TRUE(region->isChecked());
  EXPECT_EQ(scroll->widget()->findChildren<QGroupBox *>().size(), 20);
}

TEST(MainWindow, SizesDroneDockFromFirstAssignedTask)
{
  int argc = 1;
  char app_name[] = "test_main_window_task_width";
  char * argv[] = {app_name, nullptr};
  std::unique_ptr<QApplication> owned_app;
  if (QApplication::instance() == nullptr) {
    owned_app = std::make_unique<QApplication>(argc, argv);
  }

  auto model = std::make_shared<multidron_gui_lib::GuiDataModel>();
  multidron_gui_lib::DroneState drone;
  drone.drone_id = 1U;
  drone.sample_sequence = 1U;
  drone.received_steady_ns = 1;
  ASSERT_TRUE(model->UpdateDrone(drone));

  multidron_gui_lib::TaskVisual task;
  task.drone_id = 1U;
  task.task_type = "MAP_SECTION";
  task.state = "EXECUTING";
  task.region_id = "level_0_AB";
  task.detail = "candidate_unknown_with_clearance_and_pending_replan";
  model->UpdateTask(task);

  multidron_gui_lib::MainWindow window(model);
  window.show();
  ASSERT_TRUE(QMetaObject::invokeMethod(&window, "RefreshFromModel", Qt::DirectConnection));
  QCoreApplication::processEvents();

  auto * dock = window.findChild<QDockWidget *>("droneDock");
  const auto cards = window.findChildren<QGroupBox *>();
  ASSERT_NE(dock, nullptr);
  ASSERT_EQ(cards.size(), 1);
  EXPECT_GE(dock->width(), cards.front()->sizeHint().width());
}
