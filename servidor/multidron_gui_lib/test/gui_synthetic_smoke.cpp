#include "multidron_gui_lib/gui_data_model.hpp"
#include "multidron_gui_lib/main_window.hpp"

#include <QApplication>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QDir>
#include <QPushButton>
#include <QTimer>

#include <cmath>
#include <memory>

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  auto model = std::make_shared<multidron_gui_lib::GuiDataModel>();

  multidron_gui_lib::SparsePointVector points;
  points.reserve(2400U);
  for (std::uint64_t index = 0; index < 2400U; ++index) {
    const float angle = static_cast<float>(index) * 0.035F;
    multidron_gui_lib::SparsePoint point;
    point.position = QVector3D(
      8.0F * std::cos(angle), 8.0F * std::sin(angle),
      0.3F + static_cast<float>(index % 80U) * 0.025F);
    point.score = static_cast<float>(index % 100U) / 100.0F;
    point.drone_id = static_cast<std::uint32_t>(index % 2U) + 1U;
    point.source_index = index;
    points.push_back(point);
  }
  model->SetSparsePoints(std::move(points));

  multidron_gui_lib::KeyframeVector keyframes;
  for (std::int32_t index = 0; index < 24; ++index) {
    multidron_gui_lib::KeyframeVisual keyframe;
    keyframe.marker_id = index;
    keyframe.position = QVector3D(index * 0.5F - 6.0F, std::sin(index * 0.4F) * 3.0F, 1.0F);
    keyframe.line_points_world = {
      keyframe.position, keyframe.position + QVector3D(0.25F, 0.0F, 0.15F)};
    keyframes.push_back(keyframe);
  }
  model->SetKeyframes(std::move(keyframes));

  for (std::uint32_t drone_id = 1; drone_id <= 20; ++drone_id) {
    multidron_gui_lib::DroneState drone;
    drone.drone_id = drone_id;
    drone.sample_sequence = 1U;
    drone.map_epoch = 1U;
    drone.pose_revision = 1U;
    drone.tracking_state = drone_id % 5U == 0U ? 3 : 2;
    drone.pose_source = 2U;
    drone.has_world_pose = true;
    drone.lost_or_unavailable = drone_id % 5U == 0U;
    drone.received_steady_ns = 1;
    drone.position = QVector3D(
      static_cast<float>(drone_id % 5U) * 2.0F - 4.0F,
      static_cast<float>(drone_id / 5U) * 2.0F - 4.0F, 1.0F);
    model->UpdateDrone(drone);
  }

  multidron_gui_lib::FiducialObject fiducial;
  fiducial.object_id = 1;
  fiducial.size_m = QVector3D(2.0F, 2.0F, 2.0F);
  fiducial.position = QVector3D(0.0F, 0.0F, 1.0F);
  model->SetFiducials({fiducial});

  multidron_gui_lib::MissionRegionVector regions;
  for (std::uint32_t level = 0; level < 3U; ++level) {
    const float z_min = static_cast<float>(level) * 2.0F;
    const float z_max = z_min + 2.0F;
    regions.push_back(
      {"level_" + std::to_string(level) + "_AB", level, "AB",
        QVector3D(-10.0F, -10.0F, z_min), QVector3D(10.0F, 0.0F, z_max)});
    regions.push_back(
      {"level_" + std::to_string(level) + "_BC", level, "BC",
        QVector3D(0.0F, -10.0F, z_min), QVector3D(10.0F, 10.0F, z_max)});
    regions.push_back(
      {"level_" + std::to_string(level) + "_CD", level, "CD",
        QVector3D(-10.0F, 0.0F, z_min), QVector3D(10.0F, 10.0F, z_max)});
    regions.push_back(
      {"level_" + std::to_string(level) + "_DA", level, "DA",
        QVector3D(-10.0F, -10.0F, z_min), QVector3D(0.0F, 10.0F, z_max)});
  }
  model->SetMissionRegions(std::move(regions));

  model->SetVoxels(
  {
    {0, -1, 2, QVector3D(0.25F, -0.25F, 0.75F), 0.5F,
      multidron_gui_lib::VoxelState::Occupied, 0.85F},
    {1, -1, 2, QVector3D(0.75F, -0.25F, 0.75F), 0.5F,
      multidron_gui_lib::VoxelState::Free, 0.0F},
    {2, -1, 2, QVector3D(1.25F, -0.25F, 0.75F), 0.5F,
      multidron_gui_lib::VoxelState::Reserved, 0.0F}});
  multidron_gui_lib::TaskVisual task;
  task.drone_id = 1U;
  task.task_id = "map_section_level_0_AB";
  task.task_type = "MAP_SECTION";
  task.region_id = "level_0_AB";
  task.state = "ASSIGNED";
  task.detail = "Confirmada localmente";
  model->UpdateTask(task);

  multidron_gui_lib::MainWindow window(model);
  window.show();
  for (auto * checkbox : window.findChildren<QCheckBox *>()) {
    if (checkbox->text() == "Color por score" || checkbox->text() == "Filtrar score" ||
      checkbox->text() == "Ocupados" || checkbox->text() == "Libres" ||
      checkbox->text() == "Reservados")
    {
      checkbox->setChecked(true);
    }
  }
  for (auto * threshold : window.findChildren<QDoubleSpinBox *>()) {
    if (threshold->maximum() == 1.0) {
      threshold->setValue(0.35);
    }
  }
  QTimer::singleShot(
    700, [&window]() {
      for (const char * name : {
      "missionRegionToggle_level_1_BC", "missionRegionToggle_level_1_DA"})
      {
        auto * region = window.findChild<QAction *>(name);
        if (region) {
          region->trigger();
        }
      }
    });
  QTimer::singleShot(
    1400, [&window]() {
      const QString output = QDir::temp().filePath("multidron_gui_block3_synthetic.png");
      window.grab().save(output);
    });
  QTimer::singleShot(2000, &app, &QApplication::quit);
  return app.exec();
}
