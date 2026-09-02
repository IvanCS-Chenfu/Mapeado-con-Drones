#include "multidron_gui_lib/gui_data_model.hpp"
#include "multidron_gui_lib/main_window.hpp"

#include <QApplication>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QDir>
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

  multidron_gui_lib::MainWindow window(model);
  window.show();
  for (auto * checkbox : window.findChildren<QCheckBox *>()) {
    if (checkbox->text() == "Color por score" || checkbox->text() == "Filtrar score") {
      checkbox->setChecked(true);
    }
  }
  for (auto * threshold : window.findChildren<QDoubleSpinBox *>()) {
    if (threshold->maximum() == 1.0) {
      threshold->setValue(0.35);
    }
  }
  QTimer::singleShot(
    1200, [&window]() {
      const QString output = QDir::temp().filePath("multidron_gui_block1_synthetic.png");
      window.grab().save(output);
    });
  QTimer::singleShot(1800, &app, &QApplication::quit);
  return app.exec();
}
