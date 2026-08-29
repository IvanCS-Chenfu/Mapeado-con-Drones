#include "multidron_gui_lib/main_window.hpp"

#include "multidron_gui_lib/scene3d_widget.hpp"

#include <QAction>
#include <QCheckBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFontDatabase>
#include <QFrame>
#include <QGroupBox>
#include <QScrollArea>
#include <QStatusBar>
#include <QTimer>
#include <QToolBar>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace multidron_gui_lib
{
namespace
{

QString TrackingName(std::int8_t state)
{
  switch (state) {
    case -1: return "SYSTEM_NOT_READY";
    case 0: return "NO_IMAGES_YET";
    case 1: return "NOT_INITIALIZED";
    case 2: return "OK";
    case 3: return "RECENTLY_LOST";
    case 4: return "LOST";
    case 5: return "OK_KLT";
    default: return QString("UNKNOWN(%1)").arg(state);
  }
}

QString PoseSourceName(std::uint8_t source)
{
  switch (source) {
    case 1: return "ORB";
    case 2: return "GLOBAL";
    case 3: return "GT_FALLBACK (no usado por GUI)";
    default: return "INVALID";
  }
}

}  // namespace

MainWindow::MainWindow(
  std::shared_ptr<GuiDataModel> model,
  QWidget * parent)
: QMainWindow(parent), model_(std::move(model))
{
  if (!model_) {
    throw std::invalid_argument("GuiDataModel no puede ser null");
  }
  BuildUi();
  ApplyDarkTheme();

  auto * timer = new QTimer(this);
  timer->setTimerType(Qt::PreciseTimer);
  timer->setInterval(33);
  connect(timer, &QTimer::timeout, this, &MainWindow::RefreshFromModel);
  timer->start();
  RefreshFromModel();
}

void MainWindow::BuildUi()
{
  setWindowTitle("MultiDrone Mapping — Fase 7");
  resize(1500, 900);
  setMinimumSize(1000, 650);

  scene_ = new Scene3DWidget(this);
  setCentralWidget(scene_);
  connect(scene_, &Scene3DWidget::SelectionChanged, this, &MainWindow::ShowSelection);

  BuildToolbar();
  BuildDroneDock();
  BuildInspectorDock();

  counters_label_ = new QLabel("Esperando datos ROS 2...", this);
  statusBar()->addPermanentWidget(counters_label_, 1);
  statusBar()->showMessage("world | arrastrar izq: órbita | der/medio: pan | rueda: zoom | doble click: reset");
}

void MainWindow::BuildToolbar()
{
  auto * toolbar = addToolBar("Capas");
  toolbar->setMovable(false);
  toolbar->setToolButtonStyle(Qt::ToolButtonTextOnly);

  auto add_layer_action = [this, toolbar](
      const QString & text, bool checked, auto setter) {
      QAction * action = toolbar->addAction(text);
      action->setCheckable(true);
      action->setChecked(checked);
      connect(action, &QAction::toggled, scene_, setter);
      return action;
    };

  add_layer_action("Sparse", true, &Scene3DWidget::SetSparseVisible);
  add_layer_action("KeyFrames", true, &Scene3DWidget::SetKeyframesVisible);
  add_layer_action("Drones", true, &Scene3DWidget::SetDronesVisible);
  add_layer_action("Fiduciales", true, &Scene3DWidget::SetFiducialsVisible);
  add_layer_action("Trayectorias", true, &Scene3DWidget::SetTrajectoriesVisible);
  add_layer_action("Vóxeles", false, &Scene3DWidget::SetVoxelsVisible);

  toolbar->addSeparator();
  auto * score_color = new QCheckBox("Color por score", toolbar);
  score_color->setChecked(false);
  toolbar->addWidget(score_color);
  connect(
    score_color, &QCheckBox::toggled,
    scene_, &Scene3DWidget::SetScoreColorEnabled);

  auto * score_filter = new QCheckBox("Filtrar score", toolbar);
  score_filter->setChecked(false);
  toolbar->addWidget(score_filter);
  connect(
    score_filter, &QCheckBox::toggled,
    scene_, &Scene3DWidget::SetScoreFilterEnabled);

  auto * threshold = new QDoubleSpinBox(toolbar);
  threshold->setRange(0.0, 1.0);
  threshold->setSingleStep(0.05);
  threshold->setDecimals(2);
  threshold->setValue(0.0);
  threshold->setToolTip("Score mínimo visible cuando el filtro está activo");
  toolbar->addWidget(threshold);
  connect(
    threshold, qOverload<double>(&QDoubleSpinBox::valueChanged),
    this, [this](double value) {
      scene_->SetScoreThreshold(static_cast<float>(value));
    });
}

void MainWindow::BuildDroneDock()
{
  auto * dock = new QDockWidget("Drones", this);
  dock->setObjectName("droneDock");
  dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  dock->setMinimumWidth(300);

  auto * scroll = new QScrollArea(dock);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);

  drone_cards_container_ = new QWidget(scroll);
  drone_cards_layout_ = new QVBoxLayout(drone_cards_container_);
  drone_cards_layout_->setContentsMargins(8, 8, 8, 8);
  drone_cards_layout_->setSpacing(8);
  drone_cards_layout_->addStretch(1);
  scroll->setWidget(drone_cards_container_);
  dock->setWidget(scroll);
  addDockWidget(Qt::RightDockWidgetArea, dock);
}

void MainWindow::BuildInspectorDock()
{
  auto * dock = new QDockWidget("Inspector", this);
  dock->setObjectName("inspectorDock");
  dock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
  dock->setMinimumHeight(150);

  inspector_label_ = new QLabel("Sin selección", dock);
  inspector_label_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  inspector_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  inspector_label_->setWordWrap(true);
  inspector_label_->setMargin(10);
  inspector_label_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  dock->setWidget(inspector_label_);
  addDockWidget(Qt::BottomDockWidgetArea, dock);
}

void MainWindow::ApplyDarkTheme()
{
  setStyleSheet(R"(
    QMainWindow, QWidget {
      background: #11161e;
      color: #dce3ea;
    }
    QToolBar {
      background: #171e28;
      border: 0;
      spacing: 8px;
      padding: 6px;
    }
    QToolButton, QCheckBox, QDoubleSpinBox {
      color: #dce3ea;
      background: #202a36;
      border: 1px solid #2e3a48;
      border-radius: 5px;
      padding: 5px 8px;
    }
    QToolButton:checked, QCheckBox:checked {
      background: #2e526d;
      border-color: #4f86a8;
    }
    QDockWidget::title {
      background: #171e28;
      padding: 7px;
      font-weight: 600;
    }
    QGroupBox {
      border: 1px solid #2b3745;
      border-radius: 7px;
      margin-top: 8px;
      padding-top: 10px;
      font-weight: 600;
      background: #171e28;
    }
    QGroupBox::title {
      subcontrol-origin: margin;
      left: 10px;
      padding: 0 4px;
    }
    QStatusBar {
      background: #0d1218;
      color: #9fb0c1;
    }
    QScrollArea {
      border: 0;
    }
  )");
}

void MainWindow::RefreshFromModel()
{
  const GuiSnapshot snapshot = model_->Snapshot();
  scene_->SetSnapshot(snapshot);
  UpdateDroneCards(snapshot);

  const std::size_t sparse = snapshot.sparse_points ? snapshot.sparse_points->size() : 0U;
  const std::size_t keyframes = snapshot.keyframes ? snapshot.keyframes->size() : 0U;
  const std::size_t drones = snapshot.drones ? snapshot.drones->size() : 0U;
  const std::size_t fiducials = snapshot.fiducials ? snapshot.fiducials->size() : 0U;
  const std::size_t trajectories = snapshot.trajectories ? snapshot.trajectories->size() : 0U;
  const std::size_t voxels = snapshot.voxels ? snapshot.voxels->size() : 0U;
  counters_label_->setText(
    QString("gen %1 | MP %2 | KF %3 | drones %4 | fid %5 | traj %6 | vox %7")
    .arg(static_cast<qulonglong>(snapshot.generation))
    .arg(static_cast<qulonglong>(sparse))
    .arg(static_cast<qulonglong>(keyframes))
    .arg(static_cast<qulonglong>(drones))
    .arg(static_cast<qulonglong>(fiducials))
    .arg(static_cast<qulonglong>(trajectories))
    .arg(static_cast<qulonglong>(voxels)));
}

void MainWindow::ShowSelection(const QString & description)
{
  inspector_label_->setText(description);
}

void MainWindow::UpdateDroneCards(const GuiSnapshot & snapshot)
{
  if (!snapshot.drones) {
    return;
  }

  for (const auto & item : *snapshot.drones) {
    const std::uint32_t drone_id = item.first;
    const DroneState & drone = item.second;
    if (!drone_cards_.contains(drone_id)) {
      drone_cards_layout_->insertWidget(
        std::max(0, drone_cards_layout_->count() - 1),
        CreateDroneCard(drone_id).root);
    }

    DroneCard & card = drone_cards_[drone_id];
    const QString visual_state = drone.has_world_pose ?
      (drone.lost_or_unavailable ? "PERDIDO — última pose conocida" : "POSE WORLD OK") :
      "SIN POSE WORLD VÁLIDA";
    card.state->setText(visual_state);
    card.state->setStyleSheet(
      drone.lost_or_unavailable ? "color:#ffb74d;font-weight:600;" :
      "color:#81c784;font-weight:600;");

    if (drone.has_world_pose) {
      card.pose->setText(
        QString("x %1   y %2   z %3\nyaw %4 rad")
        .arg(drone.position.x(), 0, 'f', 3)
        .arg(drone.position.y(), 0, 'f', 3)
        .arg(drone.position.z(), 0, 'f', 3)
        .arg(drone.yaw_rad, 0, 'f', 3));
    } else {
      card.pose->setText("x —   y —   z —\nyaw —");
    }

    card.tracking->setText(
      QString("tracking: %1\nsource: %2\nepoch: %3   pose_rev: %4")
      .arg(TrackingName(drone.tracking_state))
      .arg(PoseSourceName(drone.pose_source))
      .arg(static_cast<qulonglong>(drone.map_epoch))
      .arg(static_cast<qulonglong>(drone.pose_revision)));

    QString task_text = "Fase 6: sin datos de tarea";
    if (snapshot.tasks) {
      const auto task = snapshot.tasks->find(drone_id);
      if (task != snapshot.tasks->end()) {
        task_text = QString("%1 | %2\n%3")
          .arg(QString::fromStdString(task->second.task_type))
          .arg(QString::fromStdString(task->second.state))
          .arg(QString::fromStdString(task->second.detail));
      }
    }
    card.task->setText(task_text);
  }
}

MainWindow::DroneCard MainWindow::CreateDroneCard(std::uint32_t drone_id)
{
  DroneCard card;
  auto * group = new QGroupBox(QString("Drone %1").arg(drone_id), drone_cards_container_);
  auto * layout = new QVBoxLayout(group);
  layout->setContentsMargins(10, 14, 10, 10);
  layout->setSpacing(6);

  card.root = group;
  card.state = new QLabel("Esperando NavigationState", group);
  card.pose = new QLabel("x —   y —   z —\nyaw —", group);
  card.tracking = new QLabel("tracking: —", group);
  card.task = new QLabel("Fase 6: sin datos de tarea", group);
  card.pose->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  card.tracking->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  card.task->setWordWrap(true);

  layout->addWidget(card.state);
  layout->addWidget(card.pose);
  layout->addWidget(card.tracking);
  layout->addWidget(card.task);
  drone_cards_.insert(drone_id, card);
  return card;
}

}  // namespace multidron_gui_lib
