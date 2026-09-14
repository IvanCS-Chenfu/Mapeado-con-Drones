#include "multidron_gui_lib/main_window.hpp"

#include "multidron_gui_lib/scene3d_widget.hpp"

#include "orbslam3_msgs/msg/navigation_state.hpp"

#include <QAction>
#include <QCheckBox>
#include <QColor>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFontDatabase>
#include <QFrame>
#include <QGroupBox>
#include <QIcon>
#include <QMenu>
#include <QPixmap>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QStatusBar>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <limits>
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
    case orbslam3_msgs::msg::NavigationState::POSE_SOURCE_ORB: return "ORB";
    case orbslam3_msgs::msg::NavigationState::POSE_SOURCE_GLOBAL: return "GLOBAL";
    case orbslam3_msgs::msg::NavigationState::POSE_SOURCE_GT_FALLBACK:
      return "GT_FALLBACK";
    case orbslam3_msgs::msg::NavigationState::POSE_SOURCE_GT_FORCED: return "GT";
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
  statusBar()->showMessage("Frame: world");
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
  add_layer_action("Plan previsto", true, &Scene3DWidget::SetTrajectoriesVisible);
  add_layer_action("Vóxeles", false, &Scene3DWidget::SetVoxelsVisible);
  auto * occupied_voxels = new QCheckBox("Ocupados", toolbar);
  occupied_voxels->setChecked(true);
  occupied_voxels->setToolTip("Mostrar evidencia OCCUPIED del mapa voxel");
  toolbar->addWidget(occupied_voxels);
  connect(
    occupied_voxels, &QCheckBox::toggled,
    scene_, &Scene3DWidget::SetOccupiedVoxelsVisible);
  auto * free_voxels = new QCheckBox("Libres", toolbar);
  free_voxels->setChecked(true);
  free_voxels->setToolTip("Mostrar evidencia FREE del mapa voxel");
  toolbar->addWidget(free_voxels);
  connect(
    free_voxels, &QCheckBox::toggled,
    scene_, &Scene3DWidget::SetFreeVoxelsVisible);
  auto * reserved_voxels = new QCheckBox("Reservados", toolbar);
  reserved_voxels->setChecked(true);
  reserved_voxels->setToolTip("Mostrar reservas transitorias de trayectorias activas");
  toolbar->addWidget(reserved_voxels);
  connect(
    reserved_voxels, &QCheckBox::toggled,
    scene_, &Scene3DWidget::SetReservedVoxelsVisible);
  mission_regions_button_ = new QToolButton(toolbar);
  mission_regions_button_->setObjectName("missionRegionsMenuButton");
  mission_regions_button_->setText("Regiones");
  mission_regions_button_->setPopupMode(QToolButton::InstantPopup);
  mission_regions_menu_ = new QMenu(mission_regions_button_);
  mission_regions_button_->setMenu(mission_regions_menu_);
  mission_regions_button_->setEnabled(false);
  toolbar->addWidget(mission_regions_button_);

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
  threshold->setFixedWidth(72);
  auto * threshold_slider = new QSlider(Qt::Horizontal, toolbar);
  threshold_slider->setRange(0, 100);
  threshold_slider->setValue(0);
  threshold_slider->setFixedWidth(120);
  threshold_slider->setToolTip("Umbral visual de score");
  toolbar->addWidget(threshold_slider);
  toolbar->addWidget(threshold);
  connect(
    threshold, qOverload<double>(&QDoubleSpinBox::valueChanged),
    this, [this, threshold_slider](double value) {
      scene_->SetScoreThreshold(static_cast<float>(value));
      const int slider_value = static_cast<int>(std::lround(value * 100.0));
      if (threshold_slider->value() != slider_value) {
        threshold_slider->setValue(slider_value);
      }
    });
  connect(
    threshold_slider, &QSlider::valueChanged,
    threshold, [threshold](int value) {
      const double score = static_cast<double>(value) / 100.0;
      if (std::abs(threshold->value() - score) > 1e-9) {
        threshold->setValue(score);
      }
    });
}

void MainWindow::BuildDroneDock()
{
  auto * dock = new QDockWidget("Drones", this);
  drone_dock_ = dock;
  dock->setObjectName("droneDock");
  dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  dock->setMinimumWidth(300);

  auto * scroll = new QScrollArea(dock);
  scroll->setObjectName("droneCardsScroll");
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
  setStyleSheet(
    R"(
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
  UpdateMissionRegionMenu(snapshot);

  const std::size_t sparse = snapshot.sparse_points ? snapshot.sparse_points->size() : 0U;
  const std::size_t keyframes = snapshot.keyframes ? snapshot.keyframes->size() : 0U;
  const std::size_t drones = snapshot.drones ? snapshot.drones->size() : 0U;
  const std::size_t fiducials = snapshot.fiducials ? snapshot.fiducials->size() : 0U;
  const std::size_t trajectories = snapshot.trajectories ? snapshot.trajectories->size() : 0U;
  const std::size_t voxels = snapshot.voxels ? snapshot.voxels->size() : 0U;
  counters_label_->setText(
    QString("gen %1 | MP %2 | KF %3 | drones %4 | fid %5 | traj %6 | vox %7 | reg %8")
    .arg(static_cast<qulonglong>(snapshot.generation))
    .arg(static_cast<qulonglong>(sparse))
    .arg(static_cast<qulonglong>(keyframes))
    .arg(static_cast<qulonglong>(drones))
    .arg(static_cast<qulonglong>(fiducials))
    .arg(static_cast<qulonglong>(trajectories))
    .arg(static_cast<qulonglong>(voxels))
    .arg(
      static_cast<qulonglong>(
        snapshot.mission_regions ? snapshot.mission_regions->size() : 0U)));
}

void MainWindow::UpdateMissionRegionMenu(const GuiSnapshot & snapshot)
{
  const void * identity = snapshot.mission_regions.get();
  if (identity == mission_regions_identity_ || !mission_regions_menu_) {
    return;
  }
  mission_regions_identity_ = identity;
  mission_regions_menu_->clear();
  mission_region_actions_.clear();
  visible_mission_region_ids_.clear();
  if (!snapshot.mission_regions || snapshot.mission_regions->empty()) {
    mission_regions_button_->setEnabled(false);
    ApplyMissionRegionVisibility();
    return;
  }

  mission_regions_button_->setEnabled(true);
  std::uint32_t current_level = std::numeric_limits<std::uint32_t>::max();
  QMenu * level_menu = nullptr;
  std::size_t region_index = 0U;
  for (const auto & region : *snapshot.mission_regions) {
    if (region.level_index != current_level) {
      current_level = region.level_index;
      level_menu = mission_regions_menu_->addMenu(QString("Nivel %1").arg(current_level));
    }
    const QString id = QString::fromStdString(region.region_id);
    QAction * action = level_menu->addAction(QString::fromStdString(region.side));
    action->setObjectName(QString("missionRegionToggle_%1").arg(id));
    action->setToolTip(id);
    action->setCheckable(true);
    QPixmap swatch(12, 12);
    swatch.fill(QColor::fromHsv(static_cast<int>((region_index * 47U) % 360U), 185, 255));
    action->setIcon(QIcon(swatch));
    connect(
      action, &QAction::toggled, this, [this, id](bool visible) {
        if (visible) {
          visible_mission_region_ids_.insert(id);
        } else {
          visible_mission_region_ids_.remove(id);
        }
        ApplyMissionRegionVisibility();
      });
    mission_region_actions_.insert(id, action);
    ++region_index;
  }

  mission_regions_menu_->addSeparator();
  QAction * show_all = mission_regions_menu_->addAction("Mostrar todas");
  connect(
    show_all, &QAction::triggered, this, [this]() {
      for (auto * action : mission_region_actions_) {
        action->setChecked(true);
      }
    });
  QAction * hide_all = mission_regions_menu_->addAction("Ocultar todas");
  connect(
    hide_all, &QAction::triggered, this, [this]() {
      for (auto * action : mission_region_actions_) {
        action->setChecked(false);
      }
    });
  ApplyMissionRegionVisibility();
}

void MainWindow::ApplyMissionRegionVisibility()
{
  scene_->SetVisibleMissionRegions(visible_mission_region_ids_);
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

  bool has_assigned_task = false;
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
        has_assigned_task = true;
        task_text = QString("%1 | %2\n%3\n%4")
          .arg(QString::fromStdString(task->second.task_type))
          .arg(QString::fromStdString(task->second.state))
          .arg(QString::fromStdString(task->second.region_id))
          .arg(QString::fromStdString(task->second.detail));
      }
    }
    card.task->setText(task_text);
    if (card.progress != nullptr) {
      bool known = false;
      float progress = 0.0F;
      if (snapshot.tasks) {
        const auto task = snapshot.tasks->find(drone_id);
        if (task != snapshot.tasks->end() && task->second.progress_known) {
          known = true;
          progress = task->second.progress;
        }
      }
      card.progress->setVisible(known);
      if (known) {
        card.progress->setValue(static_cast<int>(std::lround(progress * 100.0F)));
      }
    }
    if (snapshot.tasks) {
      const auto task = snapshot.tasks->find(drone_id);
      card.task->setProperty(
        "mission_region_id", task == snapshot.tasks->end() ? QString{} :
        QString::fromStdString(task->second.region_id));
    }
  }

  if (has_assigned_task) {
    ApplyInitialDroneDockWidth();
  }
}

void MainWindow::ApplyInitialDroneDockWidth()
{
  if (drone_dock_initial_width_applied_ || drone_dock_ == nullptr ||
    drone_cards_container_ == nullptr || drone_cards_layout_ == nullptr)
  {
    return;
  }

  drone_cards_layout_->activate();
  int widest_card = 0;
  for (const DroneCard & card : drone_cards_) {
    if (card.root != nullptr) {
      widest_card = std::max(widest_card, card.root->sizeHint().width());
    }
  }
  if (widest_card <= 0) {
    return;
  }

  const QMargins margins = drone_cards_layout_->contentsMargins();
  const int dock_chrome = std::max(0, drone_dock_->width() - drone_dock_->contentsRect().width());
  const int target_width = std::max(
    drone_dock_->minimumWidth(), widest_card + margins.left() + margins.right() + dock_chrome);
  drone_dock_initial_width_applied_ = true;
  QTimer::singleShot(
    0, this, [this, target_width]() {
      if (drone_dock_ != nullptr) {
        resizeDocks({drone_dock_}, {target_width}, Qt::Horizontal);
      }
    });
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
  card.task = new QPushButton("Fase 6: sin datos de tarea", group);
  card.task->setObjectName(QString("droneTask_%1").arg(drone_id));
  card.task->setFlat(true);
  card.task->setToolTip("Seleccionar el subROI de esta tarea en el inspector");
  card.task->setStyleSheet("text-align:left;padding:4px;");
  card.progress = new QProgressBar(group);
  card.progress->setObjectName(QString("droneTaskProgress_%1").arg(drone_id));
  card.progress->setRange(0, 100);
  card.progress->setFormat("Coverage %p%");
  card.progress->setVisible(false);
  connect(
    card.task, &QPushButton::clicked, this, [this, task_button = card.task]() {
      const QString region_id = task_button->property("mission_region_id").toString();
      if (!region_id.isEmpty()) {
        scene_->SelectMissionRegion(region_id);
      }
    });
  card.pose->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  card.tracking->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));

  layout->addWidget(card.state);
  layout->addWidget(card.pose);
  layout->addWidget(card.tracking);
  layout->addWidget(card.task);
  layout->addWidget(card.progress);
  drone_cards_.insert(drone_id, card);
  return card;
}

}  // namespace multidron_gui_lib
