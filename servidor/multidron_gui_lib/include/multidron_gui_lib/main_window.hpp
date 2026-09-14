#pragma once

#include "multidron_gui_lib/gui_data_model.hpp"

#include <QAction>
#include <QDockWidget>
#include <QLabel>
#include <QMainWindow>
#include <QMap>
#include <QMenu>
#include <QPushButton>
#include <QProgressBar>
#include <QSet>
#include <QToolButton>
#include <QVBoxLayout>

#include <cstdint>
#include <memory>

namespace multidron_gui_lib
{

class Scene3DWidget;
class MainWindow final : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(
    std::shared_ptr<GuiDataModel> model,
    QWidget * parent = nullptr);

private slots:
  void RefreshFromModel();
  void ShowSelection(const QString & description);

private:
  struct DroneCard
  {
    QWidget * root = nullptr;
    QLabel * state = nullptr;
    QLabel * pose = nullptr;
    QLabel * tracking = nullptr;
    QPushButton * task = nullptr;
    QProgressBar * progress = nullptr;
  };

  void BuildUi();
  void BuildToolbar();
  void BuildDroneDock();
  void BuildInspectorDock();
  void ApplyDarkTheme();
  void UpdateDroneCards(const GuiSnapshot & snapshot);
  void UpdateMissionRegionMenu(const GuiSnapshot & snapshot);
  void ApplyMissionRegionVisibility();
  void ApplyInitialDroneDockWidth();
  DroneCard CreateDroneCard(std::uint32_t drone_id);

  std::shared_ptr<GuiDataModel> model_;
  Scene3DWidget * scene_ = nullptr;
  QDockWidget * drone_dock_ = nullptr;
  QWidget * drone_cards_container_ = nullptr;
  QVBoxLayout * drone_cards_layout_ = nullptr;
  QLabel * inspector_label_ = nullptr;
  QLabel * counters_label_ = nullptr;
  QMap<std::uint32_t, DroneCard> drone_cards_;
  QToolButton * mission_regions_button_ = nullptr;
  QMenu * mission_regions_menu_ = nullptr;
  QMap<QString, QAction *> mission_region_actions_;
  QSet<QString> visible_mission_region_ids_;
  const void * mission_regions_identity_ = nullptr;
  bool drone_dock_initial_width_applied_ = false;
};

}  // namespace multidron_gui_lib
