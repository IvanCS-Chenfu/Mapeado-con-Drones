#pragma once

#include <QColor>
#include <QPointF>
#include <QString>
#include <QVector3D>

#include <cstdint>
#include <optional>
#include <vector>

namespace multidron_gui_lib
{

enum class EntityType : std::uint8_t
{
  MapPoint,
  Drone,
  Keyframe,
  Fiducial,
  Trajectory,
  Voxel,
  MissionRegion,
};

struct EntityKey
{
  EntityType type = EntityType::MapPoint;
  std::uint32_t drone_id = 0;
  std::uint64_t map_epoch = 0;
  std::uint64_t id = 0;
};

struct SelectedEntity
{
  EntityKey key;
  QVector3D world_position;
  QString description;
  std::uint64_t snapshot_generation = 0;
};

struct PickCandidate
{
  SelectedEntity entity;
  QPointF screen_position;
  float depth = 0.0F;
};

QColor ScoreColor(float score);
bool SparsePointVisible(float score, bool filter_enabled, float threshold);
std::optional<SelectedEntity> SelectBestCandidate(
  const std::vector<PickCandidate> & candidates,
  const QPointF & click,
  double tolerance_px);

}  // namespace multidron_gui_lib
