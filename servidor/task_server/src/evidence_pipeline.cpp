#include "task_server/evidence_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace task_server
{
namespace
{

bool IsFinite(const task_lib::Vec3 & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

task_lib::Vec3 Add(const task_lib::Vec3 & lhs, const task_lib::Vec3 & rhs)
{
  return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

task_lib::Vec3 Subtract(const task_lib::Vec3 & lhs, const task_lib::Vec3 & rhs)
{
  return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

task_lib::Vec3 Scale(const task_lib::Vec3 & value, double factor)
{
  return {value.x * factor, value.y * factor, value.z * factor};
}

double Dot(const task_lib::Vec3 & lhs, const task_lib::Vec3 & rhs)
{
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

task_lib::Vec3 Cross(const task_lib::Vec3 & lhs, const task_lib::Vec3 & rhs)
{
  return {
    lhs.y * rhs.z - lhs.z * rhs.y,
    lhs.z * rhs.x - lhs.x * rhs.z,
    lhs.x * rhs.y - lhs.y * rhs.x};
}

double Norm(const task_lib::Vec3 & value)
{
  return std::sqrt(Dot(value, value));
}

std::optional<task_lib::Vec3> Normalize(const task_lib::Vec3 & value)
{
  const double norm = Norm(value);
  if (!std::isfinite(norm) || norm <= 1e-9) {
    return std::nullopt;
  }
  return Scale(value, 1.0 / norm);
}

bool SameVec3(const task_lib::Vec3 & lhs, const task_lib::Vec3 & rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool SameSources(const KeyframeEvidenceSource & lhs, const KeyframeEvidenceSource & rhs)
{
  if (lhs.source_name != rhs.source_name || lhs.kind != rhs.kind ||
    lhs.source_revision != rhs.source_revision || !SameVec3(lhs.local_origin, rhs.local_origin) ||
    lhs.local_endpoints.size() != rhs.local_endpoints.size())
  {
    return false;
  }
  for (std::size_t index = 0U; index < lhs.local_endpoints.size(); ++index) {
    if (!SameVec3(lhs.local_endpoints[index], rhs.local_endpoints[index])) {
      return false;
    }
  }
  return true;
}

bool IsDepthSource(const std::string & source_name)
{
  return source_name.rfind("depth_", 0U) == 0U;
}

bool SamePose(const geometry_msgs::msg::Pose & lhs, const geometry_msgs::msg::Pose & rhs)
{
  return lhs.position.x == rhs.position.x && lhs.position.y == rhs.position.y &&
         lhs.position.z == rhs.position.z && lhs.orientation.x == rhs.orientation.x &&
         lhs.orientation.y == rhs.orientation.y && lhs.orientation.z == rhs.orientation.z &&
         lhs.orientation.w == rhs.orientation.w;
}

struct SparsePlaneFit
{
  task_lib::Vec3 point;
  task_lib::Vec3 normal;
  std::vector<task_lib::Vec3> inliers;
};

std::optional<SparsePlaneFit> FitSparsePlaneRansac(
  const std::vector<task_lib::Vec3> & points, std::size_t minimum_inliers,
  double residual_m)
{
  if (points.size() < minimum_inliers || !std::isfinite(residual_m) || residual_m <= 0.0) {
    return std::nullopt;
  }
  std::optional<SparsePlaneFit> best;
  double best_error = std::numeric_limits<double>::infinity();
  std::size_t tested = 0U;
  constexpr std::size_t kMaxHypotheses = 256U;
  for (std::size_t first = 0U; first + 2U < points.size() && tested < kMaxHypotheses; ++first) {
    for (std::size_t second = first + 1U;
      second + 1U < points.size() && tested < kMaxHypotheses; ++second)
    {
      for (std::size_t third = second + 1U;
        third < points.size() && tested < kMaxHypotheses; ++third, ++tested)
      {
        const auto normal = Normalize(
          Cross(
            Subtract(points[second], points[first]), Subtract(points[third], points[first])));
        if (!normal.has_value()) {
          continue;
        }
        SparsePlaneFit candidate;
        candidate.point = points[first];
        candidate.normal = *normal;
        double error = 0.0;
        for (const auto & point : points) {
          const double residual = std::abs(Dot(candidate.normal, Subtract(point, candidate.point)));
          if (residual <= residual_m) {
            candidate.inliers.push_back(point);
            error += residual;
          }
        }
        if (candidate.inliers.size() < minimum_inliers) {
          continue;
        }
        if (!best.has_value() || candidate.inliers.size() > best->inliers.size() ||
          (candidate.inliers.size() == best->inliers.size() && error < best_error))
        {
          best_error = error;
          best = std::move(candidate);
        }
      }
    }
  }
  return best;
}

struct PlaneCoordinate
{
  double u = 0.0;
  double v = 0.0;
};

double Cross2d(
  const PlaneCoordinate & origin, const PlaneCoordinate & lhs,
  const PlaneCoordinate & rhs)
{
  return (lhs.u - origin.u) * (rhs.v - origin.v) -
         (lhs.v - origin.v) * (rhs.u - origin.u);
}

std::vector<PlaneCoordinate> ConvexHull2d(std::vector<PlaneCoordinate> points)
{
  std::sort(
    points.begin(), points.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.u == rhs.u ? lhs.v < rhs.v : lhs.u < rhs.u;
    });
  points.erase(
    std::unique(
      points.begin(), points.end(), [](const auto & lhs, const auto & rhs) {
        return std::abs(lhs.u - rhs.u) <= 1e-9 && std::abs(lhs.v - rhs.v) <= 1e-9;
      }), points.end());
  if (points.size() < 3U) {
    return {};
  }
  std::vector<PlaneCoordinate> hull;
  hull.reserve(points.size() * 2U);
  for (const auto & point : points) {
    while (hull.size() >= 2U && Cross2d(hull[hull.size() - 2U], hull.back(), point) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(point);
  }
  const std::size_t lower_size = hull.size();
  for (auto point = points.rbegin() + 1; point != points.rend(); ++point) {
    while (hull.size() > lower_size &&
      Cross2d(hull[hull.size() - 2U], hull.back(), *point) <= 0.0)
    {
      hull.pop_back();
    }
    hull.push_back(*point);
  }
  hull.pop_back();
  return hull;
}

bool IsInsideConvexHull(const std::vector<PlaneCoordinate> & hull, const PlaneCoordinate & point)
{
  for (std::size_t index = 0U; index < hull.size(); ++index) {
    if (Cross2d(hull[index], hull[(index + 1U) % hull.size()], point) < -1e-9) {
      return false;
    }
  }
  return !hull.empty();
}

std::vector<task_lib::Vec3> BuildPlaneSamples(
  const std::vector<task_lib::Vec3> & points, const KeyframeEvidenceParameters & parameters)
{
  const auto fit = FitSparsePlaneRansac(
    points, parameters.plane_min_inliers, parameters.plane_residual_m);
  if (!fit.has_value()) {
    return {};
  }
  const task_lib::Vec3 axis = std::abs(fit->normal.z) < 0.9 ?
    task_lib::Vec3{0.0, 0.0, 1.0} : task_lib::Vec3{0.0, 1.0, 0.0};
  const auto tangent_u = Normalize(Cross(axis, fit->normal));
  if (!tangent_u.has_value()) {
    return {};
  }
  const auto tangent_v = Normalize(Cross(fit->normal, *tangent_u));
  if (!tangent_v.has_value()) {
    return {};
  }
  std::vector<PlaneCoordinate> projected;
  projected.reserve(fit->inliers.size());
  for (const auto & point : fit->inliers) {
    const auto offset = Subtract(point, fit->point);
    projected.push_back({Dot(offset, *tangent_u), Dot(offset, *tangent_v)});
  }
  const auto hull = ConvexHull2d(projected);
  if (hull.empty()) {
    return {};
  }
  double min_u = hull.front().u;
  double max_u = hull.front().u;
  double min_v = hull.front().v;
  double max_v = hull.front().v;
  for (const auto & point : hull) {
    min_u = std::min(min_u, point.u);
    max_u = std::max(max_u, point.u);
    min_v = std::min(min_v, point.v);
    max_v = std::max(max_v, point.v);
  }
  const double area = std::max(0.0, (max_u - min_u) * (max_v - min_v));
  const double step = std::max(
    parameters.voxel_size_m, std::sqrt(area / static_cast<double>(parameters.plane_max_samples)));
  std::vector<task_lib::Vec3> samples;
  samples.reserve(parameters.plane_max_samples);
  for (const auto & point : fit->inliers) {
    if (Norm(point) > 0.25) {
      samples.push_back(point);
    }
  }
  for (double u = min_u; u <= max_u + 1e-9 && samples.size() < parameters.plane_max_samples;
    u += step)
  {
    for (double v = min_v; v <= max_v + 1e-9 && samples.size() < parameters.plane_max_samples;
      v += step)
    {
      if (IsInsideConvexHull(hull, {u, v})) {
        const auto sample = Add(fit->point, Add(Scale(*tangent_u, u), Scale(*tangent_v, v)));
        if (Norm(sample) > 0.25) {
          samples.push_back(sample);
        }
      }
    }
  }
  return samples;
}

task_lib::Vec3 Rotate(
  const geometry_msgs::msg::Quaternion & orientation,
  const task_lib::Vec3 & value)
{
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (norm <= 1e-9) {
    return value;
  }
  const double x = orientation.x / norm;
  const double y = orientation.y / norm;
  const double z = orientation.z / norm;
  const double w = orientation.w / norm;
  const task_lib::Vec3 cross{y * value.z - z * value.y, z * value.x - x * value.z,
    x * value.y - y * value.x};
  const task_lib::Vec3 twice_cross{2.0 * cross.x, 2.0 * cross.y, 2.0 * cross.z};
  const task_lib::Vec3 second{y * twice_cross.z - z * twice_cross.y,
    z * twice_cross.x - x * twice_cross.z, x * twice_cross.y - y * twice_cross.x};
  return {value.x + w * twice_cross.x + second.x, value.y + w * twice_cross.y + second.y,
    value.z + w * twice_cross.z + second.z};
}

task_lib::Vec3 TransformPoint(
  const geometry_msgs::msg::Pose & transform,
  const task_lib::Vec3 & local)
{
  const auto rotated = Rotate(transform.orientation, local);
  return {transform.position.x + rotated.x, transform.position.y + rotated.y,
    transform.position.z + rotated.z};
}

task_lib::VoxelKey VoxelKeyFromWorld(const task_lib::Vec3 & position, double voxel_size)
{
  return {
    static_cast<std::int64_t>(std::floor(position.x / voxel_size)),
    static_cast<std::int64_t>(std::floor(position.y / voxel_size)),
    static_cast<std::int64_t>(std::floor(position.z / voxel_size))};
}

std::set<task_lib::VoxelKey> VoxelRaySupercover(
  const task_lib::Vec3 & start, const task_lib::Vec3 & end, double voxel_size)
{
  std::set<task_lib::VoxelKey> cells;
  if (voxel_size <= 0.0 || !IsFinite(start) || !IsFinite(end)) {
    return cells;
  }
  auto current = VoxelKeyFromWorld(start, voxel_size);
  const auto finish = VoxelKeyFromWorld(end, voxel_size);
  cells.insert(current);
  const double delta[3] = {end.x - start.x, end.y - start.y, end.z - start.z};
  const double origin[3] = {start.x, start.y, start.z};
  std::int64_t coordinate[3] = {current.ix, current.iy, current.iz};
  const std::int64_t target[3] = {finish.ix, finish.iy, finish.iz};
  int step[3] = {0, 0, 0};
  double t_max[3] = {
    std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity()};
  double t_delta[3] = {
    std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity()};
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(delta[axis]) <= 1e-12) {
      continue;
    }
    step[axis] = delta[axis] > 0.0 ? 1 : -1;
    const double boundary =
      static_cast<double>(coordinate[axis] + (step[axis] > 0 ? 1 : 0)) * voxel_size;
    t_max[axis] = (boundary - origin[axis]) / delta[axis];
    t_delta[axis] = voxel_size / std::abs(delta[axis]);
  }
  const std::size_t guard_limit = static_cast<std::size_t>(
    std::abs(target[0] - coordinate[0]) + std::abs(target[1] - coordinate[1]) +
    std::abs(target[2] - coordinate[2]) + 8) * 8U;
  for (std::size_t guard = 0U; guard < guard_limit && !(current == finish); ++guard) {
    const double next_t = std::min({t_max[0], t_max[1], t_max[2]});
    std::vector<int> crossed;
    for (int axis = 0; axis < 3; ++axis) {
      if (std::abs(t_max[axis] - next_t) <= 1e-10) {
        crossed.push_back(axis);
      }
    }
    const auto before = current;
    for (std::size_t mask = 1U; mask < (1U << crossed.size()); ++mask) {
      auto neighbor = before;
      for (std::size_t bit = 0U; bit < crossed.size(); ++bit) {
        if ((mask & (1U << bit)) == 0U) {
          continue;
        }
        if (crossed[bit] == 0) {
          neighbor.ix += step[0];
        } else if (crossed[bit] == 1) {
          neighbor.iy += step[1];
        } else {
          neighbor.iz += step[2];
        }
      }
      if (!(neighbor == finish)) {
        cells.insert(neighbor);
      }
    }
    for (const int axis : crossed) {
      coordinate[axis] += step[axis];
      t_max[axis] += t_delta[axis];
    }
    current = {coordinate[0], coordinate[1], coordinate[2]};
  }
  cells.erase(finish);
  return cells;
}

}  // namespace

bool KeyframeEvidenceIdentity::operator<(const KeyframeEvidenceIdentity & other) const
{
  if (drone_id != other.drone_id) {
    return drone_id < other.drone_id;
  }
  if (map_epoch != other.map_epoch) {
    return map_epoch < other.map_epoch;
  }
  return keyframe_id < other.keyframe_id;
}

bool EvidenceDatabase::ReplaceKeyframeSources(
  const KeyframeEvidenceIdentity & identity, std::uint64_t geometry_revision,
  std::uint64_t pose_revision, const geometry_msgs::msg::Pose & w_t_keyframe,
  const std::vector<KeyframeEvidenceSource> & sources)
{
  const auto tombstone = tombstones_.find(identity);
  if (tombstone != tombstones_.end() && geometry_revision <= tombstone->second.geometry_revision &&
    pose_revision <= tombstone->second.pose_revision)
  {
    return false;
  }
  std::map<std::string, KeyframeEvidenceSource> next_sources;
  const auto current = records_.find(identity);
  if (current != records_.end()) {
    for (const auto & source : current->second.sources) {
      if (IsDepthSource(source.first)) {
        next_sources.emplace(source.first, source.second);
      }
    }
  }
  for (const auto & source : sources) {
    if (!source.source_name.empty() && !source.local_endpoints.empty()) {
      next_sources[source.source_name] = source;
    }
  }
  if (current != records_.end() && geometry_revision < current->second.geometry_revision &&
    pose_revision <= current->second.pose_revision)
  {
    return false;
  }
  EvidenceTransaction transaction;
  transaction.identity = identity;
  transaction.geometry_revision = geometry_revision;
  transaction.pose_revision = pose_revision;
  transaction.w_t_keyframe = w_t_keyframe;
  if (current != records_.end()) {
    for (const auto & source : current->second.sources) {
      if (next_sources.count(source.first) == 0U) {
        transaction.deletes.push_back(source.first);
      }
    }
  }
  const bool pose_changed = current == records_.end() ||
    !SamePose(current->second.w_t_keyframe, w_t_keyframe) ||
    current->second.pose_revision != pose_revision;
  for (const auto & source : next_sources) {
    const bool source_changed = current == records_.end() ||
      current->second.sources.count(source.first) == 0U ||
      !SameSources(current->second.sources.at(source.first), source.second);
    if (source_changed || pose_changed ||
      (current != records_.end() && current->second.geometry_revision != geometry_revision))
    {
      transaction.upserts.push_back(source.second);
    }
  }
  if (transaction.upserts.empty() && transaction.deletes.empty()) {
    return false;
  }
  records_[identity] = KeyframeRecord{
    geometry_revision, pose_revision, true, w_t_keyframe, std::move(next_sources)};
  tombstones_.erase(identity);
  pending_transactions_.push_back(std::move(transaction));
  return true;
}

bool EvidenceDatabase::UpsertKeyframeSources(
  const KeyframeEvidenceIdentity & identity,
  const std::vector<KeyframeEvidenceSource> & sources)
{
  if (sources.empty() || tombstones_.count(identity) != 0U) {
    return false;
  }
  auto [record, inserted] = records_.try_emplace(identity);
  (void)inserted;
  EvidenceTransaction transaction;
  transaction.identity = identity;
  transaction.geometry_revision = record->second.geometry_revision;
  transaction.pose_revision = record->second.pose_revision;
  transaction.w_t_keyframe = record->second.w_t_keyframe;
  for (const auto & source : sources) {
    if (source.source_name.empty() || source.local_endpoints.empty()) {
      continue;
    }
    const auto existing = record->second.sources.find(source.source_name);
    if (existing != record->second.sources.end() &&
      existing->second.source_revision > source.source_revision)
    {
      continue;
    }
    if (existing != record->second.sources.end() && SameSources(existing->second, source)) {
      if (record->second.pose_available) {
        transaction.upserts.push_back(source);
      }
      continue;
    }
    record->second.sources[source.source_name] = source;
    if (record->second.pose_available) {
      transaction.upserts.push_back(source);
    }
  }
  if (transaction.upserts.empty()) {
    return !record->second.sources.empty();
  }
  pending_transactions_.push_back(std::move(transaction));
  return true;
}

bool EvidenceDatabase::UpdateKeyframePose(
  const KeyframeEvidenceIdentity & identity, std::uint64_t pose_revision,
  const geometry_msgs::msg::Pose & w_t_keyframe)
{
  const auto record = records_.find(identity);
  if (record == records_.end() || record->second.sources.empty() ||
    (record->second.pose_available && pose_revision < record->second.pose_revision))
  {
    return false;
  }
  if (record->second.pose_available && pose_revision == record->second.pose_revision &&
    SamePose(record->second.w_t_keyframe, w_t_keyframe))
  {
    return false;
  }
  record->second.pose_available = true;
  record->second.pose_revision = pose_revision;
  record->second.w_t_keyframe = w_t_keyframe;
  EvidenceTransaction transaction;
  transaction.identity = identity;
  transaction.geometry_revision = record->second.geometry_revision;
  transaction.pose_revision = pose_revision;
  transaction.w_t_keyframe = w_t_keyframe;
  for (const auto & source : record->second.sources) {
    transaction.upserts.push_back(source.second);
  }
  pending_transactions_.push_back(std::move(transaction));
  return true;
}

bool EvidenceDatabase::DeleteKeyframe(
  const KeyframeEvidenceIdentity & identity, std::uint64_t geometry_revision,
  std::uint64_t pose_revision)
{
  auto & tombstone = tombstones_[identity];
  if (geometry_revision < tombstone.geometry_revision || pose_revision < tombstone.pose_revision) {
    return false;
  }
  tombstone.geometry_revision = std::max(tombstone.geometry_revision, geometry_revision);
  tombstone.pose_revision = std::max(tombstone.pose_revision, pose_revision);
  const auto current = records_.find(identity);
  if (current == records_.end()) {
    return false;
  }
  EvidenceTransaction transaction;
  transaction.identity = identity;
  transaction.geometry_revision = geometry_revision;
  transaction.pose_revision = pose_revision;
  for (const auto & source : current->second.sources) {
    transaction.deletes.push_back(source.first);
  }
  records_.erase(current);
  pending_transactions_.push_back(std::move(transaction));
  return true;
}

std::vector<EvidenceTransaction> EvidenceDatabase::TakePendingTransactions()
{
  std::vector<EvidenceTransaction> transactions;
  transactions.swap(pending_transactions_);
  return transactions;
}

std::size_t EvidenceDatabase::SourceCount() const
{
  std::size_t count = 0U;
  for (const auto & record : records_) {
    count += record.second.sources.size();
  }
  return count;
}

bool EvidenceDatabase::HasSources(const KeyframeEvidenceIdentity & identity) const
{
  const auto record = records_.find(identity);
  return record != records_.end() && !record->second.sources.empty();
}

KeyframeEvidenceWorker::KeyframeEvidenceWorker(KeyframeEvidenceParameters parameters)
: parameters_(std::move(parameters))
{
}

KeyframeEvidenceProcessStats KeyframeEvidenceWorker::Consume(
  const mission_msgs::msg::KeyframeSparseEvidenceDelta & delta,
  EvidenceDatabase * database) const
{
  KeyframeEvidenceProcessStats stats;
  if (database == nullptr) {
    return stats;
  }
  for (const auto & removed : delta.deletes) {
    const KeyframeEvidenceIdentity identity{
      removed.drone_id, removed.map_epoch, removed.keyframe_id};
    if (database->DeleteKeyframe(identity, removed.geometry_revision, removed.pose_revision)) {
      ++stats.tombstones;
    }
  }
  for (const auto & incoming : delta.upserts) {
    const KeyframeEvidenceIdentity identity{
      incoming.drone_id, incoming.map_epoch, incoming.keyframe_id};
    std::vector<task_lib::Vec3> plane_points;
    std::vector<task_lib::Vec3> direct_endpoints;
    plane_points.reserve(incoming.points_k.size());
    direct_endpoints.reserve(incoming.points_k.size());
    for (const auto & point : incoming.points_k) {
      const task_lib::Vec3 local{point.position_k.x, point.position_k.y, point.position_k.z};
      if (!IsFinite(local)) {
        continue;
      }
      if (point.score >= parameters_.plane_min_score) {
        plane_points.push_back(local);
      }
      if (point.score >= parameters_.direct_ray_min_score) {
        direct_endpoints.push_back(local);
      }
    }
    std::vector<KeyframeEvidenceSource> sources;
    if (!direct_endpoints.empty()) {
      sources.push_back(
        {"sparse_ray_free", EvidenceSourceKind::FREE, incoming.geometry_revision, {},
          std::move(direct_endpoints)});
      ++stats.direct_sources;
    }
    const auto plane_samples = BuildPlaneSamples(plane_points, parameters_);
    if (!plane_samples.empty()) {
      sources.push_back(
        {"ransac_free", EvidenceSourceKind::FREE, incoming.geometry_revision, {}, plane_samples});
      ++stats.plane_sources;
    }
    if (database->ReplaceKeyframeSources(
        identity, incoming.geometry_revision, incoming.pose_revision,
        incoming.w_t_keyframe, sources))
    {
      ++stats.accepted_keyframes;
    }
  }
  return stats;
}

DepthIntegrationWorker::DepthIntegrationWorker(DepthIntegrationParameters parameters)
: parameters_(std::move(parameters))
{
}

DepthIntegrationProcessResult DepthIntegrationWorker::Consume(
  const DepthIntegrationJob & job, EvidenceDatabase * database) const
{
  DepthIntegrationProcessResult result;
  if (database == nullptr || job.command_id.empty() || job.observations.empty() ||
    !std::isfinite(parameters_.far_free_distance_m) || parameters_.far_free_distance_m <= 0.0 ||
    !std::isfinite(parameters_.direct_surface_max_incidence_deg) ||
    parameters_.direct_surface_max_incidence_deg <= 0.0 ||
    parameters_.direct_surface_max_incidence_deg > 90.0 ||
    !std::isfinite(parameters_.min_confidence) || parameters_.min_confidence < 0.0F ||
    parameters_.min_confidence > 1.0F)
  {
    result.reason = "invalid_depth_job";
    return result;
  }
  const double max_incidence_cos = std::cos(
    parameters_.direct_surface_max_incidence_deg * M_PI / 180.0);
  std::map<KeyframeEvidenceIdentity, std::vector<KeyframeEvidenceSource>> sources_by_keyframe;
  for (const auto & observation : job.observations) {
    const KeyframeEvidenceIdentity identity{
      observation.drone_id, observation.map_epoch, observation.local_keyframe_id};
    const task_lib::Vec3 camera_k{
      observation.k_t_camera.position.x, observation.k_t_camera.position.y,
      observation.k_t_camera.position.z};
    if (!observation.valid || identity.drone_id == 0U || identity.keyframe_id == 0U ||
      !IsFinite(camera_k) || !std::isfinite(observation.min_depth_m) ||
      !std::isfinite(observation.max_depth_m) || observation.min_depth_m <= 0.0F ||
      observation.max_depth_m < observation.min_depth_m ||
      observation.points_k.size() + observation.far_free_points_k.size() <
      parameters_.min_support_points || !std::isfinite(observation.confidence) ||
      observation.confidence < parameters_.min_confidence)
    {
      result.reason = "invalid_depth_observation";
      return result;
    }
    std::vector<task_lib::Vec3> free_endpoints;
    std::vector<task_lib::Vec3> direct_endpoints;
    std::vector<task_lib::Vec3> occupied_endpoints;
    const auto normal = Normalize(
      task_lib::Vec3{
        observation.normal_k.x, observation.normal_k.y, observation.normal_k.z});
    for (const auto & point : observation.points_k) {
      const task_lib::Vec3 endpoint_k{point.x, point.y, point.z};
      const auto delta = Subtract(endpoint_k, camera_k);
      const double distance = Norm(delta);
      if (!IsFinite(endpoint_k) || distance < observation.min_depth_m ||
        distance > parameters_.far_free_distance_m)
      {
        continue;
      }
      free_endpoints.push_back(endpoint_k);
      const auto direction = Normalize(delta);
      const bool direct_surface =
        (job.inspection_kind == DepthInspectionKind::VIEW_WALL ||
        job.inspection_kind == DepthInspectionKind::VIEW_ADVANCE) &&
        observation.normal_valid && normal.has_value() && direction.has_value() &&
        std::abs(Dot(*normal, *direction)) >= max_incidence_cos;
      if (direct_surface) {
        direct_endpoints.push_back(endpoint_k);
        occupied_endpoints.push_back(endpoint_k);
      }
    }
    for (const auto & point : observation.far_free_points_k) {
      const task_lib::Vec3 endpoint_k{point.x, point.y, point.z};
      const auto delta = Subtract(endpoint_k, camera_k);
      const double distance = Norm(delta);
      if (!IsFinite(endpoint_k) || distance <= parameters_.far_free_distance_m ||
        distance > observation.max_depth_m)
      {
        continue;
      }
      free_endpoints.push_back(
        Add(camera_k, Scale(delta, parameters_.far_free_distance_m / distance)));
    }
    if (free_endpoints.empty()) {
      result.reason = "no_usable_depth_endpoints";
      return result;
    }
    const std::string suffix = std::to_string(observation.tracking_frame_id);
    auto & sources = sources_by_keyframe[identity];
    sources.push_back(
      {"depth_free:" + suffix, EvidenceSourceKind::FREE, observation.source_revision,
        camera_k, std::move(free_endpoints)});
    if (!direct_endpoints.empty()) {
      sources.push_back(
        {"depth_direct_free:" + suffix, EvidenceSourceKind::DIRECT_FREE,
          observation.source_revision, camera_k, std::move(direct_endpoints)});
      sources.push_back(
        {"depth_occupied:" + suffix, EvidenceSourceKind::OCCUPIED,
          observation.source_revision, camera_k, std::move(occupied_endpoints)});
    }
  }
  for (const auto & entry : sources_by_keyframe) {
    if (!database->UpsertKeyframeSources(entry.first, entry.second)) {
      result.reason = "depth_source_rejected";
      return result;
    }
    for (const auto & source : entry.second) {
      result.sources.push_back({entry.first, source.source_name});
    }
  }
  result.accepted = true;
  result.reason = "depth_sources_written";
  return result;
}

VoxelMapBuilder::VoxelMapBuilder(double voxel_size_m)
: voxel_size_m_(voxel_size_m)
{
}

VoxelMaterializationStats VoxelMapBuilder::Apply(
  EvidenceDatabase * database, task_lib::ReversibleVoxelMap * voxel_map) const
{
  VoxelMaterializationStats stats;
  if (database == nullptr || voxel_map == nullptr) {
    return stats;
  }
  const auto transactions = database->TakePendingTransactions();
  stats.transactions = transactions.size();
  for (const auto & transaction : transactions) {
    for (const auto & source_name : transaction.deletes) {
      const auto source_id = SourceId(transaction.identity, source_name);
      stats.changed = voxel_map->RemoveEvidence(source_id) || stats.changed;
      stats.removed_source_ids.push_back(source_id);
      ++stats.deletes;
    }
    for (const auto & source : transaction.upserts) {
      const auto source_id = SourceId(transaction.identity, source.source_name);
      stats.applied_source_ids.push_back(source_id);
      if (source.kind == EvidenceSourceKind::OCCUPIED) {
        std::set<task_lib::VoxelKey> occupied_cells;
        for (const auto & endpoint_k : source.local_endpoints) {
          const auto endpoint_world = TransformPoint(transaction.w_t_keyframe, endpoint_k);
          occupied_cells.insert(VoxelKeyFromWorld(endpoint_world, voxel_size_m_));
        }
        if (occupied_cells.empty()) {
          stats.changed = voxel_map->RemoveEvidence(source_id) || stats.changed;
          stats.removed_source_ids.push_back(source_id);
        } else {
          stats.changed = voxel_map->ReplaceDepthOccupiedCells(source_id, occupied_cells) ||
            stats.changed;
          stats.occupied_cells_by_source.emplace(source_id, std::move(occupied_cells));
        }
        ++stats.upserts;
        continue;
      }
      std::set<task_lib::VoxelKey> free_cells;
      const auto camera_world = TransformPoint(transaction.w_t_keyframe, source.local_origin);
      for (const auto & endpoint_k : source.local_endpoints) {
        const auto endpoint_world = TransformPoint(transaction.w_t_keyframe, endpoint_k);
        const auto ray = VoxelRaySupercover(camera_world, endpoint_world, voxel_size_m_);
        free_cells.insert(ray.begin(), ray.end());
      }
      stats.free_cells += free_cells.size();
      if (free_cells.empty()) {
        stats.changed = voxel_map->RemoveEvidence(source_id) || stats.changed;
      } else if (source.kind == EvidenceSourceKind::DIRECT_FREE) {
        stats.changed = voxel_map->ReplaceDirectDepthFreeCells(source_id, free_cells) ||
          stats.changed;
      } else {
        stats.changed = voxel_map->ReplaceDepthFreeCells(source_id, free_cells) || stats.changed;
      }
      ++stats.upserts;
    }
  }
  return stats;
}

std::string VoxelMapBuilder::SourceId(
  const KeyframeEvidenceIdentity & identity, const std::string & source_name)
{
  return source_name + ":" + std::to_string(identity.drone_id) + ":" +
         std::to_string(identity.map_epoch) + ":" + std::to_string(identity.keyframe_id);
}

}  // namespace task_server
