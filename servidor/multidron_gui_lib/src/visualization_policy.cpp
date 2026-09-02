#include "multidron_gui_lib/visualization_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace multidron_gui_lib
{

QColor ScoreColor(float score)
{
  const float normalized = std::clamp(score, 0.0F, 1.0F);
  const float red = normalized <= 0.5F ? 1.0F : 2.0F * (1.0F - normalized);
  const float green = normalized <= 0.5F ? 2.0F * normalized : 1.0F;
  return QColor::fromRgbF(red, green, 0.0F, 1.0F);
}

bool SparsePointVisible(float score, bool filter_enabled, float threshold)
{
  return !filter_enabled || score >= std::clamp(threshold, 0.0F, 1.0F);
}

std::optional<SelectedEntity> SelectBestCandidate(
  const std::vector<PickCandidate> & candidates,
  const QPointF & click,
  double tolerance_px)
{
  double best_distance = std::max(0.0, tolerance_px);
  float best_depth = std::numeric_limits<float>::infinity();
  const PickCandidate * best = nullptr;
  for (const auto & candidate : candidates) {
    const double dx = candidate.screen_position.x() - click.x();
    const double dy = candidate.screen_position.y() - click.y();
    const double distance = std::sqrt(dx * dx + dy * dy);
    if (distance < best_distance ||
      (std::abs(distance - best_distance) < 0.5 && candidate.depth < best_depth))
    {
      best_distance = distance;
      best_depth = candidate.depth;
      best = &candidate;
    }
  }
  return best == nullptr ? std::nullopt : std::optional<SelectedEntity>(best->entity);
}

}  // namespace multidron_gui_lib
