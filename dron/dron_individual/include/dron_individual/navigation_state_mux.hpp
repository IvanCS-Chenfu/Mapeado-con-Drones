#ifndef DRON_INDIVIDUAL_NAVIGATION_STATE_MUX_HPP_
#define DRON_INDIVIDUAL_NAVIGATION_STATE_MUX_HPP_

#include <eigen3/Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace dron_individual
{

struct RigidPose
{
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
};

inline RigidPose Compose(const RigidPose & lhs, const RigidPose & rhs)
{
  return {lhs.rotation * rhs.rotation, lhs.translation + lhs.rotation * rhs.translation};
}

inline RigidPose Inverse(const RigidPose & pose)
{
  const Eigen::Quaterniond inverse_rotation = pose.rotation.conjugate();
  return {inverse_rotation, -(inverse_rotation * pose.translation)};
}

enum class NavigationSource
{
  INVALID,
  ORB,
  GT_FALLBACK
};

enum class FallbackReason
{
  NONE,
  STARTUP_UNANCHORED_ABSOLUTE,
  TRACKING_LOST,
  NEW_EPOCH_UNANCHORED,
  ORB_QUALIFYING,
  TRAJECTORY_SOURCE_LOCKED
};

struct SourceDecision
{
  NavigationSource source{NavigationSource::INVALID};
  FallbackReason fallback_reason{FallbackReason::NONE};
};

inline SourceDecision DecideNavigationSource(
  bool tracking_ok, bool epoch_anchored, bool previously_anchored)
{
  if (tracking_ok && epoch_anchored) {
    return {NavigationSource::ORB, FallbackReason::NONE};
  }
  if (!epoch_anchored) {
    return {
      NavigationSource::GT_FALLBACK,
      previously_anchored ? FallbackReason::NEW_EPOCH_UNANCHORED :
      FallbackReason::STARTUP_UNANCHORED_ABSOLUTE};
  }
  return {NavigationSource::GT_FALLBACK, FallbackReason::TRACKING_LOST};
}

inline const char * FallbackReasonName(FallbackReason reason)
{
  switch (reason) {
    case FallbackReason::NONE:
      return "none";
    case FallbackReason::STARTUP_UNANCHORED_ABSOLUTE:
      return "startup_unanchored_absolute";
    case FallbackReason::TRACKING_LOST:
      return "tracking_lost";
    case FallbackReason::NEW_EPOCH_UNANCHORED:
      return "new_epoch_unanchored";
    case FallbackReason::ORB_QUALIFYING:
      return "orb_qualifying";
    case FallbackReason::TRAJECTORY_SOURCE_LOCKED:
      return "trajectory_source_locked";
  }
  return "unknown";
}

inline double RotationDistance(
  const Eigen::Quaterniond & lhs, const Eigen::Quaterniond & rhs)
{
  Eigen::Quaterniond delta = lhs * rhs.conjugate();
  if (delta.w() < 0.0) {
    delta.coeffs() *= -1.0;
  }
  delta.normalize();
  return Eigen::AngleAxisd(delta).angle();
}

struct OrbQualificationResult
{
  bool qualified{false};
  bool newly_qualified{false};
  std::size_t consecutive_samples{0};
};

class OrbTransitionQualifier
{
public:
  OrbQualificationResult Update(
    uint64_t epoch, std::size_t required_samples)
  {
    if (!active_ || epoch != epoch_) {
      active_ = true;
      epoch_ = epoch;
      consecutive_samples_ = 0;
    }

    const bool was_qualified = consecutive_samples_ >= required_samples;
    if (!was_qualified) {
      ++consecutive_samples_;
    }
    const bool qualified = consecutive_samples_ >= required_samples;
    return {qualified, qualified && !was_qualified, consecutive_samples_};
  }

  void Reset()
  {
    active_ = false;
    consecutive_samples_ = 0;
  }

private:
  bool active_{false};
  uint64_t epoch_{0};
  std::size_t consecutive_samples_{0};
};

class GoalSourceLock
{
public:
  // TODO FASE 6: retirar este lock junto con GT_FALLBACK cuando exista recovery real.
  void Begin(NavigationSource current_source)
  {
    active_ = true;
    locked_source_ = current_source == NavigationSource::ORB ?
      NavigationSource::ORB : NavigationSource::GT_FALLBACK;
  }

  void End() {active_ = false;}

  SourceDecision Apply(SourceDecision decision)
  {
    if (!active_) {
      return decision;
    }
    if (locked_source_ == NavigationSource::GT_FALLBACK) {
      if (decision.source == NavigationSource::ORB) {
        return {NavigationSource::GT_FALLBACK, FallbackReason::TRAJECTORY_SOURCE_LOCKED};
      }
      return decision;
    }
    if (decision.source != NavigationSource::ORB) {
      locked_source_ = NavigationSource::GT_FALLBACK;
    }
    return decision;
  }

  bool active() const {return active_;}
  NavigationSource locked_source() const {return locked_source_;}

private:
  bool active_{false};
  NavigationSource locked_source_{NavigationSource::GT_FALLBACK};
};

class EpochAnchorLatch
{
public:
  bool Update(uint64_t epoch, bool authoritative)
  {
    if (!initialized_ || epoch != epoch_) {
      if (initialized_ && anchored_) {
        previously_anchored_ = true;
      }
      epoch_ = epoch;
      anchored_ = false;
      initialized_ = true;
    }
    anchored_ = anchored_ || authoritative;
    return anchored_;
  }

  bool previously_anchored() const {return previously_anchored_;}

private:
  bool initialized_{false};
  bool anchored_{false};
  bool previously_anchored_{false};
  uint64_t epoch_{0};
};

class ContinuousSourcePose
{
public:
  RigidPose Update(NavigationSource source, const RigidPose & source_t_body)
  {
    if (!valid_) {
      control_t_source_ = RigidPose{};
    } else if (source != source_) {
      control_t_source_ = Compose(control_t_body_, Inverse(source_t_body));
    }
    source_ = source;
    control_t_body_ = Compose(control_t_source_, source_t_body);
    control_t_body_.rotation.normalize();
    valid_ = true;
    return control_t_body_;
  }

  Eigen::Vector3d RotateVectorFromSource(const Eigen::Vector3d & vector) const
  {
    return control_t_source_.rotation * vector;
  }

private:
  bool valid_{false};
  NavigationSource source_{NavigationSource::INVALID};
  RigidPose control_t_source_;
  RigidPose control_t_body_;
};

}  // namespace dron_individual

#endif  // DRON_INDIVIDUAL_NAVIGATION_STATE_MUX_HPP_
