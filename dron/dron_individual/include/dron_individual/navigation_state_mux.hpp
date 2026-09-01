#ifndef DRON_INDIVIDUAL_NAVIGATION_STATE_MUX_HPP_
#define DRON_INDIVIDUAL_NAVIGATION_STATE_MUX_HPP_

#include <eigen3/Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <sstream>
#include <string>

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
  GT_FALLBACK,
  GT_FORCED
};

enum class Phase5NavigationSource
{
  GT,
  ORB
};

inline std::optional<Phase5NavigationSource> ParsePhase5NavigationSource(
  const std::string & value)
{
  if (value == "gt") {
    return Phase5NavigationSource::GT;
  }
  if (value == "orb") {
    return Phase5NavigationSource::ORB;
  }
  return std::nullopt;
}

inline const char * NavigationSourceName(NavigationSource source)
{
  switch (source) {
    case NavigationSource::INVALID:
      return "invalid";
    case NavigationSource::ORB:
      return "orb";
    case NavigationSource::GT_FALLBACK:
      return "gt_fallback";
    case NavigationSource::GT_FORCED:
      return "gt_forced";
  }
  return "unknown";
}

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

inline std::string ExactFailedPredicates(
  bool tracking_state_ok, bool local_valid, bool local_continuity_valid,
  bool velocity_valid, bool anchored, bool reference_valid,
  FallbackReason final_reason)
{
  std::ostringstream result;
  const auto append = [&result](const char * value) {
      if (result.tellp() > 0) {
        result << '|';
      }
      result << value;
    };
  if (!tracking_state_ok) {
    append("TRACKING_NOT_OK");
  }
  if (!local_valid) {
    append("LOCAL_INVALID");
  }
  if (!local_continuity_valid) {
    append("CONTINUITY_INVALID");
  }
  if (!velocity_valid) {
    append("VELOCITY_INVALID_NON_SOURCE_GATE");
  }
  if (!anchored) {
    append("EPOCH_UNANCHORED");
  }
  if (!reference_valid) {
    append("REFERENCE_INVALID_NON_SOURCE_GATE");
  }
  if (final_reason == FallbackReason::ORB_QUALIFYING) {
    append("ORB_QUALIFYING");
  } else if (final_reason == FallbackReason::TRAJECTORY_SOURCE_LOCKED) {
    append("TRAJECTORY_SOURCE_LOCKED");
  }
  return result.tellp() > 0 ? result.str() : "NONE";
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

class OrbShadowActivationGate
{
public:
  bool Update(
    bool prerequisites_ready, double linear_speed, double angular_speed,
    double now_sec, double settle_duration_sec, double max_linear_speed,
    double max_angular_speed)
  {
    const bool stationary = prerequisites_ready &&
      linear_speed <= max_linear_speed && angular_speed <= max_angular_speed;
    if (!stationary) {
      settling_ = false;
      ready_ = false;
      return false;
    }
    if (!settling_) {
      settling_ = true;
      settled_since_sec_ = now_sec;
    }
    ready_ = now_sec - settled_since_sec_ >= settle_duration_sec;
    return ready_;
  }

  void Reset()
  {
    settling_ = false;
    ready_ = false;
  }

  bool ready() const {return ready_;}
  double settled_since_sec() const {return settled_since_sec_;}

private:
  bool settling_{false};
  bool ready_{false};
  double settled_since_sec_{0.0};
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

enum class DiagnosticOrbControlMode
{
  NORMAL,
  POSITION_GT,
  VELOCITY_GT,
  POSITION_VELOCITY_GT,
  ORB_PV_GT_ANGULAR,
  GT_PV_ORB_ANGULAR
};

inline DiagnosticOrbControlMode ParseDiagnosticOrbControlMode(const std::string & value)
{
  if (value == "position_gt") {
    return DiagnosticOrbControlMode::POSITION_GT;
  }
  if (value == "velocity_gt") {
    return DiagnosticOrbControlMode::VELOCITY_GT;
  }
  if (value == "position_velocity_gt") {
    return DiagnosticOrbControlMode::POSITION_VELOCITY_GT;
  }
  if (value == "orb_pv_gt_angular" || value == "ORB_PV_GT_ANGULAR") {
    return DiagnosticOrbControlMode::ORB_PV_GT_ANGULAR;
  }
  if (value == "gt_pv_orb_angular" || value == "GT_PV_ORB_ANGULAR") {
    return DiagnosticOrbControlMode::GT_PV_ORB_ANGULAR;
  }
  return DiagnosticOrbControlMode::NORMAL;
}

inline bool UsesGtPosition(DiagnosticOrbControlMode mode)
{
  return mode == DiagnosticOrbControlMode::POSITION_GT ||
         mode == DiagnosticOrbControlMode::POSITION_VELOCITY_GT ||
         mode == DiagnosticOrbControlMode::GT_PV_ORB_ANGULAR;
}

inline bool UsesGtVelocity(DiagnosticOrbControlMode mode)
{
  return mode == DiagnosticOrbControlMode::VELOCITY_GT ||
         mode == DiagnosticOrbControlMode::POSITION_VELOCITY_GT ||
         mode == DiagnosticOrbControlMode::GT_PV_ORB_ANGULAR;
}

inline bool UsesGtOrientation(DiagnosticOrbControlMode mode)
{
  return mode == DiagnosticOrbControlMode::ORB_PV_GT_ANGULAR;
}

inline bool UsesGtAngularVelocity(DiagnosticOrbControlMode mode)
{
  return mode == DiagnosticOrbControlMode::ORB_PV_GT_ANGULAR;
}

inline bool IsDiagnosticChannelOverride(DiagnosticOrbControlMode mode)
{
  return mode != DiagnosticOrbControlMode::NORMAL;
}

struct TimedGtPose
{
  double stamp_sec{0.0};
  RigidPose pose;
};

struct TimedGtVelocity
{
  double stamp_sec{0.0};
  Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular{Eigen::Vector3d::Zero()};
};

struct SynchronizedGtState
{
  RigidPose pose;
  Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular{Eigen::Vector3d::Zero()};
  double effective_stamp_sec{0.0};
  double support_skew_sec{0.0};
  bool pose_interpolated{false};
  bool velocity_interpolated{false};
  bool causally_propagated{false};
};

class DiagnosticGtStateBuffer
{
public:
  void AddPose(double stamp_sec, const RigidPose & pose)
  {
    poses_.push_back({stamp_sec, pose});
    Trim(poses_);
  }

  void AddVelocity(
    double stamp_sec, const Eigen::Vector3d & linear, const Eigen::Vector3d & angular)
  {
    velocities_.push_back({stamp_sec, linear, angular});
    Trim(velocities_);
  }

  std::optional<SynchronizedGtState> Sample(double target_sec, double max_skew_sec) const
  {
    if (poses_.empty() || velocities_.empty() || max_skew_sec < 0.0) {
      return std::nullopt;
    }
    const auto pose_bracket = Bracket(poses_, target_sec);
    const auto velocity_bracket = Bracket(velocities_, target_sec);
    if (!pose_bracket.first || !velocity_bracket.first) {
      return std::nullopt;
    }

    SynchronizedGtState result;
    result.effective_stamp_sec = target_sec;
    result.pose = InterpolatePose(
      *pose_bracket.first, pose_bracket.second, target_sec, result.pose_interpolated);
    InterpolateVelocity(
      *velocity_bracket.first, velocity_bracket.second, target_sec,
      result.linear, result.angular, result.velocity_interpolated);

    const double pose_skew = SupportSkew(*pose_bracket.first, pose_bracket.second, target_sec);
    const double velocity_skew = SupportSkew(
      *velocity_bracket.first, velocity_bracket.second, target_sec);
    result.support_skew_sec = std::max(pose_skew, velocity_skew);
    if (result.support_skew_sec > max_skew_sec) {
      return std::nullopt;
    }

    if (!pose_bracket.second && target_sec > pose_bracket.first->stamp_sec) {
      const double dt = target_sec - pose_bracket.first->stamp_sec;
      result.pose.translation += result.linear * dt;
      const double angle = result.angular.norm() * dt;
      if (angle > 1e-12) {
        const Eigen::Quaterniond delta(
          Eigen::AngleAxisd(angle, result.angular.normalized()));
        result.pose.rotation = delta * result.pose.rotation;
        result.pose.rotation.normalize();
      }
      result.causally_propagated = true;
    }
    return result;
  }

private:
  template<typename T>
  static void Trim(std::deque<T> & samples)
  {
    while (samples.size() > 200) {
      samples.pop_front();
    }
  }

  template<typename T>
  static std::pair<const T *, const T *> Bracket(
    const std::deque<T> & samples, double target_sec)
  {
    const T * before = nullptr;
    const T * after = nullptr;
    for (const auto & sample : samples) {
      if (sample.stamp_sec <= target_sec) {
        before = &sample;
      } else {
        after = &sample;
        break;
      }
    }
    return {before, after};
  }

  static double SupportSkew(
    const TimedGtPose & before, const TimedGtPose * after, double target_sec)
  {
    return after ? std::max(target_sec - before.stamp_sec, after->stamp_sec - target_sec) :
           target_sec - before.stamp_sec;
  }

  static double SupportSkew(
    const TimedGtVelocity & before, const TimedGtVelocity * after, double target_sec)
  {
    return after ? std::max(target_sec - before.stamp_sec, after->stamp_sec - target_sec) :
           target_sec - before.stamp_sec;
  }

  static RigidPose InterpolatePose(
    const TimedGtPose & before, const TimedGtPose * after, double target_sec,
    bool & interpolated)
  {
    if (!after || after->stamp_sec <= before.stamp_sec) {
      interpolated = false;
      return before.pose;
    }
    const double alpha = std::clamp(
      (target_sec - before.stamp_sec) / (after->stamp_sec - before.stamp_sec), 0.0, 1.0);
    interpolated = true;
    RigidPose pose;
    pose.translation = (1.0 - alpha) * before.pose.translation + alpha * after->pose.translation;
    pose.rotation = before.pose.rotation.slerp(alpha, after->pose.rotation);
    pose.rotation.normalize();
    return pose;
  }

  static void InterpolateVelocity(
    const TimedGtVelocity & before, const TimedGtVelocity * after, double target_sec,
    Eigen::Vector3d & linear, Eigen::Vector3d & angular, bool & interpolated)
  {
    if (!after || after->stamp_sec <= before.stamp_sec) {
      linear = before.linear;
      angular = before.angular;
      interpolated = false;
      return;
    }
    const double alpha = std::clamp(
      (target_sec - before.stamp_sec) / (after->stamp_sec - before.stamp_sec), 0.0, 1.0);
    linear = (1.0 - alpha) * before.linear + alpha * after->linear;
    angular = (1.0 - alpha) * before.angular + alpha * after->angular;
    interpolated = true;
  }

  std::deque<TimedGtPose> poses_;
  std::deque<TimedGtVelocity> velocities_;
};

class DiagnosticGtControlAlignment
{
public:
  void Capture(const RigidPose & control_t_body, const RigidPose & gt_t_body)
  {
    control_t_gt_ = Compose(control_t_body, Inverse(gt_t_body));
    valid_ = true;
  }

  RigidPose TransformPose(const RigidPose & gt_t_body) const
  {
    return Compose(control_t_gt_, gt_t_body);
  }

  Eigen::Vector3d RotateVector(const Eigen::Vector3d & gt_vector) const
  {
    return control_t_gt_.rotation * gt_vector;
  }

  bool valid() const {return valid_;}

private:
  bool valid_{false};
  RigidPose control_t_gt_;
};

}  // namespace dron_individual

#endif  // DRON_INDIVIDUAL_NAVIGATION_STATE_MUX_HPP_
