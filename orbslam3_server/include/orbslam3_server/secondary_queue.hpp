#pragma once

#include "orbslam3_multi/fiducial_optimization_task.hpp"
#include "orbslam3_multi/loop_task.hpp"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <tuple>

namespace orbslam3_server
{

enum class SecondaryTaskPriority
{
  Max,
  High,
  Normal,
};

enum class SecondaryTaskKind
{
  FiducialOptimization,
  DatabaseUpdate,
  Loop,
};

struct SecondaryTask
{
  uint64_t task_id = 0;
  uint64_t enqueue_sequence = 0;
  SecondaryTaskPriority priority = SecondaryTaskPriority::Normal;
  SecondaryTaskKind kind = SecondaryTaskKind::Loop;
  std::optional<orbslam3_multi::FiducialOptimizationTask> fiducial;
  std::optional<orbslam3_multi::DatabaseUpdateTask> database_update;
  std::optional<orbslam3_multi::LoopTask> loop;
};

struct SecondaryEnqueueResult
{
  bool enqueued = false;
  bool duplicate = false;
  uint64_t enqueue_sequence = 0;
  size_t pending = 0;
};

struct SecondaryPendingStats
{
  size_t total = 0;
  size_t critical = 0;
  size_t maintenance = 0;
};

class SecondaryTaskQueue
{
public:
  SecondaryEnqueueResult PushFiducial(
    orbslam3_multi::FiducialOptimizationTask task)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      throw std::runtime_error("SecondaryTaskQueue cerrada");
    }
    const FiducialKey key{
      task.submap_id.drone_id, task.submap_id.map_epoch,
      task.keyframe_id.local_kf_id, task.fiducial_id};
    if (pending_fiducials_.count(key) != 0U || active_fiducials_.count(key) != 0U) {
      return {false, true, 0, PendingLocked()};
    }
    const uint64_t sequence = next_sequence_++;
    task.enqueue_sequence = sequence;
    SecondaryTask queued;
    queued.task_id = task.task_id;
    queued.enqueue_sequence = sequence;
    queued.priority = SecondaryTaskPriority::Max;
    queued.kind = SecondaryTaskKind::FiducialOptimization;
    queued.fiducial = std::move(task);
    max_queue_.push_back(std::move(queued));
    pending_fiducials_.insert(key);
    condition_.notify_one();
    return {true, false, sequence, PendingLocked()};
  }

  SecondaryEnqueueResult PushDatabaseUpdate(
    orbslam3_multi::DatabaseUpdateTask task)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      throw std::runtime_error("SecondaryTaskQueue cerrada");
    }
    const uint64_t sequence = next_sequence_++;
    task.enqueue_sequence = sequence;
    SecondaryTask queued;
    queued.task_id = task.task_id;
    queued.enqueue_sequence = sequence;
    queued.priority = SecondaryTaskPriority::High;
    queued.kind = SecondaryTaskKind::DatabaseUpdate;
    queued.database_update = std::move(task);
    high_queue_.push_back(std::move(queued));
    condition_.notify_one();
    return {true, false, sequence, PendingLocked()};
  }

  SecondaryEnqueueResult PushLoop(
    orbslam3_multi::LoopTask task,
    bool retry_completed_revision = false)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      throw std::runtime_error("SecondaryTaskQueue cerrada");
    }
    const LoopKey key = KeyFor(task);
    const auto completed = completed_loop_revisions_.find(task.query_keyframe_id);
    if (!retry_completed_revision &&
      completed != completed_loop_revisions_.end() && completed->second == key)
    {
      return {false, true, 0, PendingLocked()};
    }
    if (pending_loops_.count(key) != 0U || active_loops_.count(key) != 0U) {
      return {false, true, 0, PendingLocked()};
    }
    for (auto & queued : normal_queue_) {
      if (queued.loop.has_value() &&
        SameLoopIdentity(*queued.loop, task))
      {
        if (queued.loop->intent == orbslam3_multi::LoopTaskIntent::Full) {
          task.intent = orbslam3_multi::LoopTaskIntent::Full;
        }
        pending_loops_.erase(KeyFor(*queued.loop));
        task.enqueue_sequence = queued.enqueue_sequence;
        queued.task_id = task.task_id;
        queued.loop = std::move(task);
        pending_loops_.insert(KeyFor(*queued.loop));
        condition_.notify_one();
        return {true, false, queued.enqueue_sequence, PendingLocked()};
      }
    }
    const uint64_t sequence = next_sequence_++;
    task.enqueue_sequence = sequence;
    SecondaryTask queued;
    queued.task_id = task.task_id;
    queued.enqueue_sequence = sequence;
    queued.priority = SecondaryTaskPriority::Normal;
    queued.kind = SecondaryTaskKind::Loop;
    queued.loop = std::move(task);
    normal_queue_.push_back(std::move(queued));
    pending_loops_.insert(key);
    condition_.notify_one();
    return {true, false, sequence, PendingLocked()};
  }

  bool WaitPop(SecondaryTask * task)
  {
    if (task == nullptr) {
      throw std::invalid_argument("destino SecondaryTask nulo");
    }
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this]() {return closed_ || PendingLocked() > 0U;});
    if (PendingLocked() == 0U) {
      return false;
    }
    std::deque<SecondaryTask> * selected = nullptr;
    if (!max_queue_.empty()) {
      selected = &max_queue_;
    } else if (!high_queue_.empty()) {
      selected = &high_queue_;
    } else {
      selected = &normal_queue_;
    }
    *task = std::move(selected->front());
    selected->pop_front();
    if (task->fiducial.has_value()) {
      const auto key = KeyFor(*task->fiducial);
      pending_fiducials_.erase(key);
      active_fiducials_.insert(key);
    }
    if (task->loop.has_value()) {
      const auto key = KeyFor(*task->loop);
      pending_loops_.erase(key);
      active_loops_.insert(key);
    }
    return true;
  }

  void Complete(const SecondaryTask & task)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (task.fiducial.has_value()) {
      active_fiducials_.erase(KeyFor(*task.fiducial));
    }
    if (task.loop.has_value()) {
      const auto key = KeyFor(*task.loop);
      active_loops_.erase(key);
      completed_loop_revisions_[task.loop->query_keyframe_id] = key;
    }
    condition_.notify_all();
  }

  size_t Pending() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return PendingLocked();
  }

  SecondaryPendingStats PendingStats() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    SecondaryPendingStats stats;
    stats.total = PendingLocked();
    stats.critical = max_queue_.size() + high_queue_.size();
    for (const auto & task : normal_queue_) {
      if (task.loop.has_value() &&
        task.loop->intent == orbslam3_multi::LoopTaskIntent::FusionRefresh)
      {
        ++stats.maintenance;
      } else {
        ++stats.critical;
      }
    }
    return stats;
  }

  void Close()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
    condition_.notify_all();
  }

private:
  using FiducialKey = std::tuple<uint32_t, uint64_t, uint64_t, int32_t>;
  using LoopKey = std::tuple<uint32_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t>;

  static FiducialKey KeyFor(
    const orbslam3_multi::FiducialOptimizationTask & task)
  {
    return {
      task.submap_id.drone_id, task.submap_id.map_epoch,
      task.keyframe_id.local_kf_id, task.fiducial_id};
  }

  static LoopKey KeyFor(const orbslam3_multi::LoopTask & task)
  {
    return {
      task.query_keyframe_id.drone_id, task.query_keyframe_id.map_epoch,
      task.query_keyframe_id.local_kf_id, task.revision.appearance_revision,
      task.revision.geometry_revision, task.revision.anchor_revision};
  }

  static bool SameLoopIdentity(
    const orbslam3_multi::LoopTask & first,
    const orbslam3_multi::LoopTask & second)
  {
    return first.query_keyframe_id == second.query_keyframe_id;
  }

  std::deque<SecondaryTask> & QueueFor(SecondaryTaskPriority priority)
  {
    if (priority == SecondaryTaskPriority::Max) {
      return max_queue_;
    }
    return priority == SecondaryTaskPriority::High ? high_queue_ : normal_queue_;
  }

  size_t PendingLocked() const
  {
    return max_queue_.size() + high_queue_.size() + normal_queue_.size();
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<SecondaryTask> max_queue_;
  std::deque<SecondaryTask> high_queue_;
  std::deque<SecondaryTask> normal_queue_;
  std::set<FiducialKey> pending_fiducials_;
  std::set<FiducialKey> active_fiducials_;
  std::set<LoopKey> pending_loops_;
  std::set<LoopKey> active_loops_;
  std::map<orbslam3_multi::RawKeyFrameId, LoopKey> completed_loop_revisions_;
  uint64_t next_sequence_ = 1;
  bool closed_ = false;
};

inline const char * ToString(SecondaryTaskPriority priority)
{
  switch (priority) {
    case SecondaryTaskPriority::Max:
      return "MAX";
    case SecondaryTaskPriority::High:
      return "HIGH";
    case SecondaryTaskPriority::Normal:
      return "NORMAL";
  }
  return "UNKNOWN";
}

inline const char * ToString(SecondaryTaskKind kind)
{
  switch (kind) {
    case SecondaryTaskKind::FiducialOptimization:
      return "fiducial_optimization";
    case SecondaryTaskKind::DatabaseUpdate:
      return "database_update";
    case SecondaryTaskKind::Loop:
      return "loop";
  }
  return "unknown";
}

}  // namespace orbslam3_server
