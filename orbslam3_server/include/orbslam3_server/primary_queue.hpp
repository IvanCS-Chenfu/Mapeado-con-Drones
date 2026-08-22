#pragma once

#include "orbslam3_msgs/msg/orb_map.hpp"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <algorithm>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

namespace orbslam3_server
{

enum class PrimaryInputSource
{
  Live,
  Replay,
};

enum class PrimaryInputKind
{
  Delta,
  FullSnapshot,
};

inline const char * ToString(PrimaryInputSource source)
{
  return source == PrimaryInputSource::Live ? "live" : "replay";
}

inline const char * ToString(PrimaryInputKind kind)
{
  return kind == PrimaryInputKind::Delta ? "delta" : "full_snapshot";
}

struct PrimaryInput
{
  uint64_t arrival_id = 0;
  PrimaryInputSource source = PrimaryInputSource::Live;
  PrimaryInputKind kind = PrimaryInputKind::Delta;
  std::shared_ptr<const orbslam3_msgs::msg::OrbMap> map;
};

struct PrimaryEnqueueResult
{
  uint64_t arrival_id = 0;
  size_t pending = 0;
};

/// FIFO causal de entradas ORB. Un elemento reservado no puede adelantarse hasta MarkReady.
/// Live asigna arrival_id; replay conserva IDs estrictamente crecientes del record.
class PrimaryQueue
{
public:
  PrimaryEnqueueResult PushLive(
    std::shared_ptr<const orbslam3_msgs::msg::OrbMap> map,
    PrimaryInputKind kind = PrimaryInputKind::Delta)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EnsureAccepting(map);
    const uint64_t arrival_id = next_arrival_id_++;
    queue_.push_back(
      {{arrival_id, PrimaryInputSource::Live, kind, std::move(map)}, false});
    return {arrival_id, queue_.size()};
  }

  PrimaryEnqueueResult PushReplay(
    uint64_t arrival_id,
    std::shared_ptr<const orbslam3_msgs::msg::OrbMap> map)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EnsureAccepting(map);
    if (arrival_id < next_arrival_id_) {
      throw std::invalid_argument("arrival_id replay no es estrictamente creciente");
    }
    next_arrival_id_ = arrival_id + 1;
    queue_.push_back(
      {{arrival_id, PrimaryInputSource::Replay, PrimaryInputKind::Delta, std::move(map)}, false});
    return {arrival_id, queue_.size()};
  }

  void MarkReady(uint64_t arrival_id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto queued = std::find_if(
      queue_.begin(), queue_.end(),
      [arrival_id](const QueuedInput & item) {
        return item.input.arrival_id == arrival_id;
      });
    if (queued == queue_.end()) {
      throw std::invalid_argument("arrival_id no pendiente en PrimaryQueue");
    }
    queued->ready = true;
    condition_.notify_all();
  }

  bool WaitPop(PrimaryInput * input)
  {
    if (input == nullptr) {
      throw std::invalid_argument("destino PrimaryInput nulo");
    }
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(
      lock, [this]() {return closed_ || (!queue_.empty() && queue_.front().ready);});
    if (queue_.empty()) {
      return false;
    }
    *input = std::move(queue_.front().input);
    queue_.pop_front();
    condition_.notify_all();
    return true;
  }

  bool WaitUntilPendingBelow(size_t limit)
  {
    if (limit == 0) {
      throw std::invalid_argument("limite de PrimaryQueue debe ser positivo");
    }
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this, limit]() {return closed_ || queue_.size() < limit;});
    return !closed_;
  }

  size_t Pending() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  void Close()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
    for (auto & queued : queue_) {
      queued.ready = true;
    }
    condition_.notify_all();
  }

private:
  struct QueuedInput
  {
    PrimaryInput input;
    bool ready = false;
  };

  void EnsureAccepting(const std::shared_ptr<const orbslam3_msgs::msg::OrbMap> & map) const
  {
    if (closed_) {
      throw std::runtime_error("PrimaryQueue cerrada");
    }
    if (!map) {
      throw std::invalid_argument("OrbMap nulo");
    }
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<QueuedInput> queue_;
  uint64_t next_arrival_id_ = 1;
  bool closed_ = false;
};

/// Histeresis high/low que evita oscilar la señal de presión con cada dequeue.
class BackpressureHysteresis
{
public:
  BackpressureHysteresis(size_t high_watermark, size_t low_watermark)
  : high_watermark_(high_watermark), low_watermark_(low_watermark)
  {
    if (high_watermark_ == 0 || low_watermark_ >= high_watermark_) {
      throw std::invalid_argument("watermarks invalidos");
    }
  }

  std::optional<bool> Update(size_t pending)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ && pending >= high_watermark_) {
      active_ = true;
      return true;
    }
    if (active_ && pending <= low_watermark_) {
      active_ = false;
      return false;
    }
    return std::nullopt;
  }

  bool Active() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_;
  }

private:
  const size_t high_watermark_;
  const size_t low_watermark_;
  mutable std::mutex mutex_;
  bool active_ = false;
};

}  // namespace orbslam3_server
