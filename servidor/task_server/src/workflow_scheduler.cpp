#include "task_server/workflow_scheduler.hpp"

namespace task_server
{

bool WorkflowIdentity::IsValid() const
{
  return drone_id != 0U && !task_id.empty() && !workflow_id.empty() && !command_id.empty();
}

bool WorkflowScheduler::QueuedKey::operator<(const QueuedKey & other) const
{
  if (queue != other.queue) {
    return static_cast<std::uint8_t>(queue) < static_cast<std::uint8_t>(other.queue);
  }
  return workflow_id < other.workflow_id;
}

bool WorkflowScheduler::Enqueue(const WorkflowWorkItem & item)
{
  if (!item.identity.IsValid()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const QueuedKey key{item.queue, item.identity.workflow_id};
  if (!queued_keys_.insert(key).second) {
    return false;
  }
  queues_[item.queue].push_back(item);
  return true;
}

std::optional<WorkflowWorkItem> WorkflowScheduler::Dequeue(WorkflowQueue queue)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto queue_it = queues_.find(queue);
  if (queue_it == queues_.end() || queue_it->second.empty()) {
    return std::nullopt;
  }
  auto item = queue_it->second.front();
  queue_it->second.pop_front();
  queued_keys_.erase(QueuedKey{item.queue, item.identity.workflow_id});
  return item;
}

bool WorkflowScheduler::Requeue(const WorkflowWorkItem & item)
{
  return Enqueue(item);
}

std::size_t WorkflowScheduler::QueueSize(WorkflowQueue queue) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto queue_it = queues_.find(queue);
  return queue_it == queues_.end() ? 0U : queue_it->second.size();
}

bool WorkflowScheduler::HasAcceptedResult(const std::string & command_id) const
{
  if (command_id.empty()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return accepted_result_ids_.count(command_id) != 0U;
}

bool WorkflowScheduler::MarkResultAccepted(const std::string & command_id)
{
  if (command_id.empty()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return accepted_result_ids_.insert(command_id).second;
}

std::optional<std::uint64_t> WorkflowScheduler::AcceptNormalCommand(
  std::uint32_t drone_id, const std::string & command_id)
{
  if (drone_id == 0U || command_id.empty()) {
    return std::nullopt;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto active_it = active_normal_commands_.find(drone_id);
  if (active_it != active_normal_commands_.end()) {
    if (active_it->second.command_id == command_id) {
      return active_it->second.generation;
    }
    return std::nullopt;
  }
  const auto generation = ++command_generation_by_drone_[drone_id];
  active_normal_commands_.emplace(drone_id, ActiveCommand{command_id, generation});
  return generation;
}

bool WorkflowScheduler::IsCurrentNormalCommand(
  std::uint32_t drone_id, const std::string & command_id,
  std::uint64_t generation) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto active_it = active_normal_commands_.find(drone_id);
  return active_it != active_normal_commands_.end() &&
         active_it->second.command_id == command_id &&
         active_it->second.generation == generation;
}

std::uint64_t WorkflowScheduler::InvalidateNormalCommandForStop(std::uint32_t drone_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_normal_commands_.erase(drone_id);
  return ++command_generation_by_drone_[drone_id];
}

bool WorkflowScheduler::CompleteNormalCommand(
  std::uint32_t drone_id, const std::string & command_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto active_it = active_normal_commands_.find(drone_id);
  if (active_it == active_normal_commands_.end() || active_it->second.command_id != command_id) {
    return false;
  }
  active_normal_commands_.erase(active_it);
  return true;
}

}  // namespace task_server
