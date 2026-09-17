#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>

namespace task_server
{

enum class WorkflowQueue : std::uint8_t
{
  TASK_ASSIGNMENT = 0U,
  POINT_SELECTION = 1U,
  TRAJECTORY_PLANNING = 2U,
  DEPTH_INTEGRATION = 3U,
  ACTIVE_TRAJECTORY_MONITOR = 4U,
};

struct WorkflowIdentity
{
  std::uint32_t drone_id = 0U;
  std::string task_id;
  std::string workflow_id;
  std::string command_id;
  std::uint64_t map_epoch = 0U;
  std::uint64_t map_revision = 0U;

  bool IsValid() const;
};

struct WorkflowWorkItem
{
  WorkflowQueue queue = WorkflowQueue::TASK_ASSIGNMENT;
  WorkflowIdentity identity;
};

class WorkflowScheduler
{
public:
  bool Enqueue(const WorkflowWorkItem & item);
  std::optional<WorkflowWorkItem> Dequeue(WorkflowQueue queue);
  bool Requeue(const WorkflowWorkItem & item);

  std::size_t QueueSize(WorkflowQueue queue) const;
  bool HasAcceptedResult(const std::string & command_id) const;
  bool MarkResultAccepted(const std::string & command_id);

  std::optional<std::uint64_t> AcceptNormalCommand(
    std::uint32_t drone_id, const std::string & command_id);
  bool IsCurrentNormalCommand(
    std::uint32_t drone_id, const std::string & command_id,
    std::uint64_t generation) const;
  std::uint64_t InvalidateNormalCommandForStop(std::uint32_t drone_id);
  bool CompleteNormalCommand(std::uint32_t drone_id, const std::string & command_id);

private:
  struct QueuedKey
  {
    WorkflowQueue queue = WorkflowQueue::TASK_ASSIGNMENT;
    std::string workflow_id;

    bool operator<(const QueuedKey & other) const;
  };

  struct ActiveCommand
  {
    std::string command_id;
    std::uint64_t generation = 0U;
  };

  mutable std::mutex mutex_;
  std::map<WorkflowQueue, std::deque<WorkflowWorkItem>> queues_;
  std::set<QueuedKey> queued_keys_;
  std::set<std::string> accepted_result_ids_;
  std::map<std::uint32_t, ActiveCommand> active_normal_commands_;
  std::map<std::uint32_t, std::uint64_t> command_generation_by_drone_;
};

}  // namespace task_server
