#include "task_server/workflow_scheduler.hpp"

#include <gtest/gtest.h>

namespace
{

task_server::WorkflowWorkItem MakeItem(
  task_server::WorkflowQueue queue, std::uint32_t drone_id,
  const std::string & workflow_id, const std::string & command_id)
{
  task_server::WorkflowWorkItem item;
  item.queue = queue;
  item.identity.drone_id = drone_id;
  item.identity.task_id = "task_" + std::to_string(drone_id);
  item.identity.workflow_id = workflow_id;
  item.identity.command_id = command_id;
  item.identity.map_epoch = 1U;
  item.identity.map_revision = 2U;
  return item;
}

TEST(WorkflowScheduler, KeepsFifoOrderAndRequeuesAtBack)
{
  task_server::WorkflowScheduler scheduler;
  const auto first = MakeItem(
    task_server::WorkflowQueue::POINT_SELECTION, 1U, "workflow_1", "command_1");
  const auto second = MakeItem(
    task_server::WorkflowQueue::POINT_SELECTION, 2U, "workflow_2", "command_2");
  ASSERT_TRUE(scheduler.Enqueue(first));
  ASSERT_TRUE(scheduler.Enqueue(second));

  const auto dequeued_first = scheduler.Dequeue(task_server::WorkflowQueue::POINT_SELECTION);
  ASSERT_TRUE(dequeued_first.has_value());
  EXPECT_EQ(dequeued_first->identity.workflow_id, "workflow_1");
  ASSERT_TRUE(scheduler.Requeue(*dequeued_first));

  const auto dequeued_second = scheduler.Dequeue(task_server::WorkflowQueue::POINT_SELECTION);
  ASSERT_TRUE(dequeued_second.has_value());
  EXPECT_EQ(dequeued_second->identity.workflow_id, "workflow_2");
  const auto requeued = scheduler.Dequeue(task_server::WorkflowQueue::POINT_SELECTION);
  ASSERT_TRUE(requeued.has_value());
  EXPECT_EQ(requeued->identity.workflow_id, "workflow_1");
}

TEST(WorkflowScheduler, DeduplicatesQueuedWorkflowAndReportedResult)
{
  task_server::WorkflowScheduler scheduler;
  const auto item = MakeItem(
    task_server::WorkflowQueue::DEPTH_INTEGRATION, 1U, "workflow_1", "command_1");
  EXPECT_TRUE(scheduler.Enqueue(item));
  EXPECT_FALSE(scheduler.Enqueue(item));
  EXPECT_FALSE(scheduler.HasAcceptedResult("command_1"));
  EXPECT_TRUE(scheduler.MarkResultAccepted("command_1"));
  EXPECT_TRUE(scheduler.HasAcceptedResult("command_1"));
  EXPECT_FALSE(scheduler.MarkResultAccepted("command_1"));
}

TEST(WorkflowScheduler, StopInvalidatesOnlyTheCurrentNormalCommand)
{
  task_server::WorkflowScheduler scheduler;
  const auto generation = scheduler.AcceptNormalCommand(1U, "command_1");
  ASSERT_TRUE(generation.has_value());
  EXPECT_TRUE(scheduler.IsCurrentNormalCommand(1U, "command_1", *generation));
  EXPECT_FALSE(scheduler.AcceptNormalCommand(1U, "command_2").has_value());

  const auto stop_generation = scheduler.InvalidateNormalCommandForStop(1U);
  EXPECT_FALSE(scheduler.IsCurrentNormalCommand(1U, "command_1", *generation));
  const auto next_generation = scheduler.AcceptNormalCommand(1U, "command_2");
  ASSERT_TRUE(next_generation.has_value());
  EXPECT_GT(*next_generation, stop_generation);
  EXPECT_FALSE(scheduler.CompleteNormalCommand(1U, "command_1"));
  EXPECT_TRUE(scheduler.CompleteNormalCommand(1U, "command_2"));
}

}  // namespace
