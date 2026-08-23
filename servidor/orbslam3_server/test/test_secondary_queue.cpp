#include "orbslam3_server/secondary_queue.hpp"

#include <gtest/gtest.h>

namespace
{

orbslam3_multi::FiducialOptimizationTask MakeFiducialTask(
  uint64_t task_id, uint64_t keyframe_id)
{
  orbslam3_multi::FiducialOptimizationTask task;
  task.task_id = task_id;
  task.submap_id = {1, 2};
  task.keyframe_id = {1, 2, keyframe_id};
  task.fiducial_id = 2;
  return task;
}

orbslam3_multi::DatabaseUpdateTask MakeDatabaseTask(uint64_t task_id)
{
  orbslam3_multi::DatabaseUpdateTask task;
  task.task_id = task_id;
  task.source_arrival_id = task_id;
  task.submap_id = {1, 2};
  return task;
}

orbslam3_multi::LoopTask MakeLoopTask(
  uint64_t task_id, uint64_t keyframe_id, uint64_t revision = 1)
{
  orbslam3_multi::LoopTask task;
  task.task_id = task_id;
  task.query_keyframe_id = {1, 2, keyframe_id};
  task.revision = {revision, revision, revision, 0};
  return task;
}

TEST(SecondaryTaskQueue, MaxRunsBeforeHighAndNormalWithoutPreemption)
{
  orbslam3_server::SecondaryTaskQueue queue;
  queue.PushLoop(MakeLoopTask(1, 7));
  queue.PushDatabaseUpdate(MakeDatabaseTask(2));
  queue.PushFiducial(MakeFiducialTask(3, 9));

  orbslam3_server::SecondaryTask task;
  ASSERT_TRUE(queue.WaitPop(&task));
  EXPECT_EQ(task.task_id, 3U);
  queue.Complete(task);
  ASSERT_TRUE(queue.WaitPop(&task));
  EXPECT_EQ(task.task_id, 2U);
  queue.Complete(task);
  ASSERT_TRUE(queue.WaitPop(&task));
  EXPECT_EQ(task.task_id, 1U);
  queue.Complete(task);
  queue.Close();
}

TEST(SecondaryTaskQueue, DeduplicatesLoopOnlyForSameCausalRevision)
{
  orbslam3_server::SecondaryTaskQueue queue;
  EXPECT_TRUE(queue.PushLoop(MakeLoopTask(20, 9, 1)).enqueued);
  EXPECT_TRUE(queue.PushLoop(MakeLoopTask(21, 9, 1)).duplicate);
  EXPECT_TRUE(queue.PushLoop(MakeLoopTask(22, 9, 2)).enqueued);

  orbslam3_server::SecondaryTask task;
  ASSERT_TRUE(queue.WaitPop(&task));
  EXPECT_EQ(task.task_id, 22U);
  EXPECT_TRUE(queue.PushLoop(MakeLoopTask(23, 9, 2)).duplicate);
  EXPECT_TRUE(queue.PushLoop(MakeLoopTask(24, 9, 3)).enqueued);
  EXPECT_TRUE(queue.PushLoop(MakeLoopTask(25, 9, 4)).enqueued);
  queue.Complete(task);
  ASSERT_TRUE(queue.WaitPop(&task));
  EXPECT_EQ(task.task_id, 25U);
  queue.Complete(task);
  EXPECT_TRUE(queue.PushLoop(MakeLoopTask(26, 9, 4)).duplicate);
  EXPECT_TRUE(queue.PushLoop(MakeLoopTask(27, 9, 5)).enqueued);
  ASSERT_TRUE(queue.WaitPop(&task));
  EXPECT_EQ(task.task_id, 27U);
  queue.Complete(task);
  queue.Close();
}

TEST(SecondaryTaskQueue, ExplicitRetryBypassesOnlyCompletedRevisionLedger)
{
  orbslam3_server::SecondaryTaskQueue queue;
  EXPECT_TRUE(queue.PushLoop(MakeLoopTask(30, 9, 1)).enqueued);

  orbslam3_server::SecondaryTask task;
  ASSERT_TRUE(queue.WaitPop(&task));
  EXPECT_EQ(task.task_id, 30U);
  queue.Complete(task);

  EXPECT_TRUE(queue.PushLoop(MakeLoopTask(31, 9, 1)).duplicate);
  EXPECT_TRUE(queue.PushLoop(MakeLoopTask(32, 9, 1), true).enqueued);
  EXPECT_TRUE(queue.PushLoop(MakeLoopTask(33, 9, 1), true).duplicate);
  ASSERT_TRUE(queue.WaitPop(&task));
  EXPECT_EQ(task.task_id, 32U);
  queue.Complete(task);
  queue.Close();
}

TEST(SecondaryTaskQueue, FullIntentWinsWhenCoalescingFusionRefresh)
{
  for (const bool full_first : {false, true}) {
    orbslam3_server::SecondaryTaskQueue queue;
    auto first = MakeLoopTask(60, 9, 1);
    auto second = MakeLoopTask(61, 9, 2);
    first.intent = full_first ? orbslam3_multi::LoopTaskIntent::Full :
      orbslam3_multi::LoopTaskIntent::FusionRefresh;
    second.intent = full_first ? orbslam3_multi::LoopTaskIntent::FusionRefresh :
      orbslam3_multi::LoopTaskIntent::Full;
    ASSERT_TRUE(queue.PushLoop(first).enqueued);
    ASSERT_TRUE(queue.PushLoop(second).enqueued);

    orbslam3_server::SecondaryTask task;
    ASSERT_TRUE(queue.WaitPop(&task));
    ASSERT_TRUE(task.loop.has_value());
    EXPECT_EQ(task.loop->intent, orbslam3_multi::LoopTaskIntent::Full);
    queue.Complete(task);
    queue.Close();
  }
}

TEST(SecondaryTaskQueue, PreservesFifoAndDeduplicatesOnlyExactKeyFrame)
{
  orbslam3_server::SecondaryTaskQueue queue;
  EXPECT_TRUE(queue.PushFiducial(MakeFiducialTask(10, 20)).enqueued);
  EXPECT_TRUE(queue.PushFiducial(MakeFiducialTask(11, 21)).enqueued);
  EXPECT_TRUE(queue.PushFiducial(MakeFiducialTask(12, 20)).duplicate);

  orbslam3_server::SecondaryTask task;
  ASSERT_TRUE(queue.WaitPop(&task));
  EXPECT_EQ(task.task_id, 10U);
  EXPECT_TRUE(queue.PushFiducial(MakeFiducialTask(13, 20)).duplicate);
  queue.Complete(task);
  ASSERT_TRUE(queue.WaitPop(&task));
  EXPECT_EQ(task.task_id, 11U);
  queue.Complete(task);
  EXPECT_TRUE(queue.PushFiducial(MakeFiducialTask(14, 20)).enqueued);
  queue.Close();
}

TEST(SecondaryTaskQueue, FiducialCanBeRequeuedAfterStaleAttemptCompletes)
{
  orbslam3_server::SecondaryTaskQueue queue;
  orbslam3_multi::FiducialOptimizationTask task;
  task.task_id = 42;
  task.submap_id = {1, 2};
  task.keyframe_id = {1, 2, 9};
  task.fiducial_id = 7;

  ASSERT_TRUE(queue.PushFiducial(task).enqueued);
  orbslam3_server::SecondaryTask active;
  ASSERT_TRUE(queue.WaitPop(&active));
  EXPECT_TRUE(queue.PushFiducial(task).duplicate);
  queue.Complete(active);

  const auto retry = queue.PushFiducial(task);
  EXPECT_TRUE(retry.enqueued);
  EXPECT_FALSE(retry.duplicate);
  EXPECT_GT(retry.enqueue_sequence, active.enqueue_sequence);
  queue.Close();
}

TEST(SecondaryTaskQueue, SeparatesFusionRefreshMaintenanceFromCriticalPending)
{
  orbslam3_server::SecondaryTaskQueue queue;
  auto refresh_a = MakeLoopTask(70, 10);
  auto refresh_b = MakeLoopTask(71, 11);
  refresh_a.intent = orbslam3_multi::LoopTaskIntent::FusionRefresh;
  refresh_b.intent = orbslam3_multi::LoopTaskIntent::FusionRefresh;
  ASSERT_TRUE(queue.PushLoop(refresh_a).enqueued);
  ASSERT_TRUE(queue.PushLoop(refresh_b).enqueued);
  auto stats = queue.PendingStats();
  EXPECT_EQ(stats.total, 2U);
  EXPECT_EQ(stats.maintenance, 2U);
  EXPECT_EQ(stats.critical, 0U);

  ASSERT_TRUE(queue.PushLoop(MakeLoopTask(72, 12)).enqueued);
  ASSERT_TRUE(queue.PushDatabaseUpdate(MakeDatabaseTask(73)).enqueued);
  stats = queue.PendingStats();
  EXPECT_EQ(stats.total, 4U);
  EXPECT_EQ(stats.maintenance, 2U);
  EXPECT_EQ(stats.critical, 2U);
  queue.Close();
}

}  // namespace
