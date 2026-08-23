#include "orbslam3_server/primary_queue.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <thread>
#include <vector>

namespace
{

using orbslam3_msgs::msg::OrbMap;
using orbslam3_server::BackpressureHysteresis;
using orbslam3_server::PrimaryInput;
using orbslam3_server::PrimaryInputKind;
using orbslam3_server::PrimaryQueue;

TEST(PrimaryQueue, ConcurrentProducersRemainFifoByArrivalId)
{
  PrimaryQueue queue;
  auto producer = [&queue](uint32_t drone_id) {
      for (size_t i = 0; i < 50; ++i) {
        auto map = std::make_shared<OrbMap>();
        map->drone_id = drone_id;
        queue.PushLive(map);
      }
    };

  std::thread first(producer, 1);
  std::thread second(producer, 2);
  first.join();
  second.join();
  EXPECT_EQ(queue.Pending(), 100U);
  queue.Close();

  std::vector<uint64_t> arrival_ids;
  PrimaryInput input;
  while (queue.WaitPop(&input)) {
    arrival_ids.push_back(input.arrival_id);
  }
  ASSERT_EQ(arrival_ids.size(), 100U);
  for (size_t i = 0; i < arrival_ids.size(); ++i) {
    EXPECT_EQ(arrival_ids[i], i + 1);
  }
}

TEST(PrimaryQueue, ReplayPreservesOriginalArrivalIdsAndDrainsOnClose)
{
  PrimaryQueue queue;
  auto map = std::make_shared<OrbMap>();
  map->drone_id = 1;
  queue.PushReplay(4, map);
  queue.PushReplay(9, map);
  queue.Close();

  PrimaryInput input;
  ASSERT_TRUE(queue.WaitPop(&input));
  EXPECT_EQ(input.arrival_id, 4U);
  ASSERT_TRUE(queue.WaitPop(&input));
  EXPECT_EQ(input.arrival_id, 9U);
  EXPECT_FALSE(queue.WaitPop(&input));
}

TEST(PrimaryQueue, WorkerCannotPopBeforeEnqueueTelemetryIsReady)
{
  PrimaryQueue queue;
  auto map = std::make_shared<OrbMap>();
  map->drone_id = 1;
  const auto enqueued = queue.PushLive(map);

  auto waiting_pop = std::async(
    std::launch::async, [&queue]() {
      PrimaryInput input;
      queue.WaitPop(&input);
      return input.arrival_id;
    });

  EXPECT_EQ(waiting_pop.wait_for(std::chrono::milliseconds(30)), std::future_status::timeout);
  queue.MarkReady(enqueued.arrival_id);
  EXPECT_EQ(waiting_pop.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(waiting_pop.get(), enqueued.arrival_id);
  queue.Close();
}

TEST(PrimaryQueue, LiveDeltaAndSnapshotShareOneArrivalOrder)
{
  PrimaryQueue queue;
  auto map = std::make_shared<OrbMap>();
  map->drone_id = 1;
  const auto delta = queue.PushLive(map, PrimaryInputKind::Delta);
  const auto snapshot = queue.PushLive(map, PrimaryInputKind::FullSnapshot);
  queue.MarkReady(delta.arrival_id);
  queue.MarkReady(snapshot.arrival_id);
  queue.Close();

  PrimaryInput input;
  ASSERT_TRUE(queue.WaitPop(&input));
  EXPECT_EQ(input.arrival_id, delta.arrival_id);
  EXPECT_EQ(input.kind, PrimaryInputKind::Delta);
  ASSERT_TRUE(queue.WaitPop(&input));
  EXPECT_EQ(input.arrival_id, snapshot.arrival_id);
  EXPECT_EQ(input.kind, PrimaryInputKind::FullSnapshot);
}

TEST(PrimaryQueue, CapacityWaitBlocksUntilWorkerPops)
{
  PrimaryQueue queue;
  auto map = std::make_shared<OrbMap>();
  const auto first = queue.PushReplay(1, map);
  const auto second = queue.PushReplay(2, map);
  queue.MarkReady(first.arrival_id);
  queue.MarkReady(second.arrival_id);

  auto capacity = std::async(
    std::launch::async, [&queue]() {
      return queue.WaitUntilPendingBelow(2);
    });
  EXPECT_EQ(capacity.wait_for(std::chrono::milliseconds(30)), std::future_status::timeout);
  PrimaryInput input;
  ASSERT_TRUE(queue.WaitPop(&input));
  EXPECT_EQ(capacity.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_TRUE(capacity.get());
  queue.Close();
}

TEST(BackpressureHysteresis, ActivatesAndReleasesOnlyAtConfiguredThresholds)
{
  BackpressureHysteresis hysteresis(8, 2);
  EXPECT_FALSE(hysteresis.Update(7).has_value());
  ASSERT_EQ(hysteresis.Update(8), std::optional<bool>(true));
  EXPECT_TRUE(hysteresis.Active());
  EXPECT_FALSE(hysteresis.Update(7).has_value());
  EXPECT_FALSE(hysteresis.Update(3).has_value());
  ASSERT_EQ(hysteresis.Update(2), std::optional<bool>(false));
  EXPECT_FALSE(hysteresis.Active());
}

}  // namespace
