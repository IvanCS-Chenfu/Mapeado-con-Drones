#include "orbslam3_server/ground_truth_buffer.hpp"

#include <gtest/gtest.h>

TEST(GroundTruthBuffer, KeepsExactlyFiftyNewestSamplesPerDrone)
{
  orbslam3_server::GroundTruthBuffer buffer;
  for (int64_t i = 0; i < 60; ++i) {
    orbslam3_server::GroundTruthSample sample;
    sample.stamp_ns = i;
    buffer.Push(1, sample);
  }
  orbslam3_server::GroundTruthSample second_drone_sample;
  second_drone_sample.stamp_ns = 100;
  buffer.Push(2, second_drone_sample);

  const auto first = buffer.Snapshot(1);
  ASSERT_EQ(first.size(), 50U);
  EXPECT_EQ(first.front().stamp_ns, 10);
  EXPECT_EQ(first.back().stamp_ns, 59);
  EXPECT_EQ(buffer.Size(2), 1U);
}

TEST(GroundTruthBuffer, AssociatesNearestSampleWithinThreshold)
{
  std::vector<orbslam3_server::GroundTruthSample> samples(2);
  samples[0].stamp_ns = 1000000000LL;
  samples[0].fiducial_id = 2;
  samples[1].stamp_ns = 1800000000LL;
  samples[1].fiducial_id = 2;

  const auto match = orbslam3_server::GroundTruthBuffer::FindNearest(
    samples, 1700000000LL, 1.0);
  ASSERT_TRUE(match.matched);
  EXPECT_EQ(match.sample.stamp_ns, 1800000000LL);
  EXPECT_NEAR(match.dt_sec, 0.1, 1e-9);
}

TEST(GroundTruthBuffer, DiagnosesExpiredAndOutsideSamples)
{
  orbslam3_server::GroundTruthSample sample;
  sample.stamp_ns = 3000000000LL;
  const std::vector<orbslam3_server::GroundTruthSample> samples{sample};

  const auto expired = orbslam3_server::GroundTruthBuffer::FindNearest(
    samples, 1000000000LL, 1.0);
  EXPECT_FALSE(expired.matched);
  EXPECT_EQ(expired.reason, "gt_history_expired");

  const auto outside = orbslam3_server::GroundTruthBuffer::FindNearest(
    samples, 3000000000LL, 1.0);
  EXPECT_FALSE(outside.matched);
  EXPECT_EQ(outside.reason, "outside_fiducial_radius");
}
