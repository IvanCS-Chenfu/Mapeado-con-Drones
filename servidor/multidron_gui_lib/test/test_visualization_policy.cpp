#include "multidron_gui_lib/visualization_policy.hpp"

#include <gtest/gtest.h>

#include <chrono>

TEST(VisualizationPolicy, DerivesStableScoreGradientAndFilter)
{
  EXPECT_EQ(multidron_gui_lib::ScoreColor(0.0F), QColor::fromRgbF(1.0F, 0.0F, 0.0F));
  EXPECT_EQ(multidron_gui_lib::ScoreColor(0.5F), QColor::fromRgbF(1.0F, 1.0F, 0.0F));
  EXPECT_EQ(multidron_gui_lib::ScoreColor(1.0F), QColor::fromRgbF(0.0F, 1.0F, 0.0F));
  EXPECT_FALSE(multidron_gui_lib::SparsePointVisible(0.49F, true, 0.5F));
  EXPECT_TRUE(multidron_gui_lib::SparsePointVisible(0.5F, true, 0.5F));
  EXPECT_TRUE(multidron_gui_lib::SparsePointVisible(0.0F, false, 1.0F));
}

TEST(VisualizationPolicy, SelectsFromLargeSyntheticSceneWithinInteractiveBudget)
{
  using multidron_gui_lib::EntityType;
  using multidron_gui_lib::PickCandidate;
  std::vector<PickCandidate> candidates;
  candidates.reserve(100000U);
  for (std::uint64_t index = 0; index < 100000U; ++index) {
    PickCandidate candidate;
    candidate.entity.key = {EntityType::MapPoint, 1U, 1U, index};
    candidate.screen_position = QPointF(
      static_cast<double>(index % 1000U), static_cast<double>(index / 1000U));
    candidate.depth = 0.5F;
    candidates.push_back(candidate);
  }
  const auto start = std::chrono::steady_clock::now();
  const auto selected = multidron_gui_lib::SelectBestCandidate(
    candidates, QPointF(500.0, 50.0), 12.0);
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start);
  ASSERT_TRUE(selected.has_value());
  EXPECT_LT(elapsed.count(), 500);
}

TEST(VisualizationPolicy, PicksByScreenDistanceThenDepth)
{
  using multidron_gui_lib::EntityType;
  using multidron_gui_lib::PickCandidate;
  std::vector<PickCandidate> candidates(3);
  candidates[0].entity.key = {EntityType::Drone, 1U, 0U, 1U};
  candidates[0].screen_position = QPointF(102.0, 100.0);
  candidates[0].depth = 0.8F;
  candidates[1].entity.key = {EntityType::MapPoint, 2U, 3U, 7U};
  candidates[1].screen_position = QPointF(102.0, 100.0);
  candidates[1].depth = 0.2F;
  candidates[2].entity.key = {EntityType::Fiducial, 0U, 0U, 9U};
  candidates[2].screen_position = QPointF(120.0, 100.0);
  candidates[2].depth = 0.1F;

  const auto selected = multidron_gui_lib::SelectBestCandidate(
    candidates, QPointF(100.0, 100.0), 12.0);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->key.type, EntityType::MapPoint);
  EXPECT_EQ(selected->key.id, 7U);
}
