#include <gtest/gtest.h>

#include <deque>

#include "automatic_annotation/image_matching_utils.hpp"

namespace automatic_annotation {
namespace {

TEST(ImageMatchingUtilsTest, PicksEarliestCrossedSnapshot) {
  std::deque<AccumulatedCloudTimeOnlySnapshot> history = {
      {1000}, {1100}, {1200}, {1300}};

  const auto result = MatchImageToCloudTimestamp(
      /*image_time_ns=*/1150, history,
      MatchingPolicy{/*max_allowed_diff_ns=*/20, /*max_wait_ns=*/1000},
      /*now_ns=*/1150);

  EXPECT_EQ(result.status, MatchStatus::Matched);
  EXPECT_EQ(result.matched_cloud_time_ns, 1200);
}

TEST(ImageMatchingUtilsTest, WaitsWhenHistoryHasNotCaughtUp) {
  std::deque<AccumulatedCloudTimeOnlySnapshot> history = {
      {1000}, {1050}, {1100}};

  const auto result = MatchImageToCloudTimestamp(
      /*image_time_ns=*/1200, history,
      MatchingPolicy{/*max_allowed_diff_ns=*/20, /*max_wait_ns=*/1000},
      /*now_ns=*/1250);

  EXPECT_EQ(result.status, MatchStatus::Pending);
}

TEST(ImageMatchingUtilsTest, ExpiresWhenWaitingTooLong) {
  std::deque<AccumulatedCloudTimeOnlySnapshot> history = {
      {1000}, {1050}, {1100}};

  const auto result = MatchImageToCloudTimestamp(
      /*image_time_ns=*/1200, history,
      MatchingPolicy{/*max_allowed_diff_ns=*/20, /*max_wait_ns=*/1000},
      /*now_ns=*/2301);

  EXPECT_EQ(result.status, MatchStatus::DropExpired);
}

TEST(ImageMatchingUtilsTest, AcceptsNearEarlierSnapshotWithinThreshold) {
  std::deque<AccumulatedCloudTimeOnlySnapshot> history = {{1000}, {1188}};

  const auto result = MatchImageToCloudTimestamp(
      /*image_time_ns=*/1200, history,
      MatchingPolicy{/*max_allowed_diff_ns=*/20, /*max_wait_ns=*/1000},
      /*now_ns=*/1200);

  EXPECT_EQ(result.status, MatchStatus::Matched);
  EXPECT_EQ(result.matched_cloud_time_ns, 1188);
}

TEST(ImageMatchingUtilsTest, PrunesExpiredPendingImagesByRetentionWindow) {
  std::deque<int64_t> image_times = {1000, 1500, 2000, 5000};

  PruneTimestampsOlderThan(/*now_ns=*/5000, /*retention_ns=*/3000, image_times);

  ASSERT_EQ(image_times.size(), 2u);
  EXPECT_EQ(image_times.front(), 2000);
  EXPECT_EQ(image_times.back(), 5000);
}

TEST(ImageMatchingUtilsTest, ComposeTimestampNsKeepsIntegerPrecision) {
  EXPECT_EQ(ComposeTimestampNs(/*sec=*/1773905393, /*nanosec=*/600132000u),
            1773905393600132000LL);
}

}  // namespace
}  // namespace automatic_annotation
