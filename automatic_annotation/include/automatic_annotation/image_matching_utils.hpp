#ifndef AUTOMATIC_ANNOTATION_IMAGE_MATCHING_UTILS_HPP
#define AUTOMATIC_ANNOTATION_IMAGE_MATCHING_UTILS_HPP

#include <cstdint>
#include <deque>

namespace automatic_annotation {

enum class MatchStatus {
  Matched,
  Pending,
  DropExpired,
  DropHistoryTooOld,
  DropDiffExceeded,
};

struct AccumulatedCloudTimeOnlySnapshot {
  int64_t timestamp_ns = 0;
};

struct MatchingPolicy {
  int64_t max_allowed_diff_ns = 0;
  int64_t max_wait_ns = 0;
};

struct MatchDecision {
  MatchStatus status = MatchStatus::Pending;
  int64_t matched_cloud_time_ns = 0;
};

MatchDecision MatchImageToCloudTimestamp(
    int64_t image_time_ns,
    const std::deque<AccumulatedCloudTimeOnlySnapshot>& history,
    const MatchingPolicy& policy,
    int64_t now_ns);

int64_t ComposeTimestampNs(int32_t sec, uint32_t nanosec);

void PruneTimestampsOlderThan(int64_t now_ns, int64_t retention_ns,
                              std::deque<int64_t>& timestamps);

}  // namespace automatic_annotation

#endif  // AUTOMATIC_ANNOTATION_IMAGE_MATCHING_UTILS_HPP
