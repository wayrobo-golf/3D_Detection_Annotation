#include "automatic_annotation/image_matching_utils.hpp"

#include <algorithm>
#include <cstdlib>

namespace automatic_annotation {

MatchDecision MatchImageToCloudTimestamp(
    int64_t image_time_ns,
    const std::deque<AccumulatedCloudTimeOnlySnapshot>& history,
    const MatchingPolicy& policy,
    int64_t now_ns) {
  MatchDecision decision;

  if (history.empty()) {
    decision.status =
        (now_ns - image_time_ns > policy.max_wait_ns) ? MatchStatus::DropExpired
                                                      : MatchStatus::Pending;
    return decision;
  }

  for (const auto& snapshot : history) {
    if (snapshot.timestamp_ns >= image_time_ns) {
      decision.status = MatchStatus::Matched;
      decision.matched_cloud_time_ns = snapshot.timestamp_ns;
      return decision;
    }
  }

  const int64_t nearest_earlier_time_ns = history.back().timestamp_ns;
  if (std::llabs(image_time_ns - nearest_earlier_time_ns) <=
      policy.max_allowed_diff_ns) {
    decision.status = MatchStatus::Matched;
    decision.matched_cloud_time_ns = nearest_earlier_time_ns;
    return decision;
  }

  decision.status =
      (now_ns - image_time_ns > policy.max_wait_ns) ? MatchStatus::DropExpired
                                                    : MatchStatus::Pending;
  return decision;
}

int64_t ComposeTimestampNs(int32_t sec, uint32_t nanosec) {
  return static_cast<int64_t>(sec) * 1000000000LL +
         static_cast<int64_t>(nanosec);
}

void PruneTimestampsOlderThan(int64_t now_ns, int64_t retention_ns,
                              std::deque<int64_t>& timestamps) {
  const int64_t cutoff_time_ns = now_ns - retention_ns;
  while (!timestamps.empty() && timestamps.front() < cutoff_time_ns) {
    timestamps.pop_front();
  }
}

}  // namespace automatic_annotation
