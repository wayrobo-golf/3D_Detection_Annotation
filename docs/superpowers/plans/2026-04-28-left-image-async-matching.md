# Left Image Async Matching Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将左相机主链改为“轻量接收 + 累计点云历史匹配 + 后台重处理”，提升进入完整自动标注流水线的有效帧数，同时为右相机/后续图像源预留相同抽象。

**Architecture:** 新增通用图像任务和累计点云快照抽象，把左图像订阅从重处理改为轻量入队；用 ROS2 timer 在独立 callback group 中执行 matcher 和 worker。匹配逻辑抽成纯函数并先用 gtest 覆盖，重处理逻辑复用现有 `HandleLeftImageMsg()` 的后半段。

**Tech Stack:** ROS2 Humble, rclcpp, sensor_msgs, pcl, OpenCV, ament_cmake_gtest, C++17

---

## File Structure

- Modify: `automatic_annotation/include/automatic_annotation/const_value.hpp`
  - 新增异步匹配窗口、队列上限、timer 周期等默认常量
- Modify: `automatic_annotation/include/automatic_annotation/gen_prompt_point.hpp`
  - 新增 `CameraRole`、任务结构、历史缓存、队列、timer、callback group、方法声明
- Modify: `automatic_annotation/src/gen_prompt_point.cpp`
  - 左图像改轻量入队；累计点云更新历史；新增 matcher/worker；抽取重处理函数
- Create: `automatic_annotation/include/automatic_annotation/image_matching_utils.hpp`
  - 声明可测试的纯匹配逻辑与结果枚举
- Create: `automatic_annotation/src/image_matching_utils.cpp`
  - 实现纯匹配逻辑
- Create: `automatic_annotation/test/test_image_matching_utils.cpp`
  - gtest 覆盖匹配选择与超时判定
- Modify: `automatic_annotation/CMakeLists.txt`
  - 接入 `ament_cmake_gtest`，编译新源文件和测试
- Modify: `automatic_annotation/package.xml`
  - 增加 `ament_cmake_gtest` test 依赖

## Task 1: Add Testable Matching Primitives

**Files:**
- Create: `automatic_annotation/include/automatic_annotation/image_matching_utils.hpp`
- Create: `automatic_annotation/src/image_matching_utils.cpp`
- Test: `automatic_annotation/test/test_image_matching_utils.cpp`
- Modify: `automatic_annotation/CMakeLists.txt`
- Modify: `automatic_annotation/package.xml`

- [ ] **Step 1: Write the failing test**

```cpp
#include <gtest/gtest.h>

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

}  // namespace
}  // namespace automatic_annotation
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build/automatic_annotation --target test_image_matching_utils -j4`

Expected: FAIL because `image_matching_utils.hpp` / `MatchImageToCloudTimestamp` do not exist yet.

- [ ] **Step 3: Write minimal implementation**

```cpp
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
```

最小实现规则：
- `history` 为空返回 `Pending` 或 `DropExpired`
- 找到 `timestamp >= image_time_ns` 的最早快照则返回 `Matched`
- 否则若最近快照与图像差值 `<= max_allowed_diff_ns` 则返回 `Matched`
- 否则若 `now_ns - image_time_ns > max_wait_ns` 返回 `DropExpired`
- 否则返回 `Pending`

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build build/automatic_annotation --target test_image_matching_utils -j4 && ctest --test-dir build/automatic_annotation -R test_image_matching_utils --output-on-failure`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add automatic_annotation/include/automatic_annotation/image_matching_utils.hpp \
        automatic_annotation/src/image_matching_utils.cpp \
        automatic_annotation/test/test_image_matching_utils.cpp \
        automatic_annotation/CMakeLists.txt \
        automatic_annotation/package.xml
git commit -m "test: add image matching utility coverage"
```

## Task 2: Add Async Image/Cloud Task Structures

**Files:**
- Modify: `automatic_annotation/include/automatic_annotation/gen_prompt_point.hpp`
- Modify: `automatic_annotation/include/automatic_annotation/const_value.hpp`

- [ ] **Step 1: Write the failing test**

在 `automatic_annotation/test/test_image_matching_utils.cpp` 中新增一个编译型测试，引用未来要公开的结构，至少包含：

```cpp
TEST(ImageMatchingUtilsTest, CameraRoleEnumCompilesForLeftAndRight) {
  CameraRole left = CameraRole::Left;
  CameraRole right = CameraRole::Right;
  EXPECT_NE(left, right);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build/automatic_annotation --target test_image_matching_utils -j4`

Expected: FAIL because `CameraRole` does not exist yet in public headers used by the test.

- [ ] **Step 3: Write minimal implementation**

在 `gen_prompt_point.hpp` 中新增：

```cpp
enum class CameraRole { Left = 0, Right = 1 };

struct PendingImageTask {
  CameraRole role = CameraRole::Left;
  int64_t timestamp_ns = 0;
  sensor_msgs::msg::Image::SharedPtr image_msg;
};

struct AccumulatedCloudSnapshot {
  int64_t timestamp_ns = 0;
  pcl::PointCloud<PointXYZIT>::Ptr cloud;
};

struct MatchedImageTask {
  CameraRole role = CameraRole::Left;
  int64_t timestamp_ns = 0;
  sensor_msgs::msg::Image::SharedPtr image_msg;
  pcl::PointCloud<PointXYZIT>::Ptr cloud_snapshot;
};
```

在 `AutoAnnotationStatus` 中新增：

```cpp
std::deque<PendingImageTask> pending_image_tasks;
std::deque<MatchedImageTask> matched_image_tasks;
std::deque<AccumulatedCloudSnapshot> accumulated_cloud_history;
```

在 `const_value.hpp` 中新增常量：

```cpp
constexpr size_t kAccumulatedCloudHistoryLimit = 64;
constexpr int64_t kImageTaskMaxWaitNs = 1000000000LL;
constexpr int64_t kImageHistoryRetentionNs = 3000000000LL;
constexpr int64_t kCloudHistoryRetentionNs = 3000000000LL;
constexpr int64_t kMatcherTimerPeriodMs = 10;
constexpr int64_t kWorkerTimerPeriodMs = 5;
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build build/automatic_annotation --target test_image_matching_utils -j4 && ctest --test-dir build/automatic_annotation -R test_image_matching_utils --output-on-failure`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add automatic_annotation/include/automatic_annotation/gen_prompt_point.hpp \
        automatic_annotation/include/automatic_annotation/const_value.hpp \
        automatic_annotation/test/test_image_matching_utils.cpp
git commit -m "feat: add async image matching task structures"
```

## Task 3: Switch Left Image Callback to Lightweight Enqueue

**Files:**
- Modify: `automatic_annotation/include/automatic_annotation/gen_prompt_point.hpp`
- Modify: `automatic_annotation/src/gen_prompt_point.cpp`

- [ ] **Step 1: Write the failing test**

在 `test_image_matching_utils.cpp` 中新增一个纯逻辑测试，验证“队首 pending 图像时间戳保持先入先出”需要的 helper。先声明未来要新增的 helper：

```cpp
TEST(ImageMatchingUtilsTest, PrunesExpiredPendingImagesByRetentionWindow) {
  std::deque<int64_t> image_times = {1000, 1500, 2000, 5000};
  PruneTimestampsOlderThan(/*now_ns=*/5000, /*retention_ns=*/2000, image_times);
  ASSERT_EQ(image_times.size(), 2u);
  EXPECT_EQ(image_times.front(), 2000);
  EXPECT_EQ(image_times.back(), 5000);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build/automatic_annotation --target test_image_matching_utils -j4`

Expected: FAIL because `PruneTimestampsOlderThan` does not exist.

- [ ] **Step 3: Write minimal implementation**

在 `image_matching_utils.hpp/.cpp` 中新增一个可复用裁剪 helper，例如：

```cpp
void PruneTimestampsOlderThan(int64_t now_ns, int64_t retention_ns,
                              std::deque<int64_t>& timestamps);
```

然后在 `gen_prompt_point.cpp` 中：

- 将当前左图重回调改为 `EnqueueLeftImageTask(...)`
- 原始左图保存仍保留在轻量路径
- 新增把 `PendingImageTask{CameraRole::Left, timestamp, msg}` 压入 `pending_image_tasks`
- 在入队后按 `kImageHistoryRetentionNs` 裁剪历史图像任务

第一版要求：
- 该回调中不再调用 `WaitForSyncedPointCloud()`
- 该回调中不再进行深度图 / KITTI / Xtreme 导出

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build build/automatic_annotation --target test_image_matching_utils -j4 && ctest --test-dir build/automatic_annotation -R test_image_matching_utils --output-on-failure`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add automatic_annotation/src/gen_prompt_point.cpp \
        automatic_annotation/include/automatic_annotation/gen_prompt_point.hpp \
        automatic_annotation/include/automatic_annotation/image_matching_utils.hpp \
        automatic_annotation/src/image_matching_utils.cpp \
        automatic_annotation/test/test_image_matching_utils.cpp
git commit -m "feat: enqueue left images for async matching"
```

## Task 4: Preserve Accumulated Cloud History

**Files:**
- Modify: `automatic_annotation/include/automatic_annotation/gen_prompt_point.hpp`

- [ ] **Step 1: Write the failing test**

在 `test_image_matching_utils.cpp` 中新增：

```cpp
TEST(ImageMatchingUtilsTest, PrunesOldCloudHistorySnapshots) {
  std::deque<int64_t> cloud_times = {1000, 1500, 2000, 5000};
  PruneTimestampsOlderThan(/*now_ns=*/5000, /*retention_ns=*/2500, cloud_times);
  ASSERT_EQ(cloud_times.size(), 2u);
  EXPECT_EQ(cloud_times.front(), 2000);
  EXPECT_EQ(cloud_times.back(), 5000);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build/automatic_annotation --target test_image_matching_utils -j4`

Expected: FAIL if helper or history update path is not yet connected.

- [ ] **Step 3: Write minimal implementation**

在 `AccumulatePointCloudBack()` 更新 `latest_accumulated_cloud` 的位置后追加：

```cpp
AccumulatedCloudSnapshot snapshot;
snapshot.timestamp_ns = header.stamp.sec * 1000000000LL + header.stamp.nanosec;
snapshot.cloud.reset(new pcl::PointCloud<PointXYZIT>(*final_cloud_local));
annotation_status_.accumulated_cloud_history.push_back(std::move(snapshot));
```

然后：

- 以 `kCloudHistoryRetentionNs` 裁剪历史
- 额外用 `kAccumulatedCloudHistoryLimit` 做上限保护

要求：
- 保留 `latest_accumulated_cloud` 现有行为不变
- 新历史缓存只做附加，不破坏现有逻辑

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build build/automatic_annotation --target test_image_matching_utils -j4 && ctest --test-dir build/automatic_annotation -R test_image_matching_utils --output-on-failure`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add automatic_annotation/include/automatic_annotation/gen_prompt_point.hpp \
        automatic_annotation/test/test_image_matching_utils.cpp
git commit -m "feat: record accumulated cloud history snapshots"
```

## Task 5: Add Matcher and Worker Timers

**Files:**
- Modify: `automatic_annotation/include/automatic_annotation/gen_prompt_point.hpp`
- Modify: `automatic_annotation/src/gen_prompt_point.cpp`

- [ ] **Step 1: Write the failing test**

在 `test_image_matching_utils.cpp` 中新增纯逻辑测试，覆盖“历史中存在 crossed snapshot 时应优先选最早 crossed”的真实匹配规则；如果 Task 1 已覆盖，则补一个“允许 early catch”测试：

```cpp
TEST(ImageMatchingUtilsTest, AcceptsNearEarlierSnapshotWithinThreshold) {
  std::deque<AccumulatedCloudTimeOnlySnapshot> history = {
      {1000}, {1188}};
  const auto result = MatchImageToCloudTimestamp(
      /*image_time_ns=*/1200, history,
      MatchingPolicy{/*max_allowed_diff_ns=*/20, /*max_wait_ns=*/1000},
      /*now_ns=*/1200);

  EXPECT_EQ(result.status, MatchStatus::Matched);
  EXPECT_EQ(result.matched_cloud_time_ns, 1188);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build/automatic_annotation --target test_image_matching_utils -j4`

Expected: FAIL until matcher uses the intended rule set.

- [ ] **Step 3: Write minimal implementation**

在 `GenPromptPoint` 中新增：

- `callback_group_matcher_`
- `callback_group_worker_`
- `left_image_match_timer_`
- `left_image_worker_timer_`
- `TryMatchPendingLeftImageTask()`
- `ProcessMatchedLeftImageTask()`

matcher timer 行为：

- 只看 `pending_image_tasks` 队首、且只处理 `CameraRole::Left`
- 调用 `MatchImageToCloudTimestamp(...)`
- 成功则从 `accumulated_cloud_history` 复制对应快照，构造 `MatchedImageTask`
- `Pending` 则保留
- `DropExpired` / `DropDiffExceeded` 则记录日志并丢弃

worker timer 行为：

- 一次只消费一个 `MatchedImageTask`
- 调用新的 `ProcessMatchedLeftImageTask(...)`

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build build/automatic_annotation --target test_image_matching_utils -j4 && ctest --test-dir build/automatic_annotation -R test_image_matching_utils --output-on-failure`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add automatic_annotation/include/automatic_annotation/gen_prompt_point.hpp \
        automatic_annotation/src/gen_prompt_point.cpp \
        automatic_annotation/include/automatic_annotation/image_matching_utils.hpp \
        automatic_annotation/src/image_matching_utils.cpp \
        automatic_annotation/test/test_image_matching_utils.cpp
git commit -m "feat: add async left image matcher and worker timers"
```

## Task 6: Reuse Existing Left Image Heavy Processing

**Files:**
- Modify: `automatic_annotation/src/gen_prompt_point.cpp`
- Modify: `automatic_annotation/include/automatic_annotation/gen_prompt_point.hpp`

- [ ] **Step 1: Write the failing test**

先保留 `test_image_matching_utils.cpp` 全绿，再新增一个最小编译型测试，要求 `ProcessMatchedLeftImageTask(...)` 存在并可被声明引用；例如通过公开一个只在测试里引用的前向声明 helper 签名。

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build/automatic_annotation --target test_image_matching_utils -j4`

Expected: FAIL because the new processing entry point does not exist yet.

- [ ] **Step 3: Write minimal implementation**

把当前 `HandleLeftImageMsg()` 中从：

- `WaitForSyncedPointCloud()` 之后
- 到 `GenerateCameraExtrinsic()` 之前/之后的重处理逻辑

抽成：

```cpp
void ProcessMatchedLeftImageTask(const MatchedImageTask& task);
```

要求：

- 输入不再从订阅消息直接取“latest cloud”
- 而是使用 `task.cloud_snapshot`
- 继续生成：
  - `lidar_point_cloud_0`
  - `depth_image`
  - `fuse_image`
  - `label_2`
  - `camera_config`

第一版只支持 `CameraRole::Left`，若收到其他 role 直接返回并记录日志。

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build build/automatic_annotation --target test_image_matching_utils -j4 && ctest --test-dir build/automatic_annotation -R test_image_matching_utils --output-on-failure`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add automatic_annotation/src/gen_prompt_point.cpp \
        automatic_annotation/include/automatic_annotation/gen_prompt_point.hpp
git commit -m "refactor: reuse left image heavy pipeline for matched tasks"
```

## Task 7: Verify End-to-End on Single Bag

**Files:**
- Modify: `automatic_annotation/src/gen_prompt_point.cpp` (only if verification reveals logging gaps)

- [ ] **Step 1: Run single-bag verification**

Run:

```bash
/home/keyaoli/program/miniconda3/envs/nusc_env/bin/python - <<'PY'
from pathlib import Path
from pre_annotation_factory.scripts.replay_rosbag_main import run_auto_annotation_job

artifacts = run_auto_annotation_job(
    '/home/keyaoli/Data/AutoAnnotation/Wayrobo_Rosbag/Binjiang20260319_Debug/continuous_2026_03_19-15_29_53',
    'Binjiang_20260319_Debug_AsyncMatchVerify',
    workspace_path=Path('/home/keyaoli/Code/Wayrobo/3D_Detection_Annotation'),
    xtreme1_output_dir=Path('/home/keyaoli/Data/AutoAnnotation/TempVerify/XtremeUpload'),
    raw_data_archive_dir=Path('/home/keyaoli/Data/AutoAnnotation/TempVerify/RawOrigin'),
    preserve_full_raw_copy=False,
    cleanup_share_data=False,
)
print(artifacts)
PY
```

Expected:
- 回放完成
- 本地产物正常归档

- [ ] **Step 2: Check artifact counts**

Run:

```bash
python3 - <<'PY'
from pathlib import Path
root = Path('/home/keyaoli/Data/AutoAnnotation/TempVerify/RawOrigin')
for scene in sorted(root.glob('3DBox_Annotation_*_AsyncMatchVerify_origin/data_record_*/Scene_*')):
    print(scene)
    print('camera_image_0', len(list((scene/'camera_image_0').glob('*'))))
    print('lidar_point_cloud_0', len(list((scene/'lidar_point_cloud_0').glob('*.pcd'))))
    print('label_2', len(list((scene/'label_2').glob('*.txt'))))
PY
```

Expected:
- `camera_image_0` 高于当前 31 张基线
- `lidar_point_cloud_0` 高于当前 28 张基线
- `label_2` 跟随提升

- [ ] **Step 3: Inspect logs**

Run:

```bash
python3 - <<'PY'
from pathlib import Path
log = Path('/var/log/robot_log/automatic_annotation_log.log').read_text(encoding='utf-8', errors='ignore')
for key in ['Matched left image task', 'Pending left image task', 'Dropped left image task']:
    print(key, log.count(key))
PY
```

Expected:
- 能看到匹配成功/等待/丢弃的日志统计

- [ ] **Step 4: If verification passes, commit**

```bash
git add automatic_annotation
git commit -m "feat: add async left image matching pipeline"
```

## Spec Coverage Check

- 异步接收/匹配/处理三层：Task 3、Task 5、Task 6 覆盖
- 历史累计点云缓存：Task 4 覆盖
- 左相机真实接入、右相机只预留抽象：Task 2、Task 6 覆盖
- 纯逻辑最小测试先行：Task 1、Task 3、Task 4、Task 5 覆盖
- 单 bag 验证数量提升：Task 7 覆盖

