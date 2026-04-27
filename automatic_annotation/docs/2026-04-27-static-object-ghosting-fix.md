# 静态物体在累加点云中重影问题的定位与解决

## 1. 问题背景

在 `automatic_annotation` 的原始点云路径中，我们会对多帧 LiDAR 点云做累加，然后在图像时刻取出同步后的累计点云，用于：

- 保存 `lidar_point_cloud_0/*.pcd`
- 生成深度图和融合图
- 生成 KITTI 标签与验证产物

在 `Binjiang20260319_Debug` 的回放中，观察到一个典型问题：

- 车辆静止时，静态码牌等细小目标基本没有明显重影
- 车辆转向或运动时，同一个静态码牌会出现双层、分叉或拖尾
- 用 `timestamp(age)` 着色后，可以看到同一目标在多层历史点之间没有对齐好

这说明问题不是单纯的 2D 投影显示问题，而是**上游累计点云本身已经出现了静态目标层间错位**。

## 2. 我们如何判断问题出在累加点云链路

### 2.1 先排除下游显示与标注链路

最先确认的是，重影已经出现在最终保存的：

- `Scene_xx/lidar_point_cloud_0/<timestamp>.pcd`

而不是只出现在：

- `dep_fuse_*.jpg`
- `kitti_verify_*.jpg`

这意味着问题发生在：

- 点云累计
- 点云同步
- 图像时刻补偿

这些步骤里，而不是发生在图像渲染、标签投影或 Xtreme 工作流下游。

### 2.2 新版带 `timestamp(age)` 的点云给了直接证据

新版代码把累计输出点云保存为 `PointXYZIT`，其中：

- `x/y/z/intensity` 是累计后的点
- `timestamp` 表示该点相对当前累计末帧的 `age`

通过对问题帧按 `timestamp(age)` 着色，可以直接看到：

- 全局点云通常有 10 层左右历史 age
- 但静态码牌局部往往只呈现少数几种颜色
- 同一个静态目标在不同 age 层之间位置不一致

这说明：

1. 问题不是“只剩几帧被保存”
2. 而是“累计窗口里的历史帧没有稳定地对齐到同一刚体参考”

### 2.3 日志继续缩小范围

在 `accurate` deskew 初版里，日志里持续出现：

- `Accurate deskew falling back to fast`
- `cache_hit_count=0`
- `Deskew trajectory sampling failed`

同时 `fast` 模式却可以稳定产出 `Saved Aligned PCD`。

这说明：

- 问题不是原始 bag 时间轴错位
- 也不是 `AccumulatePointCloudBack()` 本身完全不可用
- 更像是 `accurate` 模式下，用于 deskew 的动态位姿缓存命中率不足

## 3. 根因分析

这次真正收敛出来的主因，不是“算法公式错误”，而是**位姿缓存的调度与消费方式不适合当前的实时回放节奏**。

### 3.1 `ins_odom` 回调被点云回调阻塞

当时的实现中：

- 原始点云订阅回调挂在 `callback_group_lidar_`
- `/novatel/oem7/ins_odom` 订阅也挂在 `callback_group_lidar_`
- 这个 callback group 是 `MutuallyExclusive`

结果就是：

- 点云回调一旦开始做 `accurate` deskew
- `ins_odom` 回调就不能并行推进 pose cache
- 虽然 bag 里 `/novatel/oem7/ins_odom` 本身是连续 50Hz
- 但缓存写入会落后于当前扫描窗

这会直接导致 `accurate` 所需的 50 个位姿采样点整体 miss，最后降级到 `fast`。

### 3.2 `accurate` 对全局 pose cache 的消费方式过于脆弱

初版 `accurate` 路径对全局 pose cache 的使用方式是：

- 直接逐次查询全局 cache
- 对每个 sample time 单独判断是否命中
- 50 个 sample 点中只要一批 miss，就容易整帧失败

这种方式有两个问题：

1. 高频加锁查询会增加竞争
2. 无法区分“缓存还没追上”与“缓存窗内有断层”

最终表现为：

- `cache_hit_count=0`
- `fallback_fast` 比例高
- 静态目标累计层不稳定

### 3.3 为什么这会放大成静态物体重影

静态物体重影的本质是：

- 多帧点云在合成一朵累计云时，没有稳定地对齐到同一个时刻参考

而 `accurate` deskew 的职责，就是在进入累计前，先把**单帧扫描内部**拉成一个更接近刚体快照。

如果 `accurate` 经常失败并降级到 `fast`，或者 cache 覆盖不足导致某些时刻的位姿补偿不稳定，就会出现：

- 某些历史层对得上
- 某些历史层对不上
- 静态细目标在 age 层之间散开

这正是码牌、杆体、薄边界目标最容易先暴露出来的现象。

## 4. 解决思路

解决思路不是改累计公式，也不是改 age 语义，而是优先解决：

1. **位姿缓存来不及写入**
2. **`accurate` 对缓存的消费方式不稳定**

我们最终采用了两步方案。

### 4.1 拆开位姿写入与点云 deskew 的调度

新增独立的：

- `callback_group_pose_`

并让：

- `pointcloud_sub_` 继续留在 `callback_group_lidar_`
- `ins_odom_sub_` 改挂到 `callback_group_pose_`

这样即使点云回调正在 deskew：

- `/novatel/oem7/ins_odom` 仍然可以并行写入 pose cache

它解决的是：

> bag 明明有 50Hz pose，但缓存总是追不上当前扫描窗

### 4.2 把 `accurate` 改成“等覆盖 + 快照 + 本地插值”

新的 `accurate` 路径不再对全局 pose cache 做 50 次逐次加锁查询，而是改成：

1. 先根据 `header.stamp + max_raw_timestamp` 求出
   - `scan_start_time_ns`
   - `scan_end_time_ns`
   - `reference_time_ns`
2. 先检查 pose cache 是否覆盖了 `[scan_start, scan_end]`
3. 如果 `newest_time_ns < scan_end_time_ns`，最多等待 `30ms`
   - 每 `1ms` 轮询一次
4. 一旦覆盖到，就从全局 cache 复制一份局部快照
   - 只保留当前扫描窗需要的姿态片段
   - 两端各多保留一个插值支撑点
5. 后续 50 个 sample 和 `reference_pose` 都只在这份局部快照上做本地插值

这个方案的关键收益是：

- 避免 50 次热点加锁查询
- 把 cache 是否覆盖扫描窗，和局部插值是否成功，拆成两个独立问题
- 一旦 cache 覆盖到，后续 deskew 全部变成纯内存插值

## 5. 实现要点

### 5.1 不改的部分

这次解决静态物体重影时，刻意没有改下面这些行为：

- `RunFastDeskew()` 的桶式 deskew
- `AccumulatePointCloudBack()` 的累计逻辑
- `WaitForSyncedPointCloud()` 的同步逻辑
- `PointXYZIT.timestamp`
  - deskew 后单帧仍为 `0.0`
  - 累计输出后仍表示 `age`

也就是说，这轮修复的重点是：

> 让进入累计前的单帧 `accurate` deskew 更稳定

而不是重写整条点云累计链路。

### 5.2 新增的 pose cache helper

围绕 `annotation_status_.ins_pose_cache` 补了 3 个 helper：

- `GetPoseCacheCoverageSnapshot()`
  - 读取当前 cache 的 `oldest_time_ns / newest_time_ns`
- `WaitForPoseCacheCoverage()`
  - 最多等待 `30ms`
  - 每 `1ms` 轮询是否覆盖到扫描窗
- `CopyPoseCacheWindowSnapshot()`
  - 从全局 cache 复制当前扫描窗对应的局部姿态片段

### 5.3 失败原因显式化

为了继续排查重影问题，这轮还把 `accurate` 的失败原因做了分类，至少区分：

- `cache_empty`
- `cache_behind_scan_end`
- `cache_before_scan_start`
- `coverage_gap_inside_window`
- `reference_pose_missing`
- `trajectory_interpolation_failed`

这些信息会出现在：

- `Deskew trajectory sampling failed`
- `Accurate deskew falling back to fast`
- `Deskew success snapshot`

这样后面再看日志时，可以直接知道：

- 是 cache 根本没追上
- 是扫描窗中间断了
- 还是参考位姿插值失败

## 6. 为什么这套方案能改善静态物体重影

要理解效果，关键是记住：

- 静态物体重影不是“点少”本身
- 而是“多层历史点在空间上没落到同一个静态几何位置”

这次方案改善的是进入累计前的单帧 deskew 稳定性：

- `ins_odom` 不再被点云回调阻塞
- `accurate` 不再频繁因 cache miss 降级
- 扫描窗内 50 个采样点更稳定地使用同一段局部姿态快照

最终效果就是：

- 更多帧真的走 `effective_path=accurate`
- 历史层之间的空间对齐更稳定
- 静态细目标在累计云里更容易收拢成一层，而不是散成双层或多层

## 7. 验证结果

### 构建验证

本次修改后，以下命令通过：

```bash
cmake --build build/automatic_annotation -- -j4
cmake --install build/automatic_annotation
```

编译输出里只有仓库原本就存在的无关 warning：

- `VerifyKITTILabelInPointCloud()` 中 `valid_box_count` 未使用

### 回放验证

验证 bag：

```text
/home/keyaoli/Data/AutoAnnotation/Wayrobo_Rosbag/Binjiang20260319_Debug/continuous_2026_03_19-15_29_53
```

验证产物目录：

```text
install/automatic_annotation/share/automatic_annotation/data/data_record_20260427193146/CacheCoverVerify_202604271933
```

产物数量：

- `camera_image_0`: `23`
- `lidar_point_cloud_0`: `23`
- `depth_image`: `23`
- `fuse_image`: `69`
- `label_2`: `23`

### 与优化前相比

按运行进程号统计总日志：

- 优化前 `pid=346237`
  - `accurate_success=10`
  - `fallback_fast_success=10`
  - `fallback_warn=10`
  - `saved_aligned=26`
  - `timeouts=3`
- 优化后 `pid=377283`
  - `accurate_success=19`
  - `fallback_fast_success=0`
  - `fallback_warn=0`
  - `saved_aligned=23`
  - `timeouts=1`

这说明这轮针对静态物体重影的核心修复已经起效：

- `accurate` 命中率显著提高
- `fallback_fast` 基本被压掉
- 累计点云输出链路保持稳定

日志里还能看到新的典型成功特征：

```text
Deskew sampled trajectory snapshot ... wait_ms=0 snapshot_size=7
Deskew success snapshot ... effective_path=accurate ... fallback_fast=false
```

## 8. 当前边界

这轮并没有一次性解决所有和重影相关的问题，仍然保留以下边界：

- `GetTimeInterpolatedTransform()` 仍可能因为 live TF 缺失而 fallback
- bag 末尾仍可能出现尾帧 `Time sync timeout`
- `raw_deskew_timestamp_unit_sec=1e-9` 与 `from_scan_start` 的时间语义没有在这轮调整

所以更准确的结论是：

> 这轮已经解决了“静态物体重影背后，`accurate` deskew 因位姿缓存覆盖不足而频繁降级”的主问题，但没有同时解决所有 TF fallback 与尾帧同步问题。

## 9. 后续建议

如果后面要继续围绕“静态物体重影”做优化，建议按这个顺序继续推进：

1. 继续跟踪 `GetTimeInterpolatedTransform()` 的 TF fallback
2. 统计尾帧 `Time sync timeout` 是否只发生在 bag 结束区间
3. 如果之后又出现 `failure_reason=coverage_gap_inside_window`，再单独评估 pose source 时间语义或输入源替换
