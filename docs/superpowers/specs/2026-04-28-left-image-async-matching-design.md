# 左相机异步匹配流水线设计

## 背景

当前 `automatic_annotation` 的左相机主链路把以下工作都放在 `HandleLeftImageMsg()` 一次回调内完成：

- 图像解码
- 等待同步累计点云
- 图像时刻点云补偿
- 深度图/融合图生成
- KITTI/Xtreme 导出

在 `Binjiang20260319_Debug` 的回放中，`/camera/left/image` 输入接近 `2Hz`，但单帧左图主链路耗时常常超过 `500ms`，甚至可达秒级。因此图像订阅来不及消费，叠加 `kCameraQueueSize = 1`，最终表现为：

- bag 中的左图像并未全部进入 `HandleLeftImageMsg()`
- `camera_image_0`、`lidar_point_cloud_0`、`label_2` 数量都低于理论值
- 即使把图像队列调大，也可能让“较旧图像”拿到“较新的累计点云”，影响时间匹配质量

目标不是只保住原始图像，而是尽可能多地让“图像 + 同时刻累计点云”一起进入完整自动标注流水线，生成更多且尽量准确的预标注结果。

## 目标

本次改造目标如下：

1. 将左相机输入从“重回调即处理”改为“轻量接收 + 异步匹配 + 后台处理”。
2. 不再只保留 `latest_accumulated_cloud`，而是保留一个短时 `accumulated_cloud_history`。
3. 让左相机图像优先与时间上最合适的累计点云配对，再进入现有重处理链。
4. 为右相机/后续其他图像源预留同样的任务抽象和接口，但第一版只让左相机真正接入异步匹配链路。
5. 尽量复用现有 deskew、累计、TF、深度图、KITTI、Xtreme 导出逻辑，不重写整套自动标注流程。

## 非目标

以下内容不在本次范围内：

- 不改 raw deskew / accurate / fallback_fast 逻辑
- 不改 `PointXYZIT.timestamp = age` 的最终语义
- 不改 `annotationctl submit/finalize` 工作流
- 不把右相机第一版真正接入异步匹配处理
- 不把系统整体重构成完全离线、非 ROS2 订阅式的 bag 读取架构

## 设计总览

### 1. 三层流水线

左相机处理链拆成三层：

1. **接收层**
   - 左图像订阅回调只负责：
     - 计数
     - 原始图像保存
     - 将图像任务入 `pending_image_tasks`
   - 不在这里做同步等待、点云补偿和自动标注

2. **匹配层**
   - 新增 matcher 定时器或后台执行单元
   - 从 `pending_image_tasks` 中取最早待处理图像
   - 在 `accumulated_cloud_history` 中寻找时间上最合适的累计点云
   - 成功则生成 `MatchedImageTask` 入 `matched_image_tasks`
   - 失败则根据原因继续等待或丢弃

3. **处理层**
   - 新增 worker 定时器或后台执行单元
   - 只处理已经匹配好的 `MatchedImageTask`
   - 重用现有左图主链中的：
     - 图像时刻点云补偿
     - 深度图生成
     - KITTI/Xtreme 导出
     - 可视化验证

### 2. 关键缓存

新增两类缓存：

- `pending_image_tasks`
  - 保存最近一段时间内尚未匹配的图像任务
- `accumulated_cloud_history`
  - 保存最近一段时间内的累计点云快照

现有 `latest_accumulated_cloud` 保留，用于兼容已有逻辑和调试；新匹配流程优先使用 `accumulated_cloud_history`。

### 3. 抽象层

为了给右相机/后续图像源预留同样模式，引入通用任务抽象：

- `CameraRole`
  - `Left`
  - `Right`
- `PendingImageTask`
- `AccumulatedCloudSnapshot`
- `MatchedImageTask`

第一版只对 `CameraRole::Left` 做真实处理；`Right` 仅作为数据结构和接口预留，不改变现有右图保存行为。

## 数据结构

建议新增以下内部结构：

### `CameraRole`

用于标识图像源，避免后续逻辑写死到左相机。

### `PendingImageTask`

字段建议：

- `CameraRole role`
- `int64_t timestamp_ns`
- `sensor_msgs::msg::Image::SharedPtr image_msg`

### `AccumulatedCloudSnapshot`

字段建议：

- `int64_t timestamp_ns`
- `pcl::PointCloud<PointXYZIT>::Ptr cloud`

### `MatchedImageTask`

字段建议：

- `CameraRole role`
- `int64_t timestamp_ns`
- `sensor_msgs::msg::Image::SharedPtr image_msg`
- `pcl::PointCloud<PointXYZIT>::Ptr cloud_snapshot`

## 匹配策略

### 1. 历史窗口

第一版使用固定窗口：

- 图像待匹配窗口：最近 `3s`
- 累计点云历史窗口：最近 `3s`

超出窗口的图像或点云快照统一裁剪，防止队列无限膨胀。

### 2. 匹配规则

对于最早的 `PendingImageTask`：

1. 优先寻找 `timestamp_ns >= image_timestamp_ns` 的最早累计点云快照
2. 若不存在“追上/越过”的快照，则允许使用 `abs(diff) <= 20ms` 的最近快照
3. 若当前历史窗口仍不足以决定，且该图像等待时长未超 `1s`，则继续保留等待
4. 若等待超过 `1s` 仍无法匹配，则丢弃并记录原因

这里延续当前 `WaitForSyncedPointCloud()` 的时间语义，但不再依赖单个 `latest_accumulated_cloud`。

### 3. 丢弃原因分类

至少记录以下原因：

- `history_empty`
- `history_too_old`
- `history_not_caught_up`
- `history_diff_exceeded`
- `image_task_expired`

## 执行模型

### 1. 回调分工

- 左图像订阅回调：轻量入队
- 点云累计回调：更新 `latest_accumulated_cloud` 并追加 `accumulated_cloud_history`
- matcher：独立 callback group
- worker：独立 callback group

### 2. 第一版实现建议

第一版优先使用 ROS2 timer + 独立 callback group，而不是手写裸线程：

- 生命周期更简单
- 更符合当前节点实现风格
- 便于和 `MultiThreadedExecutor` 配合

建议新增：

- `callback_group_matcher_`
- `callback_group_worker_`
- `left_image_match_timer_`
- `left_image_worker_timer_`

## 与现有代码的集成方式

### 保留的现有能力

以下逻辑尽量原样复用：

- `DeskewOriginalPointCloudToPointXYZIT()`
- `AccumulatePointCloudBack()`
- `GetTimeInterpolatedTransform()`
- `GenerateDepthAndFusion()`
- `GenerateKITTILabel()`
- `VerifyKITTILabel()`
- `GenerateCameraExtrinsic()`

### 需要抽取的重处理逻辑

当前 `HandleLeftImageMsg()` 内部的重处理部分需要抽成一个“处理 matched task”的函数，例如：

- `ProcessMatchedLeftImageTask(...)`

这样左图像回调只负责入队，worker 再调用这段重处理逻辑。

## 风险与缓解

### 风险 1：缓存增长导致内存上涨

缓解：

- 固定 `3s` 历史窗口
- 每次 push 后主动裁剪

### 风险 2：图像进入队列但迟迟不处理

缓解：

- `pending_image_tasks` 按时间戳排序，只处理队首
- 超过 `1s` 直接丢弃并记录原因

### 风险 3：引入异步后调试复杂度上升

缓解：

- 对 matcher 和 worker 增加独立日志
- 记录匹配成功、继续等待、丢弃原因、队列长度

### 风险 4：右相机接口预留导致首版实现过重

缓解：

- 只预留 `CameraRole` 和通用任务结构
- 右相机第一版不真正进入匹配/处理链

## 验收标准

第一版完成后，至少满足：

1. 左图像回调不再直接执行重处理流水线，而是轻量入队。
2. 系统存在 `accumulated_cloud_history`，不再只依赖 `latest_accumulated_cloud`。
3. 左相机图像能够通过异步匹配层进入完整自动标注流水线。
4. `camera_image_0` 数量提升。
5. `lidar_point_cloud_0`、`label_2` 数量也提升，而不是只有原始图像变多。
6. 新日志能明确区分：
   - 图像已匹配成功
   - 图像仍在等待更合适的累计点云
   - 图像因超时或误差超限被丢弃

## 最小实施范围

第一版只做以下闭环：

- 左图像真正接入异步匹配链路
- 右相机只预留抽象和接口
- 使用 ROS2 timer 驱动 matcher/worker
- 为匹配逻辑补最小可行单元测试
- 继续支持当前 `dev_auto` 自动化 submit 工作流

不做额外重构，不同时引入更激进的离线架构变更。
