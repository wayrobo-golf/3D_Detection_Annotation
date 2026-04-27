# 累积点云中静态障碍物重影问题分析与排查

## 1. 问题定义

本文讨论的是 `automatic_annotation` 工作流里，累积点云用于后续投影、深度图和自动标注时，静态障碍物出现“重影”的问题。这里的“重影”通常不是图像渲染问题，而是同一个静态物体在累积点云中被堆成了两个或多个略有偏移的轮廓。

常见表现包括：

- 同一静态物体边缘被拉宽，轮廓不再锐利。
- 在点云可视化里看到双层边缘、拖尾、重复轮廓。
- 深度投影到图像上时，静态物体边界发虚或出现前后两层。
- 不同帧对齐后，原本不应运动的墙体、路沿、沙堆、路牌像“轻微晃动”。

需要先明确一个判断口径：

- 如果保存下来的 `aligned_lidar_cloud` 已经重影，问题更可能发生在“累计/补偿链路”。
- 如果 `aligned_lidar_cloud` 正常，但投影图或验证图重影，问题更可能偏向“相机外参/投影链路”。

在本项目里，优先排查运行时 TF、时间同步和回放问题，再怀疑算法本身。

## 2. 本项目累计点云工作原理

本项目的累计点云不是简单把多帧原始点云直接拼接在一起，而是分成两个阶段处理。

### 2.1 阶段一：点云先对齐到 `ins_map` 后累计

核心逻辑在 `AccumulatePointCloudBack()` 中，代码位置：

- `automatic_annotation/include/automatic_annotation/gen_prompt_point.hpp:393`

处理流程如下：

1. 使用当前点云消息时间戳，尝试查询 `ins_map <- lidar_link` 的 TF。
2. 如果 direct TF 查询失败，则退化为：
   `ins_map <- ins_link (dynamic)` 乘以 `ins_link <- lidar_link` 的静态外参。
3. 把当前帧点云先转换到 `ins_map`。
4. 放入滑动窗口 `cloud_queue`，窗口大小由 `target_accumulate_pc_num` 控制。
5. 以最新一帧为目标，把窗口内历史点云从 `ins_map` 再变换回当前 `lidar_link`。
6. 为每个点注入相对时间差 `timestamp=age`，生成 `latest_accumulated_cloud`。

这意味着，累计结果是否稳定，直接依赖两件事：

- 每一帧从 `lidar_link` 到 `ins_map` 的时空变换是否准确。
- 从历史帧回到当前 `lidar_link` 时，目标帧姿态是否准确。

### 2.2 阶段二：图像到来时再做一次图像时刻补偿

图像侧逻辑在 `HandleLeftImageMsg()` 中，代码位置：

- `automatic_annotation/src/gen_prompt_point.cpp:573`

处理流程如下：

1. 图像回调触发后，调用 `WaitForSyncedPointCloud()` 获取当前可用的累计点云。
2. 根据累计点云时间 `t_lidar` 与图像时间 `t_img`，调用 `GetTimeInterpolatedTransform()`。
3. 计算 `Lidar(t_lidar) -> Lidar(t_img)` 的 time-travel TF。
4. 将累计点云再补偿到图像时刻，得到 `aligned_lidar_cloud`。
5. 后续深度图、KITTI 标注验证、投影可视化都基于这份补偿后的点云。

对应代码位置：

- `WaitForSyncedPointCloud()`：`automatic_annotation/src/gen_prompt_point.cpp:1647`
- `GetTimeInterpolatedTransform()`：`automatic_annotation/src/gen_prompt_point.cpp:1876`

因此，这条链路实际是“两阶段对齐”：

1. `Lidar(frame_i)` 先对齐到 `ins_map` 做累计。
2. 累计结果再从 `Lidar(t_lidar)` 补偿到 `Lidar(t_img)`。

只要任意一个阶段的 TF、时间戳或回退逻辑有偏差，静态物体都可能被堆出重影。

## 3. 根因树

下面把静态障碍物重影的原因归为 5 类，并说明为什么会让静态物体看起来像“动了”。

### 3.1 `Map/Lidar/Ins` TF 不准

- `ins_map <- lidar_link` 或 `ins_link <- lidar_link` 有系统误差时，每一帧点云被放进 `ins_map` 的位置就不一致。
- 同一个静态物体在不同帧会落在略有偏移的位置，累计后就会出现双轮廓。
- 这种问题在车辆转弯、加减速或姿态变化明显时通常更突出。

### 3.2 图像拿到的累计点云时刻不对

- 图像回调不是直接拿“严格同一时刻”的点云，而是通过 `WaitForSyncedPointCloud()` 从缓存里取最近可用的累计点云。
- 如果累计点云时间已经略微越过图像，或者还没追上图像但误差落在阈值内，就会被接受。
- 这会把窗口内历史帧一起带入图像时刻补偿，时间差较大时就可能形成轻微重影。

### 3.3 Time-travel TF 插值失败后回退过于保守

- `GetTimeInterpolatedTransform()` 查询 `Lidar(t_lidar) -> Lidar(t_img)` 失败时，会回退到 `fallback_extrinsic`。
- 当前调用处传入的是 `Eigen::Isometry3d::Identity()`，也就是补偿失败后直接“不补偿”。
- 如果图像时间和累计点云时间并不完全一致，缺少这一步会导致静态点云相对图像时刻残留位姿偏差。

### 3.4 bag 回放、`/clock`、QoS、TF 发布不同步

- 这类问题不会直接改动算法，但会让 TF 缓冲区里拿不到正确时刻的数据。
- 如果 `/clock` 不连续、bag 播放太快、TF 发布频率太低，或 QoS 不匹配，就会导致：
  - 某些时刻拿不到 direct TF；
  - 某些时刻 time-travel TF 插值失败；
  - 某些 bag 正常、某些 bag 异常。

### 3.5 输入点云类型或前置去畸变状态与配置不匹配

- 节点支持 `point_cloud_type=0` 原始点云和 `point_cloud_type=1` 去畸变点云。
- 如果实际上输入的是仍带运动畸变的点云，但配置或下游假定它已经稳定，那么累计时会把单帧内部的畸变一起堆进去。
- 这种情况下，重影并不完全来自多帧配准，单帧本身就可能已经带拉伸或扭曲。

### 3.6 文本化排查顺序图

```text
看到静态障碍物重影
        |
        v
先看保存的 aligned_lidar_cloud 是否重影
        |
        +-- 是 --> 问题优先在累计/补偿链路
        |           |
        |           +-- 看 debug/accumulated_pc 是否也重影
        |           |       |
        |           |       +-- 是 --> 查 ins_map / ins_link / lidar_link TF、累计窗口、输入点云质量
        |           |       |
        |           |       +-- 否 --> 查图像时刻补偿、WaitForSyncedPointCloud、time-travel TF
        |           |
        |           +-- 查日志中的 Fallback / Sync Success / Time interpolation TF failed
        |
        +-- 否 --> 问题更偏向相机外参、投影、验证图链路
```

## 4. 重点代码风险点

下面只列和“静态障碍物重影”关系最直接的几个风险点。

### 4.1 `AccumulatePointCloudBack()` 的 direct TF 失败回退

代码位置：

- `automatic_annotation/include/automatic_annotation/gen_prompt_point.hpp:410`
- `automatic_annotation/include/automatic_annotation/gen_prompt_point.hpp:418`

行为：

- 先查 `ins_map <- sensor_frame`。
- 失败后回退为 `ins_map <- ins_link` 乘以 `isometry_lidar2ins`。
- `isometry_lidar2ins` 可能来自 live TF，也可能来自默认外参。

后果：

- 如果默认 `Lidar -> InsLink` 外参不准，或者 live TF 长时间未就绪，点云被放进 `ins_map` 时会系统性偏移。
- 这种偏移会直接累积成静态障碍物的重影。

重点日志：

- `Direct TF ... failed. Fallback: Map->InsLink(dynamic) * Lidar->InsLink(static/default) used.`
- `Waiting for live TF ... Using default meanwhile.`

### 4.2 `target_accumulate_pc_num` 过大

代码位置：

- `automatic_annotation/include/automatic_annotation/gen_prompt_point.hpp:480`

行为：

- 通过滑动窗口保留最近 `target_accumulate_pc_num` 帧。

后果：

- 窗口越大，对定位精度和 TF 连续性的要求越高。
- 如果车辆短时间内姿态变化较大，哪怕每帧只有很小误差，累计后也可能被肉眼看成双层轮廓。

实务判断：

- 如果把 `target_accumulate_pc_num` 临时调小后重影明显减轻，优先排查位姿链路，而不是先改投影算法。

### 4.3 `WaitForSyncedPointCloud()` 的取云策略

代码位置：

- `automatic_annotation/src/gen_prompt_point.cpp:1647`

行为：

- 如果累计点云时间已经追上或越过图像时间，直接接受。
- 如果还没追上，但时间差小于阈值 `kLidarToCameraMaxTimeDiff`，也接受。

后果：

- 这不是严格的“同时间戳匹配”，而是“误差可接受就拿最近可用点云”。
- 当图像和点云相位关系变化、回放节奏波动、缓存更新稍慢时，容易出现偶发轻微偏差。
- 单帧看不明显，但在静态物体边缘上会表现为拖尾或双边。

重点日志：

- `Sync Success (Crossed). Diff: ...`
- `Sync Success (Caught early). Diff: ...`
- `Time sync timeout.`

### 4.4 `GetTimeInterpolatedTransform()` 插值失败回退到 `Identity`

代码位置：

- `automatic_annotation/src/gen_prompt_point.cpp:1876`
- `automatic_annotation/src/gen_prompt_point.cpp:1905`

行为：

- 查询 `lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame, ...)`
- 失败后回退到传入的 `fallback_extrinsic`

当前调用处：

- `HandleLeftImageMsg()` 里传入的是 `Eigen::Isometry3d::Identity()`。

后果：

- 一旦 time-travel TF 失败，就相当于默认 `Lidar(t_lidar)` 和 `Lidar(t_img)` 完全一致。
- 对于静止车辆或极小时间差，这可能勉强可接受。
- 但车辆有运动、或者图像和累计点云存在可见时间差时，这会直接导致补偿缺失。

重点日志：

- `Time interpolation TF failed: ... Fallback to static extrinsic.`

### 4.5 `CheckTfCallback()` 中 live TF 与 default 外参切换

代码位置：

- `automatic_annotation/src/gen_prompt_point.cpp:1092`

行为：

- 启动初期不断尝试获取相机和雷达相关静态 TF。
- 没查到前，节点会继续使用默认外参。
- 查到 live TF 后，才切换到真实 TF，并停止定时检查。

后果：

- 如果某次回放中 TF 发布慢于节点启动，前面几帧可能已经用了默认外参。
- 如果只在部分 bag 出现重影，这类“启动顺序/时序竞争”要重点怀疑。

重点日志：

- `Waiting for live TF ... Using default meanwhile.`
- `Real Transform found ... Switching to live TF!`
- `All required static TF relationships are ready.`

## 5. 现场排查步骤

建议按“先外部、后内部；先确认时间、再确认空间”的顺序排查。

### 5.1 先看保存的 `aligned_lidar_cloud` 是否已经重影

目标：

- 判断问题是在累计/补偿链路，还是在后续相机投影链路。

操作：

1. 打开节点保存的同步点云 `aligned_lidar_cloud` 对应的 `.pcd`。
2. 观察静态墙体、路边缘、固定障碍物轮廓。

判断：

- 如果这里已经重影，说明问题发生在累计点云生成或图像时刻补偿之前。
- 如果这里不重影，但投影图重影，优先查相机外参、投影或验证图链路。

### 5.2 再看 `debug/accumulated_pc` 是否已经重影

目标：

- 把问题进一步缩小到“累计阶段”还是“图像时刻补偿阶段”。

判断：

- `debug/accumulated_pc` 就重影：
  优先查 `ins_map / ins_link / lidar_link` TF、累计窗口大小、输入点云质量。
- `debug/accumulated_pc` 正常，但 `aligned_lidar_cloud` 才重影：
  优先查 `WaitForSyncedPointCloud()` 和 `GetTimeInterpolatedTransform()`。

### 5.3 检查 TF 是在用 live TF 还是 default fallback

重点看日志中是否频繁出现：

- `Waiting for live TF`
- `Direct TF ... failed. Fallback`

判断：

- 如果节点长时间停留在 default fallback，重影优先怀疑外参来源不准。
- 如果启动前几帧 fallback、后面切到 live TF，问题可能只出现在流程前段。

### 5.4 检查 `Sync Success (Crossed)` / `Caught early` 的时间差

目标：

- 判断图像是否经常拿到“误差刚好达标”的点云，而不是更理想的同相位点云。

判断：

- 偶发的小时间差通常问题不大。
- 如果频繁出现接近阈值的时间差，或者在异常 bag 中显著偏大，优先排查回放节奏、QoS 和 `/clock`。

### 5.5 检查 `Time interpolation TF failed` 是否频繁出现

目标：

- 判断图像时刻补偿是否经常失效。

判断：

- 一旦频繁出现这条日志，说明 `Lidar(t_lidar) -> Lidar(t_img)` 的补偿经常回退到 `Identity`。
- 这类问题通常会直接表现为静态物体在图像侧“轻微拖尾”。

### 5.6 检查 bag 回放、`/clock`、TF 频率、QoS、`frame_id`

这是最容易被忽视但最常见的一组外部问题。

检查点：

- `/clock` 是否正常递增。
- 点云和图像频率是否稳定。
- TF 发布频率是否足够支撑 time-travel 查询。
- 点云消息 `header.frame_id` 是否与 TF 树一致。
- bag 回放时 QoS 是否与订阅侧匹配。

### 5.7 面对 4 个常见现象时的判断

#### 现象 A：`debug/accumulated_pc` 就已重影

优先怀疑：

- `ins_map <- lidar_link` 变换不准
- fallback 外参不准
- 累计窗口过大
- 输入点云本身带畸变

#### 现象 B：`aligned_lidar_cloud` 才开始重影

优先怀疑：

- 图像回调取到的累计点云时刻不理想
- time-travel TF 失败导致补偿丢失

#### 现象 C：仅投影图重影，点云本身不重影

优先怀疑：

- 相机外参
- 投影链路
- 验证图使用的图像时刻或相机参数

#### 现象 D：某些 bag 重影、某些 bag 不重影

优先怀疑：

- bag 本身的 TF 质量差异
- 回放速率或 `/clock` 差异
- 节点启动与 TF 就绪先后顺序

## 6. 建议采集的证据与命令

以下命令尽量使用标准 `ros2` / `tf2_ros` 工具，不依赖仓库外自定义脚本。

### 6.1 时钟与话题频率

```bash
ros2 topic echo --once /clock
ros2 topic hz <pointcloud_topic>
ros2 topic hz <image_topic>
```

用途：

- 验证 `/clock` 是否存在且正常推进。
- 验证点云和图像频率是否稳定，是否存在明显抖动。

### 6.2 TF 查询

```bash
ros2 run tf2_ros tf2_echo ins_map lidar_link
ros2 run tf2_ros tf2_echo ins_link lidar_link
```

用途：

- 检查 `ins_map <- lidar_link` 是否能稳定获取。
- 检查 `ins_link <- lidar_link` 是否与配置默认外参一致或接近。

如果还需要验证相机外参，可补充：

```bash
ros2 run tf2_ros tf2_echo lidar_link camera_infra1_optical_frame
```

### 6.3 日志关键字

建议从日志中 grep 以下关键字：

```bash
grep -E "Fallback|Sync Success|Time interpolation TF failed|Waiting for live TF" <your_log_file>
```

重点看：

- `Fallback`
- `Sync Success`
- `Time interpolation TF failed`
- `Waiting for live TF`

### 6.4 建议同时保存的证据

每次排查至少保留以下材料：

- 一段出现重影的原始 bag 名称
- 对应时段的节点日志
- 一份 `aligned_lidar_cloud` 或保存下来的同步 `.pcd`
- 一张 `debug/accumulated_pc` 可视化截图
- 一张投影验证图或深度融合图

## 7. 排查结论模板

建议用下面的模板记录每次排查结果，方便后续复盘。

```text
问题现象：
- 哪个 bag / 哪个场景 / 哪类静态障碍物出现重影
- 现象是点云双轮廓、拖尾，还是仅投影重影

证据：
- aligned_lidar_cloud 是否重影
- debug/accumulated_pc 是否重影
- 是否出现 Fallback / Waiting for live TF / Time interpolation TF failed
- Sync Success 的时间差大致范围
- /clock、pointcloud、image、TF 频率是否正常

判断：
- 更像累计阶段问题 / 图像时刻补偿问题 / 相机投影问题 / 运行时 TF 问题

下一步动作：
- 修 TF 发布链路
- 修默认外参
- 降低 target_accumulate_pc_num 做 A/B 对比
- 检查 bag 回放速率、QoS、/clock
- 单独验证相机外参与投影链路
```

### 常见结论与处理建议对照表

| 结论 | 常见证据 | 优先处理建议 |
| --- | --- | --- |
| live TF 未就绪 | 日志频繁出现 `Waiting for live TF` | 先修 TF 发布链路，再看是否仍重影 |
| direct TF 经常 fallback | 日志频繁出现 `Fallback` | 校验 `ins_link <- lidar_link` 默认外参和运行时静态 TF |
| time interpolation 频繁失败 | 日志频繁出现 `Time interpolation TF failed` | 查 `/clock`、TF 缓冲、bag 回放速率、TF 发布频率 |
| 累计窗口过大放大误差 | `debug/accumulated_pc` 重影，调小窗口后改善 | 临时减小 `target_accumulate_pc_num` 做 A/B 对比 |
| 输入点云状态与配置不匹配 | 单帧就有拉伸或噪声异常 | 核对 `point_cloud_type`、去畸变前置链路和消息来源 |
| 仅投影图重影 | 点云正常，图像验证异常 | 优先检查相机外参、相机内参与投影链路 |

## 附：本分析对应的关键实现位置

- 点云累计：`automatic_annotation/include/automatic_annotation/gen_prompt_point.hpp:393`
- 图像侧取累计点云并二次补偿：`automatic_annotation/src/gen_prompt_point.cpp:573`
- 静态 TF / 默认外参降级：`automatic_annotation/src/gen_prompt_point.cpp:1092`
- 图像等待同步点云：`automatic_annotation/src/gen_prompt_point.cpp:1647`
- 时序插值补偿：`automatic_annotation/src/gen_prompt_point.cpp:1876`
