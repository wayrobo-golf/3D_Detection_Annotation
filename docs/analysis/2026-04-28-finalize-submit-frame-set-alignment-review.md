# finalize 与 submit 帧集合口径对齐评审

## 背景

当前 `annotationctl` 工作流分为两段：

- `submit`
  - 从原始自动标注产物中筛出可上传到 Xtreme 的帧
  - 打包 `camera_config / camera_image_0 / lidar_point_cloud_0 / result`
- `finalize`
  - 从 Xtreme 导出结果与原始归档重新拼装最终交付数据集

在最近一次真实任务中：

- `submit` 上传了 `70` 帧
- `finalize` 最终产出了 `71` 帧

这说明两阶段对“候选帧集合”的定义并不一致。

## 现状

### submit 当前口径

`submit` 阶段在 [replay_rosbag_main.py](../../pre_annotation_factory/scripts/replay_rosbag_main.py) 中扫描原始自动标注产物时，要求一帧必须同时具备：

- `camera_config/<frame_id>.json`
- `camera_image_0/<frame_id>.*`
- `lidar_point_cloud_0/<frame_id>.pcd`
- `label_2/<frame_id>.txt`

只有满足上述条件的帧，才会进入：

- 有效目标帧 `valid_tasks`
- 原始空帧 `empty_tasks`

因此，`submit` 的上传集本质上是：

> “已经完成自动标注并生成了 `label_2` 的帧集合”

### finalize 当前口径

`finalize` 阶段在 [MergeXtremeDynamic.py](../../pre_annotation_factory/scripts/MergeXtremeDynamic.py) 中重新扫描 `raw_origin` 时，只要求一帧具备：

- `camera_config/<frame_id>.json`
- `camera_image_0/<frame_id>.*`
- `lidar_point_cloud_0/<frame_id>.pcd`

它不要求 `label_2/<frame_id>.txt` 存在。

然后再根据 Xtreme 导出目录判断这一帧属于：

- `xtreme`
- `empty_missing_json`
- `empty_missing_scene`

因此，`finalize` 的候选集本质上是：

> “只要自动标注阶段留下了图像、点云、配置，就重新纳入最终交付候选”

## 已观测到的问题

### 1. submit 与 finalize 的帧集合不一致

在真实任务 `job_id=ca25026ff958` 中：

- `submit` 上传 `70` 帧
- `finalize` 扫描到 `72` 帧候选
- 最终保留 `71` 帧

差值来源不是 Xtreme 额外新增了数据，而是：

- 有 `2` 帧在 `submit` 时由于缺少 `label_2` 没有上传
- 但 `finalize` 重新扫描 `raw_origin` 时又把这两帧纳入候选

### 2. 当前 `missing json` 语义被污染

当前实现中，`missing json` 的真实语义不是单一的“导出缺失”，而是混合了两类情况：

1. 这帧确实上传到了 Xtreme，但导出目录中没有对应 `result/<frame_id>.json`
2. 这帧根本没有上传到 Xtreme，但因为 `finalize` 重新扫描原始归档，又被按“找不到 json”归入 `missing json`

第二类会让 `missing json` 的名字与实际行为不一致。

### 3. 未上传帧会污染最终交付集

这次任务中，那两帧未上传但被 `finalize` 重新纳入的帧还带来了后续副作用：

- 它们因为缺 TF，当初没有生成 `label_2`
- 其中至少一帧最终 `calib` 中也缺少 `Tr_velo_to_map`
- 在生成 `nusc_meta/sample.json` 时触发：
  - `Missing Tr_velo_to_map in calib`

也就是说，当前宽口径不只是“多两帧”，还会把未进入人工精修闭环的异常帧重新带入最终交付。

## 规则合理性分析

### 当前宽口径的合理之处

从“尽量不丢帧”的离线数据整合器角度看，当前宽口径有一个优点：

- 即便 Xtreme 某些导出结果缺失，也能基于原始归档把帧按空标签兜底回来

这对“最大化保留数据”是有帮助的。

### 当前宽口径的问题

对于本项目的工作流语义：

> 自动预标注 -> 上传 Xtreme -> 人工精修 -> 导出最终交付

更关键的是“闭环一致性”，而不是“尽量多捞帧”。

在这个目标下，当前宽口径的问题更明显：

- 最终交付集不等于人工实际看到的数据集
- 最终交付集可能混入从未上传、从未人工审核的帧
- `missing json` 无法准确区分“导出异常”与“未上传帧回流”
- 下游统计、复盘、质量评估都会被混淆

## 评审结论

本次评审结论如下：

1. 接受将 `finalize` 的候选帧集合收敛到 `submit` 实际上传的帧集合
2. `missing json` 的目标语义收紧为：
   - “这帧在上传清单中，但没有找到对应 Xtreme 导出 json”
3. 不保留旧的宽口径兼容开关

换句话说，后续实现的默认规则应为：

> `finalize` 只处理 `submit` 当时真正上传到 Xtreme 的帧；  
> 只在这批帧内部处理 `missing json`、`empty xtreme json` 等容错分支。

## 目标行为

优化后的行为应满足：

- 上传 `70` 帧，则 `finalize` 的候选上限就是 `70` 帧
- 最终交付帧数可以小于等于 `70`
- 最终交付帧数不能大于 `70`
- `missing json` 只表示：
  - 帧在上传清单中
  - 但在 Xtreme 导出目录中缺少对应 `result/<frame_id>.json`

## 建议实现方案

### 1. submit 阶段落一份上传清单

在 `submit` 打包 Xtreme 上传目录时，额外保存一份 manifest，记录实际上传的帧集合。

建议最小字段：

```json
[
  {
    "scene_name": "Scene_01",
    "frame_id": "1773905257000077056",
    "img_ext": ".jpg"
  }
]
```

这份清单应和 job artifacts 一起保存，供 `finalize` 读取。

### 2. finalize 阶段改为读取上传清单

`MergeXtremeDynamic.py` 不再直接通过重新扫描 `raw_origin` 来扩充候选集，而是：

- 先读取 submit manifest
- 再按 manifest 中的 `scene_name + frame_id` 定位：
  - `config_path`
  - `img_path`
  - `pcd_path`
  - `raw_label_path`
  - `xtreme_json_path`

### 3. 仅在上传清单内部做导出容错

对每一帧的分类规则改为：

- `xtreme`
  - 上传清单中存在，且 `result/<frame_id>.json` 存在
- `empty_missing_json`
  - 上传清单中存在，但 `result/<frame_id>.json` 不存在
- `empty_xtreme_json`
  - 上传清单中存在，`result/<frame_id>.json` 存在，但解析后对象为空

不再允许：

- 未上传帧进入 `empty_missing_json`
- 未上传帧进入最终数据集

## 影响范围

预计涉及文件：

- [pre_annotation_factory/scripts/replay_rosbag_main.py](../../pre_annotation_factory/scripts/replay_rosbag_main.py)
  - `submit` 时生成上传清单
- [pre_annotation_factory/scripts/MergeXtremeDynamic.py](../../pre_annotation_factory/scripts/MergeXtremeDynamic.py)
  - `finalize` 改为读取上传清单而不是扩容扫描
- [pre_annotation_factory/my_package/workflow/pipeline.py](../../pre_annotation_factory/my_package/workflow/pipeline.py)
  - 将上传清单路径传递给 `merge_final_dataset`

## 风险与注意事项

### 风险 1：旧任务没有上传清单

已有老 job 可能没有保存这份 manifest。  
由于本次结论是不保留宽口径兼容开关，因此后续实现需要明确：

- 是直接要求重新 `submit`
- 还是对旧 job 给出明确失败提示

### 风险 2：上传清单与归档文件不一致

理论上同一个 job 的 `submit` 与 `raw_origin` 应一致。  
如果出现 manifest 里有帧、但 `raw_origin` 中缺文件，应该视为数据损坏并明确报错，而不是静默跳过。

## 建议验收标准

实现完成后，至少验证以下场景：

1. `submit` 上传 `N` 帧，`finalize` 最终候选不超过 `N`
2. 上传清单内某帧缺少导出 `json`
   - 这帧应被计为 `missing json`
3. 原始归档中存在额外帧，但不在上传清单中
   - 这帧不得进入最终数据集
4. `nusc_meta` 生成阶段不再因为“未上传异常帧回流”而多出额外噪声问题

## 结论

当前 `finalize` 口径比 `submit` 宽，在“Xtreme 人工精修闭环”这个工作流下不够合理。  
更合理的默认规则是：

- 候选帧集合以上传清单为准
- 容错仅发生在这批已上传帧内部
- 不再把未上传帧回流到最终交付集

这将使 `submit`、`Xtreme`、`finalize` 三个阶段的语义保持一致，也能让 `missing json` 回到它应有的单一含义。
