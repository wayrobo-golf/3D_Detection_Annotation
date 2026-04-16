# Xtreme 服务器固化工作流设计文档

日期：2026-04-16

## 1. 背景

当前 `3D_Detection_Annotation` 的实际生产流程已经基本固定：

1. 解压 bag 数据；
2. 回放 rosbag，触发 `automatic_annotation` 生成预标注结果；
3. 固定只生成 Xtreme1 上传包，不再生成中间 KITTI 数据集；
4. 将预标注结果上传到服务器上的 Xtreme1；
5. 在 Xtreme1 中人工微调预标注结果，并补充动态障碍物；
6. 标注完成后，手动执行一条命令；
7. 系统自动从 Xtreme1 导出标注结果；
8. 使用导出结果与原始归档数据融合，生成最终交付 KITTI 数据集。

其中，第 1 到第 5 步的业务逻辑保持不变；第 6 到第 8 步改造成可在服务器上稳定复用的固定流程。原流程中的人工校准工具链不再使用。`replay_rosbag_main.py` 中的 `GENERATE_KITTI_DATASET = False` 也不再作为临时开关，而是成为服务器工作流的固定约束。

本设计的目标不是重写现有标注算法，而是将现有脚本参数化、编排化、状态化，让用户不再需要逐个修改脚本常量、逐个手工运行脚本。

## 2. 目标与非目标

### 2.1 目标

- 将现有固定工作流部署在服务器上，以命令行方式稳定运行。
- 将“准备数据并提交到 Xtreme1”收敛为一次命令。
- 将“人工标注完成后导出并生成最终交付数据集”收敛为一次命令。
- 去除当前脚本中的硬编码目录切换和手工改常量操作，改为任务清单驱动。
- 为每个任务保存完整状态，支持查询和失败后恢复。
- 复用现有 `extract_archives.py`、`replay_rosbag_main.py`、`MergeXtremeDynamic.py` 的核心业务逻辑，避免重写几何转换链路。

### 2.2 非目标

- 本阶段不实现 watcher、目录监听、守护进程。
- 本阶段不实现 Web UI。
- 本阶段不自动判断 Xtreme1 中的标注是否“已完成”，仍然由用户手动执行收尾命令。
- 本阶段不替换现有 C++ 自动预标注节点。
- 本阶段不引入数据库，状态持久化优先采用文件系统。

## 3. 用户视角下的新工作流

### 3.1 入口命令

设计一个统一命令行入口，暂定名称为 `annotationctl`。

本阶段提供 3 个子命令：

- `annotationctl submit /path/to/job.yaml`
- `annotationctl status <job_id>`
- `annotationctl finalize <job_id>`

### 3.2 用户体验

用户只需要：

1. 准备一个 `job.yaml`；
2. 执行 `annotationctl submit job.yaml`；
3. 等待系统自动解压、回放、生成上传包并上传到 Xtreme1；
4. 在 Xtreme1 中完成人工标注；
5. 执行 `annotationctl finalize <job_id>`；
6. 系统自动导出结果并生成最终交付数据集。

用户不再需要：

- 手动修改 `replay_rosbag_main.py` 中的路径常量；
- 手动切换 `GENERATE_KITTI_DATASET`；
- 手动执行上传 Xtreme1 的脚本；
- 手动下载、整理、解压导出结果；
- 手动给 `MergeXtremeDynamic.py` 改输入输出根目录。

## 4. 方案总览

### 4.1 核心思路

采用“任务清单 + 两阶段命令 + 文件系统状态机”的方案。

每一个标注任务对应一个独立的 `job_id` 和一个独立的工作目录。系统围绕这个工作目录组织所有输入、产物、日志和状态。

整体结构分为四层：

1. `CLI 层`
   负责解析命令、读取 `job.yaml`、输出用户可读状态。
2. `Pipeline Runner`
   负责编排“解压、回放、上传、导出、最终合并”等阶段，不直接承载业务几何逻辑。
3. `Xtreme Gateway`
   负责与 Xtreme1 API 通信，包括创建数据集、上传压缩包、轮询导入进度、触发导出、下载导出结果。
4. `Legacy Adapters`
   负责把现有脚本改造成可参数化调用的模块接口。

### 4.2 为什么不直接重写

当前仓库最敏感的部分是坐标系转换和 KITTI/Xtreme 互转逻辑，分别散落在：

- `automatic_annotation/src/gen_prompt_point.cpp`
- `pre_annotation_factory/scripts/replay_rosbag_main.py`
- `pre_annotation_factory/scripts/MergeXtremeDynamic.py`

这些逻辑已经与现有产物格式强绑定。此阶段直接重写会引入很高的回归风险。因此设计上只做“编排重构”和“参数入口统一”，不做“算法重写”。

## 5. 任务模型

### 5.1 `job.yaml` 结构

建议 `job.yaml` 最小字段如下：

```yaml
job_name: yinxiu_20260414_batch01
location: YinXiu_20260414

input:
  source_dir: /data/rosbags/YinXiu20260414/20260414
  recursive: false
  overwrite_extract: false

workspace:
  root_dir: /data/annotation_jobs

xtreme:
  base_url: http://127.0.0.1:8190
  token_env: XTREME1_TOKEN
  dataset_name: Xtreme1_Upload_YinXiu_20260414_batch01
  dataset_type: lidar_fusion

output:
  final_dataset_dir: /data/final_delivery
```

### 5.2 字段说明

- `job_name`
  人类可读的任务名，用于日志与目录命名。
- `location`
  最终数据集命名用的地点与时间标识，沿用现有脚本语义。
- `input.source_dir`
  bag 或压缩包所在目录。
- `workspace.root_dir`
  所有 job 的根目录。
- `xtreme.base_url`
  服务器上的 Xtreme1 API 根地址。
- `xtreme.token_env`
  读取 Bearer Token 的环境变量名，避免明文写入任务文件。
- `xtreme.dataset_name`
  在 Xtreme1 中创建的数据集名称。
- `output.final_dataset_dir`
  最终交付 KITTI 数据集 ZIP 与解包目录的输出根目录。

### 5.3 任务工作目录

每个 job 的目录结构建议如下：

```text
<workspace.root_dir>/
  jobs/
    <job_id>/
      job.yaml
      state.json
      logs/
        submit.log
        finalize.log
      artifacts/
        extracted_bags/
        raw_origin/
        xtreme_upload/
        xtreme_export/
        final_dataset/
```

其中：

- `extracted_bags/` 存放解压后的 bag；
- `raw_origin/` 存放自动预标注归档后的 `_origin` 数据；
- `xtreme_upload/` 存放待上传到 Xtreme1 的压缩包及其解包目录；
- `xtreme_export/` 存放从 Xtreme1 导出的压缩包及其解包结果；
- `final_dataset/` 存放最终 KITTI 数据集与 ZIP。

## 6. 状态机设计

### 6.1 状态定义

`state.json` 中维护一个单任务状态机，建议主状态如下：

- `created`
- `extracting`
- `replaying`
- `building_xtreme_upload`
- `uploading_xtreme`
- `waiting_human_annotation`
- `exporting_xtreme`
- `merging_final_dataset`
- `completed`
- `failed`

### 6.2 状态切换

`submit` 阶段：

- `created -> extracting`
- `extracting -> replaying`
- `replaying -> building_xtreme_upload`
- `building_xtreme_upload -> uploading_xtreme`
- `uploading_xtreme -> waiting_human_annotation`

`finalize` 阶段：

- `waiting_human_annotation -> exporting_xtreme`
- `exporting_xtreme -> merging_final_dataset`
- `merging_final_dataset -> completed`

失败时：

- 任意状态 -> `failed`

### 6.3 恢复规则

- `submit` 如果发现任务已经处于 `waiting_human_annotation` 或之后的状态，默认拒绝重复执行。
- `finalize` 仅允许从 `waiting_human_annotation` 或 `failed` 且失败点位于收尾阶段时恢复。
- 每个阶段完成后立即刷新 `state.json`，写入产物路径和外部资源 ID。
- `state.json` 至少要记录：
  - `job_id`
  - `status`
  - `current_step`
  - `last_error`
  - `xtreme.dataset_id`
  - `xtreme.import_task_serial`
  - `xtreme.export_task_id`
  - 各阶段产物路径

## 7. 命令设计

### 7.1 `submit`

`annotationctl submit /path/to/job.yaml`

职责：

1. 校验配置文件；
2. 创建 `job_id` 和工作目录；
3. 解压 bag；
4. 调用自动预标注流水线；
5. 固定生成 Xtreme1 上传包；
6. 自动上传到 Xtreme1；
7. 任务进入 `waiting_human_annotation`。

输出：

- `job_id`
- Xtreme1 `dataset_id`
- Xtreme1 数据集名称
- 当前状态与产物目录

### 7.2 `status`

`annotationctl status <job_id>`

职责：

- 读取 `state.json`；
- 输出当前状态、最近错误、Xtreme1 dataset/task 信息、关键产物路径。

本阶段 `status` 不主动查询 Xtreme1 中“是否标注完成”，只报告本地状态和已记录的远端对象标识。

### 7.3 `finalize`

`annotationctl finalize <job_id>`

职责：

1. 调用 Xtreme1 导出接口；
2. 下载导出结果；
3. 解压整理为 `MergeXtremeDynamic.py` 所需目录结构；
4. 运行最终融合；
5. 生成最终 KITTI 数据集与 ZIP；
6. 状态切换到 `completed`。

## 8. 与现有脚本的衔接方式

### 8.1 `extract_archives.py`

现状：

- 已具备安全解压能力；
- 当前通过文件顶部常量和环境变量驱动。

改造方向：

- 保留现有解压逻辑；
- 提炼出可导入函数，例如 `extract_archives(source_dir, output_dir, recursive, overwrite, remove_archive)`；
- CLI 层直接调用该函数。

### 8.2 `replay_rosbag_main.py`

现状：

- 通过大量模块级常量配置输入输出路径；
- `GENERATE_KITTI_DATASET` 已被业务固定为 `False`；
- 脚本兼做“回放 orchestrator”和“后处理 builder”。

改造方向：

将其拆成两个可调用入口：

1. `run_auto_annotation_job(config)`
   负责 `get_bags_to_process -> process_bag`
2. `build_xtreme_upload_bundle(config)`
   负责现有 `build_datasets` 的 Xtreme 上传包构建路径

约束：

- `GENERATE_KITTI_DATASET` 在新入口内强制固定为 `False`；
- 上传包输出目录由 job 工作目录决定；
- 原始归档目录由 job 工作目录决定；
- 不再允许从模块常量读取生产路径。

### 8.3 `MergeXtremeDynamic.py`

现状：

- 依赖 `XTREME_EXPORT_ROOT`、`RAW_ARCHIVE_ROOT`、`FINAL_KITTI_OUTPUT_DIR` 等模块级常量；
- 已具备当前业务真正需要的“人工判空覆盖原始标注”的规则；
- 已有针对空帧规则的测试。

改造方向：

保留核心算法与业务规则，仅提炼为：

```python
build_final_dataset(
    location_str,
    xtreme_export_root,
    raw_archive_root,
    final_kitti_output_dir,
    split_ratio=0.8,
    convert_pcd_to_bin=True,
    random_seed=42,
    max_empty_label_ratio=0.10,
)
```

这样 `finalize` 只需要把任务目录中的路径注入进去即可。

## 9. Xtreme1 对接设计

### 9.1 官方能力边界

根据官方文档，Xtreme1 提供了：

- Bearer Token 认证；
- 上传数据集压缩包；
- 查询压缩包处理进度；
- 导出标注结果；
- 查询结果相关 API。

由于服务器部署的是 `v0.6`，实现时应以该版本实例实际接口为准。设计层不直接绑定某个硬编码 endpoint 字符串，而是统一封装在 `Xtreme Gateway` 中。

### 9.2 `Xtreme Gateway` 的最小接口

建议封装为以下接口：

- `create_or_get_dataset(name, dataset_type) -> dataset_id`
- `request_upload_url(filename) -> signed_upload_url, access_url`
- `upload_archive(signed_upload_url, local_archive_path)`
- `import_archive(dataset_id, access_url) -> import_task_serial`
- `wait_import_done(dataset_id, import_task_serial)`
- `request_export(dataset_id) -> export_task_id`
- `wait_export_done(export_task_id) -> download_url`
- `download_export(download_url, local_archive_path)`

### 9.3 上传输入

上传输入优先使用 `replay_rosbag_main.py` 已生成的 Xtreme1 ZIP 包，而不是重新拼装上传包。

原因：

- 当前 ZIP 结构已经与现有业务兼容；
- 减少重复实现；
- 降低“上传包结构与当前 Xtreme 解析结构偏离”的风险。

### 9.4 导出输出

导出结果下载后，统一解压到：

```text
jobs/<job_id>/artifacts/xtreme_export/
```

并保证目录结构满足 `MergeXtremeDynamic.py` 的输入假设，即：

- `Scene_XX/result/<frame_id>.json`

如 Xtreme1 的导出压缩包外层多包了一层目录，则由 `Xtreme Gateway` 或导出整理步骤负责剥离。

## 10. 产物与目录约束

### 10.1 原始归档

`submit` 完成后，必须留下两类核心资产：

- 自动预标注原始归档 `_origin`
- Xtreme1 上传包 ZIP

即使后续 `finalize` 失败，也不能清理这两类资产。

### 10.2 最终交付

`completed` 状态下至少应产生：

- 最终 KITTI 数据集目录
- 最终 KITTI ZIP
- `nusc_meta/`
- 对应的本地导出结果目录

### 10.3 日志

每个阶段日志分文件保存，不仅输出到终端：

- `logs/submit.log`
- `logs/finalize.log`

如需更细粒度，可增加阶段子日志，但本阶段不强制。

## 11. 错误处理

### 11.1 设计原则

- 不隐藏失败。
- 不自动删除失败现场。
- 优先保留可恢复状态。

### 11.2 典型失败点

- 解压失败；
- ROS 2 节点启动失败；
- bag 回放失败；
- Xtreme1 上传 URL 申请失败；
- Xtreme1 导入超时；
- Xtreme1 导出失败；
- 导出下载成功但目录结构不匹配；
- 最终 merge 阶段发现无正样本，触发 `zero positive frames`。

### 11.3 错误记录

`state.json` 中保存：

- `failed_step`
- `error_type`
- `error_message`
- `failed_at`

同时日志文件中保留完整堆栈或命令输出。

## 12. 安全与配置约束

- Xtreme1 Token 不写入 `job.yaml` 和 `state.json`，仅通过环境变量读取。
- 工作流内部所有外部路径都来自 `job.yaml` 或系统默认配置，不再从脚本源码读取。
- 不把服务器本地绝对路径硬编码回仓库中的业务脚本。

## 13. 测试策略

### 13.1 单元测试

新增单元测试应覆盖：

- `job.yaml` 解析与校验；
- 状态机切换；
- 任务目录生成；
- `Xtreme Gateway` 的请求参数拼装；
- `finalize` 对导出目录结构的整理逻辑。

### 13.2 回归测试

保留并继续运行现有：

- `pre_annotation_factory/tests/test_merge_xtreme_dynamic.py`

这是当前“空帧判空覆盖”和“空样本比例控制”业务规则的核心回归保护。

### 13.3 集成测试

集成测试分为两档：

1. 本地假服务模式
   使用 mock Xtreme API 验证 `submit/finalize` 编排链路。
2. 真实服务器联调
   使用小规模测试包验证上传、导出、最终 merge 全链路。

## 14. 实施边界

本次设计对应的实现范围为：

1. 新增 `annotationctl` CLI；
2. 新增任务清单模型与状态文件；
3. 将 `extract_archives.py`、`replay_rosbag_main.py`、`MergeXtremeDynamic.py` 参数化；
4. 新增 `Xtreme Gateway`；
5. 打通 `submit/status/finalize`。

明确不在本次范围内的内容：

- watcher；
- 后台常驻 worker；
- Web 页面；
- 数据库存储；
- 自动判断“标注已完成”。

## 15. 推荐实施顺序

1. 先参数化现有脚本，去掉关键模块级常量依赖；
2. 再实现 job 工作目录与 `state.json`；
3. 再实现 `submit` 的本地流水线；
4. 再接 Xtreme1 上传 API；
5. 最后实现 `finalize` 的导出与最终 merge；
6. 补齐状态查询与失败恢复。

## 16. 结论

本方案以最小改动固化当前已经稳定的业务工作流。其核心不是“做一个新的标注平台”，而是把现有脚本包装成一个可部署、可参数化、可恢复、可查询的服务器命令行流水线。

在不引入 watcher、不引入服务化后台的前提下，`annotationctl submit` 和 `annotationctl finalize` 已足以覆盖当前生产模式，并且为未来扩展 watcher 或更重的服务化方案保留了清晰边界。
