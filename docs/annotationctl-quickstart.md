# annotationctl 使用说明

本文档对应当前已经落地的固定工作流：

1. 准备 `job.yaml`
2. 运行 `annotationctl submit`
3. 在 Xtreme1 中人工微调与补标动态障碍物
4. 运行 `annotationctl finalize`
5. 获取最终交付 KITTI 数据集

## 1. 运行环境

- 已完成 `3D_Detection_Annotation` 工作区编译
- 服务器或本机可访问 Xtreme1
- 预标注所需地图、XML、外参配置已准备完成
- 运行前导出 Xtreme Token：

```bash
export XTREME1_TOKEN=<your_token>
```

## 2. job.yaml 最小示例

```yaml
job_name: baoli_debug_resubmit
location: BaoLi_20260305_Debug

input:
  source_dir: /home/keyaoli/Data/AutoAnnotation/Wayrobo_Rosbag/BaoLi20260305_Debug
  recursive: false
  overwrite_extract: false

workspace:
  root_dir: /tmp/annotationctl_baoli_debug_resubmit

auto_annotation:
  base_yaml_path: /home/keyaoli/Code/Wayrobo/3D_Detection_Annotation/automatic_annotation/config/default.yaml
  overrides:
    global_pc_map_addr: /home/keyaoli/Data/AutoAnnotation/MapAnnotation/bl_20260317/velodyne_points/data/colored_lidar_merged.pcd
    pc_annotation_file_addr: /home/keyaoli/Data/AutoAnnotation/MapAnnotation/bl_20260317/tracklet_labels.xml
    default_tf_lidar_to_ins: [-0.347, 0.000, 1.277, -0.981, -0.378, -1.315]
    default_tf_lcam_to_lidar: [0.070, 0.116, 0.071, -126.100, -1.000, -91.300]

xtreme:
  base_url: http://localhost:8190
  token_env: XTREME1_TOKEN
  dataset_name: BaoLi_20260305_Debug_Resubmit_20260416
  dataset_type: lidar_fusion

output:
  final_dataset_dir: /tmp/annotationctl_baoli_debug_resubmit/final_delivery
```

说明：

- `auto_annotation.base_yaml_path` 是模板 yaml
- `auto_annotation.overrides` 会生成到 job 私有 runtime yaml，而不会改仓库里的 `default.yaml`
- `xtreme.token_env` 只写环境变量名，不在 `job.yaml` 中写明文 token

## 3. 提交任务

```bash
conda run -n nusc_env python pre_annotation_factory/scripts/annotationctl.py submit /path/to/job.yaml
```

成功后会输出：

```text
job_id=<job_id>
status=waiting_human_annotation
```

关键产物会出现在：

```text
<workspace.root_dir>/jobs/<job_id>/
```

重点关注：

- `state.json`
- `artifacts/xtreme_upload/*.zip`
- `artifacts/raw_origin/`
- `runtime/auto_annotation.yaml`

## 4. 查看状态

```bash
conda run -n nusc_env python pre_annotation_factory/scripts/annotationctl.py status <job_id> --workspace <workspace.root_dir>
```

## 5. Xtreme1 人工标注

在 Xtreme1 中打开本次 submit 创建的数据集，完成人工微调与动态障碍物标注。

当前自动上传会按 UI 中以下等价设置导入：

- `Data Format = Xtreme1`
- `Contains annotation result = true`
- `Result Type = Ground Truth`

## 6. 收尾生成最终交付数据集

```bash
conda run -n nusc_env python pre_annotation_factory/scripts/annotationctl.py finalize <job_id> --workspace <workspace.root_dir>
```

成功后会输出：

```text
job_id=<job_id>
status=completed
```

最终产物位于：

- `artifacts/final_dataset/<dataset_name>/`
- `artifacts/final_dataset/<dataset_name>.zip`

同时 `state.json` 会记录：

- `xtreme.export_serial_number`
- `paths.normalized_export_dir`
- `paths.final_dataset_dir`
- `paths.final_zip_path`

## 7. 常见排查点

### 7.1 submit 后 Xtreme 中没有预标注框

先检查：

- `artifacts/xtreme_upload/Scene_XX/result/*.json` 是否非空
- 自动上传是否带了：
  - `dataFormat=XTREME1`
  - `resultType=GROUND_TRUTH`

### 7.2 submit 后全是空帧

优先检查 `job.yaml` 中的 `auto_annotation.overrides`：

- `global_pc_map_addr`
- `pc_annotation_file_addr`
- `default_tf_lidar_to_ins`
- `default_tf_lcam_to_lidar`

### 7.3 finalize 失败

优先检查：

- Xtreme1 导出是否成功
- `artifacts/xtreme_export/normalized/Scene_XX/data/*.json` 是否存在
- `state.json` 中的 `last_error`

## 8. 多任务 manifest 管理建议

建议每一组数据使用一份新的 `job.yaml`，不要直接修改上一组已经提交过的 manifest 反复复用。

原因：

- 一个 `job.yaml` 对应一组独立输入、一套配置和一份独立产物
- `submit` 后原始 manifest 会被复制到 `jobs/<job_id>/job.yaml`
- 后续追溯某个 job 当时使用的 bag、地图、XML、外参时，会更清晰

推荐做法：

- 为每组数据单独保存一个 manifest 文件
- 复用上一份作为模板，只修改会随任务变化的字段

通常需要修改：

- `job_name`
- `location`
- `input.source_dir`
- `workspace.root_dir`
- `xtreme.dataset_name`
- 如果场景不同，再修改 `auto_annotation.overrides`

推荐命名方式例如：

- `jobs/baoli_20260305_debug.yaml`
- `jobs/yenan_20260318.yaml`
- `jobs/xihu_20260330_debug.yaml`

如果只是同一批数据重复试跑，也建议新建 manifest 或至少修改 `xtreme.dataset_name` 和 `workspace.root_dir`，避免和历史 job 混淆。
