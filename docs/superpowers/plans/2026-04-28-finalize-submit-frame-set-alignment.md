# Finalize Submit Frame Set Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 `finalize` 的候选帧集合严格等于 `submit` 实际上传到 Xtreme 的帧集合，并把 `missing json` 语义收紧为“已上传但导出缺失”。

**Architecture:** 在 `submit` 阶段生成一份上传清单 manifest，并作为 job artifact 持久化；`finalize` 不再重新扩容扫描 `raw_origin`，而是只读取这份 manifest 来定位原始文件，再在 manifest 内部做 `xtreme / missing json / empty xtreme json` 分类。缺少 manifest 的 job 直接失败，不保留旧宽口径兼容模式。

**Tech Stack:** Python 3, pytest, pathlib, dataclasses, annotationctl workflow pipeline

---

## File Structure

- Modify: `pre_annotation_factory/scripts/replay_rosbag_main.py`
  - 在 `submit` 打包阶段生成上传清单，并把路径放进返回 artifacts
- Modify: `pre_annotation_factory/scripts/MergeXtremeDynamic.py`
  - 新增读取上传清单的入口，按 manifest 构建 `tasks`
- Modify: `pre_annotation_factory/my_package/workflow/pipeline.py`
  - 把上传清单路径存进 `JobState.paths`，并在 `finalize` 时传给 merge
- Modify: `pre_annotation_factory/tests/test_merge_xtreme_dynamic.py`
  - 覆盖“只处理上传清单帧”“missing json 仅限上传清单内部”的行为
- Modify: `pre_annotation_factory/tests/test_annotationctl_pipeline.py`
  - 覆盖 manifest 产出、pipeline 路径传递、缺少 manifest 的失败提示

## Task 1: Lock Desired Finalize Semantics with Failing Tests

**Files:**
- Modify: `pre_annotation_factory/tests/test_merge_xtreme_dynamic.py`
- Test: `pre_annotation_factory/tests/test_merge_xtreme_dynamic.py`

- [ ] **Step 1: Write the failing tests**

在 `pre_annotation_factory/tests/test_merge_xtreme_dynamic.py` 中新增两个测试和一个小 helper。

新增 helper：

```python
def write_submit_manifest(manifest_path: Path, items: list[dict]):
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(items), encoding="utf-8")
```

新增测试 1，验证 `finalize` 不再把未上传帧捞回来：

```python
def test_finalize_uses_submit_manifest_as_frame_universe(tmp_path):
    module = load_merge_module()

    raw_root = tmp_path / "raw"
    xtreme_root = tmp_path / "xtreme"
    output_root = tmp_path / "output"
    scene_dir = raw_root / "data_record_20260414" / "Scene_01"

    create_raw_frame(scene_dir, "uploaded_frame", "raw-label\n")
    create_xtreme_result(xtreme_root, "Scene_01", "uploaded_frame")
    create_raw_frame(scene_dir, "not_uploaded_frame", "raw-label\n")
    create_xtreme_result(xtreme_root, "Scene_01", "not_uploaded_frame")

    manifest_path = tmp_path / "submit_manifest.json"
    write_submit_manifest(
        manifest_path,
        [{"scene_name": "Scene_01", "frame_id": "uploaded_frame", "img_ext": ".png"}],
    )

    stub_external_effects(module, output_root)
    module.MAX_EMPTY_LABEL_RATIO = 1.0
    module.parse_xtreme_to_kitti_lines = lambda *_args, **_kwargs: [
        "Car 0.00 0 0.00 0.00 0.00 10.00 10.00 1.00 1.00 1.00 0.00 0.00 10.00 0.00 0.00 0.00\n"
    ]

    module.build_final_dataset(
        "unit_test",
        xtreme_export_root=xtreme_root,
        raw_archive_root=raw_root,
        final_kitti_output_dir=output_root,
        submit_manifest_path=manifest_path,
        max_empty_label_ratio=1.0,
    )

    label_files = read_output_labels(output_root)
    assert [path.stem for path in label_files] == ["uploaded_frame"]
```

新增测试 2，验证 `missing json` 只发生在 manifest 内部：

```python
def test_finalize_marks_missing_json_only_for_uploaded_frames(tmp_path):
    module = load_merge_module()

    raw_root = tmp_path / "raw"
    xtreme_root = tmp_path / "xtreme"
    output_root = tmp_path / "output"
    scene_dir = raw_root / "data_record_20260414" / "Scene_01"

    create_raw_frame(scene_dir, "uploaded_missing_json", "raw-label\n")
    create_raw_frame(scene_dir, "not_uploaded_missing_json", "raw-label\n")
    (xtreme_root / "Scene_01" / "result").mkdir(parents=True, exist_ok=True)

    manifest_path = tmp_path / "submit_manifest.json"
    write_submit_manifest(
        manifest_path,
        [{"scene_name": "Scene_01", "frame_id": "uploaded_missing_json", "img_ext": ".png"}],
    )

    stub_external_effects(module, output_root)
    module.MAX_EMPTY_LABEL_RATIO = 1.0

    module.build_final_dataset(
        "unit_test",
        xtreme_export_root=xtreme_root,
        raw_archive_root=raw_root,
        final_kitti_output_dir=output_root,
        submit_manifest_path=manifest_path,
        max_empty_label_ratio=1.0,
    )

    label_files = read_output_labels(output_root)
    assert [path.stem for path in label_files] == ["uploaded_missing_json"]
    assert label_files[0].read_text(encoding="utf-8") == ""
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
pytest pre_annotation_factory/tests/test_merge_xtreme_dynamic.py -k "submit_manifest or missing_json_only_for_uploaded" -q
```

Expected:
- FAIL because `build_final_dataset()` does not accept `submit_manifest_path` yet
- Or FAIL because current implementation still scans all raw frames

- [ ] **Step 3: Do not implement yet — only keep the red state**

Expected failure examples:

```text
TypeError: build_final_dataset() got an unexpected keyword argument 'submit_manifest_path'
```

or

```text
AssertionError: assert ['not_uploaded_frame', 'uploaded_frame'] == ['uploaded_frame']
```

- [ ] **Step 4: Commit the red test**

```bash
git add pre_annotation_factory/tests/test_merge_xtreme_dynamic.py
git commit -m "test: define finalize frame universe by submit manifest"
```

## Task 2: Emit Submit Manifest During Xtreme Upload Bundle Build

**Files:**
- Modify: `pre_annotation_factory/scripts/replay_rosbag_main.py`
- Modify: `pre_annotation_factory/tests/test_annotationctl_pipeline.py`
- Test: `pre_annotation_factory/tests/test_annotationctl_pipeline.py`

- [ ] **Step 1: Write the failing test**

在 `pre_annotation_factory/tests/test_annotationctl_pipeline.py` 中新增一个测试，验证 `build_datasets()` 返回上传清单路径。

建议先为 `replay_rosbag_main.py` 新增一个纯函数目标：

```python
def write_submit_frame_manifest(manifest_path: Path, tasks: list[tuple]) -> Path:
    ...
```

新增测试：

```python
def test_write_submit_frame_manifest_records_scene_and_frame(tmp_path):
    module = load_replay_module()
    manifest_path = tmp_path / "submit_frames.json"
    tasks = [
        ("1773905257000077056", "cfg.json", "label.txt", "pcd.pcd", "img.jpg", "Scene_01", "/tmp/Scene_01"),
        ("1773905402200267008", "cfg2.json", "label2.txt", "pcd2.pcd", "img2.png", "Scene_02", "/tmp/Scene_02"),
    ]

    result = module.write_submit_frame_manifest(manifest_path, tasks)

    assert result == manifest_path
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert payload == [
        {"scene_name": "Scene_01", "frame_id": "1773905257000077056", "img_ext": ".jpg"},
        {"scene_name": "Scene_02", "frame_id": "1773905402200267008", "img_ext": ".png"},
    ]
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
pytest pre_annotation_factory/tests/test_annotationctl_pipeline.py -k submit_frame_manifest -q
```

Expected:

```text
AttributeError: module 'replay_rosbag_main' has no attribute 'write_submit_frame_manifest'
```

- [ ] **Step 3: Write minimal implementation**

在 `pre_annotation_factory/scripts/replay_rosbag_main.py` 中新增：

```python
def write_submit_frame_manifest(manifest_path: Path, tasks):
    payload = []
    for frame_id, _config_file, _label_path, _pcd_path, img_path, scene_name, _scene_dir in tasks:
        payload.append(
            {
                "scene_name": scene_name,
                "frame_id": frame_id,
                "img_ext": Path(img_path).suffix,
            }
        )
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return manifest_path
```

并在 `build_datasets()` 中，在 `tasks` 确认后、打包前生成：

```python
submit_manifest_path = Path(xtreme1_root_dir) / "submit_frames.json"
write_submit_frame_manifest(submit_manifest_path, tasks)
```

并把返回值增加：

```python
"submit_manifest_path": submit_manifest_path,
```

- [ ] **Step 4: Run test to verify it passes**

Run:

```bash
pytest pre_annotation_factory/tests/test_annotationctl_pipeline.py -k submit_frame_manifest -q
```

Expected:

```text
1 passed
```

- [ ] **Step 5: Commit**

```bash
git add pre_annotation_factory/scripts/replay_rosbag_main.py \
        pre_annotation_factory/tests/test_annotationctl_pipeline.py
git commit -m "feat: emit submit frame manifest for xtreme uploads"
```

## Task 3: Plumb Submit Manifest Through Workflow State and Finalize Entry

**Files:**
- Modify: `pre_annotation_factory/my_package/workflow/pipeline.py`
- Modify: `pre_annotation_factory/tests/test_annotationctl_pipeline.py`
- Test: `pre_annotation_factory/tests/test_annotationctl_pipeline.py`

- [ ] **Step 1: Write the failing test**

在 `pre_annotation_factory/tests/test_annotationctl_pipeline.py` 中新增一个 pipeline 测试，验证：

- `submit()` 后 `state.paths["submit_manifest_path"]` 存在
- `finalize()` 调用 merge 时会把该路径传下去

测试骨架：

```python
def test_finalize_passes_submit_manifest_path_to_merge(tmp_path, monkeypatch):
    from my_package.workflow.pipeline import WorkflowPipeline

    submit_manifest = tmp_path / "job_submit_frames.json"
    submit_manifest.write_text(
        json.dumps([{"scene_name": "Scene_01", "frame_id": "f1", "img_ext": ".png"}]),
        encoding="utf-8",
    )

    captured = {}

    def fake_run_auto_annotation(_target_path, _location_str, **kwargs):
        xtreme_zip = kwargs["xtreme1_output_dir"] / "upload.zip"
        xtreme_zip.parent.mkdir(parents=True, exist_ok=True)
        xtreme_zip.write_bytes(b"zip")
        raw_origin_dir = kwargs["raw_data_archive_dir"] / "origin"
        raw_origin_dir.mkdir(parents=True, exist_ok=True)
        return {
            "xtreme_zip_path": xtreme_zip,
            "raw_archive_dir": raw_origin_dir,
            "xtreme_upload_dir": kwargs["xtreme1_output_dir"],
            "submit_manifest_path": submit_manifest,
        }

    def fake_merge(_location, **kwargs):
        captured["submit_manifest_path"] = kwargs["submit_manifest_path"]
        dataset_root = tmp_path / "final_dataset"
        dataset_root.mkdir(parents=True, exist_ok=True)
        zip_path = tmp_path / "final.zip"
        zip_path.write_bytes(b"zip")
        return {"dataset_root_dir": dataset_root, "zip_path": zip_path}
```

最后断言：

```python
assert Path(captured["submit_manifest_path"]) == submit_manifest
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
pytest pre_annotation_factory/tests/test_annotationctl_pipeline.py -k finalize_passes_submit_manifest_path -q
```

Expected:
- FAIL because `submit_manifest_path` is not stored in `state.paths`
- Or FAIL because `pipeline.finalize()` does not pass it into `merge_final_dataset_fn`

- [ ] **Step 3: Write minimal implementation**

在 `pre_annotation_factory/my_package/workflow/pipeline.py` 中做三处小改动：

1. `prepare_local()` 接收 `artifacts["submit_manifest_path"]`

```python
state.paths.update(
    {
        "xtreme_upload_dir": str(artifacts["xtreme_upload_dir"]),
        "xtreme_zip_path": str(artifacts["xtreme_zip_path"]),
        "raw_archive_dir": str(artifacts["raw_archive_dir"]),
        "submit_manifest_path": str(artifacts["submit_manifest_path"]),
    }
)
```

2. `finalize()` 在调用 merge 前读取：

```python
submit_manifest_path = state.paths.get("submit_manifest_path")
if not submit_manifest_path:
    raise FileNotFoundError(
        f"Missing submit frame manifest in job state: {job_id}"
    )
```

3. 将其传给 merge：

```python
merge_result = self.merge_final_dataset_fn(
    manifest.location,
    xtreme_export_root=normalized_export_dir,
    raw_archive_root=Path(state.paths["raw_archive_dir"]),
    final_kitti_output_dir=final_output_dir,
    submit_manifest_path=Path(submit_manifest_path),
)
```

- [ ] **Step 4: Run test to verify it passes**

Run:

```bash
pytest pre_annotation_factory/tests/test_annotationctl_pipeline.py -k finalize_passes_submit_manifest_path -q
```

Expected:

```text
1 passed
```

- [ ] **Step 5: Commit**

```bash
git add pre_annotation_factory/my_package/workflow/pipeline.py \
        pre_annotation_factory/tests/test_annotationctl_pipeline.py
git commit -m "feat: pass submit frame manifest into finalize merge"
```

## Task 4: Make Finalize Strictly Consume the Submit Manifest

**Files:**
- Modify: `pre_annotation_factory/scripts/MergeXtremeDynamic.py`
- Modify: `pre_annotation_factory/tests/test_merge_xtreme_dynamic.py`
- Test: `pre_annotation_factory/tests/test_merge_xtreme_dynamic.py`

- [ ] **Step 1: Write the failing test**

在 `pre_annotation_factory/tests/test_merge_xtreme_dynamic.py` 中再补一个失败测试，锁定“manifest 缺失时直接失败”：

```python
def test_finalize_requires_submit_manifest_path(tmp_path):
    module = load_merge_module()

    raw_root = tmp_path / "raw"
    xtreme_root = tmp_path / "xtreme"
    output_root = tmp_path / "output"
    scene_dir = raw_root / "data_record_20260414" / "Scene_01"
    create_raw_frame(scene_dir, "frame_001", "raw-label\n")

    stub_external_effects(module, output_root)

    with pytest.raises(FileNotFoundError, match="submit manifest"):
        module.build_final_dataset(
            "unit_test",
            xtreme_export_root=xtreme_root,
            raw_archive_root=raw_root,
            final_kitti_output_dir=output_root,
            submit_manifest_path=tmp_path / "missing.json",
        )
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
pytest pre_annotation_factory/tests/test_merge_xtreme_dynamic.py -k "submit_manifest or requires_submit_manifest_path" -q
```

Expected:
- FAIL because `build_final_dataset()` still does not know how to consume manifest

- [ ] **Step 3: Write minimal implementation**

在 `pre_annotation_factory/scripts/MergeXtremeDynamic.py` 中做以下改动。

先给 `build_final_dataset()` 增加入参：

```python
def build_final_dataset(
    location_str,
    xtreme_export_root=None,
    raw_archive_root=None,
    final_kitti_output_dir=None,
    split_ratio=None,
    convert_pcd_to_bin=None,
    random_seed=None,
    max_empty_label_ratio=None,
    submit_manifest_path=None,
):
```

新增读取 helper：

```python
def load_submit_frame_manifest(manifest_path: Path):
    if manifest_path is None:
        raise FileNotFoundError("Missing submit manifest path for finalize.")
    manifest_path = Path(manifest_path)
    if not manifest_path.exists():
        raise FileNotFoundError(f"Missing submit manifest: {manifest_path}")
    with manifest_path.open("r", encoding="utf-8") as f:
        payload = json.load(f)
    if not isinstance(payload, list):
        raise ValueError(f"Invalid submit manifest payload: {manifest_path}")
    return payload
```

再把当前这段：

```python
scene_dirs = sorted(raw_archive_root.glob("data_record_*/Scene_*"))
scanned_tasks = []
for scene_dir in scene_dirs:
    ...
```

替换成：

```python
manifest_items = load_submit_frame_manifest(submit_manifest_path)
scanned_tasks = []

for item in manifest_items:
    scene_name = item["scene_name"]
    frame_id = item["frame_id"]
    img_ext = item["img_ext"]

    scene_matches = sorted(raw_archive_root.glob(f"data_record_*/{scene_name}"))
    if len(scene_matches) != 1:
        raise FileNotFoundError(
            f"Expected exactly one raw scene directory for {scene_name}, got {len(scene_matches)}"
        )
    scene_dir = scene_matches[0]
    xtreme_scene_dir = xtreme_export_root / scene_name

    config_path = scene_dir / "camera_config" / f"{frame_id}.json"
    img_path = scene_dir / "camera_image_0" / f"{frame_id}{img_ext}"
    pcd_path = scene_dir / "lidar_point_cloud_0" / f"{frame_id}.pcd"
    raw_label_path = scene_dir / "label_2" / f"{frame_id}.txt"
    xtreme_json_path = xtreme_scene_dir / "result" / f"{frame_id}.json"

    if not config_path.exists() or not img_path.exists() or not pcd_path.exists():
        raise FileNotFoundError(
            f"Submit manifest frame is missing raw artifacts: {scene_name}/{frame_id}"
        )

    if xtreme_json_path.exists():
        label_source = "xtreme"
        target_label_path = xtreme_json_path
    else:
        label_source = "empty_missing_json"
        target_label_path = None

    scanned_tasks.append(
        {
            "frame_id": frame_id,
            "scene_name": scene_name,
            "config_path": config_path,
            "img_path": img_path,
            "pcd_path": pcd_path,
            "label_source": label_source,
            "label_path": target_label_path,
            "raw_label_path": raw_label_path,
            "kitti_lines": None,
        }
    )
```

保持原有的：

- `xtreme` 路径
- `empty_missing_json` 路径
- `empty_xtreme_json` 路径
- 空标签比例裁剪

但彻底删除 `empty_missing_scene` 这条分支，因为 manifest 模式下不再需要“Scene 整体缺失则回流 raw frame”。

- [ ] **Step 4: Run test to verify it passes**

Run:

```bash
pytest pre_annotation_factory/tests/test_merge_xtreme_dynamic.py -k "submit_manifest or requires_submit_manifest_path" -q
```

Expected:

```text
3 passed
```

- [ ] **Step 5: Run focused regression tests**

Run:

```bash
pytest pre_annotation_factory/tests/test_merge_xtreme_dynamic.py \
       pre_annotation_factory/tests/test_annotationctl_pipeline.py -q
```

Expected:

```text
all passed
```

- [ ] **Step 6: Commit**

```bash
git add pre_annotation_factory/scripts/MergeXtremeDynamic.py \
        pre_annotation_factory/tests/test_merge_xtreme_dynamic.py \
        pre_annotation_factory/tests/test_annotationctl_pipeline.py
git commit -m "feat: align finalize frame universe with submit upload set"
```

## Task 5: Verify Real Workflow Behavior and Update Operator Docs

**Files:**
- Modify: `docs/annotationctl-quickstart.md`
- Modify: `docs/analysis/2026-04-28-finalize-submit-frame-set-alignment-review.md`

- [ ] **Step 1: Run the real workflow smoke commands**

Run:

```bash
conda run --live-stream -n nusc_env python -u pre_annotation_factory/scripts/annotationctl.py submit <job.yaml>
```

Expected:
- `state.json` contains `submit_manifest_path`
- job artifacts contain a persisted submit manifest file

Then run:

```bash
conda run --live-stream -n nusc_env python -u pre_annotation_factory/scripts/annotationctl.py finalize <job_id> --workspace <workspace.root_dir>
```

Expected:
- `finalize` candidate count never exceeds submit uploaded count
- `missing json` only refers to uploaded frames lacking export json

- [ ] **Step 2: Update operator docs**

在 `docs/annotationctl-quickstart.md` 增补一句操作说明：

```md
- `finalize` 只会处理 `submit` 实际上传到 Xtreme 的帧；未进入上传清单的原始帧不会回流进最终交付集。
```

在评审文档末尾追加实现状态摘要：

```md
## 实施后验证结果

- `submit` 上传帧数：...
- `finalize` 候选帧数：...
- `missing json` 帧数：...
- 是否仍出现“最终帧数大于上传帧数”：否
```

- [ ] **Step 3: Commit**

```bash
git add docs/annotationctl-quickstart.md \
        docs/analysis/2026-04-28-finalize-submit-frame-set-alignment-review.md
git commit -m "docs: document finalize frame universe alignment"
```

## Self-Review

- 评审结论的三个硬约束都已映射到任务：
  - `finalize` 以上传清单为准：Task 1 / Task 4
  - `missing json` 语义收紧：Task 1 / Task 4
  - 不保留宽口径兼容开关：Task 4
- 旧 job 无 manifest 的处理已明确为失败提示：Task 3 / Task 4
- 没有引入与目标无关的重构；`MergeXtremeDynamic.py` 仍保留现有空标签比例逻辑，只收紧候选集来源
