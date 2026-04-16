# Xtreme Server Workflow Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a server-friendly `annotationctl` CLI that runs the fixed Xtreme workflow through `submit`, `status`, and `finalize`, while reusing the existing pre-annotation and merge logic.

**Architecture:** Add a small workflow package under `pre_annotation_factory/my_package/workflow/` to own manifest parsing, job state persistence, Xtreme API integration, and pipeline orchestration. Keep existing scripts as legacy adapters by extracting importable functions from them instead of rewriting their geometry logic.

**Tech Stack:** Python 3.10, `pytest`, standard library `dataclasses/json/pathlib/subprocess`, existing `pre_annotation_factory` code, HTTP client for Xtreme API (`requests` if already available, otherwise `urllib` fallback).

---

## File Map

- Create: `pre_annotation_factory/my_package/workflow/__init__.py`
- Create: `pre_annotation_factory/my_package/workflow/job_models.py`
- Create: `pre_annotation_factory/my_package/workflow/job_store.py`
- Create: `pre_annotation_factory/my_package/workflow/xtreme_gateway.py`
- Create: `pre_annotation_factory/my_package/workflow/pipeline.py`
- Create: `pre_annotation_factory/scripts/annotationctl.py`
- Modify: `pre_annotation_factory/scripts/extract_archives.py`
- Modify: `pre_annotation_factory/scripts/replay_rosbag_main.py`
- Modify: `pre_annotation_factory/scripts/MergeXtremeDynamic.py`
- Create: `pre_annotation_factory/tests/test_job_models.py`
- Create: `pre_annotation_factory/tests/test_job_store.py`
- Create: `pre_annotation_factory/tests/test_xtreme_gateway.py`
- Create: `pre_annotation_factory/tests/test_annotationctl_pipeline.py`
- Modify: `pre_annotation_factory/tests/test_merge_xtreme_dynamic.py`

## Stage Boundaries

- Stage 1: Manifest parsing + job workspace + persistent state
- Stage 2: Refactor legacy scripts into importable adapters
- Stage 3: Xtreme gateway + submit/finalize orchestration
- Stage 4: CLI integration + end-to-end command tests

Each stage ends with a review checkpoint before moving on.

### Task 1: Stage 1 Scaffold - Manifest And Job State

**Files:**
- Create: `pre_annotation_factory/my_package/workflow/__init__.py`
- Create: `pre_annotation_factory/my_package/workflow/job_models.py`
- Create: `pre_annotation_factory/my_package/workflow/job_store.py`
- Create: `pre_annotation_factory/tests/test_job_models.py`
- Create: `pre_annotation_factory/tests/test_job_store.py`

- [ ] **Step 1: Write the failing manifest parsing test**

```python
# pre_annotation_factory/tests/test_job_models.py
from pathlib import Path

from my_package.workflow.job_models import load_job_manifest


def test_load_job_manifest_reads_required_fields(tmp_path: Path):
    manifest_path = tmp_path / "job.yaml"
    manifest_path.write_text(
        "\n".join(
            [
                "job_name: demo_job",
                "location: Demo_20260416",
                "input:",
                "  source_dir: /data/source",
                "  recursive: false",
                "  overwrite_extract: true",
                "workspace:",
                "  root_dir: /tmp/workspace",
                "xtreme:",
                "  base_url: http://127.0.0.1:8190",
                "  token_env: XTREME1_TOKEN",
                "  dataset_name: DemoDataset",
                "  dataset_type: lidar_fusion",
                "output:",
                "  final_dataset_dir: /tmp/final",
                "",
            ]
        ),
        encoding="utf-8",
    )

    manifest = load_job_manifest(manifest_path)

    assert manifest.job_name == "demo_job"
    assert manifest.location == "Demo_20260416"
    assert manifest.input.source_dir == Path("/data/source")
    assert manifest.input.overwrite_extract is True
    assert manifest.xtreme.dataset_name == "DemoDataset"
```

- [ ] **Step 2: Run the manifest test to verify it fails**

Run: `pytest pre_annotation_factory/tests/test_job_models.py::test_load_job_manifest_reads_required_fields -v`

Expected: FAIL with `ModuleNotFoundError` or missing `load_job_manifest`.

- [ ] **Step 3: Write the failing state persistence test**

```python
# pre_annotation_factory/tests/test_job_store.py
from pathlib import Path

from my_package.workflow.job_models import JobState
from my_package.workflow.job_store import JobStore


def test_job_store_creates_workspace_and_persists_state(tmp_path: Path):
    store = JobStore(tmp_path)

    state = JobState(
        job_id="job-001",
        status="created",
        current_step="created",
    )

    job_dir = store.initialize_job_dir("job-001")
    store.save_state("job-001", state)

    loaded = store.load_state("job-001")

    assert job_dir == tmp_path / "jobs" / "job-001"
    assert (job_dir / "artifacts").is_dir()
    assert loaded.status == "created"
    assert loaded.current_step == "created"
```

- [ ] **Step 4: Run the state test to verify it fails**

Run: `pytest pre_annotation_factory/tests/test_job_store.py::test_job_store_creates_workspace_and_persists_state -v`

Expected: FAIL with missing workflow classes.

- [ ] **Step 5: Write minimal manifest and state implementation**

```python
# pre_annotation_factory/my_package/workflow/job_models.py
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass
class InputConfig:
    source_dir: Path
    recursive: bool = False
    overwrite_extract: bool = False


@dataclass
class WorkspaceConfig:
    root_dir: Path


@dataclass
class XtremeConfig:
    base_url: str
    token_env: str
    dataset_name: str
    dataset_type: str


@dataclass
class OutputConfig:
    final_dataset_dir: Path


@dataclass
class JobManifest:
    job_name: str
    location: str
    input: InputConfig
    workspace: WorkspaceConfig
    xtreme: XtremeConfig
    output: OutputConfig


@dataclass
class JobState:
    job_id: str
    status: str
    current_step: str
    last_error: str | None = None
    xtreme: dict[str, Any] = field(default_factory=dict)
    paths: dict[str, str] = field(default_factory=dict)

    def to_json(self) -> str:
        return json.dumps(asdict(self), ensure_ascii=False, indent=2)

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "JobState":
        return cls(**payload)


def load_job_manifest(path: Path) -> JobManifest:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    return JobManifest(
        job_name=payload["job_name"],
        location=payload["location"],
        input=InputConfig(
            source_dir=Path(payload["input"]["source_dir"]),
            recursive=payload["input"].get("recursive", False),
            overwrite_extract=payload["input"].get("overwrite_extract", False),
        ),
        workspace=WorkspaceConfig(root_dir=Path(payload["workspace"]["root_dir"])),
        xtreme=XtremeConfig(**payload["xtreme"]),
        output=OutputConfig(
            final_dataset_dir=Path(payload["output"]["final_dataset_dir"])
        ),
    )
```

```python
# pre_annotation_factory/my_package/workflow/job_store.py
from __future__ import annotations

import json
from pathlib import Path

from .job_models import JobState


class JobStore:
    def __init__(self, workspace_root: Path):
        self.workspace_root = Path(workspace_root)
        self.jobs_root = self.workspace_root / "jobs"

    def initialize_job_dir(self, job_id: str) -> Path:
        job_dir = self.jobs_root / job_id
        for relative in (
            ".",
            "logs",
            "artifacts",
            "artifacts/extracted_bags",
            "artifacts/raw_origin",
            "artifacts/xtreme_upload",
            "artifacts/xtreme_export",
            "artifacts/final_dataset",
        ):
            (job_dir / relative).mkdir(parents=True, exist_ok=True)
        return job_dir

    def save_state(self, job_id: str, state: JobState) -> None:
        path = self.jobs_root / job_id / "state.json"
        path.write_text(state.to_json(), encoding="utf-8")

    def load_state(self, job_id: str) -> JobState:
        path = self.jobs_root / job_id / "state.json"
        return JobState.from_dict(json.loads(path.read_text(encoding="utf-8")))
```

- [ ] **Step 6: Run Stage 1 tests to verify they pass**

Run: `pytest pre_annotation_factory/tests/test_job_models.py pre_annotation_factory/tests/test_job_store.py -v`

Expected: PASS for both tests.

- [ ] **Step 7: Refactor with validation while keeping tests green**

```python
# add to load_job_manifest in job_models.py
required_sections = ("input", "workspace", "xtreme", "output")
for key in required_sections:
    if key not in payload:
        raise ValueError(f"Missing required manifest section: {key}")
```

- [ ] **Step 8: Re-run Stage 1 tests**

Run: `pytest pre_annotation_factory/tests/test_job_models.py pre_annotation_factory/tests/test_job_store.py -v`

Expected: PASS with no regressions.

- [ ] **Step 9: Commit Stage 1**

```bash
git add \
  pre_annotation_factory/my_package/workflow/__init__.py \
  pre_annotation_factory/my_package/workflow/job_models.py \
  pre_annotation_factory/my_package/workflow/job_store.py \
  pre_annotation_factory/tests/test_job_models.py \
  pre_annotation_factory/tests/test_job_store.py
git commit -m "feat: add job manifest and state store"
```

### Task 2: Stage 2 Refactor - Importable Legacy Adapters

**Files:**
- Modify: `pre_annotation_factory/scripts/extract_archives.py`
- Modify: `pre_annotation_factory/scripts/replay_rosbag_main.py`
- Modify: `pre_annotation_factory/scripts/MergeXtremeDynamic.py`
- Modify: `pre_annotation_factory/tests/test_merge_xtreme_dynamic.py`
- Create: `pre_annotation_factory/tests/test_annotationctl_pipeline.py`

- [ ] **Step 1: Write the failing extract adapter test**

```python
# pre_annotation_factory/tests/test_annotationctl_pipeline.py
from pathlib import Path

from pre_annotation_factory.scripts.extract_archives import extract_archives


def test_extract_archives_returns_output_dir_when_no_archives(tmp_path: Path):
    source = tmp_path / "source"
    output = tmp_path / "output"
    source.mkdir()

    result = extract_archives(source, output, recursive=False, overwrite=False)

    assert result == output
    assert output.is_dir()
```

- [ ] **Step 2: Run the extract test to verify it fails**

Run: `pytest pre_annotation_factory/tests/test_annotationctl_pipeline.py::test_extract_archives_returns_output_dir_when_no_archives -v`

Expected: FAIL because `extract_archives` does not exist yet.

- [ ] **Step 3: Write the failing replay builder test**

```python
import importlib.util
from pathlib import Path


def load_replay_module():
    module_path = Path("pre_annotation_factory/scripts/replay_rosbag_main.py").resolve()
    spec = importlib.util.spec_from_file_location("replay_rosbag_main", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_build_xtreme_upload_bundle_forces_kitti_disabled(tmp_path: Path):
    module = load_replay_module()
    called = {}

    def fake_build(_workspace_path, _location_str):
        called["generate_kitti"] = module.GENERATE_KITTI_DATASET

    module.build_datasets = fake_build
    module.run_xtreme_upload_bundle = getattr(module, "run_xtreme_upload_bundle")
    module.run_xtreme_upload_bundle("/tmp/workspace", "Demo_20260416")

    assert called["generate_kitti"] is False
```

- [ ] **Step 4: Run the replay builder test to verify it fails**

Run: `pytest pre_annotation_factory/tests/test_annotationctl_pipeline.py::test_build_xtreme_upload_bundle_forces_kitti_disabled -v`

Expected: FAIL because the adapter function is missing.

- [ ] **Step 5: Write the failing merge parameterization test**

```python
def test_build_final_dataset_accepts_runtime_roots(tmp_path: Path):
    module = load_merge_module()
    raw_root = tmp_path / "raw"
    xtreme_root = tmp_path / "xtreme"
    output_root = tmp_path / "output"

    module.build_final_dataset(
        "unit_test",
        xtreme_export_root=xtreme_root,
        raw_archive_root=raw_root,
        final_kitti_output_dir=output_root,
        max_empty_label_ratio=1.0,
    )
```

- [ ] **Step 6: Run the merge parameterization test to verify it fails**

Run: `pytest pre_annotation_factory/tests/test_merge_xtreme_dynamic.py::test_build_final_dataset_accepts_runtime_roots -v`

Expected: FAIL with unexpected keyword arguments.

- [ ] **Step 7: Write minimal adapter implementation**

```python
# extract_archives.py
def extract_archives(
    source_dir: Path,
    output_dir: Path,
    recursive: bool = False,
    overwrite: bool = False,
    remove_archive: bool = False,
) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    archives = find_archives(source_dir, recursive)
    for archive in archives:
        extract_archive(archive, archive_type(archive), output_dir, overwrite)
        if remove_archive:
            archive.unlink()
    return output_dir
```

```python
# replay_rosbag_main.py
def run_xtreme_upload_bundle(workspace_path: str, location_str: str) -> None:
    global GENERATE_KITTI_DATASET
    previous = GENERATE_KITTI_DATASET
    GENERATE_KITTI_DATASET = False
    try:
        build_datasets(workspace_path, location_str)
    finally:
        GENERATE_KITTI_DATASET = previous
```

```python
# MergeXtremeDynamic.py
def build_final_dataset(
    location_str,
    xtreme_export_root=None,
    raw_archive_root=None,
    final_kitti_output_dir=None,
    split_ratio=None,
    convert_pcd_to_bin=None,
    random_seed=None,
    max_empty_label_ratio=None,
):
    xtreme_export_root = Path(xtreme_export_root or XTREME_EXPORT_ROOT)
    raw_archive_root = Path(raw_archive_root or RAW_ARCHIVE_ROOT)
    final_kitti_output_dir = Path(final_kitti_output_dir or FINAL_KITTI_OUTPUT_DIR)
    split_ratio = SPLIT_RATIO if split_ratio is None else split_ratio
    convert_pcd_to_bin = CONVERT_PCD_TO_BIN if convert_pcd_to_bin is None else convert_pcd_to_bin
    random_seed = RANDOM_SEED if random_seed is None else random_seed
    max_empty_label_ratio = MAX_EMPTY_LABEL_RATIO if max_empty_label_ratio is None else max_empty_label_ratio
```

- [ ] **Step 8: Update existing merge tests to use the new call shape**

```python
# replace module globals injection with direct keyword args in test_merge_xtreme_dynamic.py
module.build_final_dataset(
    "unit_test",
    xtreme_export_root=xtreme_root,
    raw_archive_root=raw_root,
    final_kitti_output_dir=output_root,
    max_empty_label_ratio=1.0,
)
```

- [ ] **Step 9: Run Stage 2 tests**

Run: `pytest pre_annotation_factory/tests/test_annotationctl_pipeline.py pre_annotation_factory/tests/test_merge_xtreme_dynamic.py -v`

Expected: PASS for adapter tests and preserved merge behavior.

- [ ] **Step 10: Commit Stage 2**

```bash
git add \
  pre_annotation_factory/scripts/extract_archives.py \
  pre_annotation_factory/scripts/replay_rosbag_main.py \
  pre_annotation_factory/scripts/MergeXtremeDynamic.py \
  pre_annotation_factory/tests/test_annotationctl_pipeline.py \
  pre_annotation_factory/tests/test_merge_xtreme_dynamic.py
git commit -m "refactor: add importable workflow adapters"
```

### Task 3: Stage 3 Build - Xtreme Gateway

**Files:**
- Create: `pre_annotation_factory/my_package/workflow/xtreme_gateway.py`
- Create: `pre_annotation_factory/tests/test_xtreme_gateway.py`

- [ ] **Step 1: Write the failing token loading test**

```python
# pre_annotation_factory/tests/test_xtreme_gateway.py
import os

import pytest

from my_package.workflow.xtreme_gateway import XtremeGateway


def test_gateway_reads_token_from_environment(monkeypatch):
    monkeypatch.setenv("XTREME1_TOKEN", "secret-token")

    gateway = XtremeGateway("http://127.0.0.1:8190", "XTREME1_TOKEN")

    assert gateway.headers["Authorization"] == "Bearer secret-token"


def test_gateway_raises_when_token_missing(monkeypatch):
    monkeypatch.delenv("XTREME1_TOKEN", raising=False)

    with pytest.raises(ValueError, match="XTREME1_TOKEN"):
        XtremeGateway("http://127.0.0.1:8190", "XTREME1_TOKEN")
```

- [ ] **Step 2: Run the token tests to verify they fail**

Run: `pytest pre_annotation_factory/tests/test_xtreme_gateway.py::test_gateway_reads_token_from_environment pre_annotation_factory/tests/test_xtreme_gateway.py::test_gateway_raises_when_token_missing -v`

Expected: FAIL because `XtremeGateway` does not exist.

- [ ] **Step 3: Write the failing export normalization test**

```python
from pathlib import Path

from my_package.workflow.xtreme_gateway import normalize_export_tree


def test_normalize_export_tree_flattens_single_outer_directory(tmp_path: Path):
    source = tmp_path / "download"
    nested = source / "outer" / "Scene_01" / "result"
    nested.mkdir(parents=True)
    (nested / "frame_1.json").write_text("{}", encoding="utf-8")

    target = tmp_path / "normalized"
    normalize_export_tree(source, target)

    assert (target / "Scene_01" / "result" / "frame_1.json").exists()
```

- [ ] **Step 4: Run the normalization test to verify it fails**

Run: `pytest pre_annotation_factory/tests/test_xtreme_gateway.py::test_normalize_export_tree_flattens_single_outer_directory -v`

Expected: FAIL because the helper is missing.

- [ ] **Step 5: Write minimal gateway implementation**

```python
# pre_annotation_factory/my_package/workflow/xtreme_gateway.py
from __future__ import annotations

import os
import shutil
from pathlib import Path


class XtremeGateway:
    def __init__(self, base_url: str, token_env: str):
        token = os.environ.get(token_env)
        if not token:
            raise ValueError(f"Missing Xtreme token in environment: {token_env}")
        self.base_url = base_url.rstrip("/")
        self.headers = {"Authorization": f"Bearer {token}"}


def normalize_export_tree(download_root: Path, target_root: Path) -> Path:
    download_root = Path(download_root)
    target_root = Path(target_root)
    target_root.mkdir(parents=True, exist_ok=True)

    entries = [entry for entry in download_root.iterdir()]
    source_root = entries[0] if len(entries) == 1 and entries[0].is_dir() else download_root

    for child in source_root.iterdir():
        destination = target_root / child.name
        if child.is_dir():
            shutil.copytree(child, destination, dirs_exist_ok=True)
        else:
            shutil.copy2(child, destination)
    return target_root
```

- [ ] **Step 6: Run Stage 3 tests**

Run: `pytest pre_annotation_factory/tests/test_xtreme_gateway.py -v`

Expected: PASS.

- [ ] **Step 7: Extend the gateway with the real API surface under test**

```python
# replace the minimal class with concrete wrappers once tests exist
class XtremeGateway:
    def create_or_get_dataset(self, name: str, dataset_type: str) -> str:
        payload = {"name": name, "type": dataset_type}
        response = self._post_json("/api/dataset/create", payload)
        return response["data"]["id"]

    def request_export(self, dataset_id: str) -> str:
        response = self._post_json("/api/export/create", {"datasetId": dataset_id})
        return response["data"]["id"]

    def wait_export_done(self, export_task_id: str) -> str:
        response = self._get_json(f"/api/export/{export_task_id}")
        return response["data"]["downloadUrl"]
```

- [ ] **Step 8: Commit Stage 3**

```bash
git add \
  pre_annotation_factory/my_package/workflow/xtreme_gateway.py \
  pre_annotation_factory/tests/test_xtreme_gateway.py
git commit -m "feat: add xtreme gateway primitives"
```

### Task 4: Stage 4 Orchestration - Pipeline Runner

**Files:**
- Create: `pre_annotation_factory/my_package/workflow/pipeline.py`
- Modify: `pre_annotation_factory/tests/test_annotationctl_pipeline.py`

- [ ] **Step 1: Write the failing submit orchestration test**

```python
from pathlib import Path

from my_package.workflow.job_models import load_job_manifest
from my_package.workflow.pipeline import WorkflowPipeline


def test_submit_runs_extract_then_upload_then_waiting_state(tmp_path: Path, monkeypatch):
    manifest_path = tmp_path / "job.yaml"
    manifest_path.write_text(
        "\n".join(
            [
                "job_name: demo_job",
                "location: Demo_20260416",
                "input:",
                f"  source_dir: {tmp_path / 'source'}",
                "workspace:",
                f"  root_dir: {tmp_path / 'workspace'}",
                "xtreme:",
                "  base_url: http://127.0.0.1:8190",
                "  token_env: XTREME1_TOKEN",
                "  dataset_name: DemoDataset",
                "  dataset_type: lidar_fusion",
                "output:",
                f"  final_dataset_dir: {tmp_path / 'final'}",
                "",
            ]
        ),
        encoding="utf-8",
    )
    (tmp_path / "source").mkdir()
    monkeypatch.setenv("XTREME1_TOKEN", "token")

    manifest = load_job_manifest(manifest_path)
    pipeline = WorkflowPipeline.for_tests(tmp_path)

    job_id = pipeline.submit(manifest)
    state = pipeline.job_store.load_state(job_id)

    assert state.status == "waiting_human_annotation"
    assert "dataset_id" in state.xtreme
```

- [ ] **Step 2: Run the submit orchestration test to verify it fails**

Run: `pytest pre_annotation_factory/tests/test_annotationctl_pipeline.py::test_submit_runs_extract_then_upload_then_waiting_state -v`

Expected: FAIL because `WorkflowPipeline` does not exist.

- [ ] **Step 3: Write the failing finalize orchestration test**

```python
def test_finalize_runs_export_and_merge_to_completed(tmp_path: Path, monkeypatch):
    monkeypatch.setenv("XTREME1_TOKEN", "token")
    pipeline = WorkflowPipeline.for_tests(tmp_path)
    job_id = pipeline.seed_waiting_job("job-123")

    pipeline.finalize(job_id)
    state = pipeline.job_store.load_state(job_id)

    assert state.status == "completed"
    assert "final_dataset_dir" in state.paths
```

- [ ] **Step 4: Run the finalize orchestration test to verify it fails**

Run: `pytest pre_annotation_factory/tests/test_annotationctl_pipeline.py::test_finalize_runs_export_and_merge_to_completed -v`

Expected: FAIL because the finalize path is missing.

- [ ] **Step 5: Write minimal pipeline implementation**

```python
# pre_annotation_factory/my_package/workflow/pipeline.py
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from uuid import uuid4

from .job_models import JobManifest, JobState
from .job_store import JobStore


@dataclass
class WorkflowPipeline:
    job_store: JobStore

    @classmethod
    def for_tests(cls, workspace_root: Path) -> "WorkflowPipeline":
        return cls(job_store=JobStore(workspace_root))

    def submit(self, manifest: JobManifest) -> str:
        job_id = uuid4().hex[:12]
        self.job_store.initialize_job_dir(job_id)
        state = JobState(
            job_id=job_id,
            status="waiting_human_annotation",
            current_step="uploading_xtreme",
            xtreme={"dataset_id": manifest.xtreme.dataset_name},
        )
        self.job_store.save_state(job_id, state)
        return job_id

    def seed_waiting_job(self, job_id: str) -> str:
        self.job_store.initialize_job_dir(job_id)
        self.job_store.save_state(
            job_id,
            JobState(
                job_id=job_id,
                status="waiting_human_annotation",
                current_step="waiting_human_annotation",
            ),
        )
        return job_id

    def finalize(self, job_id: str) -> None:
        state = self.job_store.load_state(job_id)
        state.status = "completed"
        state.current_step = "completed"
        state.paths["final_dataset_dir"] = str(
            self.job_store.jobs_root / job_id / "artifacts" / "final_dataset"
        )
        self.job_store.save_state(job_id, state)
```

- [ ] **Step 6: Run Stage 4 pipeline tests**

Run: `pytest pre_annotation_factory/tests/test_annotationctl_pipeline.py -v`

Expected: PASS for the orchestration skeleton.

- [ ] **Step 7: Replace the skeleton calls with real adapters one by one under tests**

```python
# submit path
extract_archives(
    manifest.input.source_dir,
    extracted_dir,
    recursive=manifest.input.recursive,
    overwrite=manifest.input.overwrite_extract,
)
run_auto_annotation_job(runtime_config)
run_xtreme_upload_bundle(str(workspace_path), manifest.location)
dataset_id = gateway.create_or_get_dataset(
    manifest.xtreme.dataset_name,
    manifest.xtreme.dataset_type,
)

# finalize path
export_task_id = gateway.request_export(state.xtreme["dataset_id"])
download_url = gateway.wait_export_done(export_task_id)
gateway.download_export(download_url, export_archive_path)
normalize_export_tree(export_unpack_dir, normalized_export_dir)
merge_build_final_dataset(
    manifest.location,
    xtreme_export_root=normalized_export_dir,
    raw_archive_root=raw_origin_dir,
    final_kitti_output_dir=final_dataset_dir,
)
```

- [ ] **Step 8: Commit Stage 4**

```bash
git add \
  pre_annotation_factory/my_package/workflow/pipeline.py \
  pre_annotation_factory/tests/test_annotationctl_pipeline.py
git commit -m "feat: add workflow pipeline orchestration"
```

### Task 5: Stage 5 Integration - CLI

**Files:**
- Create: `pre_annotation_factory/scripts/annotationctl.py`
- Modify: `pre_annotation_factory/tests/test_annotationctl_pipeline.py`

- [ ] **Step 1: Write the failing CLI status test**

```python
import subprocess
import sys
from pathlib import Path


def test_annotationctl_status_prints_saved_state(tmp_path: Path):
    script = Path("pre_annotation_factory/scripts/annotationctl.py").resolve()
    result = subprocess.run(
        [sys.executable, str(script), "status", "job-001", "--workspace", str(tmp_path)],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert "job-001" in result.stdout
```

- [ ] **Step 2: Run the CLI status test to verify it fails**

Run: `pytest pre_annotation_factory/tests/test_annotationctl_pipeline.py::test_annotationctl_status_prints_saved_state -v`

Expected: FAIL because the CLI script does not exist yet.

- [ ] **Step 3: Write minimal CLI implementation**

```python
# pre_annotation_factory/scripts/annotationctl.py
from __future__ import annotations

import argparse
from pathlib import Path

from my_package.workflow.job_store import JobStore


def main() -> int:
    parser = argparse.ArgumentParser("annotationctl")
    parser.add_argument("--workspace", default=".")
    subparsers = parser.add_subparsers(dest="command", required=True)

    status_parser = subparsers.add_parser("status")
    status_parser.add_argument("job_id")

    args = parser.parse_args()
    store = JobStore(Path(args.workspace))

    if args.command == "status":
        state = store.load_state(args.job_id)
        print(f"job_id={state.job_id}")
        print(f"status={state.status}")
        return 0

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Run the CLI test to verify it passes**

Run: `pytest pre_annotation_factory/tests/test_annotationctl_pipeline.py::test_annotationctl_status_prints_saved_state -v`

Expected: PASS.

- [ ] **Step 5: Add submit/finalize command tests before implementing them**

```python
def test_annotationctl_submit_creates_job_and_prints_job_id(tmp_path: Path):
    script = Path("pre_annotation_factory/scripts/annotationctl.py").resolve()
    manifest = tmp_path / "job.yaml"
    source_dir = tmp_path / "source"
    source_dir.mkdir()
    manifest.write_text(
        "\n".join(
            [
                "job_name: demo_job",
                "location: Demo_20260416",
                "input:",
                f"  source_dir: {source_dir}",
                "workspace:",
                f"  root_dir: {tmp_path / 'workspace'}",
                "xtreme:",
                "  base_url: http://127.0.0.1:8190",
                "  token_env: XTREME1_TOKEN",
                "  dataset_name: DemoDataset",
                "  dataset_type: lidar_fusion",
                "output:",
                f"  final_dataset_dir: {tmp_path / 'final'}",
                "",
            ]
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [sys.executable, str(script), "submit", str(manifest)],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert "job_id=" in result.stdout


def test_annotationctl_finalize_marks_job_completed(tmp_path: Path):
    script = Path("pre_annotation_factory/scripts/annotationctl.py").resolve()
    workspace = tmp_path / "workspace"
    store = JobStore(workspace)
    store.initialize_job_dir("job-001")
    store.save_state(
        "job-001",
        JobState(
            job_id="job-001",
            status="waiting_human_annotation",
            current_step="waiting_human_annotation",
        ),
    )

    result = subprocess.run(
        [sys.executable, str(script), "finalize", "job-001", "--workspace", str(workspace)],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert "completed" in result.stdout
```

- [ ] **Step 6: Implement minimal submit/finalize wiring and run the full suite**

Run: `pytest pre_annotation_factory/tests/test_job_models.py pre_annotation_factory/tests/test_job_store.py pre_annotation_factory/tests/test_xtreme_gateway.py pre_annotation_factory/tests/test_annotationctl_pipeline.py pre_annotation_factory/tests/test_merge_xtreme_dynamic.py -v`

Expected: PASS for the workflow suite and preserved merge regression tests.

- [ ] **Step 7: Commit Stage 5**

```bash
git add \
  pre_annotation_factory/scripts/annotationctl.py \
  pre_annotation_factory/tests/test_annotationctl_pipeline.py
git commit -m "feat: add annotation workflow cli"
```

## Self-Review

- Spec coverage:
  - `submit/status/finalize`: covered by Tasks 4 and 5.
  - `job.yaml` and `state.json`: covered by Task 1.
  - Legacy script parameterization: covered by Task 2.
  - Xtreme gateway and export normalization: covered by Task 3.
  - Failure-safe incremental development through TDD: every task starts with failing tests and stage commits.
- Placeholder scan:
  - No `TODO` or `TBD` placeholders remain in task steps.
  - One intentional `...` appears in future test names under Task 5 Step 5 as shorthand and should be replaced with real test bodies during execution before code is written.
- Type consistency:
  - Workflow package names are consistent: `job_models`, `job_store`, `xtreme_gateway`, `pipeline`.
  - CLI command name is consistent: `annotationctl`.

## Fixups Before Execution

- Keep Stage 4 skeleton intentionally minimal; once the tests are green, swap in real adapter calls under the existing tests instead of inventing new architecture.

## Review Cadence

- Review 1: After Task 1 commit
- Review 2: After Task 2 commit
- Review 3: After Task 3 commit
- Review 4: After Tasks 4 and 5 complete

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-04-16-xtreme-server-workflow.md`. Two execution options:

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

Which approach?
