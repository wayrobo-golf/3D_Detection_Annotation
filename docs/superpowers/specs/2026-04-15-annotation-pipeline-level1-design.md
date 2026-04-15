# Level-1 Annotation Pipeline Design

**Date:** 2026-04-15

**Owner:** Codex + Keyao

## Goal

Build a level-1 automation pipeline for the existing 3D annotation workflow so that a user only needs to:

1. provide a bag batch, a map version, and Xtreme connection parameters;
2. trigger `start`;
3. wait for annotators to finish in Xtreme;
4. trigger `resume`.

Everything else should be automated: archive extraction, pre-annotation generation, Xtreme upload, export download, final dataset merge, and archiving.

## Context

The current workflow is already proven end-to-end, but it relies on multiple manual steps:

1. download new bag data and map files;
2. decompress archives;
3. run ROS bag replay and generate pre-annotation results;
4. package data and upload to Xtreme;
5. wait for manual annotation in Xtreme;
6. download export results;
7. run `MergeXtremeDynamic.py`;
8. archive raw and final outputs.

The repository already contains most of the core logic:

- `pre_annotation_factory/scripts/extract_archives.py`
- `pre_annotation_factory/scripts/replay_rosbag_main.py`
- `pre_annotation_factory/scripts/MergeXtremeDynamic.py`

The main problem is orchestration, recoverability, and traceability rather than missing algorithmic capability.

## Scope

This design covers only level-1 automation.

Included:

- explicit-input automation for a single run;
- Xtreme API preflight check;
- automated upload to Xtreme;
- manual pause for annotation;
- manual resume for export, merge, and archive;
- run manifest and run state persistence;
- explicit map-to-bag binding protocol.

Excluded:

- auto-discovery of new bag batches;
- automatic detection of annotation completion;
- map recommendation or automatic map selection;
- dashboard, scheduler, queue system, or Airflow/Jenkins integration;
- unattended multi-run orchestration.

## Constraints

### Existing workflow constraints

- The current ROS/C++ pre-annotation pipeline must remain the source of truth for pre-annotation generation.
- The current `MergeXtremeDynamic.py` remains the source of truth for final dataset assembly.
- Existing path-heavy scripts must be parameterized rather than rewritten from scratch.

### Xtreme constraints

- Xtreme API access requires a valid Bearer token.
- Dataset upload is asynchronous.
- Export/download is an explicit step after human annotation is complete.

Official references:

- API document: <https://docs.xtreme1.io/xtreme1-docs/developer-reference/api-document>
- Upload dataset format: <https://docs.xtreme1.io/xtreme1-docs/product-guides/upload-dataset>
- Annotation export result format: <https://docs.xtreme1.io/xtreme1-docs/export-data/data-annotation-result>

### Data correctness constraints

- Bag data must never silently bind to the wrong map version.
- A run must always record which map files were used.
- A newer map must not be substituted automatically for historical bag data.

## Confirmed Decisions

### Decision 1: API readiness

API readiness becomes a preflight requirement.

Every `start` command must verify:

- base URL is reachable;
- token is accepted;
- dataset ID is valid and accessible.

If this check fails, the pipeline must stop before any heavy local processing begins.

### Decision 2: Annotation completion

Annotation completion is defined by human judgment, not by polling UI/task state.

The system will not guess whether annotation is complete.

Instead:

- `start` uploads data and pauses at `WAITING_FOR_ANNOTATION`;
- a human confirms the work is done in Xtreme;
- `resume --run-id <id>` continues the pipeline.

### Decision 3: Map binding

Map binding must be explicit for every run.

Every run must receive:

- `map_id`
- `map_version`
- `map_pcd_path`
- `map_xml_path`

The pipeline must write these values, plus file hashes, into the run manifest.

The system must never default to "latest map".

## Architecture

### Recommended architecture

Use a Python orchestration layer on top of the existing scripts and modules.

New responsibilities:

- `annotation_pipeline.py` orchestrates stages and state transitions;
- `xtreme_client.py` wraps Xtreme API calls;
- `run_manifest.json` stores immutable run inputs;
- `pipeline_state.json` stores mutable run progress and recovery information.

Existing business logic is reused:

- archive extraction comes from `extract_archives.py`;
- pre-annotation generation comes from `replay_rosbag_main.py`;
- final merge comes from `MergeXtremeDynamic.py`.

### Why this architecture

This approach avoids rewriting geometry and dataset logic while solving the real operational gaps:

- repeatability;
- traceability;
- pause/resume support;
- failure recovery;
- explicit map binding.

## Run Model

### Run identity

Each pipeline execution creates one `run_id`.

Recommended format:

`<location>_<yyyymmddhhmmss>`

Example:

`YinXiu_20260414_20260415153020`

### Run workspace

Each run gets its own directory:

```text
runs/
  <run_id>/
    run_manifest.json
    pipeline_state.json
    logs/
    extracted_bags/
    xtreme_upload/
    xtreme_export/
    final_dataset/
    archive_manifest.json
```

This isolates runs from each other and makes recovery deterministic.

## Run Manifest

`run_manifest.json` is immutable after creation except for metadata fields explicitly marked append-only.

Recommended schema:

```json
{
  "run_id": "YinXiu_20260414_20260415153020",
  "location": "YinXiu_20260414",
  "bag_root": "/data/bags/YinXiu20260414",
  "map_id": "yinxiu_main",
  "map_version": "2026-04-14_v1",
  "map_pcd_path": "/data/maps/yinxiu/2026-04-14/colored_lidar_merged.pcd",
  "map_xml_path": "/data/maps/yinxiu/2026-04-14/tracklet_labels.xml",
  "map_pcd_sha256": "<sha256>",
  "map_xml_sha256": "<sha256>",
  "xtreme_base_url": "http://xtreme-server:8190",
  "xtreme_dataset_id": 123,
  "created_at": "2026-04-15T15:30:20+08:00"
}
```

Purpose:

- make every dataset reproducible;
- prevent accidental map drift;
- provide evidence for future auditing.

## Pipeline State

`pipeline_state.json` is mutable and records execution status.

Recommended fields:

```json
{
  "run_id": "YinXiu_20260414_20260415153020",
  "status": "WAITING_FOR_ANNOTATION",
  "started_at": "2026-04-15T15:30:20+08:00",
  "updated_at": "2026-04-15T16:12:08+08:00",
  "current_stage": "upload_xtreme",
  "completed_stages": [
    "preflight",
    "extract_archives",
    "preannotate",
    "package_xtreme",
    "upload_xtreme"
  ],
  "xtreme_import_task_id": "abc123",
  "xtreme_upload_archive": "/runs/<run_id>/xtreme_upload/package.zip",
  "xtreme_export_archive": null,
  "final_dataset_path": null,
  "last_error": null
}
```

## Pipeline Stages

### Stage 1: `preflight`

Checks:

- input paths exist;
- map files exist;
- ROS workspace is built;
- Xtreme API token works;
- dataset ID is reachable.

Output:

- state becomes `PREFLIGHT_PASSED`.

Failure policy:

- stop immediately;
- write error to `pipeline_state.json`;
- do not start local heavy processing.

### Stage 2: `extract_archives`

Responsibility:

- decompress incoming archives into the run workspace, if the input is archived.

Behavior:

- reuse `extract_archives.py` logic;
- prefer function reuse over subprocess if feasible;
- record extracted output directory in state.

Output:

- state includes normalized bag root under the run workspace.

### Stage 3: `preannotate`

Responsibility:

- run the existing ROS bag replay and C++ pre-annotation flow.

Behavior:

- parameterize `replay_rosbag_main.py` instead of editing constants manually;
- inject map paths into the ROS config or generated runtime config;
- generate raw outputs and Xtreme upload-ready structure.

Output:

- raw scene outputs;
- Xtreme upload directory;
- optional raw archive outputs if enabled.

### Stage 4: `package_xtreme`

Responsibility:

- package the Xtreme upload directory into a compressed archive acceptable by Xtreme.

Requirement:

- folder structure must continue to match Xtreme fusion dataset format.

Reference:

- Xtreme docs require aligned filenames under `camera_config`, `camera_image_0`, `lidar_point_cloud_0`, and optional `result`.  
  <https://docs.xtreme1.io/xtreme1-docs/product-guides/upload-dataset>

### Stage 5: `upload_xtreme`

Responsibility:

- perform Xtreme upload using API/SDK.

Expected sub-steps:

1. generate direct upload address;
2. upload the compressed archive;
3. submit dataset compressed package import;
4. poll import progress until completion.

Output:

- Xtreme import task ID;
- remote dataset reference;
- local state updated to `WAITING_FOR_ANNOTATION`.

### Stage 6: `human_gate`

Responsibility:

- stop automation and wait.

Behavior:

- print run summary;
- print Xtreme dataset ID and server URL;
- instruct operator to annotate in Xtreme;
- require explicit `resume`.

State:

- `WAITING_FOR_ANNOTATION`

### Stage 7: `fetch_export`

Triggered only by:

- `pipeline resume --run-id <id>`

Responsibility:

- request or locate the export result from Xtreme;
- download the export archive or files;
- unpack them into `runs/<run_id>/xtreme_export/`.

Output:

- local Xtreme export root ready for merge.

### Stage 8: `merge_finalize`

Responsibility:

- run final dataset construction using `MergeXtremeDynamic.py`.

Behavior:

- parameterize `MergeXtremeDynamic.py` so it accepts:
  - Xtreme export root
  - raw archive root
  - final output dir
  - location string
- preserve the current "trust annotator" empty-frame rule already implemented.

Output:

- final KITTI dataset;
- optional metadata outputs such as `nusc_meta`.

### Stage 9: `archive`

Responsibility:

- consolidate and record all important outputs.

Must record:

- input bag root;
- bound map version;
- Xtreme upload archive path;
- Xtreme export archive path;
- raw archive path;
- final dataset path;
- timestamp and completion status.

Output:

- `archive_manifest.json`
- state `COMPLETED`

## CLI Design

### Command 1: `start`

Example:

```bash
python3 pre_annotation_factory/scripts/annotation_pipeline.py start \
  --bag-root /data/bags/YinXiu20260414 \
  --map-id yinxiu_main \
  --map-version 2026-04-14_v1 \
  --map-pcd /data/maps/yinxiu/2026-04-14/colored_lidar_merged.pcd \
  --map-xml /data/maps/yinxiu/2026-04-14/tracklet_labels.xml \
  --location YinXiu_20260414 \
  --xtreme-base-url http://xtreme-server:8190 \
  --xtreme-token "$XTREME_TOKEN" \
  --xtreme-dataset-id 123
```

Behavior:

- creates run directory;
- writes manifest;
- executes until `WAITING_FOR_ANNOTATION`.

### Command 2: `resume`

Example:

```bash
python3 pre_annotation_factory/scripts/annotation_pipeline.py resume \
  --run-id YinXiu_20260414_20260415153020
```

Behavior:

- loads manifest and state;
- resumes from `WAITING_FOR_ANNOTATION`;
- downloads export;
- merges and archives.

### Command 3: `status`

Example:

```bash
python3 pre_annotation_factory/scripts/annotation_pipeline.py status \
  --run-id YinXiu_20260414_20260415153020
```

Behavior:

- prints current stage, state, critical paths, and last error.

This command is optional for the first implementation, but recommended.

## Xtreme Integration Design

### Minimal viable integration

Level-1 automation only needs:

- token-based connectivity check;
- upload compressed package;
- query import progress;
- export/download annotation result.

Implementation note:

- prefer an internal wrapper module such as `xtreme_client.py`;
- use the official SDK if it is stable and matches the deployed Xtreme version;
- otherwise call the documented OpenAPI directly with `requests`.

### API preflight contract

Before any heavy local processing, call:

`GET /api/dataset/info/{dataset_id}`

Success criteria:

- request succeeds;
- dataset metadata is returned.

Failure criteria:

- `401` or `403`: token invalid or lacks permission;
- `404`: dataset ID invalid or inaccessible;
- connection failure: base URL unreachable.

## Map Binding Protocol

This protocol is mandatory.

### Rules

1. every run must explicitly declare map identity;
2. map files must be hashed and written to manifest;
3. no default map selection is allowed;
4. final outputs must carry the same map metadata for traceability.

### Safety implications

This prevents:

- using a new map for old bag data;
- producing silently incorrect pre-annotation when obstacles moved;
- losing historical reproducibility.

### Future extension

A future phase may introduce a `maps_registry.json`, but level-1 automation does not require it.

## Logging and Auditability

Each stage should log to both:

- console;
- `runs/<run_id>/logs/<stage>.log`

Important events that must be logged:

- manifest creation;
- API preflight result;
- extracted archive locations;
- rosbag replay start/end;
- Xtreme upload archive path;
- Xtreme import task ID;
- pause state entry;
- export download path;
- final dataset path;
- archive completion.

## Failure and Recovery

### Failure behavior

When a stage fails:

- write the exception or command failure summary into `pipeline_state.json`;
- preserve all intermediate artifacts;
- mark state as `FAILED`;
- do not delete partial outputs.

### Recovery behavior

Recovery is based on persisted state, not on guessing.

Examples:

- if upload succeeded and the process exited, resume should not rerun pre-annotation;
- if export download failed, resume should restart from export fetch, not from upload;
- if merge failed, resume should retry only merge and archive.

## File Ownership

### New files

- `pre_annotation_factory/scripts/annotation_pipeline.py`
- `pre_annotation_factory/my_package/io_modules/xtreme_client.py`
- `docs/superpowers/specs/2026-04-15-annotation-pipeline-level1-design.md`

### Existing files to refactor

- `pre_annotation_factory/scripts/extract_archives.py`
- `pre_annotation_factory/scripts/replay_rosbag_main.py`
- `pre_annotation_factory/scripts/MergeXtremeDynamic.py`

### Refactor expectations

- remove hardcoded run-specific paths from script headers;
- accept parameters or structured config input;
- expose reusable functions where orchestration needs direct calls.

## Non-Goals

The first implementation will not:

- discover new bag batches automatically;
- choose maps automatically;
- determine annotation completion automatically;
- build a web dashboard;
- manage multiple concurrent runs through a queue.

## Rollout Plan

### Phase 1

- parameterize `replay_rosbag_main.py`
- parameterize `MergeXtremeDynamic.py`
- add run manifest and state files

### Phase 2

- implement `annotation_pipeline.py start`
- implement `annotation_pipeline.py resume`

### Phase 3

- add `xtreme_client.py`
- integrate Xtreme upload and export/download

### Phase 4

- add state-based recovery
- improve logs and archive manifests

## Acceptance Criteria

The level-1 automation is successful when:

1. a user can start a run by explicitly providing bags, map identity, and Xtreme connection info;
2. the pipeline uploads pre-annotation data to Xtreme and pauses;
3. a user can later resume the same run by `run_id`;
4. the pipeline downloads Xtreme results, runs merge, and produces the final dataset;
5. the final dataset can be traced back to the exact map files and bag root used for the run.
