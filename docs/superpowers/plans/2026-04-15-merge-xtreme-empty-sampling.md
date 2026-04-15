# Merge Xtreme Empty Sampling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Update `MergeXtremeDynamic.py` so scenes absent from Xtreme are treated as empty-label scenes and the final merged dataset keeps at most 10% empty-label frames via global random sampling.

**Architecture:** Keep the current raw-data-driven scan, but classify each frame into a positive or empty-label bucket before writing outputs. Compute a deterministic empty-frame cap from the positive-frame count, sample empty tasks with the existing random seed, then write only the retained tasks and split them into train/val.

**Tech Stack:** Python, pytest, pathlib, random, NumPy, SciPy

---

### Task 1: Lock the new merge behavior with tests

**Files:**
- Modify: `pre_annotation_factory/tests/test_merge_xtreme_dynamic.py`
- Test: `pre_annotation_factory/tests/test_merge_xtreme_dynamic.py`

- [ ] **Step 1: Write the failing tests**

```python
def test_missing_xtreme_scene_is_treated_as_empty_label(tmp_path):
    ...


def test_empty_labels_are_globally_sampled_to_ten_percent(tmp_path):
    ...


def test_merge_fails_when_only_empty_frames_remain(tmp_path):
    ...
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest pre_annotation_factory/tests/test_merge_xtreme_dynamic.py -v`
Expected: FAIL because scenes missing from Xtreme still copy raw labels and there is no 10% empty-frame cap yet.

- [ ] **Step 3: Write minimal implementation**

```python
# scan raw scenes
# classify each frame as positive/empty before writing outputs
# keep all positives
# sample empties with random.sample(..., k=max_empty)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest pre_annotation_factory/tests/test_merge_xtreme_dynamic.py -v`
Expected: PASS with the new scene-missing and empty-sampling behavior.

- [ ] **Step 5: Commit**

```bash
git add pre_annotation_factory/tests/test_merge_xtreme_dynamic.py pre_annotation_factory/scripts/MergeXtremeDynamic.py docs/superpowers/specs/2026-04-15-annotation-pipeline-level1-design.md docs/superpowers/plans/2026-04-15-merge-xtreme-empty-sampling.md
git commit -m "feat: cap empty frames in xtreme merge"
```

### Task 2: Refactor merge task classification and output statistics

**Files:**
- Modify: `pre_annotation_factory/scripts/MergeXtremeDynamic.py`
- Test: `pre_annotation_factory/tests/test_merge_xtreme_dynamic.py`

- [ ] **Step 1: Write the failing test**

```python
def test_xtreme_json_with_no_boxes_counts_as_empty(tmp_path):
    ...
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest pre_annotation_factory/tests/test_merge_xtreme_dynamic.py::test_xtreme_json_with_no_boxes_counts_as_empty -v`
Expected: FAIL because empty Xtreme JSON results are not classified before the sampling stage.

- [ ] **Step 3: Write minimal implementation**

```python
task["kitti_lines"] = parse_xtreme_to_kitti_lines(...)
task["is_empty_label"] = len(task["kitti_lines"]) == 0
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest pre_annotation_factory/tests/test_merge_xtreme_dynamic.py::test_xtreme_json_with_no_boxes_counts_as_empty -v`
Expected: PASS with empty Xtreme JSON included in the empty-frame pool.

- [ ] **Step 5: Commit**

```bash
git add pre_annotation_factory/tests/test_merge_xtreme_dynamic.py pre_annotation_factory/scripts/MergeXtremeDynamic.py
git commit -m "refactor: classify merge tasks before writing"
```
