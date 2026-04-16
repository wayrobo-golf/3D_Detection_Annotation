import importlib.util
import json
from pathlib import Path

import pytest


def load_merge_module():
    module_path = (
        Path(__file__).resolve().parents[1] / "scripts" / "MergeXtremeDynamic.py"
    )
    spec = importlib.util.spec_from_file_location("merge_xtreme_dynamic", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def write_camera_config(config_path: Path):
    config_path.parent.mkdir(parents=True, exist_ok=True)
    payload = [
        {
            "camera_internal": {
                "fx": 1000.0,
                "fy": 1000.0,
                "cx": 960.0,
                "cy": 540.0,
            },
            "width": 1920,
            "height": 1080,
            "camera_external": [
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
        }
    ]
    config_path.write_text(json.dumps(payload), encoding="utf-8")


def create_raw_frame(scene_dir: Path, frame_id: str, raw_label: str):
    write_camera_config(scene_dir / "camera_config" / f"{frame_id}.json")
    (scene_dir / "camera_image_0").mkdir(parents=True, exist_ok=True)
    (scene_dir / "camera_image_0" / f"{frame_id}.png").write_bytes(b"fake-image")
    (scene_dir / "lidar_point_cloud_0").mkdir(parents=True, exist_ok=True)
    (scene_dir / "lidar_point_cloud_0" / f"{frame_id}.pcd").write_text(
        "VERSION .7\nFIELDS x y z intensity\nDATA binary\n",
        encoding="utf-8",
    )
    (scene_dir / "label_2").mkdir(parents=True, exist_ok=True)
    (scene_dir / "label_2" / f"{frame_id}.txt").write_text(raw_label, encoding="utf-8")


def create_xtreme_result(xtreme_root: Path, scene_name: str, frame_id: str, payload=None):
    result_dir = xtreme_root / scene_name / "result"
    result_dir.mkdir(parents=True, exist_ok=True)
    if payload is None:
        payload = [{"objects": []}]
    (result_dir / f"{frame_id}.json").write_text(json.dumps(payload), encoding="utf-8")


def stub_external_effects(module, dataset_root: Path):
    def fake_pcd_to_bin(_pcd_path, bin_path):
        Path(bin_path).write_bytes(b"\x00" * 16)
        return True

    def fake_archive(base_name, format, root_dir, base_dir):
        archive_path = Path(f"{base_name}.{format}")
        archive_path.write_bytes(b"zip")
        return str(archive_path)

    module.pcd_to_bin_fixed = fake_pcd_to_bin
    module.generate_nuscenes_metadata = lambda *_args, **_kwargs: None
    module.shutil.make_archive = fake_archive
    module.FINAL_KITTI_OUTPUT_DIR = dataset_root


def read_output_labels(output_root: Path):
    return sorted(
        (output_root).glob("3DBox_Annotation_Final_*_unit_test/training/label_2/*.txt")
    )


def test_xtreme_scene_without_json_is_trusted_as_empty_frame(tmp_path):
    module = load_merge_module()

    raw_root = tmp_path / "raw"
    xtreme_root = tmp_path / "xtreme"
    output_root = tmp_path / "output"
    frame_id = "1710000000000000000"
    scene_dir = raw_root / "data_record_20260414" / "Scene_01"
    create_raw_frame(scene_dir, frame_id, "Pole 0 0 0 0 0 10 10 1 1 1 0 0 10 0 0 0\n")

    (xtreme_root / "Scene_01" / "result").mkdir(parents=True, exist_ok=True)

    module.RAW_ARCHIVE_ROOT = raw_root
    module.XTREME_EXPORT_ROOT = xtreme_root
    stub_external_effects(module, output_root)
    module.MAX_EMPTY_LABEL_RATIO = 1.0

    module.build_final_dataset("unit_test")

    label_files = read_output_labels(output_root)
    assert len(label_files) == 1
    assert label_files[0].read_text(encoding="utf-8") == ""


def test_missing_xtreme_scene_is_treated_as_empty_label(tmp_path):
    module = load_merge_module()

    raw_root = tmp_path / "raw"
    output_root = tmp_path / "output"
    frame_id = "1710000000000000001"
    raw_label = "Pole 0 0 0 0 0 10 10 1 1 1 0 0 10 0 0 0\n"
    scene_dir = raw_root / "data_record_20260414" / "Scene_01"
    create_raw_frame(scene_dir, frame_id, raw_label)

    module.RAW_ARCHIVE_ROOT = raw_root
    module.XTREME_EXPORT_ROOT = tmp_path / "xtreme"
    stub_external_effects(module, output_root)
    module.MAX_EMPTY_LABEL_RATIO = 1.0

    module.build_final_dataset("unit_test")

    label_files = read_output_labels(output_root)
    assert len(label_files) == 1
    assert label_files[0].read_text(encoding="utf-8") == ""


def test_build_final_dataset_accepts_runtime_roots(tmp_path):
    module = load_merge_module()

    raw_root = tmp_path / "raw"
    xtreme_root = tmp_path / "xtreme"
    output_root = tmp_path / "output"
    scene_dir = raw_root / "data_record_20260414" / "Scene_01"

    create_raw_frame(scene_dir, "frame_001", "raw-label\n")
    create_xtreme_result(xtreme_root, "Scene_01", "frame_001")

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
        max_empty_label_ratio=1.0,
    )

    label_files = read_output_labels(output_root)
    assert len(label_files) == 1


def test_empty_labels_are_globally_sampled_to_ten_percent(tmp_path):
    module = load_merge_module()

    raw_root = tmp_path / "raw"
    xtreme_root = tmp_path / "xtreme"
    output_root = tmp_path / "output"
    positive_scene_dir = raw_root / "data_record_20260414" / "Scene_01"
    empty_scene_dir = raw_root / "data_record_20260414" / "Scene_02"

    for index in range(9):
        frame_id = f"positive_{index}"
        create_raw_frame(positive_scene_dir, frame_id, "raw-label\n")
        create_xtreme_result(xtreme_root, "Scene_01", frame_id)

    for index in range(9):
        frame_id = f"empty_{index}"
        create_raw_frame(empty_scene_dir, frame_id, "raw-label\n")

    module.RAW_ARCHIVE_ROOT = raw_root
    module.XTREME_EXPORT_ROOT = xtreme_root
    stub_external_effects(module, output_root)
    module.parse_xtreme_to_kitti_lines = lambda *_args, **_kwargs: [
        "Car 0.00 0 0.00 0.00 0.00 10.00 10.00 1.00 1.00 1.00 0.00 0.00 10.00 0.00 0.00 0.00\n"
    ]

    module.build_final_dataset("unit_test")

    label_files = read_output_labels(output_root)
    assert len(label_files) == 10
    empty_count = sum(
        1 for label_file in label_files if label_file.read_text(encoding="utf-8") == ""
    )
    assert empty_count == 1


def test_xtreme_json_with_no_boxes_counts_as_empty(tmp_path):
    module = load_merge_module()

    raw_root = tmp_path / "raw"
    xtreme_root = tmp_path / "xtreme"
    output_root = tmp_path / "output"
    scene_dir = raw_root / "data_record_20260414" / "Scene_01"

    create_raw_frame(scene_dir, "positive_frame", "raw-label\n")
    create_xtreme_result(xtreme_root, "Scene_01", "positive_frame")
    create_raw_frame(scene_dir, "empty_xtreme_frame", "raw-label\n")
    create_xtreme_result(xtreme_root, "Scene_01", "empty_xtreme_frame")

    module.RAW_ARCHIVE_ROOT = raw_root
    module.XTREME_EXPORT_ROOT = xtreme_root
    stub_external_effects(module, output_root)
    module.MAX_EMPTY_LABEL_RATIO = 1.0
    module.parse_xtreme_to_kitti_lines = lambda xtreme_json_path, _config_path: (
        []
        if Path(xtreme_json_path).stem == "empty_xtreme_frame"
        else [
            "Car 0.00 0 0.00 0.00 0.00 10.00 10.00 1.00 1.00 1.00 0.00 0.00 10.00 0.00 0.00 0.00\n"
        ]
    )

    module.build_final_dataset("unit_test")

    label_texts = [path.read_text(encoding="utf-8") for path in read_output_labels(output_root)]
    assert len(label_texts) == 2
    assert "" in label_texts
    assert any(text.strip() for text in label_texts)


def test_merge_fails_when_only_empty_frames_remain(tmp_path):
    module = load_merge_module()

    raw_root = tmp_path / "raw"
    output_root = tmp_path / "output"
    scene_dir = raw_root / "data_record_20260414" / "Scene_01"

    create_raw_frame(scene_dir, "empty_0", "raw-label\n")
    create_raw_frame(scene_dir, "empty_1", "raw-label\n")

    module.RAW_ARCHIVE_ROOT = raw_root
    module.XTREME_EXPORT_ROOT = tmp_path / "xtreme"
    stub_external_effects(module, output_root)

    with pytest.raises(ValueError, match="zero positive frames"):
        module.build_final_dataset("unit_test")
