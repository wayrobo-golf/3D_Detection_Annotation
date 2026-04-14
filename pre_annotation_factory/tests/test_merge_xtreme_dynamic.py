import importlib.util
import json
from pathlib import Path


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

    module.build_final_dataset("unit_test")

    label_files = sorted((output_root).glob("3DBox_Annotation_Final_*_unit_test/training/label_2/*.txt"))
    assert len(label_files) == 1
    assert label_files[0].read_text(encoding="utf-8") == ""


def test_raw_label_is_used_when_scene_never_entered_xtreme(tmp_path):
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

    module.build_final_dataset("unit_test")

    label_files = sorted((output_root).glob("3DBox_Annotation_Final_*_unit_test/training/label_2/*.txt"))
    assert len(label_files) == 1
    assert label_files[0].read_text(encoding="utf-8") == raw_label
