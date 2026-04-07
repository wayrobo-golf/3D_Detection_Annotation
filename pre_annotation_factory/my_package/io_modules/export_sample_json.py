from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np


def parse_calib(calib_path: Path) -> Dict[str, np.ndarray]:
    data: Dict[str, np.ndarray] = {}
    with calib_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or ":" not in line:
                continue
            key, value_str = line.split(":", 1)
            values = np.fromstring(value_str, sep=" ", dtype=np.float64)
            if key.startswith("P") and values.size == 12:
                data[key] = values.reshape(3, 4)
            elif key in {"R0_rect", "R_rect"} and values.size == 9:
                data[key] = values.reshape(3, 3)
            elif key in {"Tr_velo_to_cam", "Tr_velo_cam", "Tr_cam_to_velo", "Tr_velo_to_map"} and values.size == 12:
                data[key] = values.reshape(3, 4)
            else:
                data[key] = values

    if "Tr_velo_to_map" not in data:
        raise KeyError(f"Missing Tr_velo_to_map in calib: {calib_path}")
    return data


def to_homogeneous_4x4(matrix_3x4: np.ndarray) -> np.ndarray:
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :4] = matrix_3x4
    return mat


def resolve_image_file(image_dir: Path, sample_id: str) -> Optional[Path]:
    for ext in (".jpg", ".png", ".jpeg", ".bmp"):
        path = image_dir / f"{sample_id}{ext}"
        if path.exists():
            return path
    return None


def build_samples(dataset_root: Path, split: str, location: str) -> List[Dict[str, object]]:
    split_dir = dataset_root / split
    calib_dir = split_dir / "calib"
    velodyne_dir = split_dir / "velodyne"
    image_dir = split_dir / "image_2"

    if not calib_dir.exists():
        raise FileNotFoundError(f"Missing directory: {calib_dir}")
    if not velodyne_dir.exists():
        raise FileNotFoundError(f"Missing directory: {velodyne_dir}")
    if not image_dir.exists():
        raise FileNotFoundError(f"Missing directory: {image_dir}")

    samples: List[Dict[str, object]] = []
    for calib_path in sorted(calib_dir.glob("*.txt")):
        if "_origin" in calib_path.name:
            continue
            
        sample_id = calib_path.stem
        lidar_path = velodyne_dir / f"{sample_id}.bin"
        image_path = resolve_image_file(image_dir, sample_id)

        if not lidar_path.exists() or image_path is None:
            continue

        calib = parse_calib(calib_path)
        ego2global = to_homogeneous_4x4(calib["Tr_velo_to_map"]).round(9).tolist()

        samples.append(
            {
                "token": sample_id,
                "timestamp": int(sample_id),
                "location": location,
                "ego2global": ego2global,  # 从车体坐标系到地图坐标系的变换矩阵
                "lidar_points": {
                    "lidar_path": str(lidar_path.relative_to(dataset_root)).replace("\\", "/"),
                },
                "images": {
                    "CAM_FRONT": {
                        "img_path": str(image_path.relative_to(dataset_root)).replace("\\", "/"),
                    }
                },
            }
        )

    return samples


def main() -> None:
    parser = argparse.ArgumentParser(description="Export sample.json from KITTI calib, velodyne and image data")
    parser.add_argument("--data-root", type=Path, required=True, help="Dataset root, e.g. 3DBox_Annotation_xxx")
    parser.add_argument("--split", type=str, default="training", choices=["training", "testing"])
    parser.add_argument("--location", type=str, default=None, help="Location name, default uses dataset folder name")
    parser.add_argument("--output", type=Path, default=None, help="Output path, default: <data-root>/sample.json")
    args = parser.parse_args()

    location = args.location if args.location is not None else args.data_root.name
    output_path = args.output if args.output is not None else args.data_root / "sample.json"

    samples = build_samples(args.data_root, args.split, location)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(samples, f, ensure_ascii=False, indent=2)

    print(f"Saved: {output_path}")
    print(f"Samples: {len(samples)}")


if __name__ == "__main__":
    main()
