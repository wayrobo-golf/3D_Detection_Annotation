from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, List, Tuple

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
                data[key] = values.reshape(3, 4)  # 相机投影矩阵，从立体校正后的相机坐标系到图像
            elif key in {"R0_rect", "R_rect"} and values.size == 9:
                data[key] = values.reshape(3, 3)  # 修正旋转矩阵，从相机0坐标系到立体校正后的相机坐标系
            elif key in {"Tr_velo_to_cam", "Tr_velo_cam", "Tr_cam_to_velo", "Tr_velo_to_map"} and values.size == 12:
                data[key] = values.reshape(3, 4)  # 相机外参，从lidar坐标系到相机0坐标系
            else:
                data[key] = values

    if "Tr_velo_to_cam" not in data and "Tr_velo_cam" in data:
        data["Tr_velo_to_cam"] = data["Tr_velo_cam"]
    if "R0_rect" not in data and "R_rect" in data:
        data["R0_rect"] = data["R_rect"]
    if "R0_rect" not in data:
        data["R0_rect"] = np.eye(3, dtype=np.float64)
    if "Tr_velo_to_cam" not in data:
        raise KeyError(f"Missing Tr_velo_to_cam in calib: {calib_path}")
    if "P2" not in data:
        raise KeyError(f"Missing P2 in calib: {calib_path}")
    return data


def to_homogeneous_4x4(matrix_3x4: np.ndarray) -> np.ndarray:
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :4] = matrix_3x4
    return mat


def matrix_to_quaternion_xyzw(rotation: np.ndarray) -> List[float]:
    m = rotation
    trace = float(np.trace(m))

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    else:
        if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s

    quat = np.array([w, x, y, z], dtype=np.float64)
    quat /= np.linalg.norm(quat) + 1e-12
    return quat.tolist()


def average_rotation(rotations: List[np.ndarray]) -> np.ndarray:
    mean_rot = np.mean(np.stack(rotations, axis=0), axis=0)
    u, _, vt = np.linalg.svd(mean_rot)
    rot = u @ vt
    if np.linalg.det(rot) < 0:
        u[:, -1] *= -1.0
        rot = u @ vt
    return rot


def collect_calibs(calib_dir: Path) -> List[Dict[str, np.ndarray]]:
    files = sorted([p for p in calib_dir.glob("*.txt") if "_origin" not in p.name])
    if not files:
        raise FileNotFoundError(f"No calib files found in: {calib_dir}")
    return [parse_calib(p) for p in files]


def build_calibrated_sensor(calibs: List[Dict[str, np.ndarray]]) -> List[Dict[str, object]]:
    p2_list = [c["P2"] for c in calibs]
    intrinsic = np.mean(np.stack([p[:3, :3] for p in p2_list], axis=0), axis=0)

    lidar_to_cam_rect_list = []
    cam_to_ego_rot_list = []
    cam_to_ego_trans_list = []

    for calib in calibs:
        tr_velo_to_cam = to_homogeneous_4x4(calib["Tr_velo_to_cam"])
        r0_rect = np.eye(4, dtype=np.float64)
        r0_rect[:3, :3] = calib["R0_rect"]
        lidar_to_cam_rect = r0_rect @ tr_velo_to_cam
        lidar_to_cam_rect_list.append(lidar_to_cam_rect)

        cam_to_ego = np.linalg.inv(lidar_to_cam_rect)
        cam_to_ego_rot_list.append(cam_to_ego[:3, :3])
        cam_to_ego_trans_list.append(cam_to_ego[:3, 3])

    cam_to_ego_rot = average_rotation(cam_to_ego_rot_list)
    cam_to_ego_trans = np.mean(np.stack(cam_to_ego_trans_list, axis=0), axis=0)

    return [
        {
            "token": "calib_lidar_top",  # lidar到车体坐标系
            "sensor_token": "sensor_lidar_top",
            "translation": [0.0, 0.0, 0.0],
            "rotation": [1.0, 0.0, 0.0, 0.0],
            "camera_intrinsic": [],
            "camera_distortion": [],
            "camera_model": "",
        },
        {
            "token": "calib_cam_front",  # 相机到车体坐标系
            "sensor_token": "sensor_cam_front",
            "translation": cam_to_ego_trans.round(9).tolist(),
            "rotation": [round(v, 9) for v in matrix_to_quaternion_xyzw(cam_to_ego_rot)],
            "camera_intrinsic": intrinsic.round(9).tolist(),
            "camera_distortion": [],
            "camera_model": "pinhole",
        },
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description="Export calibrated_sensor.json from KITTI calib files")
    parser.add_argument("--data-root", type=Path, required=True, help="Dataset root, e.g. 3DBox_Annotation_xxx")
    parser.add_argument("--split", type=str, default="training", choices=["training", "testing"])
    parser.add_argument("--output", type=Path, default=None, help="Output path, default: <data-root>/calibrated_sensor.json")
    args = parser.parse_args()

    calib_dir = args.data_root / args.split / "calib"
    output_path = args.output if args.output is not None else args.data_root / "calibrated_sensor.json"

    calibs = collect_calibs(calib_dir)
    result = build_calibrated_sensor(calibs)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    print(f"Saved: {output_path}")
    print(f"Entries: {len(result)}")


if __name__ == "__main__":
    main()
