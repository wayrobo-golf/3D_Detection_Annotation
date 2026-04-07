from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from collections import defaultdict
from typing import Any, Dict, List

import numpy as np
import scipy.spatial.transform


def make_rotation_global(yaw: float, roll: float = 0.0, pitch: float = 0.0) -> Dict[str, float]:
    return {
        "roll": float(roll),
        "pitch": float(pitch),
        "yaw": float(yaw),
    }


def circular_mean(angles: np.ndarray) -> float:
    if angles.size == 0:
        return 0.0
    return float(math.atan2(np.sin(angles).mean(), np.cos(angles).mean()))


def union_find_clusters(points: np.ndarray, distance_threshold: float) -> List[List[int]]:
    n = len(points)
    parent = list(range(n))

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a: int, b: int) -> None:
        ra = find(a)
        rb = find(b)
        if ra != rb:
            parent[rb] = ra

    for i in range(n):
        for j in range(i + 1, n):
            if np.linalg.norm(points[i] - points[j]) <= distance_threshold:
                union(i, j)

    groups: Dict[int, List[int]] = defaultdict(list)
    for i in range(n):
        groups[find(i)].append(i)
    return list(groups.values())


def choose_representative_bbox(instances: List[Dict[str, Any]]) -> List[float]:
    valid = []
    for ins in instances:
        bbox = ins.get("bbox_2d", [])
        if len(bbox) == 4 and any(float(v) >= 0 for v in bbox):
            valid.append([float(v) for v in bbox])
    if not valid:
        return [-1.0, -1.0, -1.0, -1.0]
    arr = np.asarray(valid, dtype=np.float64)
    return arr.mean(axis=0).tolist()


def merge_instances(data: Dict[str, Any], distance_threshold: float) -> Dict[str, Any]:
    by_class: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for ins in data.get("instances", []):
        by_class[str(ins.get("class_name", "Unknown"))].append(ins)

    merged_instances: List[Dict[str, Any]] = []

    for class_name, instances in sorted(by_class.items()):
        centers = np.asarray([ins["center_global"] for ins in instances], dtype=np.float64)
        clusters = union_find_clusters(centers, distance_threshold=distance_threshold)

        for cluster_idx, cluster in enumerate(clusters):
            group = [instances[i] for i in cluster]
            group_sorted = sorted(group, key=lambda x: str(x.get("sample_id", "")))
            centers_arr = np.asarray([ins["center_global"] for ins in group_sorted], dtype=np.float64)
            dims_arr = np.asarray([ins["dimensions_hwl"] for ins in group_sorted], dtype=np.float64)
            yaws_arr = np.asarray([float(ins["rotation_global"]["yaw"]) for ins in group_sorted], dtype=np.float64)

            yaw_global = circular_mean(yaws_arr)

            merged_instances.append(
                {
                    "instance_id": f"{class_name.lower()}_{cluster_idx:03d}",
                    "class_name": class_name,
                    "num_observations": len(group_sorted),
                    "dimensions_hwl": dims_arr.mean(axis=0).round(6).tolist(),
                    "center_global": centers_arr.mean(axis=0).round(6).tolist(),
                    "rotation_global": make_rotation_global(yaw=yaw_global),
                }
            )

    return {
        "dataset_root": data.get("dataset_root"),
        "split": data.get("split"),
        "coordinate_system": data.get("coordinate_system", "global"),
        "merge_distance_threshold": float(distance_threshold),
        "num_instances": len(merged_instances),
        "instances": merged_instances,
    }


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

    if "Tr_velo_to_cam" not in data and "Tr_velo_cam" in data:
        data["Tr_velo_to_cam"] = data["Tr_velo_cam"]

    if "R0_rect" not in data and "R_rect" in data:
        data["R0_rect"] = data["R_rect"]

    if "R0_rect" not in data:
        data["R0_rect"] = np.eye(3, dtype=np.float64)

    if "Tr_velo_to_cam" not in data:
        raise KeyError(f"Missing Tr_velo_to_cam in calib: {calib_path}")
    if "Tr_velo_to_map" not in data:
        raise KeyError(f"Missing Tr_velo_to_map in calib: {calib_path}")

    return data


def to_homogeneous_4x4(matrix_3x4: np.ndarray) -> np.ndarray:
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :4] = matrix_3x4
    return mat


def get_rect_to_velo(calib: Dict[str, np.ndarray]) -> np.ndarray:
    tr_velo_to_cam = to_homogeneous_4x4(calib["Tr_velo_to_cam"])
    r0_rect = np.eye(4, dtype=np.float64)
    r0_rect[:3, :3] = calib["R0_rect"]
    tr_velo_to_rect = r0_rect @ tr_velo_to_cam
    return np.linalg.inv(tr_velo_to_rect)


def parse_label_line(line: str) -> Dict[str, Any]:
    parts = line.strip().split()
    if len(parts) < 15:
        raise ValueError(f"Invalid label line (<15 fields): {line}")

    extra_values: List[float] = []
    for value in parts[15:]:
        try:
            extra_values.append(float(value))
        except ValueError:
            pass

    return {
        "type": parts[0],
        "truncated": float(parts[1]),
        "occluded": int(float(parts[2])),
        "alpha": float(parts[3]),
        "bbox_2d": [float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])],
        "dimensions": [float(parts[8]), float(parts[9]), float(parts[10])],  # h, w, l
        "location_cam": [float(parts[11]), float(parts[12]), float(parts[13])],
        "rotation_y": float(parts[14]), # yaw(ry)
        "extra": extra_values, # pitch(rx), roll(rz)
    }

def make_box_corners_local(h: float, w: float, l: float):
    x_corners = np.array([-l / 2, -l / 2, -l / 2, -l / 2, l / 2, l / 2, l / 2, l / 2], dtype=np.float64)
    y_corners = np.array([-h, 0, 0, -h, -h, 0, 0, -h], dtype=np.float64)
    z_corners = np.array([-w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2, w / 2], dtype=np.float64)

    corners = np.vstack([x_corners, y_corners, z_corners])
    return corners.T

def make_box_corners_camera(h: float, w: float, l: float, x: float, y: float, z: float, rx: float, ry: float, rz: float) -> np.ndarray:
    corners_local = make_box_corners_local(h, w, l)

    rot = scipy.spatial.transform.Rotation.from_euler("xyz", [rx, ry, rz], degrees=False).as_matrix().astype(np.float64)
    translation = np.array([x, y, z], dtype=np.float64)

    corners_camera = np.dot(corners_local, rot.T) + translation
    return corners_camera


def transform_points(points_xyz: np.ndarray, transform: np.ndarray) -> np.ndarray:
    points_h = np.concatenate([points_xyz, np.ones((points_xyz.shape[0], 1), dtype=np.float64)], axis=1)
    transformed = (transform @ points_h.T).T
    return transformed[:, :3]


def transform_point(point_xyz: np.ndarray, transform: np.ndarray) -> np.ndarray:
    point_h = np.array([point_xyz[0], point_xyz[1], point_xyz[2], 1.0], dtype=np.float64)
    transformed = transform @ point_h
    return transformed[:3]

def compute_rpy_global(rpy: float, rect_to_velo: np.ndarray, velo_to_map: np.ndarray) -> float:
    # 框本体到相机坐标系的旋转
    rot = scipy.spatial.transform.Rotation.from_euler("xyz", rpy, degrees=False).as_matrix().astype(np.float64)
    # 框本体到global坐标系的旋转
    rot_velo = rect_to_velo[:3, :3] @ rot
    rot_global = velo_to_map[:3, :3] @ rot_velo 

    rpy_global = scipy.spatial.transform.Rotation.from_matrix(rot_global).as_euler("xyz", degrees=False)
    return rpy_global

def compute_yaw_global(rpy: float, rect_to_velo: np.ndarray, velo_to_map: np.ndarray) -> float:
    rpy_global = compute_rpy_global(rpy, rect_to_velo, velo_to_map)
    # 忽略小角度的roll和pitch
    if abs(rpy_global[0] / np.pi + 0.5) * 180 > 3.0 or abs(rpy_global[1]) / np.pi * 180 > 3.0:
        print(f"Warning: large roll/pitch in global frame: roll={rpy_global[0]:.3f}, pitch={rpy_global[1]:.3f}")
    return rpy_global[2]

def collect_label_files(dataset_root: Path, split: str) -> List[Path]:
    label_dir = dataset_root / split / "label_2"
    if not label_dir.exists():
        raise FileNotFoundError(f"Missing label directory: {label_dir}")
    return sorted([p for p in label_dir.glob("*.txt") if "_origin" not in p.name])


def build_static_instances(dataset_root: Path, split: str, include_dontcare: bool) -> Dict[str, Any]:
    records: List[Dict[str, Any]] = []
    label_files = collect_label_files(dataset_root, split)

    for label_path in label_files:
        sample_id = label_path.stem
        calib_path = dataset_root / split / "calib" / f"{sample_id}.txt"
        if not calib_path.exists():
            continue

        calib = parse_calib(calib_path)
        rect_to_velo = get_rect_to_velo(calib)  # 从立体校正后的相机坐标系到lidar坐标系的外参
        velo_to_map = to_homogeneous_4x4(calib["Tr_velo_to_map"])  # 从车体坐标系到地图坐标系的变换矩阵

        with label_path.open("r", encoding="utf-8") as f:
            for obj_idx, raw_line in enumerate(f):
                raw_line = raw_line.strip()
                if not raw_line:
                    continue

                ann = parse_label_line(raw_line)  # 解析一个标注结果
                cls = ann["type"]
                if not include_dontcare and cls == "DontCare":
                    continue

                h, w, l = ann["dimensions"]
                x, y, z = ann["location_cam"]  # 框的底面中心（可能对应相机坐标系下框的顶面中心）
                rpy = [0.0, ann["rotation_y"], 0.0]
                if len(ann["extra"]) >= 2:
                    rpy = [ann["extra"][0], ann["rotation_y"], ann["extra"][1]]

                corners_cam = make_box_corners_camera(h, w, l, x, y, z, *rpy)  # 计算框的8个角点在相机坐标系下的位置（可选）
                corners_cam_homogeneous = np.concatenate(
                    [corners_cam, np.ones((corners_cam.shape[0], 1), dtype=np.float64)], axis=1
                )
                corners_velo_homogeneous = np.dot(corners_cam_homogeneous, rect_to_velo.T)
                corners_global_homogeneous = np.dot(corners_velo_homogeneous, velo_to_map.T)
                center_global = np.average(corners_global_homogeneous, axis=0)[:3]
                
                yaw_global = compute_yaw_global(rpy, rect_to_velo, velo_to_map)

                records.append(
                    {
                        "instance_id": f"{sample_id}_{obj_idx:03d}",
                        "sample_id": sample_id,
                        "class_name": cls,
                        "bbox_2d": ann["bbox_2d"],
                        "dimensions_hwl": [float(h), float(w), float(l)],
                        "center_global": [float(v) for v in center_global.tolist()],
                        "rotation_global": make_rotation_global(yaw=yaw_global),
                        "source": {
                            "location_cam": [float(x), float(y), float(z)],
                            "rotation_y": float(ann["rotation_y"]),
                            "truncated": float(ann["truncated"]),
                            "occluded": int(ann["occluded"]),
                            "alpha": float(ann["alpha"]),
                            "extra": ann["extra"],
                        },
                    }
                )

    return {
        "dataset_root": str(dataset_root),
        "split": split,
        "coordinate_system": "global",
        "num_instances": len(records),
        "instances": records,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Extract KITTI label_2 annotations and export static_instance.json in global coordinates"
    )
    parser.add_argument("--data-root", type=Path, required=True, help="KITTI dataset root, e.g. 3DBox_Annotation_xxx")
    parser.add_argument("--split", type=str, default="training", choices=["training", "testing"])
    parser.add_argument("--output", type=Path, default=None, help="Output json path, default: <data-root>/static_instance.json")
    parser.add_argument("--include-dontcare", action="store_true", help="Include DontCare annotations")
    parser.add_argument("--merge-nearby", action="store_true", help="Merge same-class instances with nearby global centers")
    parser.add_argument("--merge-distance-threshold", type=float, default=1.0, help="Merge threshold in meters when --merge-nearby is enabled")
    args = parser.parse_args()

    output_path = args.output if args.output is not None else args.data_root / "static_instance.json"

    result = build_static_instances(
        dataset_root=args.data_root,
        split=args.split,
        include_dontcare=args.include_dontcare,
    )

    if args.merge_nearby:
        result = merge_instances(result, distance_threshold=args.merge_distance_threshold)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    print(f"Saved: {output_path}")
    if args.merge_nearby:
        print(f"Merged instances: {result['num_instances']}")
    else:
        print(f"Instances: {result['num_instances']}")


if __name__ == "__main__":
    main()
