import numpy as np
from my_package.core.data_types import BoundingBox3D

class TransformMath:
    @staticmethod
    def euler_to_matrix(rx, ry, rz) -> np.ndarray:
        Rx = np.array([[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]])
        Ry = np.array([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]])
        Rz = np.array([[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    @staticmethod
    def get_box_corners_map(box: BoundingBox3D) -> np.ndarray:
        hl, hw, hh = box.l / 2, box.w / 2, box.h / 2
        corners_local = np.array([
            [hl, hw, hh], [hl, -hw, hh], [-hl, -hw, hh], [-hl, hw, hh],
            [hl, hw, -hh], [hl, -hw, -hh], [-hl, -hw, -hh], [-hl, hw, -hh]
        ]).T
        R = TransformMath.euler_to_matrix(box.rx, box.ry, box.rz)
        T = np.array([[box.tx], [box.ty], [box.tz]])
        return np.vstack((R @ corners_local + T, np.ones((1, 8))))

class Projector:
    @staticmethod
    def project_to_lidar(corners_map_h: np.ndarray, T_lidar2map_new: np.ndarray) -> np.ndarray:
        T_map2lidar = np.linalg.inv(T_lidar2map_new)
        return (T_map2lidar @ corners_map_h)[:3, :]

    @staticmethod
    def project_to_image(corners_lidar: np.ndarray, T_lidar2cam: np.ndarray, K: np.ndarray) -> np.ndarray:
        corners_lidar_h = np.vstack((corners_lidar, np.ones((1, 8))))
        corners_cam_h = T_lidar2cam @ corners_lidar_h
        corners_cam = corners_cam_h[:3, :]
        if np.any(corners_cam[2, :] < 0.1): return None
        uv = K @ corners_cam
        return (uv[:2, :] / uv[2, :]).astype(int)

    @staticmethod
    def count_points_in_box(pcd_points: np.ndarray, corners_lidar: np.ndarray, box: BoundingBox3D) -> int:
        """ 实现 AABB 粗筛与局部坐标精确过滤"""
        if len(pcd_points) == 0: return 0
        min_p, max_p = np.min(corners_lidar, axis=1), np.max(corners_lidar, axis=1)
        mask = (pcd_points[:, 0] >= min_p[0]) & (pcd_points[:, 0] <= max_p[0]) & \
               (pcd_points[:, 1] >= min_p[1]) & (pcd_points[:, 1] <= max_p[1]) & \
               (pcd_points[:, 2] >= min_p[2]) & (pcd_points[:, 2] <= max_p[2])
        pts_aabb = pcd_points[mask]
        if len(pts_aabb) < 50: return len(pts_aabb)
        
        center = np.mean(corners_lidar, axis=1)
        v_x = (corners_lidar[:, 0] - corners_lidar[:, 3]) / np.linalg.norm(corners_lidar[:, 0] - corners_lidar[:, 3])
        v_y = (corners_lidar[:, 0] - corners_lidar[:, 1]) / np.linalg.norm(corners_lidar[:, 0] - corners_lidar[:, 1])
        v_z = (corners_lidar[:, 0] - corners_lidar[:, 4]) / np.linalg.norm(corners_lidar[:, 0] - corners_lidar[:, 4])
        R_inv = np.column_stack((v_x, v_y, v_z)).T
        pts_local = (R_inv @ (pts_aabb - center).T).T
        mask_exact = (np.abs(pts_local[:, 0]) <= box.l/2) & (np.abs(pts_local[:, 1]) <= box.w/2) & (np.abs(pts_local[:, 2]) <= box.h/2)
        return np.sum(mask_exact)