import numpy as np
import open3d as o3d
from dataclasses import dataclass
from typing import List

@dataclass
class BoundingBox3D:
    """定义全局地图下的 3D 边界框"""
    type: str
    l: float; w: float; h: float
    tx: float; ty: float; tz: float
    rx: float; ry: float; rz: float

@dataclass
class SensorData:
    """统一管理一帧内的所有多模态数据"""
    image: np.ndarray
    pcd: o3d.geometry.PointCloud
    K: np.ndarray              
    T_lidar2cam: np.ndarray    
    T_lidar2map: np.ndarray    
    global_boxes: List[BoundingBox3D]