import cv2, json, os, glob
import open3d as o3d
import numpy as np
import xml.etree.ElementTree as ET
from typing import List, Dict, Any
from my_package.core.data_types import BoundingBox3D, SensorData

class DataLoader:
    def __init__(self, data_root_dir: str, xml_path: str):
        self.data_root_dir = data_root_dir
        self.xml_path = xml_path
        # 初始化时就解析 XML，因为这些全局真值在所有帧里都是静态共享的
        self.global_boxes = self._parse_xml()

    def _parse_xml(self) -> List[BoundingBox3D]:
        """解析 XML 获取 Map 坐标系下的全局 3D 框"""
        print(f"Loading global map annotations from: {self.xml_path}")
        tree = ET.parse(self.xml_path)
        root = tree.getroot()
        tracklets = root.find('tracklets')
        boxes = []
        
        valid_classes = {
            "Distance Marker": "Distance_Marker", 
            "Structure": "Unloading_Station", 
            "flag": "Pole",
            "Chipping Net": "Chipping_Net"
        }
        
        for item in tracklets.findall('item'):
            obj_type_elem = item.find('objectType')
            if obj_type_elem is None or obj_type_elem.text not in valid_classes:
                continue
            
            obj_type = valid_classes[obj_type_elem.text]
            
            raw_h = float(item.find('h').text)
            raw_w = float(item.find('w').text)
            raw_l = float(item.find('l').text)
            
            real_h = raw_l  
            real_l = raw_w  
            real_w = raw_h  
            
            poses = item.find('poses')
            if poses is not None:
                pose_item = poses.find('item') 
                if pose_item is not None:
                    tx = float(pose_item.find('tx').text)
                    ty = float(pose_item.find('ty').text)
                    tz = float(pose_item.find('tz').text)
                    rx = float(pose_item.find('rx').text)
                    ry = float(pose_item.find('ry').text)
                    rz = float(pose_item.find('rz').text)
                    
                    boxes.append(BoundingBox3D(
                        type=obj_type, 
                        l=real_l, w=real_w, h=real_h, 
                        tx=tx, ty=ty, tz=tz, 
                        rx=rx, ry=ry, rz=rz
                    ))
        print(f"Loaded {len(boxes)} valid bounding boxes.")
        return boxes

    def get_all_frames(self) -> List[Dict[str, Any]]:
        """跨 Scene 深度扫描所有有效数据帧"""
        frames = []
        print(f"\n🔍 正在深度扫描目录: {self.data_root_dir}")
        
        for root, dirs, files in os.walk(self.data_root_dir):
            if "camera_config" in dirs:
                scene_dir = root
                scene_name = os.path.basename(root)
                config_dir = os.path.join(scene_dir, "camera_config")
                
                json_files = glob.glob(os.path.join(config_dir, "*.json"))
                valid_json_count = 0
                skipped_due_to_missing = 0 # 记录因为缺文件被跳过的帧数
                
                for json_path in json_files:
                    # 【核心修复】：只检查文件名，绝不能检查完整的绝对路径！
                    filename = os.path.basename(json_path)
                    if "_origin" in filename:
                        continue 
                    
                    frame_id = filename.replace(".json", "")
                    
                    # 严谨校验：确保图像和点云也同时存在
                    img_pattern = os.path.join(scene_dir, "camera_image_0", f"{frame_id}.*")
                    img_exists = len(glob.glob(img_pattern)) > 0
                    pcd_path = os.path.join(scene_dir, "lidar_point_cloud_0", f"{frame_id}.pcd")
                    pcd_exists = os.path.exists(pcd_path)
                    
                    # 如果数据不全，记录跳过并继续
                    if not (img_exists and pcd_exists):
                        skipped_due_to_missing += 1
                        continue 
                        
                    valid_json_count += 1
                    
                    origin_json = os.path.join(config_dir, f"{frame_id}_origin.json")
                    is_edited = os.path.exists(origin_json)
                    
                    frames.append({
                        "frame_id": frame_id,
                        "scene_dir": scene_dir,
                        "scene_name": scene_name,
                        "is_edited": is_edited
                    })
                    
                # 打印当前 Scene 的扫描报告
                if valid_json_count > 0 or skipped_due_to_missing > 0:
                    print(f"  -> 📁 '{scene_name}': 找到 {valid_json_count} 帧完整数据 "
                          f"(因缺失图/点云跳过 {skipped_due_to_missing} 帧残缺数据)")
        
        frames.sort(key=lambda x: x["frame_id"])
        print(f"✅ 全局扫描完成！共找到 {len(frames)} 帧可微调数据。")
        return frames

    def load_frame(self, frame_info: Dict[str, Any]) -> SensorData:
        """根据全局索引字典，定位并加载特定帧"""
        frame_id = frame_info["frame_id"]
        scene_dir = frame_info["scene_dir"]
        
        img_pattern = os.path.join(scene_dir, "camera_image_0", f"{frame_id}.*")
        img_paths = glob.glob(img_pattern)
        if not img_paths:
            raise FileNotFoundError(f"Image not found for frame {img_pattern}")
        img = cv2.imread(img_paths[0])
        
        pcd_path = os.path.join(scene_dir, "lidar_point_cloud_0", f"{frame_id}.pcd")
        pcd = o3d.io.read_point_cloud(pcd_path)
        
        json_path = os.path.join(scene_dir, "camera_config", f"{frame_id}.json")
        with open(json_path, 'r') as f:
            config = json.load(f)[0] 
            
        intrinsics = config["camera_internal"]
        K = np.array([
            [intrinsics["fx"], 0, intrinsics["cx"]],
            [0, intrinsics["fy"], intrinsics["cy"]],
            [0, 0, 1]
        ])
        
        T_lidar2cam = np.array(config["camera_external"]).reshape((4, 4), order='F')
        
        if "tf_lidar_to_map" in config:
            T_lidar2map = np.array(config["tf_lidar_to_map"]).reshape((4, 4), order='F')
        else:
            T_lidar2map = np.eye(4)
            
        return SensorData(img, pcd, K, T_lidar2cam, T_lidar2map, self.global_boxes)