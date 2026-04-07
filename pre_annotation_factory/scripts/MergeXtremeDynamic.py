import os
import sys
import json
import math
import shutil
import glob
import random
import numpy as np
from pathlib import Path
from datetime import datetime
from scipy.spatial.transform import Rotation as R

# 尝试导入 NuScenes 元数据生成函数
try:
    root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    if root_path not in sys.path:
        sys.path.append(root_path)
    from my_package.io_modules.nusc_json_exporter import generate_nuscenes_metadata
except ImportError:
    print("⚠️ 警告: 未能找到 my_package.io_modules, NuScenes 元数据生成功能将不可用。")

# ================= 配置区 =================
# 1. Xtreme1 导出的标注数据根目录 (解压后的 Scene_XX 所在目录)
XTREME_EXPORT_ROOT = Path("/home/keyaoli/Data/AutoAnnotation/Xtreme/ExportFromXtreme/BaoLi_20260305-20260402122402")

# 2. 你的原始数据集归档目录 (_origin 后缀的那个文件夹)
RAW_ARCHIVE_ROOT = Path("/home/keyaoli/Data/3DBox_Annotation_20260401143738_BaoLi_20260305_origin")

# 3. 最终训练集输出目录
FINAL_KITTI_OUTPUT_DIR = Path("/home/keyaoli/Data/AutoAnnotation/Wayrobo_KITTI_Dataset/Debug")

# 数据集配置
SPLIT_RATIO = 0.8         # 80% 训练集, 20% 验证集
CONVERT_PCD_TO_BIN = True # 是否转换为 OpenPCDet 需要的 bin
RANDOM_SEED = 42

# ================= 核心数学与转换工具 =================
R_xtreme2kitti = np.array([
    [1.0,  0.0,  0.0],
    [0.0,  0.0,  1.0],
    [0.0, -1.0,  0.0]
])

def load_camera_config(config_path):
    with open(config_path, 'r') as f:
        data = json.load(f)
    if isinstance(data, list): data = data[0]
    if "camera_internal" not in data: data = data[list(data.keys())[0]]
        
    intrinsics = data["camera_internal"]
    K = np.array([[intrinsics["fx"], 0, intrinsics["cx"]],
                  [0, intrinsics["fy"], intrinsics["cy"]],
                  [0, 0, 1]])
    img_w, img_h = data.get("width", 1920), data.get("height", 1080)
    T_lidar2cam = np.array(data["camera_external"]).reshape((4, 4), order='F')
    
    T_lidar2map = None
    if "tf_lidar_to_map" in data:
        T_lidar2map = np.array(data["tf_lidar_to_map"]).reshape((4, 4), order='F')
        
    return K, T_lidar2cam, T_lidar2map, img_w, img_h

def generate_calib(config_path, out_calib):
    K, T_lidar2cam, T_lidar2map, _, _ = load_camera_config(config_path)
    with open(out_calib, 'w') as f:
        P_str = " ".join([f"{x:.6f}" for x in np.pad(K, ((0,0),(0,1))).flatten()])
        f.writelines([f"P{i}: {P_str}\n" for i in range(4)])
        f.write("R0_rect: 1 0 0 0 1 0 0 0 1\n")
        Tr = " ".join([f"{x:.6f}" for x in T_lidar2cam[:3, :].flatten()])
        f.write(f"Tr_velo_to_cam: {Tr}\nTr_imu_to_velo: 1 0 0 0 0 1 0 0 0 0 1 0\n")
        if T_lidar2map is not None:
            Tr_velo2map = " ".join([f"{x:.6f}" for x in T_lidar2map[:3, :].flatten()])
            f.write(f"Tr_velo_to_map: {Tr_velo2map}\n")

def pcd_to_bin_fixed(pcd_path, bin_path):
    try:
        with open(pcd_path, 'rb') as f:
            while True:
                line = f.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('DATA binary'): break
                if not line: return False
            data = np.fromfile(f, dtype=np.float32)
        points = data.reshape(-1, 4)
        points.tofile(bin_path)
        return True
    except Exception as e:
        print(f"  [Error] 转换 PCD 失败 {pcd_path}: {e}")
        return False

# ================= 核心标签解析引擎 =================
def parse_xtreme_to_kitti_lines(xtreme_json_path, config_file):
    K, T_lidar2cam, _, img_w, img_h = load_camera_config(config_file)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    with open(xtreme_json_path, 'r') as f:
        xtreme_data = json.load(f)
        
    objects_to_write = []
    if xtreme_data and "objects" in xtreme_data[0]:
        for obj in xtreme_data[0].get("objects", []):
            if obj["type"] != "3D_BOX": continue
            
            # 【核心修复】：双重回退读取逻辑 (className -> modelClass)
            raw_class_name = obj.get('className')
            
            # 如果 className 是 None 或空字符串，则尝试读取 modelClass
            if not raw_class_name:
                raw_class_name = obj.get('modelClass')
                
            # 如果两个都为空，那说明这个框真的坏了，跳过它
            if not raw_class_name:
                print(f"⚠️ 警告: 发现缺失类别的异常目标 (className 与 modelClass 均为空)，已跳过。文件: {xtreme_json_path.name}")
                continue

            class_name = raw_class_name.replace(' ', '_')
            
            s, c, rot = obj["contour"]["size3D"], obj["contour"]["center3D"], obj["contour"]["rotation3D"]
            h, w, l = s["z"], s["y"], s["x"]
            
            R_lidar_to_box = R.from_euler('xyz', [rot["x"], rot["y"], rot["z"]]).as_matrix()
            center_lidar = np.array([c["x"], c["y"], c["z"]])
            
            R_6dof = T_lidar2cam[:3, :3] @ R_lidar_to_box @ R_xtreme2kitti
            center_cam_geo = T_lidar2cam[:3, :3] @ center_lidar + T_lidar2cam[:3, 3]
            bottom_cam = center_cam_geo + R_6dof @ np.array([0.0, h/2.0, 0.0])
            rx, ry, rz = R.from_matrix(R_6dof).as_euler('xyz')
            
            corners_local = np.array([
                [ l/2, -h,  w/2], [ l/2, -h, -w/2], [ l/2, 0, -w/2], [ l/2, 0,  w/2],
                [-l/2, -h,  w/2], [-l/2, -h, -w/2], [-l/2, 0, -w/2], [-l/2, 0,  w/2]
            ]).T
            corners_cam = R_6dof @ corners_local + bottom_cam.reshape(3, 1)
            
            is_camera_visible = True
            bbox_left, bbox_top, bbox_right, bbox_bottom = 0.0, 0.0, 0.0, 0.0
            
            valid_idx = corners_cam[2, :] > 0.1
            if not np.any(valid_idx):
                is_camera_visible = False
            else:
                u = fx * corners_cam[0, valid_idx] / corners_cam[2, valid_idx] + cx
                v = fy * corners_cam[1, valid_idx] / corners_cam[2, valid_idx] + cy
                bbox_left = max(0.0, min(float(img_w) - 1.0, np.min(u)))
                bbox_right = max(0.0, min(float(img_w) - 1.0, np.max(u)))
                bbox_top = max(0.0, min(float(img_h) - 1.0, np.min(v)))
                bbox_bottom = max(0.0, min(float(img_h) - 1.0, np.max(v)))
                if bbox_right <= bbox_left or bbox_bottom <= bbox_top:
                    is_camera_visible = False
                    
            alpha = ry - math.atan2(bottom_cam[0], bottom_cam[2])
            alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
            
            bbox_str = f"{bbox_left:.2f} {bbox_top:.2f} {bbox_right:.2f} {bbox_bottom:.2f}" if is_camera_visible else "-1.00 -1.00 -1.00 -1.00"
            trunc, occ = ("0.00", "0") if is_camera_visible else ("1.00", "3")
            alpha_val = f"{alpha:.2f}" if is_camera_visible else "-10.00"

            line = (f"{class_name} {trunc} {occ} {alpha_val} {bbox_str} "
                    f"{h:.4f} {w:.4f} {l:.4f} "
                    f"{bottom_cam[0]:.4f} {bottom_cam[1]:.4f} {bottom_cam[2]:.4f} "
                    f"{ry:.4f} {rx:.4f} {rz:.4f}\n")
            objects_to_write.append(line)
            
    return objects_to_write

# ================= 编译流水线 =================
def build_final_dataset(location_str):
    print(f"\n{'='*50}")
    print("🚀 开始终极编译：融合 Xtreme1 人工精修与原始数据...")
    print(f"{'='*50}")

    current_time_str = datetime.now().strftime("%Y%m%d%H%M%S")
    dataset_folder_name = f"3DBox_Annotation_Final_{current_time_str}_{location_str}"
    dataset_root_dir = FINAL_KITTI_OUTPUT_DIR / dataset_folder_name
    
    training_base = dataset_root_dir / "training"
    imagesets_dir = dataset_root_dir / "ImageSets"

    for fol in ["label_2", "calib", "image_2", "velodyne"]:
        (training_base / fol).mkdir(parents=True, exist_ok=True)
    imagesets_dir.mkdir(parents=True, exist_ok=True)

    scene_dirs = sorted(RAW_ARCHIVE_ROOT.glob("data_record_*/Scene_*"))
    tasks = []

    for scene_dir in scene_dirs:
        scene_name = scene_dir.name
        xtreme_scene_dir = XTREME_EXPORT_ROOT / scene_name
        has_xtreme_export = xtreme_scene_dir.exists()
        
        config_files = glob.glob(str(scene_dir / "camera_config" / "*.json"))
        
        for config_path in config_files:
            frame_id = Path(config_path).stem
            img_matches = glob.glob(str(scene_dir / "camera_image_0" / f"{frame_id}.*"))
            if not img_matches: continue
            
            pcd_path = scene_dir / "lidar_point_cloud_0" / f"{frame_id}.pcd"
            if not pcd_path.exists(): continue
            
            xtreme_json_path = xtreme_scene_dir / "result" / f"{frame_id}.json"
            if has_xtreme_export and xtreme_json_path.exists():
                label_source = "xtreme"
                target_label_path = xtreme_json_path
            else:
                label_source = "raw"
                target_label_path = scene_dir / "label_2" / f"{frame_id}.txt"
                if not target_label_path.exists(): continue
                
            tasks.append({
                "frame_id": frame_id,
                "config_path": Path(config_path),
                "img_path": Path(img_matches[0]),
                "pcd_path": pcd_path,
                "label_source": label_source,
                "label_path": target_label_path
            })

    if not tasks:
        print("❌ 未发现任何有效帧数据，整合终止。")
        return

    random.seed(RANDOM_SEED)
    random.shuffle(tasks)
    
    split_idx = int(len(tasks) * SPLIT_RATIO)
    train_ids, val_ids = [], []
    xtreme_used_count, raw_used_count = 0, 0

    print(f"🔎 扫描完毕。共捕获 {len(tasks)} 帧有效数据，开始转换装配...")

    for i, task in enumerate(tasks):
        frame_id = task["frame_id"]
        img_ext = task["img_path"].suffix
        
        out_label = training_base / "label_2" / f"{frame_id}.txt"
        out_calib = training_base / "calib" / f"{frame_id}.txt"
        out_bin = training_base / "velodyne" / f"{frame_id}.bin"
        out_img = training_base / "image_2" / f"{frame_id}{img_ext}"
        
        if task["label_source"] == "xtreme":
            kitti_lines = parse_xtreme_to_kitti_lines(task["label_path"], task["config_path"])
            with open(out_label, 'w') as f:
                f.writelines(kitti_lines)
            xtreme_used_count += 1
        else:
            shutil.copy2(task["label_path"], out_label)
            raw_used_count += 1
            
        generate_calib(task["config_path"], out_calib)
        shutil.copy2(task["img_path"], out_img)
        
        if CONVERT_PCD_TO_BIN:
            pcd_to_bin_fixed(task["pcd_path"], out_bin)
        else:
            shutil.copy2(task["pcd_path"], training_base / "velodyne" / f"{frame_id}.pcd")

        if i < split_idx: train_ids.append(frame_id)
        else: val_ids.append(frame_id)

        if (i + 1) % 50 == 0:
            print(f"  已装配 {i + 1} / {len(tasks)} 帧...")

    with open(imagesets_dir / "train.txt", "w") as f: f.write("\n".join(train_ids))
    with open(imagesets_dir / "val.txt", "w") as f: f.write("\n".join(val_ids))
    
    if 'generate_nuscenes_metadata' in globals():
        try:
            generate_nuscenes_metadata(str(dataset_root_dir), location_str)
        except Exception as e:
            print(f"⚠️ NuScenes Metadata 生成跳过: {e}")

    print(f"\n✅ 编译完成！共使用 Xtreme精修: {xtreme_used_count}帧, 自动标注兜底: {raw_used_count}帧。")
    print(f"📊 划分情况 -> 训练集: {len(train_ids)}帧，验证集: {len(val_ids)}帧。")
    
    print(f"📦 正在打包最终极训练数据集 (ZIP)...")
    zip_path = shutil.make_archive(
        base_name=str(dataset_root_dir), 
        format='zip',
        root_dir=FINAL_KITTI_OUTPUT_DIR,  
        base_dir=dataset_folder_name
    )
    print(f"🎉 终极交付准备就绪！数据集已打包至 -> {zip_path}")


if __name__ == "__main__":
    print("\n" + "="*50)
    location_str = input("👉 请输入本次数据集的【采集地点与时间】(例如: Binjiang_20260319): ").strip()
    if not location_str:
        location_str = "Unknown_Location"
        print("⚠️ 未输入标识，将使用默认名 'Unknown_Location'")
    print("="*50 + "\n")
    
    build_final_dataset(location_str)