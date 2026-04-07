import os
import sys
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if root_path not in sys.path:
    sys.path.append(root_path)

from my_package.io_modules.nusc_json_exporter import generate_nuscenes_metadata
import shutil
import glob
import json
import random
import numpy as np
import tkinter as tk
from tkinter import filedialog
from datetime import datetime

# ================= 配置区 =================
# 1. KITTI 数据集输出的根目录
KITTI_OUTPUT_DIR = "/home/keyaoli/Data/AutoAnnotation/Wayrobo_KITTI_Dataset/Debug"

# 2. 数据集划分与处理参数
SPLIT_RATIO = 0.8         
CONVERT_PCD_TO_BIN = True 
RANDOM_SEED = 42
MAX_EMPTY_FRAME_RATIO = 0.1 

# ================= 核心转换函数 =================
def pcd_to_bin_fixed(pcd_path, bin_path):
    """高速无损 PCD 转 BIN"""
    try:
        with open(pcd_path, 'rb') as f:
            while True:
                line = f.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('DATA binary'):
                    break
                if not line: return False
            data = np.fromfile(f, dtype=np.float32)
        points = data.reshape(-1, 4)
        points.tofile(bin_path)
        return True
    except Exception as e:
        print(f"  [Error] 转换 PCD 失败 {pcd_path}: {e}")
        return False

def load_camera_config(config_path):
    """从 JSON 中提取内参、Lidar->Cam 外参以及 Map->Lidar 外参"""
    with open(config_path, 'r') as f: 
        data = json.load(f)
    if isinstance(data, list): data = data[0]
    intrinsics = data["camera_internal"]
    K = np.array([[intrinsics["fx"], 0, intrinsics["cx"]],
                  [0, intrinsics["fy"], intrinsics["cy"]],
                  [0, 0, 1]])
    T_lidar2cam = np.array(data["camera_external"]).reshape((4, 4), order='F')
    T_lidar2map = None
    if "tf_lidar_to_map" in data:
        T_lidar2map = np.array(data["tf_lidar_to_map"]).reshape((4, 4), order='F')
    return K, T_lidar2cam, T_lidar2map

def generate_calib(config_path, out_calib):
    """生成 KITTI 标准的 calib.txt"""
    K, T_lidar2cam, T_lidar2map = load_camera_config(config_path)
    with open(out_calib, 'w') as f:
        P_str = " ".join([f"{x:.6f}" for x in np.pad(K, ((0,0),(0,1))).flatten()])
        f.writelines([f"P{i}: {P_str}\n" for i in range(4)])
        f.write("R0_rect: 1 0 0 0 1 0 0 0 1\n")
        Tr = " ".join([f"{x:.6f}" for x in T_lidar2cam[:3, :].flatten()])
        f.write(f"Tr_velo_to_cam: {Tr}\nTr_imu_to_velo: 1 0 0 0 0 1 0 0 0 0 1 0\n")
        if T_lidar2map is not None:
            Tr_velo2map = " ".join([f"{x:.6f}" for x in T_lidar2map[:3, :].flatten()])
            f.write(f"Tr_velo_to_map: {Tr_velo2map}\n")

# ================= 主体导出逻辑 =================
def export_tuned_dataset(data_root_dir, location_str):
    print(f"\n{'='*50}")
    print("🚀 开始将微调后的数据打包为 KITTI 数据集...")
    print(f"{'='*50}")

    # ---------------------------------------------------------
    # 【修改点】：根据所选目录动态生成数据集名称与地点
    # ---------------------------------------------------------
    # 1. 获取文件夹原名 (如 3DBox_Annotation_20260320151216_滨江_origin)
    raw_folder_name = os.path.basename(data_root_dir.rstrip('/'))
    
    # 2. 如果结尾有 _origin，将其去掉
    if raw_folder_name.endswith("_origin"):
        base_name = raw_folder_name[:-7] 
    else:
        base_name = raw_folder_name

    # 3. 拼接 Tuned_ 前缀
    dataset_folder_name = f"Tuned_{base_name}"


    print(f"📦 目标数据集名称: {dataset_folder_name}")
    print(f"📍 提取到采集地点: {location_str}")
    # ---------------------------------------------------------

    dataset_root_dir = os.path.join(KITTI_OUTPUT_DIR, dataset_folder_name)
    training_base = os.path.join(dataset_root_dir, "training")
    imagesets_dir = os.path.join(dataset_root_dir, "ImageSets")

    # 建立 KITTI 目录结构
    for fol in ["label_2", "calib", "image_2", "velodyne"]:
        os.makedirs(os.path.join(training_base, fol), exist_ok=True)
    os.makedirs(imagesets_dir, exist_ok=True)

    # 扫描所有子目录
    scene_dirs = []
    for root, dirs, files in os.walk(data_root_dir):
        if "camera_config" in dirs:
            scene_dirs.append(root)

    if not scene_dirs:
        print(f"❌ 未在 {data_root_dir} 找到有效数据，打包程序退出。")
        return

    valid_tasks, empty_tasks = [], []
    for scene_dir in sorted(scene_dirs):
        config_files = glob.glob(os.path.join(scene_dir, "camera_config", "*.json"))
        for config_file in config_files:
            if "_origin" in os.path.basename(config_file): continue
            frame_id = os.path.basename(config_file).split('.')[0]
            label_path = os.path.join(scene_dir, "label_2", f"{frame_id}.txt")
            pcd_path = os.path.join(scene_dir, "lidar_point_cloud_0", f"{frame_id}.pcd")
            img_matches = glob.glob(os.path.join(scene_dir, "camera_image_0", f"{frame_id}.*"))
            img_path = img_matches[0] if img_matches else None

            if os.path.exists(label_path) and os.path.exists(pcd_path) and img_path:
                is_empty = True
                with open(label_path, 'r') as f:
                    if any(line.strip() for line in f): is_empty = False
                task_tuple = (frame_id, config_file, label_path, pcd_path, img_path)
                if is_empty: empty_tasks.append(task_tuple)
                else: valid_tasks.append(task_tuple)

    # 控制空帧
    max_empty_count = int(len(valid_tasks) * MAX_EMPTY_FRAME_RATIO)
    random.seed(RANDOM_SEED)
    random.shuffle(empty_tasks)
    kept_empty_tasks = empty_tasks[:max_empty_count]
    tasks = valid_tasks + kept_empty_tasks
    random.shuffle(tasks)

    print(f"🚀 共整合 {len(tasks)} 帧数据，开始转换流水线...")

    split_idx = int(len(tasks) * SPLIT_RATIO)
    train_ids, val_ids = [], []

    for i, (frame_id, config_file, label_path, pcd_path, img_path) in enumerate(tasks):
        out_label = os.path.join(training_base, "label_2", f"{frame_id}.txt")
        out_calib = os.path.join(training_base, "calib", f"{frame_id}.txt")
        out_bin = os.path.join(training_base, "velodyne", f"{frame_id}.bin")
        img_ext = os.path.splitext(img_path)[1] 
        out_img = os.path.join(training_base, "image_2", f"{frame_id}{img_ext}")

        shutil.copy2(label_path, out_label)
        generate_calib(config_file, out_calib)
        shutil.copy2(img_path, out_img)
        if CONVERT_PCD_TO_BIN: pcd_to_bin_fixed(pcd_path, out_bin)
        else: shutil.copy2(pcd_path, os.path.join(training_base, "velodyne", f"{frame_id}.pcd"))

        if i < split_idx: train_ids.append(frame_id)
        else: val_ids.append(frame_id)
        if (i + 1) % 50 == 0: print(f"  已转换 {i + 1} / {len(tasks)} 帧...")

    with open(os.path.join(imagesets_dir, "train.txt"), "w") as f: f.write("\n".join(train_ids))
    with open(os.path.join(imagesets_dir, "val.txt"), "w") as f: f.write("\n".join(val_ids))
    
    # ================= 🌟 新增：生成 nuScenes 风格 JSON 🌟 =================
    generate_nuscenes_metadata(dataset_root_dir, location_str)
    # =======================================================================

    print(f"📦 正在打包为 ZIP...")
    zip_path = shutil.make_archive(base_name=dataset_root_dir, format='zip', root_dir=KITTI_OUTPUT_DIR, base_dir=dataset_folder_name)
    print(f"🎉 终极交付：微调版数据集已打包至 -> {zip_path}\n")

if __name__ == "__main__":
    # 1. 选择目录
    tk_root = tk.Tk()
    tk_root.withdraw() 
    print("👉 请选择微调后的原始数据根目录...")
    data_root_dir = filedialog.askdirectory(title="选择包含 Scene 的微调数据根目录")
    tk_root.destroy()

    if not data_root_dir:
        print("❌ 未选择目录，程序退出。")
        sys.exit(0)

    # 2. 手动确认采集地点（保持与 replay_rosbag_main.py 一致）
    print("\n" + "="*50)
    suggested_loc = "Unknown_Location"
    try:
        # 获取基础名称，例如 3DBox_Annotation_20260327162217_XiangXue_20260323
        raw_name = os.path.basename(data_root_dir.rstrip('/'))
        if raw_name.endswith("_origin"):
            raw_name = raw_name[:-7]
            
        parts = raw_name.split('_')
        # 如果长度大于 3，说明索引 3 之后的全是 location_str 的内容
        if len(parts) > 3: 
            suggested_loc = "_".join(parts[3:])  # 【关键修改】：连接索引 3 之后的所有片段
    except Exception as e:
        print(f"⚠️ 解析建议地点失败: {e}")

    location_str = input(f"👉 请确认/输入本次数据集的【采集地点】\n(默认建议: {suggested_loc}): ").strip()
    if not location_str:
        location_str = suggested_loc
    print("="*50 + "\n")

    # 3. 执行导出
    export_tuned_dataset(data_root_dir, location_str)