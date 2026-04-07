import os
import json
import shutil
from pathlib import Path

# --- 配置路径 ---
source_dir = "/home/keyaoli/Data/AutoAnnotation/DataRecord/data_record_test/data_record_20260228135009/Xtreme1_Upload/Scene_02"
output_dir = "/home/keyaoli/Data/AutoAnnotation/DataRecord/data_record_test/data_record_20260228135009/CVAT_Upload"

# 创建输出结构
pcd_out = Path(output_dir) / "pointcloud"
img_out = Path(output_dir) / "related_images"
pcd_out.mkdir(parents=True, exist_ok=True)
img_out.mkdir(parents=True, exist_ok=True)

all_calib_items = []

def col_major_to_row_major(m):
    return [
        m[0], m[4], m[8],  m[12],
        m[1], m[5], m[9],  m[13],
        m[2], m[6], m[10], m[14],
        m[3], m[7], m[11], m[15]
    ]

# 遍历 lidar 文件夹
pcd_files = sorted(Path(source_dir).glob("lidar_point_cloud_0/*.pcd"))

print(f"开始转换数据...")

for pcd_path in pcd_files:
    file_id = pcd_path.stem
    shutil.copy(pcd_path, pcd_out / f"{file_id}.pcd")
    
    img_subfolder = img_out / f"{file_id}_pcd"
    img_subfolder.mkdir(exist_ok=True)
    src_img = Path(source_dir) / "camera_image_0" / f"{file_id}.png"
    
    if src_img.exists():
        shutil.copy(src_img, img_subfolder / "image_0.png")

    config_path = Path(source_dir) / "camera_config" / f"{file_id}.json"
    if config_path.exists():
        with open(config_path, 'r') as f:
            raw_data = json.load(f)[0]
            internal = raw_data["camera_internal"]
            external_col = raw_data["camera_external"]
            external_row = col_major_to_row_major(external_col)
            
            item = {
                "name": file_id,
                "intrinsics": {
                    "fx": internal["fx"], "fy": internal["fy"],
                    "cx": internal["cx"], "cy": internal["cy"]
                },
                "extrinsics": external_row
            }
            all_calib_items.append(item)

# 写入标定文件
with open(Path(output_dir) / "calibration.json", "w") as f:
    json.dump({"items": all_calib_items}, f, indent=4)

# --- 自动打包并移动到 output_dir ---
zip_name = Path(source_dir).name  # Scene_02
temp_zip_base = Path(output_dir).parent / zip_name  # 先在父目录创建临时路径
final_zip_path = Path(output_dir) / f"{zip_name}.zip"

print(f"正在打包 {zip_name}.zip ...")

# 1. 在 output_dir 之外创建压缩包，避免“包含自身”的问题
archive_path = shutil.make_archive(str(temp_zip_base), 'zip', output_dir)

# 2. 将生成的 zip 移动到 output_dir 内部
shutil.move(archive_path, final_zip_path)

print("-" * 30)