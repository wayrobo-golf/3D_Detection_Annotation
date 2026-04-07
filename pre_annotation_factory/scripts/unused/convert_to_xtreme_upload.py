import os
import shutil
from pathlib import Path

def convert_dataset_structure():
    # --- 配置路径 ---
    # 原始数据根目录
    src_root = Path("/home/keyaoli/Code/Wayrobo/Robot/install/automatic_annotation/share/automatic_annotation/data/history/3DBox_Annotation_20260328150222_Binjiang_20260319_origin")
    
    # 目标数据根目录 (所有 Scene 将会被放在这里面)
    dst_base_dir = Path("/home/keyaoli/Data/AutoAnnotation/DataRecord/Changcheng/Xtreme1_Upload")

    # 需要提取的文件夹列表
    folders_to_copy = [
        "camera_config",
        "camera_image_0",
        "lidar_point_cloud_0"
    ]

    print(f"开始批量转换数据结构...")
    print(f"源基础路径: {src_root}")
    print(f"目标基础路径: {dst_base_dir}\n")

    # 查找所有的 Scene 文件夹 (匹配 data_record_*/Scene_*)
    scene_dirs = sorted(src_root.glob("data_record_*/Scene_*"))
    
    if not scene_dirs:
        print("[ERROR] 未在源路径下找到任何 Scene 文件夹，请检查路径。")
        return

    # 1. 遍历每个 Scene 文件夹进行数据同步
    for src_scene_path in scene_dirs:
        scene_name = src_scene_path.name  # 例如 "Scene_01"
        dst_scene_path = dst_base_dir / scene_name  # 对应的目标路径

        print(f"{'-'*40}")
        print(f"正在处理: {scene_name}")
        
        # 确保目标 Scene 目录存在
        if not dst_scene_path.exists():
            dst_scene_path.mkdir(parents=True, exist_ok=True)

        # 遍历并复制指定的传感器文件夹
        for folder in folders_to_copy:
            src_folder_path = src_scene_path / folder
            dst_folder_path = dst_scene_path / folder

            if src_folder_path.exists() and src_folder_path.is_dir():
                if dst_folder_path.exists():
                    print(f"  [WARN] 目标 {folder} 已存在，正在覆盖...")
                    shutil.rmtree(dst_folder_path)
                
                shutil.copytree(src_folder_path, dst_folder_path)
                file_count = len(list(dst_folder_path.glob('*')))
                print(f"  [SUCCESS] 已同步 {folder}: 共 {file_count} 个文件")
            else:
                print(f"  [ERROR] 找不到源文件夹: {src_folder_path}")

    # 2. 所有数据同步完成后，进行统一打包
    print(f"\n{'='*40}")
    # 设置压缩包存放路径（放在目标文件夹的上一级目录，避免把自己打包进去）
    zip_export_path = dst_base_dir.parent / "Scene"
    
    print(f"所有场景文件同步完毕，开始整体打包...")
    print(f"正在生成 {zip_export_path}.zip ...")
    
    shutil.make_archive(
        base_name=str(zip_export_path), 
        format='zip', 
        root_dir=dst_base_dir  # 将 dst_base_dir (Xtreme1_Upload) 下的所有内容作为根目录打包
    )
    
    print(f"[SUCCESS] 整体压缩包已生成！位置: {zip_export_path}.zip")

if __name__ == "__main__":
    convert_dataset_structure()