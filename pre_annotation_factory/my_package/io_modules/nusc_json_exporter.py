# my_package/io_modules/nusc_json_exporter.py
import json
from pathlib import Path

# 使用相对导入引入同目录下的基础模块
from . import export_calibrated_sensor
from . import export_sample_json
from . import export_static_instance

def generate_nuscenes_metadata(dataset_root_str: str, location_str: str, split: str = "training"):
    """
    为标准的 KITTI 目录结构生成 nuScenes 风格的 JSON 描述文件，
    并统一归档到 nusc_meta 文件夹中。
    """
    root_path = Path(dataset_root_str)
    
    # 🌟 核心改动：在数据集根目录下创建 nusc_meta 文件夹
    meta_dir = root_path / "nusc_meta"
    meta_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"\n📊 开始生成全局 JSON 描述文件，归档至: {meta_dir.name}/ ...")
    try:
        # 1. 生成 calibrated_sensor.json
        calib_dir = root_path / split / "calib"
        calibs = export_calibrated_sensor.collect_calibs(calib_dir)
        calib_data = export_calibrated_sensor.build_calibrated_sensor(calibs)
        # 注意：这里改成了 meta_dir / "..."
        with open(meta_dir / "calibrated_sensor.json", "w", encoding="utf-8") as f:
            json.dump(calib_data, f, ensure_ascii=False, indent=2)
        print("  [-] 已生成 calibrated_sensor.json")

        # 2. 生成 sample.json
        samples_data = export_sample_json.build_samples(root_path, split, location_str)
        with open(meta_dir / "sample.json", "w", encoding="utf-8") as f:
            json.dump(samples_data, f, ensure_ascii=False, indent=2)
        print("  [-] 已生成 sample.json")

        # 3. 生成 static_instance.json (合并距离设为 1.0 米)
        static_data = export_static_instance.build_static_instances(root_path, split, include_dontcare=False)
        static_data = export_static_instance.merge_instances(static_data, distance_threshold=1.0)
        with open(meta_dir / "static_instance.json", "w", encoding="utf-8") as f:
            json.dump(static_data, f, ensure_ascii=False, indent=2)
        print("  [-] 已生成 static_instance.json")
        
    except Exception as e:
        print(f"  ⚠️ 生成 JSON 描述文件时发生意外错误: {e}")