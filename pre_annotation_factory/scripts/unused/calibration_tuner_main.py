# scripts/calibration_tuner_main.py
# 微调程序入口，负责加载数据并启动 UI 界面
import os
import sys
# 确保能够找到 my_package
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
if root_path not in sys.path:
    sys.path.append(root_path)
os.environ["WAYLAND_DISPLAY"] = ""  # 强制屏蔽 Wayland，自动回退到 X11 兼容模式
import tkinter as tk
from tkinter import filedialog

from my_package.io_modules.data_loader import DataLoader
from my_package.ui.app import CalibrationApp

if __name__ == "__main__":
    # 初始化 tkinter 隐藏主窗口
    tk_root = tk.Tk()
    tk_root.withdraw() 
    
    print("👉 请在弹出的窗口中选择包含所有 Scene 的数据根目录...")
    # 例如选择 3DBox_Annotation_20260323151038_origin_滨江_精标20mdebug
    data_root_dir = filedialog.askdirectory(title="1. 选择包含所有 Scene 的数据根目录")
    if not data_root_dir:
        print("❌ 未选择目录，程序退出。")
        sys.exit(0)

    print("👉 请在弹出的窗口中选择 tracklet_labels.xml 文件...")
    xml_path = filedialog.askopenfilename(title="2. 选择 tracklet_labels.xml", filetypes=[("XML Files", "*.xml")])
    if not xml_path:
        print("❌ 未选择 XML 文件，程序退出。")
        sys.exit(0)
    
    # 销毁 tkinter 实例，避免与 OpenCV 窗口冲突
    tk_root.destroy()
    
    print(f"\n✅ 成功加载配置！\n 📁 根目录: {data_root_dir}\n 📄 XML: {xml_path}\n")

    # 初始化 DataLoader
    loader = DataLoader(data_root_dir=data_root_dir, xml_path=xml_path)
    all_frames = loader.get_all_frames()
    
    if not all_frames:
        print(f"❌ 在 {data_root_dir} 下未找到任何有效的数据帧！")
        sys.exit(1)

    # 启动 App，传入全局帧序列
    app = CalibrationApp(loader, all_frames)
    app.run()