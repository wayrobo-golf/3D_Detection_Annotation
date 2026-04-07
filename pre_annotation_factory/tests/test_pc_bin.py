import sys
import os
import numpy as np
import open3d as o3d
import argparse

def visualize_kitti_bin(bin_path):
    """
    读取并可视化 KITTI/OpenPCDet 格式的 .bin 点云文件
    """
    if not os.path.exists(bin_path):
        print(f"❌ 错误: 文件不存在 -> {bin_path}")
        return

    print(f"📂 正在加载点云: {bin_path}")
    
    # 1. 读取二进制文件
    # 数据格式为 float32，N个点，每个点4个特征 (x, y, z, intensity)
    point_cloud_data = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    print(f"📊 成功加载，共计 {point_cloud_data.shape[0]} 个点。")

    # 2. 提取空间坐标 (x, y, z)
    xyz = point_cloud_data[:, :3]

    # 3. 提取并归一化反射强度 (intensity) 用于着色
    intensity = point_cloud_data[:, 3]
    # 将 intensity 限制在合理范围并归一化到 0~1 之间
    # Livox/Velodyne 的 intensity 通常在 0-255 之间
    intensity_norm = np.clip(intensity, 0, 255) / 255.0
    
    # 构造颜色数组 (N, 3)，这里我们用灰度显示，R=G=B=intensity
    colors = np.zeros((xyz.shape[0], 3))
    colors[:, 0] = intensity_norm
    colors[:, 1] = intensity_norm
    colors[:, 2] = intensity_norm

    # 4. 构建 Open3D 的 PointCloud 对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # 5. 可视化设置
    print("🚀 启动可视化窗口 (按 'Q' 或 'Esc' 退出，鼠标左键旋转，右键平移，滚轮缩放)...")
    
    # 坐标轴提示：红色=X, 绿色=Y, 蓝色=Z
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=2.0, origin=[0, 0, 0]
    )

    # 绘制点云和坐标系
    o3d.visualization.draw_geometries(
        [pcd, mesh_frame], 
        window_name=f"BIN Point Cloud Viewer - {os.path.basename(bin_path)}",
        width=1024, 
        height=768,
        point_show_normal=False
    )

if __name__ == "__main__":
    # 使用 argparse 支持命令行传参
    parser = argparse.ArgumentParser(description="Visualize KITTI format .bin point cloud.")
    parser.add_argument("file_path", type=str, help="Path to the .bin file")
    
    args = parser.parse_args()
    visualize_kitti_bin(args.file_path)