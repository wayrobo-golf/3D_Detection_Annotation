import numpy as np
from scipy.spatial.transform import Rotation as R

def quat_trans_to_yaml(name, quat_wxyz, trans):
    # 将 [w, x, y, z] 转换为 scipy 需要的 [x, y, z, w]
    quat_xyzw = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]
    
    # 建立旋转矩阵，转换为欧拉角 (ZYX顺序，对应 Yaw, Pitch, Roll)
    rot = R.from_quat(quat_xyzw)
    euler_zyx = rot.as_euler('ZYX', degrees=True)
    
    # scipy 输出的是 [yaw, pitch, roll]，你的代码需要 [roll, pitch, yaw]
    yaw, pitch, roll = euler_zyx
    
    print(f"{name}: [{trans[0]:.6f}, {trans[1]:.6f}, {trans[2]:.6f}, {roll:.2f}, {pitch:.2f}, {yaw:.2f}]")

# 1. Camera to LiDAR
quat_cam2lidar = [-0.332288, 0.626261, -0.62299,  0.330553]
trans_cam2lidar = [0.07000, 0.115999, 0.071299]
quat_trans_to_yaml("default_tf_lcam_to_lidar", quat_cam2lidar, trans_cam2lidar)

# 2. LiDAR to INS (LiDAR->IMU + IMU->INS)
quat_lidar2imu = [0.999552, -0.000540978, -0.0101732, -0.0281514]
trans_lidar2imu = np.array([0.501, 0.0526,0.986])
trans_imu2ins = np.array([-0.837824806, -0.048043063,  0.31324089])

# 合并平移
trans_lidar2ins = trans_lidar2imu + trans_imu2ins
quat_trans_to_yaml("default_tf_lidar_to_ins", quat_lidar2imu, trans_lidar2ins)