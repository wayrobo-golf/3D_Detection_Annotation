import cv2, numpy as np, open3d as o3d
import os, json, shutil
import tkinter as tk
from tkinter import simpledialog
from my_package.core.math_utils import TransformMath, Projector
from my_package.core.data_types import SensorData

class CalibrationState:
    def __init__(self):
        self.dx = self.dy = self.dz = 0.0
        self.droll = self.dpitch = self.dyaw = 0.0
        
    def get_delta_matrix(self) -> np.ndarray:
        T_delta = np.eye(4)
        T_delta[:3, :3] = TransformMath.euler_to_matrix(self.droll, self.dpitch, self.dyaw)
        T_delta[:3, 3] = [self.dx, self.dy, self.dz]
        return T_delta

class CalibrationApp:
    def __init__(self, loader, frames: list):
        self.loader = loader
        self.frames = frames
        self.total_frames = len(frames)
        self.current_idx = 0
        
        self.should_quit = False
        
        # 【关键修复】：提前初始化 state 和锁，防止 OpenCV 创建滑块时触发回调崩溃！
        self.state = CalibrationState() 
        self._is_setting_trackbar = True 
        
        # 定义屏幕上的按钮热区
        self.buttons = {
            "save":  [20,  300, 120, 340, (0, 200, 0), "SAVE (S)"],
            "quit":  [130, 300, 230, 340, (0, 0, 200), "QUIT (Q)"],
            "top":   [20,  360, 90,  400, (200, 150, 0), "TOP"],
            "front": [100, 360, 180, 400, (200, 150, 0), "FRONT"],
            "side":  [190, 360, 270, 400, (200, 150, 0), "SIDE"],
            "reset": [280, 360, 360, 400, (100, 100, 100), "RESET"],
            "prev":  [20,  420, 90,  460, (150, 50, 50), "PREV"],
            "next":  [100, 420, 170, 460, (150, 50, 50), "NEXT"],
            "jump":  [180, 420, 270, 460, (50, 150, 150), "JUMP (J)"]
        }

        # 现在可以安全地初始化 UI 了
        self._setup_cv_ui()
        
        # 初始化 O3D 视窗
        self.o3d_vis = o3d.visualization.Visualizer()
        self.o3d_vis.create_window(window_name="LiDAR View (Open3D)", width=1000, height=800)
        self.line_set = o3d.geometry.LineSet()

        # 加载初始帧
        self.load_current_frame()
        
        # 【关键修复】：界面和数据全部就绪，解锁滑块回调
        self._is_setting_trackbar = False

    def load_current_frame(self):
        """核心：加载索引指定的帧数据，更新 UI"""
        self.current_frame_info = self.frames[self.current_idx]
        self.data = self.loader.load_frame(self.current_frame_info)
        self.base_image = self.data.image.copy()
        
        self.state = CalibrationState() # 切换帧时清空微调参数
        
        # Z-Coloring 点云染色
        points = np.asarray(self.data.pcd.points)
        if len(points) > 0:
            z_vals = points[:, 2]
            z_min, z_max = np.percentile(z_vals, 5), np.percentile(z_vals, 95)
            z_vals_clipped = np.clip(z_vals, z_min, z_max)
            z_norm = (z_vals_clipped - z_min) / (z_max - z_min + 1e-6)
            z_uint8 = (z_norm * 255).astype(np.uint8)
            colors_bgr = cv2.applyColorMap(z_uint8, cv2.COLORMAP_JET).reshape(-1, 3)
            self.data.pcd.colors = o3d.utility.Vector3dVector(colors_bgr[:, ::-1] / 255.0)

        # 刷新 Open3D 几何体
        self.o3d_vis.clear_geometries()
        self.o3d_vis.add_geometry(self.data.pcd)
        self.o3d_vis.add_geometry(self.line_set)

        # 刷新 OpenCV 滑块位置
        self._is_setting_trackbar = True
        cv2.setTrackbarPos('Global Frame', self.win_name, self.current_idx)
        cv2.setTrackbarPos('X (m)', self.win_name, 100)
        cv2.setTrackbarPos('Y (m)', self.win_name, 100)
        cv2.setTrackbarPos('Z (m)', self.win_name, 100)
        cv2.setTrackbarPos('Roll (deg)', self.win_name, 100)
        cv2.setTrackbarPos('Pitch (deg)', self.win_name, 100)
        cv2.setTrackbarPos('Yaw (deg)', self.win_name, 100)
        self._is_setting_trackbar = False

    def _setup_cv_ui(self):
        self.win_name = "Camera View (OpenCV) & Controls"
        cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow(self.win_name, 1280, 850) 
        
        def update_frame(val):
            if not self._is_setting_trackbar:
                self.current_idx = val
                self.load_current_frame()

        def update_x(val):
            if not self._is_setting_trackbar: self.state.dx = (val - 100) / 100.0
        def update_y(val):
            if not self._is_setting_trackbar: self.state.dy = (val - 100) / 100.0
        def update_z(val):
            if not self._is_setting_trackbar: self.state.dz = (val - 100) / 100.0
        def update_roll(val):
            if not self._is_setting_trackbar: self.state.droll = np.deg2rad((val - 100) / 10.0)
        def update_pitch(val):
            if not self._is_setting_trackbar: self.state.dpitch = np.deg2rad((val - 100) / 10.0)
        def update_yaw(val):
            if not self._is_setting_trackbar: self.state.dyaw = np.deg2rad((val - 100) / 10.0)

        cv2.createTrackbar('Global Frame', self.win_name, 0, max(1, self.total_frames - 1), update_frame)
        cv2.createTrackbar('X (m)', self.win_name, 100, 200, update_x)
        cv2.createTrackbar('Y (m)', self.win_name, 100, 200, update_y)
        cv2.createTrackbar('Z (m)', self.win_name, 100, 200, update_z)
        cv2.createTrackbar('Roll (deg)', self.win_name, 100, 200, update_roll)
        cv2.createTrackbar('Pitch (deg)', self.win_name, 100, 200, update_pitch)
        cv2.createTrackbar('Yaw (deg)', self.win_name, 100, 200, update_yaw)
        cv2.setMouseCallback(self.win_name, self.on_mouse)

    def set_view(self, view_type: str):
        ctr = self.o3d_vis.get_view_control()
        ctr.change_field_of_view(step=-90) 
        if view_type == "top":
            ctr.set_front([0, 0, 1]); ctr.set_up([1, 0, 0])
        elif view_type == "front":
            ctr.set_front([1, 0, 0]); ctr.set_up([0, 0, 1])
        elif view_type == "side":
            ctr.set_front([0, 1, 0]); ctr.set_up([0, 0, 1])
        elif view_type == "reset":
            self.o3d_vis.reset_view_point(True)
            return
        ctr.set_zoom(0.3)

    def draw_2d_box(self, img: np.ndarray, uv: np.ndarray):
        if uv is None: return
        color = (0, 255, 0)
        edges = [[0,1], [1,2], [2,3], [3,0], [4,5], [5,6], [6,7], [7,4], [0,4], [1,5], [2,6], [3,7]]
        for edge in edges:
            cv2.line(img, tuple(uv[:, edge[0]]), tuple(uv[:, edge[1]]), color, 2)

    def handle_jump(self):
        """处理精确 ID 定位 JUMP 操作"""
        tk_root = tk.Tk()
        tk_root.withdraw()
        target_id = simpledialog.askstring("Jump to Frame", "请输入要跳转的目标 FRAME_ID:", parent=tk_root)
        tk_root.destroy()
        
        if target_id:
            target_id = target_id.strip()
            found = False
            for i, f in enumerate(self.frames):
                if f["frame_id"] == target_id:
                    self.current_idx = i
                    self.load_current_frame()
                    found = True
                    break
            if not found:
                print(f"\n⚠️ 未在当前目录内找到指定的 Frame_ID: {target_id}\n")

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            for action, btn in self.buttons.items():
                x1, y1, x2, y2, color, text = btn
                if x1 <= x <= x2 and y1 <= y <= y2:
                    if action == "save":
                        self.save_calibration()
                    elif action == "quit":
                        self.should_quit = True
                    elif action in ["top", "front", "side", "reset"]:
                        self.set_view(action)
                    elif action == "next":
                        if self.current_idx < self.total_frames - 1:
                            self.current_idx += 1
                            self.load_current_frame()
                    elif action == "prev":
                        if self.current_idx > 0:
                            self.current_idx -= 1
                            self.load_current_frame()
                    elif action == "jump":
                        self.handle_jump()
                    break

    def update_render(self):
        T_delta = self.state.get_delta_matrix()
        T_lidar2map_new = self.data.T_lidar2map @ T_delta
        img_display = self.base_image.copy()
        
        points_list, edges_list, colors_list = [], [], []
        pcd_points = np.asarray(self.data.pcd.points)
        valid_box_count = 0 

        for box in self.data.global_boxes:
            corners_map_h = TransformMath.get_box_corners_map(box)
            corners_lidar = Projector.project_to_lidar(corners_map_h, T_lidar2map_new)
            
            if Projector.count_points_in_box(pcd_points, corners_lidar, box) < 50:
                continue 
                
            points_list.append(corners_lidar.T)
            base_idx = valid_box_count * 8 
            box_edges = [
                [base_idx, base_idx+1], [base_idx+1, base_idx+2], [base_idx+2, base_idx+3], [base_idx+3, base_idx],
                [base_idx+4, base_idx+5], [base_idx+5, base_idx+6], [base_idx+6, base_idx+7], [base_idx+7, base_idx+4],
                [base_idx, base_idx+4], [base_idx+1, base_idx+5], [base_idx+2, base_idx+6], [base_idx+3, base_idx+7]
            ]
            edges_list.extend(box_edges)
            colors_list.extend([[1.0, 1.0, 0.0] for _ in range(12)]) 

            uv = Projector.project_to_image(corners_lidar, self.data.T_lidar2cam, self.data.K)
            self.draw_2d_box(img_display, uv)
            valid_box_count += 1
        
        frame_id = self.current_frame_info["frame_id"]
        scene_name = self.current_frame_info["scene_name"]
        is_edited = self.current_frame_info["is_edited"]
        status_str = "Edited" if is_edited else "Original"
        
        hud_lines = [
            f"Frame: {self.current_idx + 1}/{self.total_frames}  [{frame_id}]",
            f"Scene: {scene_name} | Status: {status_str}",
            f"1. X (m):     {self.state.dx:+.2f}",
            f"2. Y (m):     {self.state.dy:+.2f}",
            f"3. Z (m):     {self.state.dz:+.2f}",
            f"4. Roll (deg):  {np.rad2deg(self.state.droll):+.2f}",
            f"5. Pitch (deg): {np.rad2deg(self.state.dpitch):+.2f}",
            f"6. Yaw (deg):   {np.rad2deg(self.state.dyaw):+.2f}"
        ]
        
        start_y = 30
        for i, text in enumerate(hud_lines):
            pos = (20, start_y + i * 32)
            text_color = (150, 255, 150) if is_edited and i < 2 else (255, 255, 255)
            cv2.putText(img_display, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(img_display, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)

        for action, btn in self.buttons.items():
            x1, y1, x2, y2, bg_color, text = btn
            cv2.rectangle(img_display, (x1, y1), (x2, y2), bg_color, -1)
            cv2.rectangle(img_display, (x1, y1), (x2, y2), (255, 255, 255), 2)
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            text_x = x1 + (x2 - x1 - text_size[0]) // 2
            text_y = y1 + (y2 - y1 + text_size[1]) // 2
            cv2.putText(img_display, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow(self.win_name, img_display)
        
        if points_list:
            self.line_set.points = o3d.utility.Vector3dVector(np.vstack(points_list))
            self.line_set.lines = o3d.utility.Vector2iVector(edges_list)
            self.line_set.colors = o3d.utility.Vector3dVector(colors_list)
        else:
            self.line_set.points = o3d.utility.Vector3dVector(np.zeros((0, 3)))
            self.line_set.lines = o3d.utility.Vector2iVector(np.zeros((0, 2), dtype=np.int32))
            
        self.o3d_vis.update_geometry(self.line_set)

    def run(self):
        print("💡 App started. Use Trackbars or On-Screen Buttons to tune.")
        while not self.should_quit:
            self.update_render()
            self.o3d_vis.poll_events()
            self.o3d_vis.update_renderer()
            
            key = cv2.waitKey(10) & 0xFF
            if key == ord('q'): self.should_quit = True
            elif key == ord('s'): self.save_calibration()
            elif key == ord('1'): self.set_view("top")
            elif key == ord('2'): self.set_view("front")
            elif key == ord('3'): self.set_view("side")
            elif key == ord('4'): self.set_view("reset")
            elif key == ord('j'): self.handle_jump()
                
        self.o3d_vis.destroy_window()
        cv2.destroyAllWindows()

    def save_calibration(self):
        frame_id = self.current_frame_info["frame_id"]
        scene_dir = self.current_frame_info["scene_dir"]
        
        T_delta = self.state.get_delta_matrix()
        T_lidar2map_new = self.data.T_lidar2map @ T_delta
        print(f"\n✅ Saving New Data for frame {frame_id}...")

        json_path = os.path.join(scene_dir, "camera_config", f"{frame_id}.json")
        backup_json = os.path.join(scene_dir, "camera_config", f"{frame_id}_origin.json")
        if not os.path.exists(backup_json):
            shutil.copy2(json_path, backup_json)

        with open(json_path, 'r') as f: config = json.load(f)
        config[0]["tf_lidar_to_map"] = T_lidar2map_new.flatten(order='F').tolist()
        with open(json_path, 'w') as f: json.dump(config, f, indent=2)

        txt_path = os.path.join(scene_dir, "label_2", f"{frame_id}.txt")
        backup_txt = os.path.join(scene_dir, "label_2", f"{frame_id}_origin.txt")
        if not os.path.exists(backup_txt):
            shutil.copy2(txt_path, backup_txt)

        T_map2cam = self.data.T_lidar2cam @ np.linalg.inv(T_lidar2map_new)
        K = self.data.K
        img_h, img_w = self.data.image.shape[:2]
        pcd_points = np.asarray(self.data.pcd.points)

        lines_to_write = []

        for box in self.data.global_boxes:
            R_local2map = TransformMath.euler_to_matrix(box.rx, box.ry, box.rz)
            T_local2map = np.eye(4)
            T_local2map[:3, :3] = R_local2map
            T_local2map[:3, 3] = [box.tx, box.ty, box.tz]

            hl, hw, hh = box.l / 2.0, box.w / 2.0, box.h / 2.0

            bottom_local = np.array([0.0, 0.0, -hh, 1.0])
            bottom_cam = T_map2cam @ (T_local2map @ bottom_local)
            if np.linalg.norm(bottom_cam[:3]) > 20.0:
                continue

            corners_map_h = TransformMath.get_box_corners_map(box)
            corners_lidar = Projector.project_to_lidar(corners_map_h, T_lidar2map_new)
            if Projector.count_points_in_box(pcd_points, corners_lidar, box) < 50:
                continue

            center_local = np.array([0.0, 0.0, 0.0, 1.0])
            center_cam = T_map2cam @ (T_local2map @ center_local)

            is_camera_visible = True
            bbox_left, bbox_top, bbox_right, bbox_bottom = float(img_w), float(img_h), 0.0, 0.0

            if center_cam[2] < 0.1:
                is_camera_visible = False
            else:
                corners_cam_h = T_map2cam @ corners_map_h
                corners_cam = corners_cam_h[:3, :]
                front_mask = corners_cam[2, :] > 0.1

                if not np.any(front_mask):
                    is_camera_visible = False
                else:
                    uv = K @ corners_cam[:, front_mask]
                    uv = uv[:2, :] / uv[2, :]
                    bbox_left = max(0.0, min(float(img_w - 1), float(np.min(uv[0, :]))))
                    bbox_top = max(0.0, min(float(img_h - 1), float(np.min(uv[1, :]))))
                    bbox_right = max(0.0, min(float(img_w - 1), float(np.max(uv[0, :]))))
                    bbox_bottom = max(0.0, min(float(img_h - 1), float(np.max(uv[1, :]))))

                    if bbox_right <= bbox_left or bbox_bottom <= bbox_top:
                        is_camera_visible = False

            R_local2cam = T_map2cam[:3, :3] @ R_local2map
            R_kitti2ros = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
            R_kitti_to_cam = R_local2cam @ R_kitti2ros

            sy = np.sqrt(R_kitti_to_cam[0,0]**2 + R_kitti_to_cam[1,0]**2)
            if sy > 1e-6:
                rx = np.arctan2(R_kitti_to_cam[2,1], R_kitti_to_cam[2,2])
                ry_kitti = np.arctan2(-R_kitti_to_cam[2,0], sy)
                rz = np.arctan2(R_kitti_to_cam[1,0], R_kitti_to_cam[0,0])
            else:
                rx = np.arctan2(-R_kitti_to_cam[1,2], R_kitti_to_cam[1,1])
                ry_kitti = np.arctan2(-R_kitti_to_cam[2,0], sy)
                rz = 0.0

            alpha = ry_kitti - np.arctan2(bottom_cam[0], bottom_cam[2])
            while alpha > np.pi: alpha -= 2 * np.pi
            while alpha < -np.pi: alpha += 2 * np.pi

            class_name = box.type.replace(" ", "_")
            if is_camera_visible:
                trunc = "0.00"
                occ = "0"
                bbox_str = f"{bbox_left:.2f} {bbox_top:.2f} {bbox_right:.2f} {bbox_bottom:.2f}"
                alpha_str = f"{alpha:.2f}"
            else:
                trunc = "1.00"
                occ = "3"
                bbox_str = "-1.00 -1.00 -1.00 -1.00"
                alpha_str = "-10.00"

            line = f"{class_name} {trunc} {occ} {alpha_str} {bbox_str} {box.h:.4f} {box.w:.4f} {box.l:.4f} " \
                   f"{bottom_cam[0]:.4f} {bottom_cam[1]:.4f} {bottom_cam[2]:.4f} {ry_kitti:.4f} {rx:.4f} {rz:.4f}\n"
            lines_to_write.append(line)

        with open(txt_path, 'w') as f:
            f.writelines(lines_to_write)
        print(f"  -> TXT label updated: {txt_path} (Valid boxes: {len(lines_to_write)})")

        self.current_frame_info["is_edited"] = True  
        print(f"  -> Data saved successfully!")