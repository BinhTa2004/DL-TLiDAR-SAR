"""Open3D viewer and target wireframes; map accumulation stays outside the renderer."""
import json
import os
import threading
import time
import numpy as np
import open3d as o3d
from .detection_policy import COLORS
from .runtime_support import get_logger

log = get_logger("lidar_3d")


def generate_target_box_wires(center_enu, extent_enu=(1.2, 1.2, 1.8), beacon_h=3.5, diamond_r=0.45, diamond_h=0.6):
    """
    Tạo hình học Khung Đánh Dấu 3D (3D Bounding Box Wireframe) trong hệ tọa độ Open3D ENU:
    - 12 cạnh khung hộp không gian 3 chiều bao quanh nạn nhân (1.2m x 1.2m x 1.8m)
    - Dấu chữ thập tâm nóc hộp (Top Crosshair)
    - Vòng định vị chân đế trên mặt đất (Ground Footprint Diamond)
    - Cột tiêu cứu hộ thẳng đứng vươn lên không gian (Vertical Rescue Beacon Pole cao 3.5m)
    - Kim cương định vị 3D trên đỉnh cột tiêu (3D Octahedron Beacon)
    """
    cx, cy, cz = float(center_enu[0]), float(center_enu[1]), float(center_enu[2])
    hx = max(0.45, float(extent_enu[0]) / 2.0)
    hy = max(0.45, float(extent_enu[1]) / 2.0)
    hz = max(0.45, float(extent_enu[2]) / 2.0)

    # 8 góc của hình hộp chữ nhật
    corners = [
        [cx - hx, cy - hy, cz - hz],  # 0
        [cx + hx, cy - hy, cz - hz],  # 1
        [cx + hx, cy + hy, cz - hz],  # 2
        [cx - hx, cy + hy, cz - hz],  # 3
        [cx - hx, cy - hy, cz + hz],  # 4
        [cx + hx, cy - hy, cz + hz],  # 5
        [cx + hx, cy + hy, cz + hz],  # 6
        [cx - hx, cy + hy, cz + hz],  # 7
    ]

    edges = [
        [0, 1], [1, 2], [2, 3], [3, 0],  # Khung đáy
        [4, 5], [5, 6], [6, 7], [7, 4],  # Khung nóc
        [0, 4], [1, 5], [2, 6], [3, 7],  # 4 trụ đứng góc
        [4, 6], [5, 7],                  # Dấu X tâm nóc
    ]

    if beacon_h <= 0:
        # Live animals and unclassified heat signals use a plain box.
        return corners, edges

    # Vòng định vị chân đế trên mặt đất
    r_base = 0.85
    b_idx = len(corners)
    corners.extend([
        [cx + r_base, cy, cz - hz],
        [cx - r_base, cy, cz - hz],
        [cx, cy + r_base, cz - hz],
        [cx, cy - r_base, cz - hz],
    ])
    edges.extend([
        [b_idx, b_idx + 2], [b_idx + 2, b_idx + 1],
        [b_idx + 1, b_idx + 3], [b_idx + 3, b_idx],
        [b_idx, b_idx + 1], [b_idx + 2, b_idx + 3]
    ])

    # Cột tiêu cứu hộ thẳng đứng
    z_top = cz + hz
    z_pole = z_top + beacon_h
    p_idx = len(corners)
    corners.append([cx, cy, z_top])       # p_idx
    corners.append([cx, cy, z_pole])      # p_idx + 1
    edges.append([p_idx, p_idx + 1])

    # Kim cương định vị trên đỉnh cột tiêu (Octahedron Beacon)
    p_east  = [cx + diamond_r, cy, z_pole]
    p_west  = [cx - diamond_r, cy, z_pole]
    p_north = [cx, cy + diamond_r, z_pole]
    p_south = [cx, cy - diamond_r, z_pole]
    p_up    = [cx, cy, z_pole + diamond_h]
    p_down  = [cx, cy, z_pole - diamond_h]

    d_idx = len(corners)
    corners.extend([p_east, p_west, p_north, p_south, p_up, p_down])
    edges.extend([
        [d_idx, d_idx + 2], [d_idx + 2, d_idx + 1], [d_idx + 1, d_idx + 3], [d_idx + 3, d_idx], # Vành ngang
        [d_idx, d_idx + 4], [d_idx + 1, d_idx + 4], [d_idx + 2, d_idx + 4], [d_idx + 3, d_idx + 4], # Chóp trên
        [d_idx, d_idx + 5], [d_idx + 1, d_idx + 5], [d_idx + 2, d_idx + 5], [d_idx + 3, d_idx + 5], # Chóp dưới
    ])

    return corners, edges


class LidarDisplayMixin:
    """Render independent map snapshots and saved/live target markers."""

    def update_target_bounding_boxes(self, vis, uav_pos_enu=None):
        """
        Cập nhật toàn bộ khung Bounding Box và Cột Tiêu Cứu Hộ 3D trên bản đồ:
        - Nạn nhân đã chốt đánh dấu [Y]: Khung ĐỎ CỨU HỘ + Cột tiêu cao 3.8m + Kim cương định vị
        - Mục tiêu trực tiếp đang phát hiện (Active Target): Khung VÀNG CAM nổi bật
        - Các mục tiêu nhận diện trực tiếp từ AI YOLO: Khung XANH NEON
        """
        now = time.perf_counter()
        if now - self.last_targets_poll < 0.20:
            return
        self.last_targets_poll = now

        target_ls = self.target_lineset
        if target_ls is None or not self.show_targets:
            if target_ls is not None and getattr(self, "_has_active_marker_geo", False):
                dummy_pts = np.array([[0.0, 0.0, -9999.0], [0.0, 0.0, -9998.0]], dtype=np.float64)
                target_ls.points = o3d.utility.Vector3dVector(dummy_pts)
                target_ls.lines = o3d.utility.Vector2iVector(np.array([[0, 1]], dtype=np.int32))
                target_ls.colors = o3d.utility.Vector3dVector(np.zeros((1, 3), dtype=np.float64))
                try:
                    vis.update_geometry(target_ls)
                except Exception:
                    pass
                self._has_active_marker_geo = False
            return

        # Lấy bản chụp điểm tĩnh an toàn cho thuật toán snap
        with self.lock:
            snap_pts = self.cached_snap_pts

        # Đọc dữ liệu từ Data/live_targets.json
        targets_file = self.live_targets_path
        data = None
        if os.path.exists(targets_file):
            try:
                mtime = os.path.getmtime(targets_file)
                if mtime != self.cached_targets_mtime:
                    with open(targets_file, "r", encoding="utf-8") as f:
                        fresh_data = json.load(f)
                    self.cached_target_data = fresh_data
                    self.cached_targets_mtime = mtime
                data = self.cached_target_data
            except Exception:
                data = self.cached_target_data
        elif self.cached_target_data is not None:
            data = self.cached_target_data

        if (self.stream_session_id is None or
                not data or data.get("session_id") != self.stream_session_id):
            data = None

        all_corners = []
        all_edges = []
        all_colors = []
        vertex_offset = 0

        # Màu sắc chiến thuật chuẩn GIS / Cứu hộ cứu nạn
        COLOR_MARKED = np.array([1.0, 0.10, 0.20], dtype=np.float64)   # Đỏ cứu hộ rực rỡ
        COLOR_START = np.array([0.15, 1.0, 0.35], dtype=np.float64)
        COLOR_ROUTE = np.array([1.0, 0.9, 0.05], dtype=np.float64)

        drawn_positions = []
        marked_centers = []

        # 1. Vẽ các nạn nhân đã được chốt mốc cứu hộ [Phím Y]
        marked_list = []
        if data and "marked_persons" in data and isinstance(data["marked_persons"], list):
            marked_list = data["marked_persons"]

        for m in marked_list:
            if "x_enu" in m and "y_enu" in m:
                raw_enu = np.array([float(m["x_enu"]), float(m["y_enu"]), float(m.get("z_enu", 0.0))], dtype=np.float32)
            elif "x" in m and "y" in m:
                raw_enu = np.array([float(m["x"]), float(m["y"]), float(m.get("z", 0.0))], dtype=np.float32)
            else:
                continue

            center_enu = self.snap_target_to_pointcloud(raw_enu, snap_pts)
            corners, edges = generate_target_box_wires(center_enu, extent_enu=(1.2, 1.2, 1.8), beacon_h=3.8)
            for c in corners:
                all_corners.append(c)
            for e in edges:
                all_edges.append([e[0] + vertex_offset, e[1] + vertex_offset])
                all_colors.append(COLOR_MARKED)
            vertex_offset += len(corners)
            drawn_positions.append(center_enu[:2])
            marked_centers.append(center_enu)

        # Điểm xuất phát được chụp một lần từ AirSim khi GCS khởi động.
        start_pos = data.get("start_pos_enu") if data else None
        if isinstance(start_pos, (list, tuple)) and len(start_pos) == 3:
            start_enu = np.asarray(start_pos, dtype=np.float64)
            if np.all(np.isfinite(start_enu)):
                sx, sy, sz = start_enu
                start_vertices = [
                    [sx, sy, sz], [sx, sy, sz + 1.5],
                    [sx - 0.65, sy, sz + 1.5], [sx + 0.65, sy, sz + 1.5],
                    [sx, sy - 0.65, sz + 1.5], [sx, sy + 0.65, sz + 1.5],
                ]
                start_edges = [[0, 1], [2, 3], [4, 5]]
                all_corners.extend(start_vertices)
                for a, b in start_edges:
                    all_edges.append([vertex_offset + a, vertex_offset + b])
                    all_colors.append(COLOR_START)
                vertex_offset += len(start_vertices)

                if data.get("route_lines_enabled", False):
                    route_start = [sx, sy, sz + 1.5]
                    for center in marked_centers:
                        if not np.all(np.isfinite(center)):
                            continue
                        # Đường nối trực tiếp trong không gian 3D; không phải đường bay tránh vật cản.
                        all_corners.extend([route_start, center.tolist()])
                        all_edges.append([vertex_offset, vertex_offset + 1])
                        all_colors.append(COLOR_ROUTE)
                        vertex_offset += 2

        # 2. Vẽ mục tiêu đang phát hiện trực tiếp (Active Target)
        active_t = data.get("active_target") if data else None
        if active_t:
            if "x_enu" in active_t and "y_enu" in active_t:
                raw_enu = np.array([float(active_t["x_enu"]), float(active_t["y_enu"]), float(active_t.get("z_enu", 0.0))], dtype=np.float32)
            elif "x" in active_t and "y" in active_t:
                raw_enu = np.array([float(active_t["x"]), float(active_t["y"]), float(active_t.get("z", 0.0))], dtype=np.float32)
            else:
                raw_enu = None

            if raw_enu is not None:
                is_duplicate = False
                for dp in drawn_positions:
                    if np.hypot(dp[0] - raw_enu[0], dp[1] - raw_enu[1]) < 4.0:
                        is_duplicate = True
                        break
                if not is_duplicate:
                    center_enu = self.snap_target_to_pointcloud(raw_enu, snap_pts)
                    kind = active_t.get("classification", "thermal")
                    extent = (1.2, 1.2, 1.8) if kind == "person" else (1.2, 1.2, 1.2)
                    corners, edges = generate_target_box_wires(
                        center_enu, extent_enu=extent, beacon_h=2.8 if kind == "person" else 0.0)
                    active_color = np.asarray(COLORS.get(kind, COLORS["thermal"]), dtype=np.float64) / 255.0
                    for c in corners:
                        all_corners.append(c)
                    for e in edges:
                        all_edges.append([e[0] + vertex_offset, e[1] + vertex_offset])
                        all_colors.append(active_color)
                    vertex_offset += len(corners)
                    drawn_positions.append(center_enu[:2])

        # 3. Vẽ các mục tiêu từ live_detections
        live_dets = data.get("live_detections", []) if data else []
        for det in live_dets:
            if "x_enu" in det and "y_enu" in det:
                raw_enu = np.array([float(det["x_enu"]), float(det["y_enu"]), float(det.get("z_enu", 0.0))], dtype=np.float32)
            elif "x" in det and "y" in det:
                raw_enu = np.array([float(det["x"]), float(det["y"]), float(det.get("z", 0.0))], dtype=np.float32)
            else:
                continue

            is_duplicate = False
            for dp in drawn_positions:
                if np.hypot(dp[0] - raw_enu[0], dp[1] - raw_enu[1]) < 3.5:
                    is_duplicate = True
                    break
            if not is_duplicate:
                center_enu = self.snap_target_to_pointcloud(raw_enu, snap_pts)
                kind = det.get("classification", "thermal")
                extent = (1.2, 1.2, 1.8) if kind == "person" else (1.2, 1.2, 1.2)
                corners, edges = generate_target_box_wires(
                    center_enu, extent_enu=extent, beacon_h=2.2 if kind == "person" else 0.0)
                live_color = np.asarray(COLORS.get(kind, COLORS["thermal"]), dtype=np.float64) / 255.0
                for c in corners:
                    all_corners.append(c)
                for e in edges:
                    all_edges.append([e[0] + vertex_offset, e[1] + vertex_offset])
                    all_colors.append(live_color)
                vertex_offset += len(corners)
                drawn_positions.append(center_enu[:2])

        # 4. Cập nhật LineSet lên GPU an toàn
        try:
            if all_corners and all_edges:
                target_ls.points = o3d.utility.Vector3dVector(np.asarray(all_corners, dtype=np.float64))
                target_ls.lines = o3d.utility.Vector2iVector(np.asarray(all_edges, dtype=np.int32))
                target_ls.colors = o3d.utility.Vector3dVector(np.asarray(all_colors, dtype=np.float64))
                vis.update_geometry(target_ls)
                self._has_active_marker_geo = True
            else:
                if getattr(self, "_has_active_marker_geo", False):
                    dummy_pts = np.array([[0.0, 0.0, -9999.0], [0.0, 0.0, -9998.0]], dtype=np.float64)
                    target_ls.points = o3d.utility.Vector3dVector(dummy_pts)
                    target_ls.lines = o3d.utility.Vector2iVector(np.array([[0, 1]], dtype=np.int32))
                    target_ls.colors = o3d.utility.Vector3dVector(np.zeros((1, 3), dtype=np.float64))
                    vis.update_geometry(target_ls)
                    self._has_active_marker_geo = False
        except Exception:
            pass

    def run_viewer(self):
        """Khởi chạy Visualizer Open3D với điều khiển tương tác chuột 100% tự do mượt mà"""
        # Khởi động các luồng ngầm
        threading.Thread(target=self.lidar_worker, daemon=True, name="lidar_worker").start()
        threading.Thread(target=self.map_consolidation_worker, daemon=True, name="map_consolidation").start()

        # Tạo Visualizer Open3D
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(
            window_name="HE THONG DUNG DIA HINH 3D LIDAR [MEDIEVAL VILLAGE]",
            width=1280, height=720, left=40, top=40, visible=True
        )

        render_opt = vis.get_render_option()
        render_opt.background_color = np.array([0.05, 0.05, 0.07])
        render_opt.point_size = self.point_size
        render_opt.light_on = True

        # Đám mây điểm hình học Open3D trên GPU (được quản lý riêng bởi visualizer)
        pcd = o3d.geometry.PointCloud()
        dummy_pts = np.array([[0, 0, 0], [1, 1, 1]], dtype=np.float64)
        pcd.points = o3d.utility.Vector3dVector(dummy_pts)
        pcd.colors = o3d.utility.Vector3dVector(np.zeros((2, 3), dtype=np.float64))
        vis.add_geometry(pcd)

        # Mô hình hiển thị vị trí Drone (Tọa độ 3 trục RGB XYZ)
        drone_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.5, origin=[0, 0, 0])
        vis.add_geometry(drone_frame)

        # Mô hình hiển thị Khung Đánh Dấu Vị Trí Người & Cột Tiêu Cứu Hộ 3D (LineSet)
        self.target_lineset = o3d.geometry.LineSet()
        dummy_ls_pts = np.array([[0.0, 0.0, -9999.0], [0.0, 0.0, -9998.0]], dtype=np.float64)
        self.target_lineset.points = o3d.utility.Vector3dVector(dummy_ls_pts)
        self.target_lineset.lines = o3d.utility.Vector2iVector(np.array([[0, 1]], dtype=np.int32))
        self.target_lineset.colors = o3d.utility.Vector3dVector(np.zeros((1, 3), dtype=np.float64))
        vis.add_geometry(self.target_lineset)

        # Đăng ký các phím chức năng
        def cb_toggle_scan(v):
            self.toggle_scanning()
            return False

        def cb_save_map(v):
            self.save_point_cloud()
            return False

        def cb_clear_map(v):
            now = time.perf_counter()
            if now - getattr(self, "_last_clear_t", 0.0) > 1.5:
                self._last_clear_t = now
                log.warning(">> [CẢNH BÁO] Bạn vừa bấm phím C. Nhấn C thêm một lần nữa trong vòng 1.5s nếu thực sự muốn XÓA BẢN ĐỒ 3D!")
                return False
            self._last_clear_t = 0.0
            with self.lock:
                self._clear_map_requested = True
                self.new_points_buffer = []
                self.cached_snap_pts = None
                self.pending_render_data = None
            pcd.clear()
            vis.update_geometry(pcd)
            log.debug(">> [PHÍM C] ĐÃ XÓA TOÀN BỘ BẢN ĐỒ 3D! Bắt đầu quét lại từ đầu.")
            return False

        def cb_change_cmap(v):
            self.colormap_idx = (self.colormap_idx + 1) % len(self.colormaps)
            log.debug(f">> [PHÍM H] Bảng màu độ cao hiện tại: {self.colormaps[self.colormap_idx].upper()}")
            self.recolor_needed = True
            return False

        def cb_reset_view(v):
            """Đặt lại góc nhìn camera về trung tâm Drone và ngôi làng"""
            with self.lock:
                uav = np.copy(self.uav_pos_enu)
            ctr = vis.get_view_control()
            ctr.set_lookat(uav.astype(np.float64))
            ctr.set_front([-0.6, -0.6, 0.5])
            ctr.set_up([0, 0, 1])
            ctr.set_zoom(0.7)
            log.debug(">> [PHÍM R] Đã đặt lại góc nhìn camera về vị trí Drone!")
            return False

        def cb_toggle_follow(v):
            self.follow_uav = not self.follow_uav
            mode = "BÁM THEO UAV" if self.follow_uav else "TỰ DO (Orbit Free Camera)"
            log.debug(f">> [PHÍM F] Chế độ Camera: {mode}")
            return False

        def cb_increase_point_size(v):
            self.point_size = min(5.0, self.point_size + 0.5)
            vis.get_render_option().point_size = self.point_size
            log.debug(f">> [PHÍM +] Kích thước điểm: {self.point_size:.1f}px")
            return False

        def cb_decrease_point_size(v):
            self.point_size = max(0.5, self.point_size - 0.5)
            vis.get_render_option().point_size = self.point_size
            log.debug(f">> [PHÍM -] Kích thước điểm: {self.point_size:.1f}px")
            return False

        def cb_toggle_markers(v):
            self.show_targets = not self.show_targets
            state = "HIỆN" if self.show_targets else "ẨN"
            log.debug(f">> [PHÍM M] Khung đánh dấu vị trí người trên bản đồ 3D: [{state}]")
            self.update_target_bounding_boxes(vis, self.uav_pos_enu)
            return False

        def cb_clear_markers(v):
            if os.path.exists(self.live_targets_path):
                try:
                    with open(self.live_targets_path, "w", encoding="utf-8") as f:
                        json.dump({"session_id": self.stream_session_id, "timestamp": time.time(),
                                   "marked_persons": [], "live_detections": []}, f, indent=2)
                except Exception:
                    pass
            self.cached_target_data = {"session_id": self.stream_session_id, "timestamp": time.time(),
                                       "marked_persons": [], "live_detections": []}
            self.update_target_bounding_boxes(vis, self.uav_pos_enu)
            log.debug(">> [DEL / BACKSPACE] ĐÃ XÓA TOÀN BỘ CÁC MỐC ĐÁNH DẤU NẠN NHÂN!")
            return False

        def cb_ignore(v):
            # Giữ an toàn tuyệt đối khi người dùng lái Drone bằng các phím trên bàn phím
            return False

        def cb_esc_exit(v):
            log.debug(">> [ESC] Đang đóng cửa sổ dựng địa hình 3D...")
            self.running = False
            return False

        # Đăng ký phím chức năng
        vis.register_key_callback(ord('J'), cb_toggle_scan)
        vis.register_key_callback(ord('j'), cb_toggle_scan)
        vis.register_key_callback(ord('S'), cb_save_map)
        vis.register_key_callback(ord('s'), cb_save_map)
        vis.register_key_callback(ord('P'), cb_save_map)
        vis.register_key_callback(ord('p'), cb_save_map)
        vis.register_key_callback(ord('C'), cb_clear_map)
        vis.register_key_callback(ord('c'), cb_clear_map)
        vis.register_key_callback(ord('H'), cb_change_cmap)
        vis.register_key_callback(ord('h'), cb_change_cmap)
        vis.register_key_callback(ord('R'), cb_reset_view)
        vis.register_key_callback(ord('r'), cb_reset_view)
        vis.register_key_callback(ord('F'), cb_toggle_follow)
        vis.register_key_callback(ord('f'), cb_toggle_follow)
        vis.register_key_callback(ord('='), cb_increase_point_size)
        vis.register_key_callback(ord('+'), cb_increase_point_size)
        vis.register_key_callback(ord('-'), cb_decrease_point_size)
        vis.register_key_callback(ord('_'), cb_decrease_point_size)
        vis.register_key_callback(ord('M'), cb_toggle_markers)
        vis.register_key_callback(ord('m'), cb_toggle_markers)
        vis.register_key_callback(127, cb_clear_markers)
        vis.register_key_callback(8, cb_clear_markers)

        # KHÓA CHẶN TOÀN DIỆN MỌI PHÍM ĐIỀU KHIỂN BAY CỦA UAV:
        # Ngăn tuyệt đối Open3D tự động đóng cửa sổ khi bấm WASD, Q, E, Space, Shift, Ctrl, Alt, Tab, V, Y, T, K, U, I...
        safe_keys = [
            81, 113,  # Q, q (Yaw Left)
            69, 101,  # E, e (Yaw Right)
            87, 119,  # W, w (Forward)
            65, 97,   # A, a (Left)
            68, 100,  # D, d (Right)
            76, 108,  # L, l (Takeoff/Land)
            32,       # Space (Climb)
            340, 344, # Left/Right Shift (Descend)
            341, 345, # Left/Right Ctrl (Boost)
            342, 346, # Left/Right Alt
            258,      # Tab
            86, 118,  # V, v (Cam FPV/Chase)
            89, 121,  # Y, y (Mark Target)
            84, 116,  # T, t (Lock Target)
            75, 107,  # K, k (Record Data)
            85, 117,  # U, u (Thermal Cam)
            73, 105,  # I, i (Invert Pitch)
            79, 111,  # O, o (HUD Box)
            71, 103,  # G, g (Gimbal Preset)
            49, 50, 51, # 1, 2, 3 (Preset Gimbal)
            262, 263, 264, 265, # Arrow keys
            290, 291, 298, 299, 300 # F1, F2, F9, F10, F11
        ]
        for k in safe_keys:
            try:
                vis.register_key_callback(k, cb_ignore)
            except Exception:
                pass

        vis.register_key_callback(256, cb_esc_exit)  # Chỉ ESC mới đóng cửa sổ có chủ đích

        log.debug("\n" + "="*75)
        log.debug("  HƯỚNG DẪN ĐIỀU KHIỂN CỬA SỔ DỰNG ĐỊA HÌNH 3D LIDAR")
        log.debug("="*75)
        log.debug("  [D-Pad Phải ▶] / [J] : BẬT / DỪNG QUÉT ĐỊA HÌNH (Lần 1: Bật, Lần 2: Dừng)")
        log.debug("  [D-Pad Lên ▲]       : HIỆN / ẨN ĐƯỜNG NỐI ĐIỂM XUẤT PHÁT VỚI NGƯỜI ĐÃ ĐÁNH DẤU")
        log.debug("  [S] / [P]           : LƯU BẢN ĐỒ 3D (.PLY) + TỌA ĐỘ MỤC TIÊU CỨU HỘ")
        log.debug("  [Chuột Trái + Kéo]   : Xoay góc nhìn 3D tự do quanh ngôi làng (Orbit)")
        log.debug("  [Chuột Phải / Giữa]  : Trượt di chuyển bản đồ theo ý muốn (Pan)")
        log.debug("  [Lăn Chuột]          : Phóng to / Thu nhỏ chi tiết (Zoom In/Out)")
        log.debug("  [M]                  : Bật / Tắt khung đánh dấu vị trí người trên bản đồ 3D")
        log.debug("  [DEL] / [Backspace]  : Xóa toàn bộ các mốc đánh dấu nạn nhân")
        log.debug("  [R]                  : Đặt lại góc nhìn camera về vị trí Drone")
        log.debug("  [F]                  : Bật / Tắt chế độ Camera tự động bám theo Drone")
        log.debug("  [H]                  : Đổi bảng màu độ cao (Turbo -> Jet -> Viridis)")
        log.debug("  [+] / [-]            : Tăng / Giảm kích thước hạt điểm (Point Size)")
        log.debug("  [C] (2 lần)          : Xác nhận xóa bản đồ để quét lại từ đầu")
        log.debug("  [Esc]                : Thoát cửa sổ dựng hình 3D")
        log.debug("  * Chú ý: Phím bay WASD, Q, E, Space, Shift không bao giờ làm tắt cửa sổ map!")
        log.debug("="*75 + "\n")

        # Thiết lập view ban đầu
        first_frame = True
        last_info_t = time.perf_counter()
        prev_drone_marker_pos = np.zeros(3, dtype=np.float32)
        rendered_session_id = None
        last_drone_update_t = time.perf_counter()

        # VÒNG LẶP RENDER ~40 FPS (CÁCH LY AN TOÀN TUYỆT ĐỐI)
        while self.running:
            try:
                # Kiểm tra nút bấm từ tay cầm Xbox
                edge_right, _ = self.gamepad.poll_buttons()
                if edge_right:
                    self.toggle_scanning()

                now = time.perf_counter()

                with self.lock:
                    session_id = self.stream_session_id
                if session_id != rendered_session_id:
                    rendered_session_id = session_id
                    pcd.clear()
                    vis.update_geometry(pcd)
                    self.target_lineset.clear()
                    vis.update_geometry(self.target_lineset)
                    self._has_active_marker_geo = False
                    drone_frame.translate(-prev_drone_marker_pos, relative=True)
                    prev_drone_marker_pos = np.zeros(3, dtype=np.float32)
                    vis.update_geometry(drone_frame)
                    first_frame = True

                # 1. Cập nhật đám mây điểm lên GPU CHỈ KHI CÓ SNAPSHOT MỚI (4 Hz)
                if self.need_gpu_update:
                    with self.lock:
                        data = self.pending_render_data
                        self.pending_render_data = None
                        self.need_gpu_update = False

                    if data is not None:
                        pts_arr, colors_arr = data
                        if len(pts_arr) == 0:
                            pcd.clear()
                            vis.update_geometry(pcd)
                            first_frame = True
                        if len(pts_arr) > 0 and len(pts_arr) == len(colors_arr):
                            pcd.points = o3d.utility.Vector3dVector(pts_arr)
                            pcd.colors = o3d.utility.Vector3dVector(colors_arr)
                            vis.update_geometry(pcd)

                            if first_frame:
                                vis.reset_view_point(True)
                                first_frame = False

                # 2. Cập nhật vị trí Marker Drone & Khung Đánh Dấu Nạn Nhân 3D (10-15 Hz)
                if now - last_drone_update_t >= 0.08:
                    last_drone_update_t = now
                    with self.lock:
                        curr_uav = np.copy(self.uav_pos_enu)

                    delta_uav = curr_uav - prev_drone_marker_pos
                    if np.linalg.norm(delta_uav) > 0.12:
                        drone_frame.translate(delta_uav, relative=True)
                        prev_drone_marker_pos = np.copy(curr_uav)
                        vis.update_geometry(drone_frame)

                    if self.follow_uav and not first_frame:
                        vis.get_view_control().set_lookat(curr_uav.astype(np.float64))

                    # Cập nhật Khung Đánh Dấu Nạn Nhân 3D & Cột Tiêu Cứu Hộ từ live_targets.json
                    try:
                        self.update_target_bounding_boxes(vis, curr_uav)
                    except Exception:
                        pass

                # 3. Hiển thị thông số trên Console định kỳ mỗi 2 giây
                if log.isEnabledFor(10) and now - last_info_t >= 2.0:
                    last_info_t = now
                    with self.lock:
                        n_pts = len(self.cached_snap_pts) if self.cached_snap_pts is not None else 0
                        curr_pos = np.copy(self.uav_pos_enu)
                    scan_str = "▶ ĐANG QUÉT" if self.lidar_scanning else "⏸ ĐÃ DỪNG"
                    log.debug(f"[3D MAP] Trạng thái: {scan_str} | Điểm: {n_pts:,} pts | UAV: (X:{curr_pos[0]:.1f}m, Y:{curr_pos[1]:.1f}m, Cao:{curr_pos[2]:.1f}m) | [J]: Bật/Dừng | [S]: Lưu PLY")

                # 4. Xử lý sự kiện cửa sổ & chuột ở nhịp ổn định
                if not vis.poll_events():
                    break
                vis.update_renderer()

            except Exception:
                time.sleep(0.01)

            time.sleep(0.025)  # ~40 FPS: đủ mượt, giảm đáng kể tải OpenGL/GPU

        try:
            vis.destroy_window()
        except Exception:
            pass
        self.running = False
