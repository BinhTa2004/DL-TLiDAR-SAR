# -*- coding: utf-8 -*-
"""Accumulate and persist the shared physical LiDAR stream (Report Section 3.2.7).

The map worker owns the mutable Open3D cloud and publishes independent NumPy
snapshots. The renderer is implemented separately in app.lidar_display.
"""

from configs.config import ROOT, local_path, MAPPING
from app.lidar_display import LidarDisplayMixin, generate_target_box_wires as generate_target_box_wires
from app.runtime_support import get_logger
log = get_logger("lidar_3d")

import os
import sys
import ctypes
import time
import json
import threading
import numpy as np

if sys.platform.startswith("win"):
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace', line_buffering=True)
        sys.stderr.reconfigure(encoding='utf-8', errors='replace', line_buffering=True)
    except Exception:
        pass

import open3d as o3d
import matplotlib as mpl
from app.shared_lidar_stream import LidarStreamReader

__all__ = ["UAVLiDAR3DReconstruction", "generate_target_box_wires", "main",
           "quat_to_R", "euler_to_R"]

SCRIPT_DIR = str(ROOT)

# ---------------------------------------------------------------------------
# CẤU HÌNH TOÁN HỌC & HỆ TỌA ĐỘ
# ---------------------------------------------------------------------------
def quat_to_R(q):
    """Chuyển đổi Quaternion sang ma trận xoay 3x3"""
    w = getattr(q, 'w_val', getattr(q, 'w', 1.0))
    x = getattr(q, 'x_val', getattr(q, 'x', 0.0))
    y = getattr(q, 'y_val', getattr(q, 'y', 0.0))
    z = getattr(q, 'z_val', getattr(q, 'z', 0.0))
    return np.array([
        [1.0 - 2.0*(y*y + z*z),       2.0*(x*y - z*w),       2.0*(x*z + y*w)],
        [      2.0*(x*y + z*w), 1.0 - 2.0*(x*x + z*z),       2.0*(y*z - x*w)],
        [      2.0*(x*z - y*w),       2.0*(y*z + x*w), 1.0 - 2.0*(x*x + y*y)]
    ], dtype=np.float32)

def euler_to_R(pitch_deg, roll_deg, yaw_deg):
    """Ma trận xoay Euler Pitch-Roll-Yaw"""
    p, r, y = np.radians(pitch_deg), np.radians(roll_deg), np.radians(yaw_deg)
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]], dtype=np.float32)
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]], dtype=np.float32)
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]], dtype=np.float32)
    return Rz @ Ry @ Rx


# ---------------------------------------------------------------------------
# BỘ ĐỌC TAY CẦM XBOX XINPUT (WINDOWS ZERO-LATENCY C-API)
# ---------------------------------------------------------------------------
class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [
        ("wButtons", ctypes.c_ushort),
        ("bLeftTrigger", ctypes.c_ubyte),
        ("bRightTrigger", ctypes.c_ubyte),
        ("sThumbLX", ctypes.c_short),
        ("sThumbLY", ctypes.c_short),
        ("sThumbRX", ctypes.c_short),
        ("sThumbRY", ctypes.c_short),
    ]

class XINPUT_STATE(ctypes.Structure):
    _fields_ = [
        ("dwPacketNumber", ctypes.c_ulong),
        ("Gamepad", XINPUT_GAMEPAD),
    ]

class XInputGamepadReader:
    def __init__(self):
        self.xinput = None
        for dll in ("xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll"):
            try:
                self.xinput = ctypes.windll.LoadLibrary(dll)
                break
            except Exception:
                pass
        self._last_dpad_right = False
        self._last_poll_t = 0.0
        self.state = XINPUT_STATE()

    def poll_buttons(self):
        """Đọc cạnh nhấn D-Pad phải; D-Pad lên do GCS xử lý và đồng bộ qua JSON."""
        if not self.xinput:
            return False, False
        now = time.perf_counter()
        if now - self._last_poll_t < 0.04:  # Tối đa 25 Hz
            return False, False
        self._last_poll_t = now

        try:
            res = self.xinput.XInputGetState(0, ctypes.byref(self.state))
            if res != 0:
                self._last_dpad_right = False
                return False, False
            btns = self.state.Gamepad.wButtons
            dpad_right = bool(btns & 0x0008)

            edge_right = dpad_right and not self._last_dpad_right

            self._last_dpad_right = dpad_right

            return edge_right, False
        except Exception:
            return False, False


# ---------------------------------------------------------------------------
# HÌNH HỌC KHUNG ĐÁNH DẤU MỤC TIÊU 3D & CỘT TIÊU CỨU HỘ QUANH NẠN NHÂN
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# CLASS CHÍNH: 3D LIDAR RECONSTRUCTION ENGINE (SIÊU ỔN ĐỊNH & ZERO-CRASH)
# ---------------------------------------------------------------------------
class UAVLiDAR3DReconstruction(LidarDisplayMixin):
    def __init__(self):
        self.running = True
        self.lock = threading.Lock()

        # Cấu hình cảm biến LiDAR Ouster OS1-64 trên UAV
        self.lidar_name = "Lidar1"
        self.vehicle_name = "UAV1"
        self.lidar_mount_pos = np.array([0.35, 0.0, 0.15], dtype=np.float32)
        self.lidar_mount_R = euler_to_R(-20.0, 0.0, 0.0)

        # Trạng thái Drone (hệ tọa độ thế giới ENU: +X Đông, +Y Bắc, +Z Cao độ)
        self.uav_pos_enu = np.zeros(3, dtype=np.float32)
        self.connected_airsim = False
        self.stream_session_id = None
        self._map_session_id = None
        self._clear_map_requested = False

        # Bản đồ đám mây điểm tích lũy C++ Open3D (sở hữu độc quyền bởi worker thread)
        self.global_pcd = o3d.geometry.PointCloud()
        self.voxel_size = MAPPING.scan_voxel_m             # Lọc từng đợt điểm LiDAR
        self.global_voxel_size = MAPPING.global_voxel_m      # Lọc lại toàn map khi vượt ngưỡng điểm
        self.max_map_points = MAPPING.max_points      # Chặn các đợt khựng do cloud nhiều triệu điểm

        # Trạng thái quét LiDAR: Mặc định BẬT đồng bộ cùng hệ thống
        self.lidar_scanning = True
        self.paused = False
        self._last_toggle_t = 0.0
        self.gamepad = XInputGamepadReader()

        # Hàng đợi đệm điểm LiDAR mới nhận
        self.new_points_buffer = []

        # Cơ chế cách ly dữ liệu hiển thị (Thread-Safe Snapshot):
        # Renderer nhận bản sao NumPy độc lập, không dùng chung con trỏ C++ thô
        self.need_gpu_update = False
        self.pending_render_data = None    # tuple: (pts_arr, colors_arr)
        self.cached_snap_pts = None        # Mảng NumPy tĩnh cho thuật toán snap chân đế
        self.recolor_needed = True

        # Bảng màu hiển thị độ cao
        self.colormaps = ["turbo", "jet", "viridis"]
        self.colormap_idx = 0
        self.follow_uav = False            # MẶC ĐỊNH: TẮT follow để người dùng điều khiển chuột tự do 100%
        self.point_size = 1.0              # Kích thước hạt điểm 1.0px sắc nét chuẩn khảo sát địa hình LiDAR

        # Dải độ cao Z được cập nhật động mượt mà
        self.z_min = -5.0
        self.z_max = 35.0
        self._cached_z_bounds = (self.z_min, self.z_max)

        # Thư mục xuất dữ liệu
        self.output_dir = os.path.join(SCRIPT_DIR, "outputs")
        os.makedirs(self.output_dir, exist_ok=True)

        # Cấu hình Khung Đánh Dấu Vị Trí Người & Nạn Nhân Cứu Hộ 3D
        self.target_lineset = None
        self.show_targets = True
        self.live_targets_path = os.path.join(self.output_dir, "live_targets.json")
        self.lidar_stream_path = os.path.join(self.output_dir, "lidar_stream.bin")
        self.last_targets_poll = 0.0
        self.cached_targets_mtime = 0.0
        self.cached_target_data = None
        self._has_active_marker_geo = False
        self.known_actors_enu = []

        # Đồng bộ trạng thái quét LiDAR hai chiều với GCS (chống xung đột timestamp cũ)
        self.lidar_sync_file = os.path.join(self.output_dir, "lidar_state.json")
        self._last_lidar_sync_ts = time.time()  # Chỉ lắng nghe thay đổi sau thời điểm khởi chạy
        self._cached_sync_mtime = 0.0

    def _sync_lidar_state(self, from_self=False):
        """Đồng bộ trạng thái Bật/Tắt LiDAR hai chiều giữa GCS và Bản đồ 3D an toàn"""
        sync_file = self.lidar_sync_file
        if from_self:
            try:
                now_ts = time.time()
                self._last_lidar_sync_ts = now_ts
                tmp = sync_file + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump({"lidar_enabled": bool(self.lidar_scanning), "source": "3d_map",
                               "timestamp": now_ts, "session_id": self.stream_session_id}, f)
                os.replace(tmp, sync_file)
            except Exception:
                pass
        else:
            if os.path.exists(sync_file):
                try:
                    mtime = os.path.getmtime(sync_file)
                    if mtime != getattr(self, "_cached_sync_mtime", 0.0):
                        self._cached_sync_mtime = mtime
                        with open(sync_file, "r", encoding="utf-8") as f:
                            data = json.load(f)
                        ts = data.get("timestamp", 0.0)
                        src = data.get("source", "")
                        if (self.stream_session_id is not None and
                                data.get("session_id") == self.stream_session_id and
                                src != "3d_map" and ts > self._last_lidar_sync_ts):
                            self._last_lidar_sync_ts = ts
                            req_enabled = bool(data.get("lidar_enabled", True))
                            if req_enabled != self.lidar_scanning:
                                self.toggle_scanning(force_val=req_enabled)
                except Exception:
                    pass

    def toggle_scanning(self, force_val=None):
        """Bật / Tắt quét và tích lũy đám mây điểm LiDAR (D-Pad Phải / Phím J) có chống rung"""
        now = time.perf_counter()
        if force_val is None and (now - self._last_toggle_t < 0.35):
            return
        self._last_toggle_t = now

        if force_val is None:
            self.lidar_scanning = not self.lidar_scanning
        else:
            self.lidar_scanning = bool(force_val)
        self.paused = not self.lidar_scanning

        if self.lidar_scanning:
            log.debug("\n" + "="*70)
            log.debug("  >> [LIDAR SCAN] ▶ BẮT ĐẦU QUÉT ĐỊA HÌNH 3D [D-PAD PHẢI ▶ / PHÍM J]")
            log.debug("="*70)
        else:
            log.debug("\n" + "="*70)
            log.debug("  >> [LIDAR SCAN] ⏸ ĐÃ TẠM DỪNG / DỪNG QUÉT ĐỊA HÌNH [D-PAD PHẢI ▶ / PHÍM J]")
            log.debug("="*70)

        if force_val is None:
            self._sync_lidar_state(from_self=True)

    # -----------------------------------------------------------------------
    # LUỒNG 1: NHẬN SNAPSHOT TỪ LUỒNG LIDAR DUY NHẤT CỦA GCS
    # -----------------------------------------------------------------------
    def _consume_lidar_snapshot(self, snapshot):
        """Invalidate old snapshots immediately when a new GCS session appears."""
        session_id = snapshot["session_id"]
        with self.lock:
            if self.stream_session_id is not None and session_id < self.stream_session_id:
                return
            if session_id != self.stream_session_id:
                self.stream_session_id = session_id
                self.new_points_buffer = []
                self.cached_snap_pts = None
                self.pending_render_data = None
                self.need_gpu_update = False
                self.cached_target_data = None
                self.cached_targets_mtime = 0.0
                self.uav_pos_enu = np.zeros(3, dtype=np.float32)
            if not self.lidar_scanning or self.paused or not len(snapshot["points_ned"]):
                return
            world_ned = snapshot["points_ned"]
            pos_ned = snapshot["uav_pos_ned"]
            self.uav_pos_enu = np.array([pos_ned[1], pos_ned[0], -pos_ned[2]], dtype=np.float32)
            self.new_points_buffer.append(np.column_stack([
                world_ned[:, 1], world_ned[:, 0], -world_ned[:, 2]
            ]).astype(np.float32, copy=False))

    def lidar_worker(self):
        """Đọc memory-map do GCS xuất; tiến trình này không kết nối AirSim."""
        log.debug(">> [LIDAR SHARE] Đang chờ snapshot LiDAR dùng chung từ GCS...")
        reader = LidarStreamReader(self.lidar_stream_path)
        last_scan_key = None
        last_sync_lidar_t = 0.0
        announced = False

        while self.running:
            try:
                now_t = time.perf_counter()

                # Đồng bộ trạng thái Bật/Tắt định kỳ (3 Hz, không gây áp lực ổ đĩa)
                if now_t - last_sync_lidar_t >= 0.30:
                    last_sync_lidar_t = now_t
                    self._sync_lidar_state(from_self=False)

                snapshot = reader.read_latest()
                if snapshot is None:
                    self.connected_airsim = False
                    time.sleep(0.05)
                    continue

                age_s = max(0.0, (time.time_ns() - snapshot["timestamp_ns"]) / 1e9)
                self.connected_airsim = age_s < 2.0
                scan_key = (snapshot["session_id"], snapshot["sequence"])
                if scan_key != last_scan_key:
                    last_scan_key = scan_key
                    if not announced:
                        log.debug(">> [LIDAR SHARE] ĐÃ NHẬN LUỒNG LIDAR CHUNG TỪ GCS!")
                        announced = True
                    self._consume_lidar_snapshot(snapshot)

                time.sleep(0.04)  # 25 Hz polling, dữ liệu nguồn vẫn 10 Hz
            except Exception:
                self.connected_airsim = False
                time.sleep(0.1)

        reader.close()

    # -----------------------------------------------------------------------
    # LUỒNG 2: HỢP NHẤT BẢN ĐỒ BẰNG C++ VOXEL GRID (CHẠY ĐỊNH KỲ 2 HZ)
    # -----------------------------------------------------------------------
    def _compute_point_colors(self, z_array, z_min, z_max, cmap_name):
        """Tính bảng màu độ cao chuẩn xác và hiệu quả"""
        z_range = max(1.0, z_max - z_min)
        norm_z = np.clip((z_array - z_min) / z_range, 0.0, 1.0)
        cmap = mpl.colormaps[cmap_name]
        return cmap(norm_z)[:, :3].astype(np.float64)

    def map_consolidation_worker(self):
        """
        Hợp nhất các điểm mới vào bản đồ Open3D C++:
        Sở hữu độc quyền self.global_pcd và xuất bản sao NumPy tĩnh an toàn cho render
        """
        while self.running:
            time.sleep(0.50)  # 2 Hz: gom nhiều scan, giảm số lần sao chép cloud toàn cục

            # 1. Rút các mảng điểm mới từ bộ đệm
            with self.lock:
                session_id = self.stream_session_id
                reset_map = session_id != self._map_session_id or self._clear_map_requested
                self._clear_map_requested = False
                if not self.new_points_buffer:
                    scans_to_merge = []
                else:
                    scans_to_merge = self.new_points_buffer
                    self.new_points_buffer = []

            if reset_map:
                # Only this worker owns the C++ cloud; the render thread gets
                # independent empty arrays, never a concurrent clear() call.
                self._reset_map_for_session(session_id)

            if not scans_to_merge or self.paused:
                if self.recolor_needed:
                    self._trigger_full_recolor()
                continue

            try:
                all_new = np.vstack(scans_to_merge)
                if len(all_new) < 5:
                    continue

                # Tạo PointCloud tạm thời và lọc voxel cục bộ C++
                temp_pcd = o3d.geometry.PointCloud()
                temp_pcd.points = o3d.utility.Vector3dVector(all_new.astype(np.float64))
                temp_pcd = temp_pcd.voxel_down_sample(self.voxel_size)

                n_new_pts = len(temp_pcd.points)
                if n_new_pts == 0:
                    continue

                # Ước tính dải Z động
                new_z = np.asarray(temp_pcd.points)[:, 2]
                curr_min, curr_max = self._cached_z_bounds
                z_lo = min(curr_min, float(new_z.min()))
                z_hi = max(curr_max, float(new_z.max()))

                # Nếu dải Z thay đổi quá 15%, đánh dấu cần tô lại toàn bộ màu
                range_delta = (z_hi - z_lo) - (curr_max - curr_min)
                need_recolor = (range_delta > 0.15 * max(1.0, curr_max - curr_min)) or self.recolor_needed
                self._cached_z_bounds = (z_lo, z_hi)

                # Gán màu độ cao cho mảng điểm mới
                cmap_name = self.colormaps[self.colormap_idx]
                new_colors = self._compute_point_colors(new_z, z_lo, z_hi, cmap_name)
                temp_pcd.colors = o3d.utility.Vector3dVector(new_colors)

                # Hợp nhất vào đám mây toàn cục (chỉ luồng này sở hữu)
                self.global_pcd += temp_pcd

                # Lọc voxel toàn cục khi số điểm vượt ngưỡng để tránh spike bộ nhớ
                if len(self.global_pcd.points) > self.max_map_points:
                    self.global_pcd = self.global_pcd.voxel_down_sample(self.global_voxel_size)
                    need_recolor = True

                if need_recolor:
                    pts = np.asarray(self.global_pcd.points)
                    if len(pts) > 10:
                        z_vals = pts[:, 2]
                        z_min = float(np.percentile(z_vals, 2)) if len(z_vals) > 100 else float(z_vals.min())
                        z_max = float(np.percentile(z_vals, 98)) if len(z_vals) > 100 else float(z_vals.max())
                        self._cached_z_bounds = (z_min, z_max)
                        full_colors = self._compute_point_colors(z_vals, z_min, z_max, cmap_name)
                        self.global_pcd.colors = o3d.utility.Vector3dVector(full_colors)
                        self.z_min = z_min
                        self.z_max = z_max
                    self.recolor_needed = False

                # TẠO BẢN CHỤP NUMPY ĐỘC LẬP (Snapshot Isolation):
                # Triệt tiêu hoàn toàn race condition trong VCRUNTIME140.dll & NumPy umath
                pts_snapshot = np.array(self.global_pcd.points, copy=True, dtype=np.float64)
                colors_snapshot = np.array(self.global_pcd.colors, copy=True, dtype=np.float64)

                with self.lock:
                    if session_id == self.stream_session_id and not self._clear_map_requested:
                        self.pending_render_data = (pts_snapshot, colors_snapshot)
                        self.cached_snap_pts = pts_snapshot
                        self.need_gpu_update = True

            except Exception:
                pass

    def _reset_map_for_session(self, session_id):
        """Run exclusively on the consolidation worker, including manual clears."""
        self.global_pcd = o3d.geometry.PointCloud()
        self._map_session_id = session_id
        self.z_min, self.z_max = -5.0, 35.0
        self._cached_z_bounds = (self.z_min, self.z_max)
        self.recolor_needed = True
        with self.lock:
            if session_id == self.stream_session_id:
                empty = np.empty((0, 3), dtype=np.float64)
                self.cached_snap_pts = empty
                self.pending_render_data = (empty, empty.copy())
                self.need_gpu_update = True

    def _trigger_full_recolor(self):
        """Tô lại màu toàn bộ bản đồ khi người dùng đổi bảng màu [F5]"""
        try:
            pts = np.asarray(self.global_pcd.points)
            if len(pts) > 0:
                z_vals = pts[:, 2]
                z_min, z_max = self._cached_z_bounds
                cmap_name = self.colormaps[self.colormap_idx]
                full_colors = self._compute_point_colors(z_vals, z_min, z_max, cmap_name)
                self.global_pcd.colors = o3d.utility.Vector3dVector(full_colors)
                pts_snapshot = np.array(self.global_pcd.points, copy=True, dtype=np.float64)
                colors_snapshot = np.array(self.global_pcd.colors, copy=True, dtype=np.float64)
                with self.lock:
                    if self._map_session_id == self.stream_session_id and not self._clear_map_requested:
                        self.pending_render_data = (pts_snapshot, colors_snapshot)
                        self.cached_snap_pts = pts_snapshot
                        self.need_gpu_update = True
            self.recolor_needed = False
        except Exception:
            pass

    # -----------------------------------------------------------------------
    # XUẤT ĐÁM MÂY ĐIỂM SANG FILE .PLY & TỌA ĐỘ NẠN NHÂN .JSON/.CSV
    # -----------------------------------------------------------------------
    def save_point_cloud(self, prefix="medieval_village_3d_pointcloud"):
        """Lưu toàn bộ bản đồ 3D ra file chuẩn Open3D / MeshLab / CloudCompare"""
        with self.lock:
            if self.cached_snap_pts is None or len(self.cached_snap_pts) == 0:
                log.debug(">> [LƯU BẢN ĐỒ] Bản đồ hiện tại trống, không có điểm để lưu!")
                return None
            pts_copy = np.copy(self.cached_snap_pts)

        pcd_copy = o3d.geometry.PointCloud()
        pcd_copy.points = o3d.utility.Vector3dVector(pts_copy)

        z = pts_copy[:, 2]
        cmap_name = self.colormaps[self.colormap_idx]
        norm_colors = self._compute_point_colors(z, self.z_min, self.z_max, cmap_name)
        pcd_copy.colors = o3d.utility.Vector3dVector(norm_colors)

        out_path = str(local_path(os.path.join(self.output_dir, f"{prefix}_{time.time_ns()}.ply")))

        # Lọc sạch ngoại lai nhẹ trước khi lưu
        pcd_clean = pcd_copy
        if len(pts_copy) > 16:
            filtered, _ = pcd_copy.remove_statistical_outlier(nb_neighbors=16, std_ratio=2.0)
            if len(filtered.points):
                pcd_clean = filtered

        # Xuất kèm danh sách tọa độ các nạn nhân đã đánh dấu nếu có
        td = self.cached_target_data or {}
        if os.path.exists(self.live_targets_path):
            try:
                with open(self.live_targets_path, "r", encoding="utf-8") as f:
                    td = json.load(f)
                if self.stream_session_id is None or td.get("session_id") != self.stream_session_id:
                    td = {}
            except (OSError, ValueError) as ex:
                log.warning(f">> [CẢNH BÁO] Dùng bản mục tiêu đã đọc trước đó: {ex}")
                td = self.cached_target_data or {}
        targets_to_export = td.get("marked_persons", [])

        # Always save a sidecar, including an empty list. Never load live targets
        # when reopening an older map.
        try:
            if not o3d.io.write_point_cloud(out_path, pcd_clean, write_ascii=False):
                raise OSError("Open3D không ghi được file PLY")
            out_targets_json = out_path.replace(".ply", "_targets.json")
            out_targets_csv = out_path.replace(".ply", "_targets.csv")
            with open(out_targets_json, "w", encoding="utf-8") as f:
                json.dump(targets_to_export, f, indent=2, ensure_ascii=False, allow_nan=False)
            with open(out_path.replace(".ply", "_metadata.json"), "w", encoding="utf-8") as f:
                json.dump({
                    "version": 1,
                    "start_pos_enu": td.get("start_pos_enu"),
                    "start_pos_ned": td.get("start_pos_ned"),
                    "route_lines_enabled": bool(td.get("route_lines_enabled", False)),
                }, f, indent=2, ensure_ascii=False, allow_nan=False)
            import csv
            with open(out_targets_csv, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["ID", "TIME", "X_ENU", "Y_ENU", "Z_ENU", "X_NED", "Y_NED", "Z_NED", "LABEL"])
                for t in targets_to_export:
                    writer.writerow([
                        t.get("id", 1), t.get("time", ""),
                        t.get("x_enu", ""), t.get("y_enu", ""), t.get("z_enu", ""),
                        t.get("x_ned", ""), t.get("y_ned", ""), t.get("z_ned", ""),
                        t.get("label", "")
                    ])
        except (OSError, ValueError, TypeError, RuntimeError) as ex:
            # A partial bundle must not look like a successfully saved map.
            for partial in (out_path, out_path.replace(".ply", "_targets.json"),
                            out_path.replace(".ply", "_targets.csv"),
                            out_path.replace(".ply", "_metadata.json")):
                if os.path.isfile(partial):
                    os.remove(partial)
            log.warning(f">> [LỖI LƯU BẢN ĐỒ] {ex}")
            return None

        log.info(f"\n>> [LƯU THÀNH CÔNG] {len(pcd_clean.points):,} điểm, {len(targets_to_export)} mốc cứu hộ.")
        log.debug(f"   -> {out_path}")
        log.debug("   Mở OPEN_SAVED_3D_MAP.bat để xem lại bản đồ và vị trí nạn nhân.\n")

        return out_path

    # -----------------------------------------------------------------------
    # KHÓA CHỐT & CẬP NHẬT KHUNG ĐÁNH DẤU NẠN NHÂN 3D TRÊN BẢN ĐỒ
    # -----------------------------------------------------------------------
    def snap_target_to_pointcloud(self, center_enu, snap_pts):
        """
        Khóa chốt và tinh chỉnh cao độ mục tiêu 3D vào đúng bề mặt đám mây điểm đã quét:
        - Sử dụng snapshot NumPy tĩnh an toàn tuyệt đối, triệt tiêu 100% crash trong umath
        - Căn đáy Bounding Box khớp với cao độ mặt đất thực tế tại khu vực bán kính 2.5m
        """
        if snap_pts is None or len(snap_pts) < 30:
            return center_enu

        cx, cy, cz = float(center_enu[0]), float(center_enu[1]), float(center_enu[2])
        d_xy_sq = (snap_pts[:, 0] - cx) ** 2 + (snap_pts[:, 1] - cy) ** 2
        near_m = d_xy_sq <= 6.25  # bán kính 2.5m
        if np.count_nonzero(near_m) >= 3:
            local_pts = snap_pts[near_m]
            z_diff = np.abs(local_pts[:, 2] - cz)
            z_close_m = z_diff <= 4.0
            cand_pts = local_pts[z_close_m] if np.count_nonzero(z_close_m) >= 3 else local_pts
            z_ground = float(np.percentile(cand_pts[:, 2], 40))
            res = np.copy(center_enu)
            res[2] = z_ground + 0.9  # Tâm hộp cao 1.8m nằm tại z_ground + 0.9m
            return res

        return center_enu


    # -----------------------------------------------------------------------
    # CỬA SỔ TRỰC QUAN HÓA 3D OPEN3D (RENDER 40 FPS ỔN ĐỊNH)
    # -----------------------------------------------------------------------


# ---------------------------------------------------------------------------
# ĐIỂM VÀO THỰC THI (ENTRY POINT)
# ---------------------------------------------------------------------------
def main():
    """Run the viewer with the original KeyboardInterrupt cleanup."""
    app = UAVLiDAR3DReconstruction()
    try:
        app.run_viewer()
    except KeyboardInterrupt:
        app.running = False
        log.debug("\n>> Đang dừng hệ thống dựng hình 3D...")


if __name__ == "__main__":
    main()
