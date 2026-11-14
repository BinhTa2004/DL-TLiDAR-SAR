# -*- coding: utf-8 -*-
"""UAV control, application state and sensor workers. Entry point: uav_gcs_station.py."""

from pathlib import Path
import sys

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from configs.config import ROOT, local_path, DETECTOR, RUNTIME
from pipeline.acquisition import parse_image_response, recent_lidar_points
from pipeline.thermal_detector import detect_thermal_targets, synthesize_radiometric_flir, temperature_from_intensity
from app.gcs_display import GCSDisplayMixin
from app.flight_control import (
    FlightControlMixin, quat_multiply as quat_multiply, quat_inverse as quat_inverse,
)
from app.sensor_workers import SensorWorkersMixin
from app.mission_records import MissionRecordsMixin
from app.input_devices import XInputController
from pipeline.frustum_alignment import quat_to_rotation as quat_to_R
from app.layout import DEFAULT_GCS_W, DEFAULT_GCS_H, CANVAS_W, CANVAS_H
from app.runtime_support import connect_airsim, get_logger, start_live_session
from app.detection_policy import SEARCH_CLASS_IDS, target_presentation
log = get_logger("gcs")

import sys
try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace', line_buffering=True)
    sys.stderr.reconfigure(encoding='utf-8', errors='replace', line_buffering=True)
except Exception:
    pass

import logging
logging.getLogger("tornado.general").setLevel(logging.ERROR)
logging.getLogger("tornado.application").setLevel(logging.ERROR)

import os
import time
import math
import json
import threading
import queue
import ctypes
from collections import deque
from app.shared_lidar_stream import LidarStreamWriter

__all__ = [
    "UAVUnifiedSystem", "main", "quat_to_R", "euler_to_R", "quat_multiply",
    "quat_inverse", "auto_arrange_windows", "open_or_focus_3d_map",
]

# ---------------------------------------------------------------------------
# WINDOWS WIN32 & XINPUT ZERO-LATENCY APIS
# ---------------------------------------------------------------------------
user32 = ctypes.windll.user32
try:
    ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(), 0x00008000)
except Exception:
    pass


try:
    import win32gui
    import win32con
    HAS_WIN32 = True
except Exception:
    HAS_WIN32 = False

def auto_arrange_windows(restore=False):
    """Hide the extra Unreal view without changing user window placement.

    Restoring normal window sizes is reserved for an explicit F10/button action.
    """
    if not HAS_WIN32:
        return
    def enum_handler(hwnd, _):
        if not win32gui.IsWindowVisible(hwnd):
            return True
        title = win32gui.GetWindowText(hwnd)
        cls = win32gui.GetClassName(hwnd)
        if ("MedievalVillage" in title or "Africa" in title or "UnrealWindow" in cls) and "UAV GCS" not in title:
            try:
                if os.environ.get("UAV_NO_VIEWPORT") == "1":
                    win32gui.ShowWindow(hwnd, win32con.SW_HIDE)
                elif restore:
                    win32gui.ShowWindow(hwnd, win32con.SW_SHOWNORMAL)
            except Exception: pass
        elif restore and "UAV GCS" in title:
            try: win32gui.ShowWindow(hwnd, win32con.SW_SHOWNORMAL)
            except Exception: pass
        elif restore and ("DUNG DIA HINH 3D LIDAR" in title or "POINTR" in title or "OPEN3D" in title):
            try: win32gui.ShowWindow(hwnd, win32con.SW_SHOWNOACTIVATE)
            except Exception: pass
        return True
    try:
        win32gui.EnumWindows(enum_handler, None)
    except Exception:
        pass

def open_or_focus_3d_map():
    """Tìm và kích hoạt cửa sổ 3D Open3D lên phía trước hoặc khởi chạy nếu chưa chạy"""
    if HAS_WIN32:
        found = []
        def enum_cb(hwnd, _):
            title = win32gui.GetWindowText(hwnd)
            if "DUNG DIA HINH 3D LIDAR" in title or "POINTR" in title:
                found.append(hwnd)
            return True
        try:
            win32gui.EnumWindows(enum_cb, None)
        except Exception:
            pass
        if found:
            hwnd = found[0]
            try:
                if win32gui.IsIconic(hwnd):
                    win32gui.ShowWindow(hwnd, win32con.SW_RESTORE)
                elif not win32gui.IsWindowVisible(hwnd):
                    win32gui.ShowWindow(hwnd, win32con.SW_SHOW)
                win32gui.SetForegroundWindow(hwnd)
            except Exception:
                log.debug("[MAP 3D] Cửa sổ đã mở; Windows chưa cho chuyển tiêu điểm.", exc_info=True)
            # Do not create another viewer if Windows rejects a focus request.
            return True
    # Nếu chưa mở, khởi chạy tiến trình app/viewer_3d.py
    script_path = os.path.join(str(ROOT), "app", "viewer_3d.py")
    if os.path.exists(script_path):
        try:
            import subprocess
            creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
            subprocess.Popen(
                [sys.executable, script_path], cwd=SCRIPT_DIR,
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, creationflags=creationflags,
            )
            return True
        except Exception:
            pass
    return False

import airsim
import numpy as np
import cv2
cv2.setNumThreads(2)
import pipeline as sfc

try:
    import torch
    from ultralytics import YOLO
    HAS_YOLO = True
except Exception:
    HAS_YOLO = False

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
import pygame
import pygame._sdl2.controller as sdl2_controller

# ---------------------------------------------------------------------------
# 1. HẰNG SỐ & BỐ CỤC GIAO DIỆN
# ---------------------------------------------------------------------------
SCRIPT_DIR = str(ROOT)

GROUND_Z_DATUM = RUNTIME.ground_z_ned_m      # Z (NED) mặt đất tại bãi xuất phát MedievalVillage
THERMAL_FOV_DEG = RUNTIME.thermal_fov_deg      # FLIR Boson ống kính tầm xa
DEFAULT_GIMBAL_PITCH = RUNTIME.default_gimbal_pitch_deg  # Góc nghiêng mặc định: 0.0 = Cùng hướng UAV (nhìn thẳng phía trước)
CMD_PERIOD = RUNTIME.command_period_s          # 33Hz Heartbeat (phản hồi tức thì)
CMD_DURATION = RUNTIME.command_duration_s         # 80ms Watchdog đệm mượt (loại bỏ hủy trajectory của SimpleFlight)
THERMAL_CAPTURE_PERIOD = RUNTIME.capture_period_s  # RGB va IR cua front_camera duoc chup cung mot lan RPC
RAW_IMAGE_IS_BGR = RUNTIME.raw_image_is_bgr

# ---------------------------------------------------------------------------
# 2. CẤU HÌNH PHẦN CỨNG & HÌNH HỌC
# ---------------------------------------------------------------------------
def load_hw_config():
    cfg = {
        "lidar": {"X": 0.35, "Y": 0.0, "Z": 0.15, "Roll": 0.0, "Pitch": -20.0, "Yaw": 0.0},
        "cam": {"X": 0.35, "Y": 0.0, "Z": 0.15, "Pitch": 0.0, "W": 640, "H": 512},
    }
    for path in (str(ROOT / "simulation/airsim_settings.json"),
                 os.path.join(SCRIPT_DIR, "settings.json")):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            veh = next(iter(data.get("Vehicles", {}).values()))
            lid = next(iter(veh.get("Sensors", {}).values()))
            for k in ("X", "Y", "Z", "Roll", "Pitch", "Yaw"):
                if k in lid: cfg["lidar"][k] = float(lid[k])
            cam = veh.get("Cameras", {}).get("front_camera", {})
            for k in ("X", "Y", "Z", "Pitch"):
                if k in cam: cfg["cam"][k] = float(cam[k])
            for cs in cam.get("CaptureSettings", []):
                if cs.get("ImageType") == 7:
                    cfg["cam"]["W"] = int(cs.get("Width", cfg["cam"]["W"]))
                    cfg["cam"]["H"] = int(cs.get("Height", cfg["cam"]["H"]))
            break
        except Exception:
            continue
    return cfg


def euler_to_R(pitch_deg, roll_deg=0.0, yaw_deg=0.0):
    q = airsim.to_quaternion(math.radians(pitch_deg), math.radians(roll_deg), math.radians(yaw_deg))
    return quat_to_R((q.w_val, q.x_val, q.y_val, q.z_val))

# ---------------------------------------------------------------------------
# 3. CLASS HỆ THỐNG GCS ĐA CẢM BIẾN CHỐNG RƠI & TỰ PHỤC HỒI KẾT NỐI
# ---------------------------------------------------------------------------
class UAVUnifiedSystem(GCSDisplayMixin, FlightControlMixin, SensorWorkersMixin, MissionRecordsMixin):
    synthesize_radiometric_flir = staticmethod(synthesize_radiometric_flir)
    temp_of = staticmethod(temperature_from_intensity)

    def parse_img(self, resp):
        """Report Section 3.2.2: decode a captured sensor frame."""
        return parse_image_response(resp)

    def detect_persons(self, th_gray, infrared_frame=None):
        """Report Section 3.2.3: delegate inference without drawing or flight commands."""
        detections, self.yolo_fps = detect_thermal_targets(
            th_gray, infrared_frame, model=self.yolo_model, enabled=self.yolo_enabled,
            device=self.yolo_device, imgsz=self.yolo_imgsz, confidence=self.yolo_conf,
            person_threshold=self.person_thresh, previous_fps=self.yolo_fps,
        )
        return detections

    def __init__(self):
        self.session_id = (
            int(os.environ["UAV_SESSION_ID"])
            if os.environ.get("UAV_SESSION_ID")
            else start_live_session()
        )
        self.running = True
        self.lock = threading.Lock()
        self.vehicle_name = "UAV1"
        self.win_w = DEFAULT_GCS_W
        self.win_h = DEFAULT_GCS_H
        self.fullscreen = False

        self.hw = load_hw_config()
        lc = self.hw["lidar"]
        self.lidar_mount_pos = np.array([lc["X"], lc["Y"], lc["Z"]], dtype=np.float32)
        self.lidar_mount_R = euler_to_R(lc["Pitch"], lc["Roll"], lc["Yaw"])
        cc = self.hw["cam"]
        self.cam_mount_pos = airsim.Vector3r(cc["X"], cc["Y"], cc["Z"])
        self.cam_mount_np = np.array([cc["X"], cc["Y"], cc["Z"]], dtype=np.float32)
        self.cam_w, self.cam_h = cc["W"], cc["H"]
        self.ground_z_datum = GROUND_Z_DATUM
        self.terrain_agl = None         # Độ cao AGL thực tế đo bằng LiDAR

        # Telemetry
        self.telemetry = {
            "connected": False,
            "status": "ĐANG KẾT NỐI...",
            "x": 0.0, "y": 0.0, "z": 0.0,
            "alt": 0.0,
            "alt_asl": 0.0,
            "alt_agl": 0.0,
            "vx": 0.0, "vy": 0.0, "vz": 0.0,
            "speed": 0.0,
            "pitch": 0.0, "roll": 0.0, "yaw": 0.0,
            "quat": (1.0, 0.0, 0.0, 0.0),
            "battery": 100.0,
            "flight_time": 0.0,
            "points_count": 0
        }

        # Gimbal Camera Angle: 0.0 = Ngang chân trời, -45 = Nghiêng tìm kiếm, -90.0 = Xuống mặt đất
        self.target_gimbal_pitch = DEFAULT_GIMBAL_PITCH
        self.gimbal_pitch = DEFAULT_GIMBAL_PITCH
        self.gimbal_applied = None
        self._last_gimbal_set_time = 0.0
        self.invert_cam_pitch = False   # Chế độ đảo chiều góc cam [I]

        # Điều khiển vận tốc & Tự động giữ độ cao (Altitude Hold)
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.cmd_yaw_rate = 0.0
        self.smooth_vx = 0.0
        self.smooth_vy = 0.0
        self.smooth_vz = 0.0
        self.smooth_yr = 0.0
        self.curr_accel_x = 0.0
        self.curr_accel_y = 0.0
        self.filtered_drone_pitch = 0.0
        self.filtered_drone_roll = 0.0
        self.smooth_radar_yaw = 0.0
        self.gimbal_stabilization_enabled = True   # Chống rung gimbal 2 trục chủ động (Roll Horizon + Pitch Decoupling)
        self._last_applied_gimbal_quat = None
        self.cmd_event = threading.Event()

        # Trạng thái bay & Cất cánh thủ công
        self.target_z = None
        self.req_takeoff = False
        self.req_land = False
        self.is_taking_off = False
        self.is_landing = False
        self.is_airborne = False
        self.flight_armed = False
        self._vertical_input_active = False
        self.is_hover = False
        self.is_boost = False
        self.searchlight_on = True

        self.normal_speed = 8.0
        self.boost_speed = 18.0

        # XInput Controller
        self.xinput = XInputController()
        self._last_pad_b = False
        self._last_pad_x = False
        self._last_pad_y = False
        self._last_pad_lb = False
        self._last_pad_rb = False
        self._last_pad_back = False
        self._last_pad_start = False
        self._last_pad_l3 = False
        self._last_pad_r3 = False
        self._last_pad_dpad_right = False
        self._last_pad_dpad_left = False
        self._last_pad_dpad_down = False
        self._last_pad_dpad_up = False

        self.controller = None
        self.joystick = None
        self.controller_name = "Chưa kết nối"
        self._last_reported_ctrl = None

        # Camera & Sensor Streams
        self.rgb_img = None
        self.ir_img = None
        self.latest_ai_frame = None
        self.latest_camera_pose = None
        self.latest_ir_seg = None
        self.detected_rgb_img = None
        self.detected_ir_img = None
        self.chase_img = None
        self.chase_available = True
        self.rgb_view_target = 0      # 0 = CAM MŨI UAV (HƯỚNG UAV), 1 = Chase Cam

        # Bảng màu nhiệt FLIR
        self.thermal_palettes = [
            ("White-Hot", "white_hot"),
            ("Ironbow", "ironbow"),
            ("Rainbow", "rainbow"),
            ("Black-Hot", "black_hot"),
        ]
        self.palette_idx = 0
        self.person_thresh = DETECTOR.person_threshold
        self.detections = []
        self.detected_trees = []
        self.marked_persons = []
        self.marker_counter = 0
        self.start_pos_ned = None
        self.route_lines_enabled = False
        self.show_bbox = True            # Mặc định: BẬT hiển thị khung nhận diện AI YOLO
        self._thermal_calibrated = False
        self._thermal_setup_lock = threading.Lock()

        # AI YOLO26m person search; reveal classes only within 40 m.
        self.yolo_model = None
        self.yolo_enabled = True         # Mặc định: Tự động BẬT AI YOLO26m khi phát hiện mô hình
        self.yolo_device = "cpu"
        self.yolo_conf = DETECTOR.confidence            # Nguong phat hien YOLO (25%)
        self.yolo_imgsz = DETECTOR.runtime_imgsz            # Active checkpoint was trained at imgsz=960.
        self.yolo_fps = 0.0
        self.yolo_status_str = "ĐANG KHỞI TẠO"

        if HAS_YOLO:
            model_path = str(local_path(DETECTOR.model_path))
            if os.path.isfile(model_path):
                try:
                    dev = "cuda:0" if torch.cuda.is_available() else "cpu"
                    self.yolo_device = dev
                    dev_name = torch.cuda.get_device_name(0) if dev.startswith("cuda") else "CPU"
                    log.debug(f">> [AI YOLO] Đang nạp mô hình YOLO26m ({os.path.basename(model_path)}) trên {dev_name}...")
                    self.yolo_model = YOLO(model_path)
                    dummy = np.zeros((512, 640, 3), dtype=np.uint8)
                    self.yolo_model(dummy, device=dev, imgsz=self.yolo_imgsz,
                                    classes=list(SEARCH_CLASS_IDS), verbose=False)
                    self.yolo_enabled = True
                    self.show_bbox = True
                    dev_short = "GPU CUDA" if dev.startswith("cuda") else "CPU"
                    self.yolo_status_str = f"BẬT ({dev_short})"
                    log.debug(f">> [AI YOLO] Khởi tạo YOLO26m tìm người trên {dev_name} ({model_path})!")
                except Exception as e:
                    log.warning(f">> [AI YOLO CẢNH BÁO] Khởi tạo YOLO thất bại: {e}")
                    self.yolo_model = None
                    self.yolo_enabled = False
                    self.show_bbox = False
                    self.yolo_status_str = "LỖI NẠP"
            else:
                self.yolo_enabled = False
                self.show_bbox = False
                log.warning(f">> [AI YOLO] Không tìm thấy file trọng số: {model_path}")
                self.yolo_status_str = "THIẾU FILE"

        # LiDAR Ouster OS1-64 (Gọn nhẹ)
        self.lidar_name = "Lidar1"
        self.lidar_enabled = True       # Mặc định khi chạy: BẬT quét LiDAR đồng bộ toàn hệ thống
        self.thermal_enabled = True     # Mặc định khi chạy: BẬT camera nhiệt FLIR & AI YOLO26m phát hiện nạn nhân
        self.lidar_pts = np.empty((0, 3), dtype=np.float32)
        # Keep the full physical cloud for camera/LiDAR association. The
        # down-sampled cloud remains dedicated to smooth UI rendering.
        self.lidar_full_pts = np.empty((0, 3), dtype=np.float32)
        self.lidar_recent_scans = deque(maxlen=RUNTIME.recent_scan_count)
        self.lidar_ranges = [30.0, 60.0, 120.0]
        self.lidar_range_idx = 1
        self.lidar_colormaps = [
            ("Turbo", cv2.COLORMAP_TURBO),
            ("Viridis", cv2.COLORMAP_VIRIDIS),
            ("Plasma", cv2.COLORMAP_PLASMA),
        ]
        self.lidar_cmap_idx = 0
        self.radar_heading_up = True  # True: HEADING-UP (Mũi UAV hướng lên 12H), False: NORTH-UP (Bắc lên trên)

        # Mục tiêu và khóa mục tiêu
        self.target_locked = False
        self.locked_pos = None
        self.target_world = None
        self.target_time = 0.0
        self.active_fusion = None
        self.locked_fusion = None

        # Đường bay
        self.flight_trail = []
        self.last_trail_time = 0.0

        # Tần số thực tế
        self.cmd_hz = 33.0
        self.img_hz = 0.0
        self.rgb_hz = 0.0
        self.lidar_hz = 0.0
        self.ui_fps = 0.0

        # Thông báo
        self.last_ping_msg = "[SẴN SÀNG] UAV Ở MẶT ĐẤT. BẤM [L] HOẶC [SPACE] / CÒ RT ĐỂ CẤT CÁNH"
        self.last_ping_time = time.time()
        self.show_help = False

        # Thư mục lưu dữ liệu tìm kiếm (Nhiệt + LiDAR) và danh sách mục tiêu cứu hộ vào outputs/
        self.data_dir = os.path.join(SCRIPT_DIR, "outputs")
        os.makedirs(self.data_dir, exist_ok=True)
        self.markers_dir = os.path.join(self.data_dir, "rescue_mission_logs")
        self.csv_path = os.path.join(self.data_dir, "rescue_targets_log.csv")
        self.lidar_sync_file = os.path.join(self.data_dir, "lidar_state.json")
        self.lidar_stream_path = os.path.join(self.data_dir, "lidar_stream.bin")
        self.lidar_stream_writer = None
        try:
            self.lidar_stream_writer = LidarStreamWriter(self.lidar_stream_path, session_id=self.session_id)
        except Exception as exc:
            log.warning(f">> [LIDAR SHARE] Không thể tạo luồng dùng chung: {exc}")
        self._last_lidar_sync_ts = time.time()
        self._cached_sync_mtime = 0.0
        self._last_telemetry_sync_t = 0.0
        self._sync_lidar_state(from_self=True)

        self.recording = False
        self.record_session_dir = None
        self.record_frame_id = 0
        self.record_start_time = 0.0
        self.record_queue = queue.Queue(maxsize=300)
        self.last_record_sample_time = 0.0
        self.record_lock = threading.Lock()
        self._last_record_toggle_time = 0.0

        # Cache surfaces & font cho 60 FPS mượt mà
        self._fonts = {}
        self._text_cache = {}
        self._last_db = {}
        self.canvas = None
        self.panes = {}
        self.sidebar_buttons = []
        self.cached_colorbar_surf = None
        self.cached_colorbar_cmap = None
        self.lidar_colors = None
        self.lidar_max_elev = 10.0

    # -----------------------------------------------------------------------
    # HÌNH HỌC VÀ TỌA ĐỘ
    # -----------------------------------------------------------------------
    def body_to_world(self, body_pts, quat):
        R = quat_to_R(quat)
        return body_pts @ R.T

    def sensor_to_world(self, pts, quat, uav_pos):
        body_pts = pts @ self.lidar_mount_R.T + self.lidar_mount_pos
        return self.body_to_world(body_pts, quat) + uav_pos


    # -----------------------------------------------------------------------
    # GIMBAL / CAMERA / MỤC TIÊU
    # -----------------------------------------------------------------------


    def _sync_lidar_state(self, from_self=False):
        """Đồng bộ trạng thái Bật/Tắt LiDAR hai chiều giữa GCS và Bản đồ 3D"""
        sync_file = getattr(self, "lidar_sync_file", None)
        if not sync_file:
            data_dir = getattr(self, "data_dir", os.path.join(SCRIPT_DIR, "outputs"))
            os.makedirs(data_dir, exist_ok=True)
            sync_file = os.path.join(data_dir, "lidar_state.json")
            self.lidar_sync_file = sync_file
            self._last_lidar_sync_ts = 0.0

        if from_self:
            try:
                now_ts = time.time()
                self._last_lidar_sync_ts = now_ts
                tmp = sync_file + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump({"lidar_enabled": bool(self.lidar_enabled), "source": "gcs",
                               "timestamp": now_ts, "session_id": self.session_id}, f)
                os.replace(tmp, sync_file)
            except Exception:
                pass
        else:
            if not os.path.exists(sync_file):
                return
            try:
                mtime = os.path.getmtime(sync_file)
                if mtime != getattr(self, "_cached_sync_mtime", 0.0):
                    self._cached_sync_mtime = mtime
                    with open(sync_file, "r", encoding="utf-8") as f:
                        data = json.load(f)
                    ts = data.get("timestamp", 0.0)
                    src = data.get("source", "")
                    if (data.get("session_id") == self.session_id and
                            src != "gcs" and ts > getattr(self, "_last_lidar_sync_ts", 0.0)):
                        self._last_lidar_sync_ts = ts
                        req_enabled = bool(data.get("lidar_enabled", True))
                        if req_enabled != self.lidar_enabled:
                            self.toggle_lidar(force_val=req_enabled)
            except Exception:
                pass

    def toggle_lidar(self, force_val=None):
        if force_val is None:
            if not self._debounce("toggle_lidar", 0.25): return
            self.lidar_enabled = not self.lidar_enabled
        else:
            self.lidar_enabled = bool(force_val)

        if self.lidar_enabled:
            self.notify("[LIDAR] ĐÃ BẬT QUÉT OUSTER LIDAR OS1-64 [D-PAD ▶ / J]")
        else:
            with self.lock:
                self.lidar_pts = np.empty((0, 3), dtype=np.float32)
                self.lidar_full_pts = np.empty((0, 3), dtype=np.float32)
                self.lidar_recent_scans.clear()
                self.lidar_colors = None
                self.telemetry["points_count"] = 0
            self.notify("[LIDAR] ĐÃ TẮT QUÉT LIDAR (TIẾT KIỆM TÀI NGUYÊN) [D-PAD ▶ / J]")

        if force_val is None:
            self._sync_lidar_state(from_self=True)
        self.export_live_targets_json()

    def toggle_thermal(self):
        if not self._debounce("toggle_thermal", 0.25): return
        self.thermal_enabled = not self.thermal_enabled
        if self.thermal_enabled:
            self.notify("[CAMERA NHIỆT] ĐÃ BẬT CAMERA HỒNG NGOẠI FLIR [D-PAD ◀ / U]")
        else:
            with self.lock:
                self.ir_img = None
                self.detections = []
            self.notify("[CAMERA NHIỆT] ĐÃ TẮT CAMERA NHIỆT (TIẾT KIỆM TÀI NGUYÊN) [D-PAD ◀ / U]")

    def toggle_yolo_mode(self):
        if not self._debounce("toggle_yolo", 0.25): return
        if self.yolo_model is None:
            self.notify(f"[AI YOLO] KHÔNG CÓ MÔ HÌNH {os.path.basename(DETECTOR.model_path)} ĐỂ BẬT!")
            return
        self.yolo_enabled = not self.yolo_enabled
        if self.yolo_enabled:
            self.show_bbox = True
            dev_str = "GPU CUDA" if self.yolo_device.startswith("cuda") else "CPU"
            self.yolo_status_str = f"BẬT ({dev_str})"
            self.notify(f"[AI YOLO] ĐÃ BẬT MÔ HÌNH YOLO26M ({dev_str}) [PHÍM M]")
        else:
            self.show_bbox = False
            self.yolo_status_str = "TẮT (NGƯỠNG)"
            self.notify("[AI YOLO] ĐÃ TẮT - CHUYỂN SANG BỘ LỌC NHIỆT TRUYỀN THỐNG [PHÍM M]")

    def toggle_bbox(self):
        if not self._debounce("toggle_bbox", 0.25): return
        self.show_bbox = not getattr(self, "show_bbox", True)
        status_txt = "ĐÃ BẬT KHUNG NHẬN DIỆN" if self.show_bbox else "ĐÃ ẨN TOÀN BỘ KHUNG (CHỈ XEM ẢNH THUẦN)"
        self.notify(f"[KHUNG NHẬN DIỆN] {status_txt} [PHÍM O]")

    def toggle_rgb_view(self):
        if not self._debounce("toggle_rgb", 0.25): return
        self.rgb_view_target = 1 - self.rgb_view_target
        name = "CHASE CAM BÁM ĐUÔI" if self.rgb_view_target == 1 else "CAM MŨI UAV (HƯỚNG UAV)"
        self.notify(f"[CAM RGB] CHUYỂN GÓC NHÌN: {name}")

    def toggle_radar_heading(self):
        if not self._debounce("toggle_heading", 0.25): return
        self.radar_heading_up = not self.radar_heading_up
        mode_str = "HEADING-UP (Mũi UAV hướng lên 12H)" if self.radar_heading_up else "NORTH-UP (Bắc cố định lên trên)"
        self.notify(f"[RADAR BEV] CHUYỂN GÓC: {mode_str} [N]")

    def toggle_target_lock(self):
        if not self._debounce("toggle_lock", 0.25): return
        if self.target_locked:
            self.target_locked = False
            self.locked_pos = None
            self.locked_fusion = None
            self.notify("[MỤC TIÊU] ĐÃ MỞ KHÓA - TỰ ĐỘNG BÁM TÍN HIỆU NHIỆT")
            return
        pos = None
        fusion = None
        if self.target_world is not None and (time.time() - self.target_time) < 3.0:
            pos = self.target_world.copy()
            fusion = dict(self.active_fusion) if self.active_fusion else None
        elif self.marked_persons:
            m = self.marked_persons[-1]
            pos = np.array([m["tgt_x"], m["tgt_y"], m.get("tgt_z", self.ground_z_datum)], dtype=np.float32)
            fusion = {"class_id": 0, "is_yolo": True, "fusion_verdict": "CONFIRMED_HUMAN",
                      "lidar_verified": bool(m.get("lidar_verified", False))}
        if pos is None:
            self.notify("[MỤC TIÊU] CHƯA CÓ TÍN HIỆU NHIỆT ĐỂ KHÓA")
            return
        self.locked_pos = pos
        self.locked_fusion = fusion
        self.target_locked = True
        self.notify(f"[MỤC TIÊU] ĐÃ KHÓA TẠI (X:{pos[0]:.1f}, Y:{pos[1]:.1f}, Z:{pos[2]:.1f})")

    def current_target(self):
        if self.target_locked and self.locked_pos is not None:
            return self.locked_pos
        if self.target_world is not None and (time.time() - self.target_time) < 2.0:
            return self.target_world
        return None

    def notify(self, msg):
        self.last_ping_msg = msg
        self.last_ping_time = time.time()
        if "LỖI" in msg or "LOI" in msg:
            log.warning(">> %s", msg)
        else:
            log.debug(">> %s", msg)


    def setup_thermal_and_targets(self, client):
        with self._thermal_setup_lock:
            if self._thermal_calibrated:
                return
            from simulation import scene_setup
            self._thermal_calibrated = scene_setup.setup_scene(client)


    def project_pixel_to_ground(self, u, v, w_im, h_im):
        with self.lock:
            quat = self.telemetry["quat"]
            pos = np.array([self.telemetry["x"], self.telemetry["y"], self.telemetry["z"]], dtype=np.float32)
            yaw_deg = float(self.telemetry.get("yaw", 0.0))
            t_agl = self.terrain_agl
            lidar_sparse = self.lidar_pts
        cam_pitch = self.gimbal_applied if self.gimbal_applied is not None else self.gimbal_pitch
        R_body = quat_to_R(quat)

        # 1. Hướng tia ngắm chuẩn thế giới: Tách rời góc chúi/lắc thân UAV khi Gimbal đang chống rung
        if getattr(self, "gimbal_stabilization_enabled", True):
            R_cam_world = euler_to_R(cam_pitch, 0.0, yaw_deg)
        else:
            R_cam_body = euler_to_R(cam_pitch, 0.0, 0.0)
            R_cam_world = R_body @ R_cam_body

        f = (w_im / 2.0) / math.tan(math.radians(THERMAL_FOV_DEG / 2.0))
        d_cam = np.array([1.0, (u - w_im / 2.0) / f, (v - h_im / 2.0) / f], dtype=np.float32)
        d_w = R_cam_world @ d_cam
        d_w = d_w / (np.linalg.norm(d_w) + 1e-9)
        if d_w[2] <= 0.02:
            return None
        origin = pos + R_body @ self.cam_mount_np

        # 2. Ước tính cao độ mặt đất sơ bộ từ LiDAR AGL hoặc datum
        ground_z_plane = (pos[2] + t_agl) if (t_agl is not None and t_agl > 0.5) else self.ground_z_datum
        t = (ground_z_plane - origin[2]) / d_w[2]
        if t <= 0 or t > 450:
            return None
        p = (origin + t * d_w).astype(np.float32)

        # 3. Tinh chỉnh cao độ giao điểm thực tế theo đám mây điểm LiDAR cục bộ
        if lidar_sparse is not None and len(lidar_sparse) >= 20:
            d_sq = (lidar_sparse[:, 0] - p[0]) ** 2 + (lidar_sparse[:, 1] - p[1]) ** 2
            near_m = d_sq <= 12.25  # bán kính 3.5m
            if np.count_nonzero(near_m) >= 4:
                # Trong hệ NED, Z dương lớn nhất là mặt đất thấp nhất
                z_terrain = float(np.percentile(lidar_sparse[near_m, 2], 85))
                t_refined = (z_terrain - origin[2]) / d_w[2]
                if 0 < t_refined < 450:
                    p = (origin + t_refined * d_w).astype(np.float32)

        # 4. Snap chính xác vào vị trí nạn nhân 3D trong map nếu cự ly tia ngắm < 6.5m
        return p

    def locate_target_world(self, det, w_im, h_im, thermal_gray=None, camera_pose=None):
        x0, y0, x1, y1 = det["bbox"]
        u = float(det.get("cx", (x0 + x1) / 2.0))
        now = time.time()
        with self.lock:
            quat = self.telemetry["quat"]
            pos = np.array([
                self.telemetry["x"], self.telemetry["y"], self.telemetry["z"]
            ], dtype=np.float32)
            yaw_deg = float(self.telemetry.get("yaw", 0.0))
            terrain_agl = self.terrain_agl
            lidar_cloud = self.lidar_full_pts
            recent_scans = tuple(self.lidar_recent_scans)

        # Ghép vài lượt quét ngắn trong hệ tọa độ NED để người ở xa có đủ
        # điểm hình học, kể cả khi một lượt quét chỉ chạm 2–4 điểm cơ thể.
        lidar_cloud = recent_lidar_points(recent_scans, now, lidar_cloud)

        cam_pitch = self.gimbal_applied if self.gimbal_applied is not None else self.gimbal_pitch
        result = sfc.locate_target_sensor_fusion(
            u_center=u,
            v_bottom=float(y1),
            bbox_2d=(x0, y0, x1, y1),
            w_im=w_im,
            h_im=h_im,
            fov_deg=THERMAL_FOV_DEG,
            uav_pos_ned=pos,
            uav_quat=quat,
            cam_pitch_deg=cam_pitch,
            yaw_deg=yaw_deg,
            lidar_points_ned=lidar_cloud,
            ground_z_datum=self.ground_z_datum,
            terrain_agl=terrain_agl,
            cam_mount_frd=self.cam_mount_np,
            gimbal_stabilized=getattr(self, "gimbal_stabilization_enabled", True),
            thermal_confidence=float(det.get("conf", 0.50)),
            thermal_temp_c=float(det.get("temp", 37.0)),
            thermal_image=thermal_gray,
            camera_position_ned=camera_pose[0] if camera_pose else None,
            camera_quat=camera_pose[1] if camera_pose else None,
            target_class_id=det.get("class_id"),
        )
        if result is None:
            det.update({
                "verdict": "NO_GEOMETRIC_SOLUTION",
                "fusion_verdict": "NO_GEOMETRIC_SOLUTION",
                "is_human": False,
                "s_final": 0.0,
                "s_geo": 0.0,
                "lidar_verified": False,
                "sar_reason": "Khong tim thay giao diem camera-LiDAR hop le",
                "_fusion_pos_ned": None,
            })
            det.update(target_presentation(det, pos))
            return None

        det.update({
            "verdict": result["verdict"],
            "fusion_verdict": result["verdict"],
            "is_human": bool(result["is_human"]),
            "s_final": float(result["s_final"]),
            "s_geo": float(result.get("s_geo", 0.0)),
            "planarity": float(result.get("planarity", 0.0)),
            "lidar_points": int(result.get("points_count", 0)),
            "lidar_bbox_2d": result.get("lidar_bbox_2d"),
            "mean_lidar_temp_c": float(result.get("mean_temp_c", det.get("temp", 37.0))),
            "lidar_verified": bool(result.get("lidar_verified", False)),
            "sar_reason": result.get("reason", ""),
            "dimensions_m": result.get("dimensions_m"),
            "_fusion_pos_ned": np.asarray(result["pos_ned"], dtype=np.float32),
            "_fusion_time": time.time(),
        })
        det.update(target_presentation(det, pos))
        return det["_fusion_pos_ned"]

    def detect_foliage_trees(self, th_gray):
        # Su dung segmentation map ID 24-26 (foliage/tree) neu co de nhan dien chinh xac 100%
        with self.lock:
            seg = getattr(self, "latest_ir_seg", None)
        if seg is not None and seg.shape[:2] == th_gray.shape[:2]:
            mask = ((seg >= 22) & (seg <= 28)).astype(np.uint8)
        else:
            # Fallback: cay co nhiet do tan la am trong dai 95-185
            mask = ((th_gray >= 95) & (th_gray <= 185)).astype(np.uint8)
        if float(mask.mean()) < 0.002:
            return []
        h_im, w_im = th_gray.shape[:2]
        f = (w_im / 2.0) / math.tan(math.radians(THERMAL_FOV_DEG / 2.0))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        n, _, stats, cents = cv2.connectedComponentsWithStats(mask_clean, connectivity=8)
        trees = []
        with self.lock:
            uav_x = self.telemetry["x"]
            uav_y = self.telemetry["y"]
        for i in range(1, n):
            x, y, bw, bh, area = [int(v) for v in stats[i]]
            if area < 120 or area > 100000:
                continue
            u_base = float(cents[i][0])
            v_base = float(min(h_im - 1, y + bh))
            p_base = self.project_pixel_to_ground(u_base, v_base, w_im, h_im)
            if p_base is None:
                continue
            dist = float(np.hypot(p_base[0] - uav_x, p_base[1] - uav_y))
            if dist < 3.0 or dist > 95.0:
                continue
            tree_h = min(8.0, max(2.5, dist * (bh / f)))
            tree_r = min(5.0, max(1.5, dist * (bw / f) * 0.5))
            trees.append({
                "x": float(p_base[0]),
                "y": float(p_base[1]),
                "z": float(p_base[2]),
                "dist": dist,
                "h": tree_h,
                "r": tree_r,
                "area": area,
            })
        trees.sort(key=lambda t: t["dist"])
        return trees[:12]


    # -----------------------------------------------------------------------
    # GHI DỮ LIỆU TÌM KIẾM (NHIỆT + LIDAR) VÀO THƯ MỤC Data/
    # HỖ TRỢ: BẬT KHI ẤN LẦN 1, TẮT & LƯU HOÀN TẤT KHI ẤN LẦN 2
    # -----------------------------------------------------------------------


    # -----------------------------------------------------------------------
    # 4. KẾT NỐI RPC VÀ TỰ ĐỘNG PHỤC HỒI (RESILIENT AUTO-RECONNECT)
    # -----------------------------------------------------------------------
    def _connect_client(self, timeout=15, max_retries=None):
        """Tạo kết nối MultirotorClient có timeout bảo vệ, tự động thử lại khi mất mạng."""
        attempts = 0
        while self.running:
            try:
                client = connect_airsim(timeout=timeout)
                return client
            except Exception:
                attempts += 1
                if max_retries is not None and attempts >= max_retries:
                    return None
                time.sleep(0.3)
        return None


    def auto_arrange_windows(self):
        auto_arrange_windows(restore=True)
        self.notify("[WINDOW] ĐÃ BỐ TRÍ CỬA SỔ UNREAL, GCS & MAP 3D!")

    def open_or_focus_3d_map(self):
        """Mở hoặc đưa Cửa sổ Dựng Địa Hình 3D (Open3D LiDAR + PoinTr) lên phía trước [F2]"""
        if not self._debounce("open_3d_map", 0.35):
            return
        if not self.lidar_enabled:
            self.toggle_lidar(force_val=True)
        res = open_or_focus_3d_map()
        if res:
            self.notify("[BẢN ĐỒ 3D] ĐÃ MỞ / CHUYỂN TỚI CỬA SỔ DỰNG ĐỊA HÌNH 3D [F2]")
        else:
            self.notify("[BẢN ĐỒ 3D] ĐANG KHỞI CHẠY TIẾN TRÌNH DỰNG ĐỊA HÌNH 3D...")

    def cycle_lidar_range(self):
        """Phóng to / Thu nhỏ dải đo khoảng cách Radar LiDAR"""
        if not self._debounce("cycle_lidar_range", 0.20):
            return
        self.lidar_range_idx = (self.lidar_range_idx + 1) % len(self.lidar_ranges)
        self.notify(f"[RADAR LIDAR] THANG ĐO CỰ LY: {self.lidar_ranges[self.lidar_range_idx]:.0f}m [X]")

    def window_management_loop(self):
        # The simulator is already ready before GCS starts. Hide its extra
        # viewport once; never restore/maximize/minimize the user's GCS window.
        if self.running:
            auto_arrange_windows()


    # -----------------------------------------------------------------------
    # LUỒNG 2: ĐIỀU KHIỂN & CHỐNG RUNG GIMBAL 2 TRỤC CHỦ ĐỘNG (30Hz)
    # -----------------------------------------------------------------------


    # -----------------------------------------------------------------------
    # LUỒNG 3: LIDAR OUSTER OS1-64 PIPELINE (10Hz GỌN NHẸ)
    # -----------------------------------------------------------------------


    _CANVAS_CACHE = {}


    # -----------------------------------------------------------------------
    # 7. VÒNG LẶP CHÍNH (60 FPS MƯỢT MÀ)
    # -----------------------------------------------------------------------
    def run(self):
        pygame.init()
        pygame.font.init()
        try:
            pygame.transform.set_smoothscale_backend("SSE2")
        except Exception:
            pass
        self.init_controller()
        self.set_display(False)
        pygame.display.set_caption("HE THONG UAV GCS TIM KIEM CUU HO [UE 4.27 MEDIEVAL VILLAGE]")
        self.build_surfaces()

        clock = pygame.time.Clock()

        # 3 luồng chuyên biệt, không chồng chéo, tự phục hồi khi mất kết nối
        threads = [
            threading.Thread(target=self.flight_control_telemetry_loop, daemon=True, name="flight_telemetry"),
            threading.Thread(target=self.gimbal_loop, daemon=True, name="gimbal_smooth"),
            threading.Thread(target=self.image_loop, daemon=True, name="images"),
            threading.Thread(target=self.ai_worker_loop, daemon=True, name="ai_worker"),
            threading.Thread(target=self.ground_truth_retry_loop, daemon=True, name="ground_truth_retry"),
            threading.Thread(target=self.lidar_loop, daemon=True, name="lidar"),
            threading.Thread(target=self.data_recorder_worker, daemon=True, name="recorder_worker"),
            threading.Thread(target=self.window_management_loop, daemon=True, name="window_mgr"),
        ]
        for t in threads:
            t.start()

        log.info("[GCS] Sẵn sàng. Giữ Space / RT để bay lên thủ công.")

        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.VIDEORESIZE and not self.fullscreen:
                    self.win_w, self.win_h = max(640, event.w), max(360, event.h)
                elif event.type in (pygame.JOYDEVICEADDED, getattr(pygame, "CONTROLLERDEVICEADDED", 0x650)):
                    self.init_controller()
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    sw, sh = pygame.display.get_surface().get_size()
                    s = min(sw / float(CANVAS_W), sh / float(CANVAS_H))
                    dw, dh = max(1, int(CANVAS_W * s)), max(1, int(CANVAS_H * s))
                    ox, oy = (sw - dw) // 2, (sh - dh) // 2
                    mx, my = event.pos
                    if ox <= mx < ox + dw and oy <= my < oy + dh:
                        canvas_x = int((mx - ox) / s)
                        canvas_y = int((my - oy) / s)
                        self.handle_mouse_click(canvas_x, canvas_y)
                elif event.type in (getattr(pygame, "CONTROLLERBUTTONDOWN", 0x651), pygame.JOYBUTTONDOWN):
                    btn = getattr(event, "button", -1)
                    if btn in (11, getattr(sdl2_controller, "CONTROLLER_BUTTON_DPAD_UP", 11)):
                        self.toggle_rescue_routes()
                    elif btn in (12, getattr(sdl2_controller, "CONTROLLER_BUTTON_DPAD_DOWN", 12)):
                        self.toggle_recording()
                    elif btn in (13, getattr(sdl2_controller, "CONTROLLER_BUTTON_DPAD_LEFT", 13)):
                        self.toggle_thermal()
                    elif btn in (14, getattr(sdl2_controller, "CONTROLLER_BUTTON_DPAD_RIGHT", 14)):
                        self.toggle_lidar()
                elif event.type == pygame.JOYHATMOTION:
                    hat_val = getattr(event, "value", (0, 0))
                    if hat_val[1] == 1: self.toggle_rescue_routes()
                    elif hat_val[1] == -1: self.toggle_recording()
                    elif hat_val[0] == -1: self.toggle_thermal()
                    elif hat_val[0] == 1: self.toggle_lidar()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        if self.fullscreen: self.set_display(False)
                        else: self.running = False
                    # Other shortcuts are polled once in poll_flight_inputs,
                    # including F8, so focusing this window cannot fire twice.
                    elif event.key == pygame.K_F11: self.set_display(not self.fullscreen)
                    elif event.key == pygame.K_F1: self.show_help = not self.show_help

            self.ui_fps = clock.get_fps()
            try:
                with self.lock:
                    if self.thermal_enabled and self.detections and self.detected_rgb_img is not None:
                        self._display_rgb = self.detected_rgb_img
                        self._display_ir = self.detected_ir_img
                    else:
                        self._display_rgb = self.rgb_img
                        self._display_ir = self.ir_img
                    self._display_chase = self.chase_img
                self.render_thermal_pane()
                self.render_rgb_pane()
                self.render_lidar_radar_pane()
                self.render_roi_pane()
                self.render_sidebar()
                self.draw_dashboard(clock)
                self.present()
            except Exception:
                pass
            clock.tick(60)

        self.running = False
        if self.recording:
            self.toggle_recording()
        try:
            self.record_queue.put_nowait(None)
        except Exception:
            pass
        for worker in threads:
            worker.join(timeout=0.75)
        if self.lidar_stream_writer is not None:
            try:
                self.lidar_stream_writer.close()
            except Exception:
                pass
        pygame.quit()
        log.info(">> [GCS STATION] Đã dừng phiên làm việc.")

def main():
    system = UAVUnifiedSystem()
    system.run()

if __name__ == "__main__":
    main()
