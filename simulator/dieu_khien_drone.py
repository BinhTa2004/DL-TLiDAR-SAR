# -*- coding: utf-8 -*-
r"""
=============================================================================
MÔI TRƯỜNG SIMULATOR UAV AIRSIM - BAY LIÊN TỤC 100% & CẢM BIẾN NHIỆT / LIDAR
=============================================================================
1. BẢO VỆ BAY LIÊN TỤC (CHỐNG RESET):
   - Sử dụng driver `keyboard.block_key('backspace')` để chặn 100% phím Backspace
     do bộ gõ Tiếng Việt (UniKey / EVKey Telex) vô tình kích hoạt.
   - Luồng Heartbeat 20Hz (50ms) gửi vận tốc liên tục, tự động re-enable API Control.
2. HỆ THỐNG CẢM BIẾN ĐỒNG TRỤC (SENSOR DATA STREAM):
   - Camera Đa Kênh (640x512, FOV 70°):
     + DepthVis (Chiều sâu chi tiết 3D)
     + Infrared (Hồng ngoại nhiệt Planck)
     + Scene (Camera màu RGB)
   - Cảm biến LiDAR Ouster OS1-64 (64 kênh, 100m, 131,072 pts/s, pitch -15°):
     + Live BEV Radar hiển thị cự ly 25m, 50m.
   - Xuất dữ liệu qua RPC API AirSim (Port 41451) phục vụ các pipeline AI bên ngoài.
=============================================================================
"""

import sys
try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

import os
import time
import math
import threading
import ctypes
from ctypes import wintypes
import numpy as np
import cv2

# Thư viện AirSim
AIRSIM_PATH = r"D:\simulator\AirSim\PythonClient"
if os.path.exists(AIRSIM_PATH) and AIRSIM_PATH not in sys.path:
    sys.path.insert(0, AIRSIM_PATH)

import airsim
import pygame

# Cài đặt và sử dụng thư viện keyboard để chặn BackSpace triệt để
try:
    import keyboard
    HAS_KEYBOARD_LIB = True
except ImportError:
    HAS_KEYBOARD_LIB = False

CMD_PERIOD = 0.05    # Chu kỳ Heartbeat gửi lệnh: 50ms (20Hz)
CMD_DURATION = 0.25  # Thời hạn mỗi lệnh vận tốc: 250ms (chống SimpleFlight timeout)
NORMAL_SPEED = 6.0   # Tốc độ bay thường (m/s)
BOOST_SPEED = 14.0   # Tốc độ bay nhanh (m/s)

user32 = ctypes.windll.user32
kernel32 = ctypes.windll.kernel32

# Virtual-Key Windows để bắt phím toàn cục
VK_KEYS = {
    'W': 0x57, 'S': 0x53, 'A': 0x41, 'D': 0x44,
    'UP': 0x26, 'DOWN': 0x28, 'LEFT': 0x25, 'RIGHT': 0x27,
    'SPACE': 0x20, 'SHIFT': 0x10, 'Q': 0x51, 'E': 0x45,
    'TAB': 0x09, 'CTRL': 0x11, 'T': 0x54, 'H': 0x48,
    'V': 0x56, 'C': 0x43, 'L': 0x4C, 'M': 0x4D, 'P': 0x50,
    'LBRACKET': 0xDB, 'RBRACKET': 0xDD,
    'PGUP': 0x21, 'PGDN': 0x22,
    'ESC': 0x1B, 'BACK': 0x08
}

def is_key_down(vk_code):
    try:
        return (user32.GetAsyncKeyState(vk_code) & 0x8000) != 0
    except Exception:
        return False

# ---------------------------------------------------------------------------
# TOÁN HỌC CHUYỂN ĐỔI KHÔNG GIAN 3D (SENSOR -> BODY -> WORLD NED)
# ---------------------------------------------------------------------------
def quat_to_R(q):
    """Quaternion (w, x, y, z) -> Ma trận quay 3x3."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)

def euler_to_R(pitch_deg, roll_deg=0.0, yaw_deg=0.0):
    """Góc Euler (Pitch, Roll, Yaw) -> Ma trận quay 3x3."""
    q = airsim.to_quaternion(math.radians(pitch_deg), math.radians(roll_deg), math.radians(yaw_deg))
    return quat_to_R((q.w_val, q.x_val, q.y_val, q.z_val))

# Ma trận giá gắn đồng trục cảm biến trên mũi UAV (theo settings.json)
LIDAR_MOUNT_R = euler_to_R(pitch_deg=-15.0)
LIDAR_MOUNT_POS = np.array([0.35, 0.0, 0.15], dtype=np.float32)

CAM_MOUNT_R = euler_to_R(pitch_deg=-10.0)
CAM_MOUNT_POS = np.array([0.35, 0.0, 0.15], dtype=np.float32)

# ---------------------------------------------------------------------------
# BỘ CHẶN PHÍM BACKSPACE TOÀN CỤC (CHỐNG UNIKEY RESET GAME)
# ---------------------------------------------------------------------------
_hook_active = False

def enable_anti_reset():
    global _hook_active
    if HAS_KEYBOARD_LIB:
        try:
            keyboard.block_key('backspace')
            _hook_active = True
            print(">> [BẢO VỆ GAME] Đã kích hoạt bộ chặn phím Reset tự động (Chống lỗi UniKey Telex)!", flush=True)
            return True
        except Exception as e:
            print(f">> [WARN] Không thể kích hoạt keyboard hook: {e}", flush=True)
    return False

def disable_anti_reset():
    global _hook_active
    if HAS_KEYBOARD_LIB and _hook_active:
        try:
            keyboard.unblock_key('backspace')
            _hook_active = False
            print(">> [BẢO VỆ GAME] Đã gỡ bỏ bộ chặn phím Reset.", flush=True)
        except Exception:
            pass

# ---------------------------------------------------------------------------
# HIỆU CHUẨN DẢI NHIỆT PLANCK (CHỐNG LÓA TRẮNG)
# ---------------------------------------------------------------------------
def calibrate_thermal_camera(client):
    r"""
    Hiệu chuẩn dải nhiệt độ Planck cho Camera Hồng Ngoại (ImageType 7).
    Trích xuất từ D:\DL-TLiDAR-SAR\AirSim_Africa_UE427\setup_africa_targets.py:
    - Đưa nền địa hình và cây cối về mức lạnh/tối (35-65), nước về 20.
    - Con người / mục tiêu phát sáng trắng rực White-Hot (255) trên nền tối.
    """
    try:
        client.simSetSegmentationObjectID(".*", 0, True)
        client.simSetSegmentationObjectID(".*Base_Terrain.*", 65, True)
        client.simSetSegmentationObjectID(".*landscape.*", 65, True)
        client.simSetSegmentationObjectID(".*soil.*", 65, True)
        client.simSetSegmentationObjectID(".*mountain.*", 65, True)
        client.simSetSegmentationObjectID(".*rock.*", 50, True)
        client.simSetSegmentationObjectID(".*InstancedFoliageActor.*", 35, True)
        client.simSetSegmentationObjectID(".*tree.*", 35, True)
        client.simSetSegmentationObjectID(".*bush.*", 35, True)
        client.simSetSegmentationObjectID(".*acacia.*", 35, True)
        client.simSetSegmentationObjectID(".*Water.*", 20, True)
        client.simSetSegmentationObjectID(".*elephant.*", 215, True)
        client.simSetSegmentationObjectID(".*zebra.*", 225, True)
        client.simSetSegmentationObjectID(".*Crocodile.*", 205, True)
        client.simSetSegmentationObjectID(".*Rhinoceros.*", 220, True)
        client.simSetSegmentationObjectID(".*Hippo.*", 210, True)
        client.simSetSegmentationObjectID(".*poacher.*", 255, True)
        client.simSetSegmentationObjectID(".*human.*", 255, True)
        client.simSetSegmentationObjectID(".*person.*", 255, True)
        client.simSetSegmentationObjectID(".*victim.*", 255, True)
        client.simSetSegmentationObjectID(".*mannequin.*", 255, True)
        print(">> [FLIR THERMAL] Đã nạp bảng nhiệt Planck chuẩn từ Africa!", flush=True)
        return True
    except Exception as e:
        print(f">> [FLIR THERMAL WARN] {e}", flush=True)
        return False

# ---------------------------------------------------------------------------
# BẢNG CHẾ ĐỘ CAMERA VÀ CẢM BIẾN (TỐI ƯU CHO BÀI TOÁN CỨU HỘ SAR)
# ---------------------------------------------------------------------------
CAMERA_MODES = [
    ("FLIR Thermal (Ảnh Nhiệt Cứu Hộ)", airsim.ImageType.Infrared),
    ("DepthVis (Chiều Sâu Địa Hình 3D)", airsim.ImageType.DepthVis),
    ("Scene (Camera RGB Quang Học)", airsim.ImageType.Scene)
]

THERMAL_PALETTES = [
    ("White-Hot (FLIR Trắng Nóng)", "white_hot"),
    ("Ironbow (FLIR Cứu Hộ)", "ironbow"),
    ("Black-Hot (FLIR Đen Nóng)", "black_hot"),
    ("Rainbow (Phổ Nhiệt Jet)", "rainbow")
]

LIDAR_MODES = [
    ("BEV Cao Độ (Địa hình Turbo)", "bev_elevation"),
    ("Phối Cảnh 3D (Isometric)", "iso_3d"),
    ("Cảnh Báo Cự Ly (Proximity)", "proximity")
]

# Các mức điều chỉnh mật độ điểm LiDAR (Chống giật lag & Cảnh báo độ thưa phù hợp bài toán)
LIDAR_DENSITY_LEVELS = [
    {
        "ratio": 1.0,
        "name": "100% (Toàn Phần)",
        "tag": "100% GỐC",
        "color": (255, 200, 80),
        "status": "Mật độ LiDAR 100%: Toàn bộ mây điểm thô OS1-64 (Chi tiết cao nhất, tải tính toán nặng).",
        "warning": None
    },
    {
        "ratio": 0.5,
        "name": "50% [CHUẨN CỨU HỘ]",
        "tag": "50% TỐI ƯU",
        "color": (0, 255, 160),
        "status": "Mật độ LiDAR 50% [KHUYÊN DÙNG]: Chuẩn cứu hộ SAR (Mượt 60 FPS + Đủ rõ nét địa hình & nạn nhân).",
        "warning": None
    },
    {
        "ratio": 0.25,
        "name": "25% (Tiết Kiệm)",
        "tag": "25% NHẸ",
        "color": (100, 200, 255),
        "status": "Mật độ LiDAR 25%: Chế độ nhẹ máy, tiết kiệm tài nguyên khi chạy đa tác vụ AI.",
        "warning": None
    },
    {
        "ratio": 0.1,
        "name": "10% [CẢNH BÁO: QUÁ THƯA]",
        "tag": "10% THƯA",
        "color": (255, 80, 80),
        "status": "CẢNH BÁO: Mật độ LiDAR 10% quá thưa! Khoảng cách điểm lớn, rất khó nhận diện người và vật cản.",
        "warning": "CẢNH BÁO: ĐIỂM QUÁ THƯA (<10%) - KHÓ NHẬN DIỆN NẠN NHÂN!"
    }
]

def colorize_thermal(th_gray, pal_mode):
    """Áp dụng bảng màu nhiệt chuẩn FLIR / SAR."""
    if len(th_gray.shape) == 3 and th_gray.shape[2] == 1:
        th_gray = th_gray[:, :, 0]
    elif len(th_gray.shape) == 3 and th_gray.shape[2] == 3:
        th_gray = cv2.cvtColor(th_gray, cv2.COLOR_RGB2GRAY)

    if pal_mode == "white_hot":
        return cv2.cvtColor(th_gray, cv2.COLOR_GRAY2RGB)
    if pal_mode == "black_hot":
        return cv2.cvtColor(255 - th_gray, cv2.COLOR_GRAY2RGB)
    if pal_mode == "ironbow":
        return cv2.cvtColor(cv2.applyColorMap(th_gray, cv2.COLORMAP_INFERNO), cv2.COLOR_BGR2RGB)
    if pal_mode == "rainbow":
        return cv2.cvtColor(cv2.applyColorMap(th_gray, cv2.COLORMAP_JET), cv2.COLOR_BGR2RGB)
    return cv2.cvtColor(th_gray, cv2.COLOR_GRAY2RGB)

# ---------------------------------------------------------------------------
# CLASS ĐIỀU KHIỂN CHÍNH
# ---------------------------------------------------------------------------
class DroneController:
    def __init__(self):
        self.running = True
        self.lock = threading.Lock()
        self.vehicle_name = "UAV1"
        self.connected = False

        # Vận tốc điều khiển hiện tại (Body Frame)
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.cmd_yaw_rate = 0.0

        self.is_boost = False
        self.is_hover = False
        self.cmd_hz = 20.0

        # Thông số đo đạc
        self.ground_z = None
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.alt_agl = 0.0
        self.speed = 0.0
        self.yaw = 0.0

        # Camera & Màu nhiệt chuyên dụng cứu hộ
        self.thermal_surface = None
        self.cam_mode_idx = 0      # Mặc định FLIR Thermal cứu hộ
        self.palette_idx = 0       # Mặc định White-Hot (FLIR Trắng Nóng chuẩn cứu hộ)
        self.thermal_fps = 0.0

        # LiDAR Ouster OS1-64 & Mây điểm 3D (Đa chế độ & Điều chỉnh mật độ chống lag)
        self.lidar_surface = None
        self.lidar_mode_idx = 0       # Mặc định BEV Cao Độ (Turbo Elevation Map)
        self.lidar_density_idx = 1    # Mặc định 50% [CHUẨN CỨU HỘ] (Mượt 60 FPS & Giữ nguyên cấu trúc)
        self.lidar_pts_count = 0
        self.lidar_fps = 0.0
        self.uav_quat = (1.0, 0.0, 0.0, 0.0)
        self.latest_world_pts = np.empty((0, 3), dtype=np.float32)

        # Góc ngẩng / chúi của Gimbal Camera & Góc nhìn 3D (PageUp / PageDown)
        self.gimbal_pitch = -10.0  # Mặc định chúi nhẹ -10° theo settings.json
        self.hud_status_msg = "Môi trường AirSim sẵn sàng | Cảm biến FLIR Thermal & LiDAR OS1-64"

        self.client_main = None

    def update_gimbal_pitch(self, delta_deg):
        """Điều chỉnh góc ngẩng / chúi của Gimbal Camera (Page Up: ngửa, Page Down: chúi đất)."""
        with self.lock:
            self.gimbal_pitch = float(np.clip(self.gimbal_pitch + delta_deg, -90.0, 20.0))
            new_pitch = self.gimbal_pitch
        try:
            if self.client_main:
                q = airsim.to_quaternion(math.radians(new_pitch), 0.0, 0.0)
                pos = airsim.Vector3r(0.35, 0.0, 0.15)
                self.client_main.simSetCameraPose("front_camera", airsim.Pose(pos, q), vehicle_name=self.vehicle_name)
        except Exception:
            pass
        status_lbl = "Nhìn ngang" if abs(new_pitch) < 3 else ("Chúi thẳng xuống đất (Top-down)" if new_pitch <= -85 else f"Chúi xiên {new_pitch:.0f}°")
        print(f">> [GÓC NHÌN GIMBAL] Pitch = {new_pitch:+.1f}° ({status_lbl})", flush=True)

    def connect_and_init(self):
        """Kết nối đồng bộ tới AirSim, xác thực quyền điều khiển một lần duy nhất."""
        print("Đang tìm kiếm kết nối tới AirSim...", end="", flush=True)
        while self.running:
            try:
                c = airsim.MultirotorClient(timeout_value=3)
                c.confirmConnection()
                self.client_main = c
                print(" [OK! ĐÃ KẾT NỐI]")
                break
            except Exception:
                print(".", end="", flush=True)
                time.sleep(1.0)

        time.sleep(0.5)

        try:
            vehicles = self.client_main.listVehicles()
            if vehicles:
                self.vehicle_name = vehicles[0]
            print(f">> Nhận diện phương tiện: '{self.vehicle_name}'")
        except Exception:
            pass

        try:
            self.client_main.enableApiControl(True, vehicle_name=self.vehicle_name)
            self.client_main.armDisarm(True, vehicle_name=self.vehicle_name)
            print(f">> [API CONTROL] Đã kích hoạt quyền điều khiển UAV '{self.vehicle_name}'!", flush=True)
        except Exception as e:
            print(f">> [API CONTROL WARN] {e}")

        # Hiệu chuẩn nhiệt độ Planck
        calibrate_thermal_camera(self.client_main)
        try:
            self.client_main.simSetCameraFov("front_camera", 70.0, vehicle_name=self.vehicle_name)
        except Exception:
            pass

        self.connected = True

    # -----------------------------------------------------------------------
    # LUỒNG GỬI LỆNH BAY ĐỘC LẬP (HEARTBEAT 20Hz)
    # -----------------------------------------------------------------------
    def heartbeat_thread(self):
        """
        Luồng gửi lệnh vận tốc độc lập liên tục 20Hz (50ms).
        Chỉ gửi lệnh moveByVelocityBodyFrameAsync.
        Nếu mất API control, lập tức re-enable tại chỗ.
        """
        client_cmd = airsim.MultirotorClient(timeout_value=5)
        client_cmd.confirmConnection()
        try:
            client_cmd.enableApiControl(True, vehicle_name=self.vehicle_name)
        except Exception:
            pass

        print(f">> [HEARTBEAT 20Hz] Luồng gửi lệnh bay liên tục đã chạy!", flush=True)

        next_t = time.perf_counter()
        last_t = time.perf_counter()

        while self.running:
            try:
                if self.is_hover:
                    vx, vy, vz, yr = 0.0, 0.0, 0.0, 0.0
                else:
                    vx = self.cmd_vx
                    vy = self.cmd_vy
                    vz = self.cmd_vz
                    yr = self.cmd_yaw_rate

                client_cmd.moveByVelocityBodyFrameAsync(
                    vx, vy, vz,
                    duration=CMD_DURATION,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yr),
                    vehicle_name=self.vehicle_name
                )

                now_c = time.perf_counter()
                dt = max(1e-3, now_c - last_t)
                self.cmd_hz = self.cmd_hz * 0.9 + (1.0 / dt) * 0.1
                last_t = now_c
            except Exception:
                try:
                    client_cmd.enableApiControl(True, vehicle_name=self.vehicle_name)
                except Exception:
                    pass
                time.sleep(0.05)

            next_t += CMD_PERIOD
            sleep_time = next_t - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif sleep_time < -0.2:
                next_t = time.perf_counter()

        try:
            client_cmd.hoverAsync(vehicle_name=self.vehicle_name)
        except Exception:
            pass

    # -----------------------------------------------------------------------
    # LUỒNG ĐỌC TELEMETRY & DUY TRÌ QUYỀN ĐIỀU KHIỂN
    # -----------------------------------------------------------------------
    def telemetry_thread(self):
        client_tele = airsim.MultirotorClient(timeout_value=5)
        client_tele.confirmConnection()
        last_maintain = time.time()

        while self.running:
            loop_start = time.perf_counter()

            # Kiểm tra và duy trì quyền API Control nếu bị mất
            if time.time() - last_maintain > 2.0:
                try:
                    if not client_tele.isApiControlEnabled(vehicle_name=self.vehicle_name):
                        client_tele.enableApiControl(True, vehicle_name=self.vehicle_name)
                    last_maintain = time.time()
                except Exception:
                    pass

            try:
                st = client_tele.getMultirotorState(vehicle_name=self.vehicle_name)
                pos = st.kinematics_estimated.position
                vel = st.kinematics_estimated.linear_velocity
                ori = st.kinematics_estimated.orientation
                _, _, yaw_val = airsim.to_eularian_angles(ori)

                with self.lock:
                    self.pos_x = pos.x_val
                    self.pos_y = pos.y_val
                    self.pos_z = pos.z_val
                    self.uav_quat = (ori.w_val, ori.x_val, ori.y_val, ori.z_val)
                    if self.ground_z is None:
                        self.ground_z = pos.z_val
                    self.alt_agl = max(0.0, self.ground_z - pos.z_val)
                    self.speed = math.hypot(vel.x_val, vel.y_val)
                    self.yaw = math.degrees(yaw_val)
            except Exception:
                pass

            sleep_time = 0.10 - (time.perf_counter() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    # -----------------------------------------------------------------------
    # LUỒNG THU THẬP & HIỂN THỊ CAMERA NHIỆT / CHIỀU SÂU (ĐỘC LẬP 15Hz)
    # -----------------------------------------------------------------------
    def thermal_camera_thread(self):
        """
        Luồng thu thập ảnh camera độc lập (15Hz).
        Hỗ trợ DepthVis (ảnh 2), Infrared, Scene.
        Không làm nghẽn luồng bay Heartbeat 20Hz.
        """
        client_cam = airsim.MultirotorClient(timeout_value=5)
        client_cam.confirmConnection()
        print(">> [CAMERA LIVE] Luồng thu thập ảnh đã sẵn sàng!", flush=True)

        cam_names = ["front_camera", "0"]
        active_cam = cam_names[0]

        last_t = time.perf_counter()

        while self.running:
            loop_start = time.perf_counter()
            try:
                with self.lock:
                    _, cur_img_type = CAMERA_MODES[self.cam_mode_idx]
                    _, cur_pal_mode = THERMAL_PALETTES[self.palette_idx]

                req = [airsim.ImageRequest(active_cam, cur_img_type, False, False)]
                responses = client_cam.simGetImages(req, vehicle_name=self.vehicle_name)
                if not responses or len(responses[0].image_data_uint8) == 0:
                    for cname in cam_names:
                        if cname != active_cam:
                            active_cam = cname
                            break
                    time.sleep(0.1)
                    continue

                resp = responses[0]
                raw = resp.image_data_uint8
                w, h = resp.width, resp.height

                # Chuyển đổi mảng byte sang ảnh
                if cur_img_type == airsim.ImageType.Scene:
                    # Ảnh RGB thường
                    if len(raw) == w * h * 3:
                        rgb_frame = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 3))[:, :, ::-1] # BGR to RGB
                    elif len(raw) == w * h * 4:
                        rgb_frame = cv2.cvtColor(np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 4)), cv2.COLOR_BGRA2RGB)
                    else:
                        arr = np.frombuffer(raw, dtype=np.uint8)
                        rgb_frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        if rgb_frame is not None:
                            rgb_frame = rgb_frame[:, :, ::-1]
                else:
                    # Ảnh DepthVis hoặc Infrared -> chuyển sang grayscale rồi áp dụng Palette
                    if len(raw) == w * h * 3:
                        th_gray = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 3))[:, :, 0]
                    elif len(raw) == w * h * 4:
                        th_gray = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 4))[:, :, 0]
                    elif len(raw) == w * h:
                        th_gray = np.frombuffer(raw, dtype=np.uint8).reshape((h, w))
                    else:
                        arr = np.frombuffer(raw, dtype=np.uint8)
                        th_gray = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)

                    if th_gray is not None:
                        rgb_frame = colorize_thermal(th_gray, cur_pal_mode)
                    else:
                        rgb_frame = None

                if rgb_frame is not None:
                    disp_w, disp_h = 436, 348
                    rgb_disp = cv2.resize(rgb_frame, (disp_w, disp_h))

                    # Vẽ tâm ngắm Reticle chữ thập
                    cx, cy = disp_w // 2, disp_h // 2
                    ret_col = (0, 255, 180) if cur_pal_mode in ("white_hot", "ironbow") else (255, 60, 60)
                    cv2.line(rgb_disp, (cx - 16, cy), (cx - 4, cy), ret_col, 1)
                    cv2.line(rgb_disp, (cx + 4, cy), (cx + 16, cy), ret_col, 1)
                    cv2.line(rgb_disp, (cx, cy - 16), (cx, cy - 4), ret_col, 1)
                    cv2.line(rgb_disp, (cx, cy + 4), (cx, cy + 16), ret_col, 1)

                    # Thước đo nhiệt độ mô phỏng chuẩn FLIR SAR (20°C - 38°C) chỉ vẽ khi đang ở Infrared
                    if cur_img_type == airsim.ImageType.Infrared:
                        tbar_h = 100
                        tbar_x = disp_w - 24
                        tgrad = np.linspace(255, 0, tbar_h, dtype=np.uint8)
                        tcol = colorize_thermal(tgrad[:, None], cur_pal_mode)[:, 0, :]
                        rgb_disp[100:100+tbar_h, tbar_x:tbar_x+8] = np.repeat(tcol[:, None, :], 8, axis=1)
                        cv2.rectangle(rgb_disp, (tbar_x, 100), (tbar_x+8, 100+tbar_h), (200, 200, 200), 1)
                        cv2.putText(rgb_disp, "38C", (tbar_x - 30, 98), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 220, 100), 1)
                        cv2.putText(rgb_disp, "20C", (tbar_x - 30, 100+tbar_h+12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (140, 180, 255), 1)

                    surf = pygame.image.frombuffer(rgb_disp.tobytes(), (disp_w, disp_h), "RGB")
                    with self.lock:
                        self.thermal_surface = surf

                    now_c = time.perf_counter()
                    dt = max(1e-3, now_c - last_t)
                    self.thermal_fps = 0.9 * self.thermal_fps + 0.1 * (1.0 / dt)
                    last_t = now_c

            except Exception as e:
                time.sleep(0.08)

            # Chạy ở tần số 15-20Hz để tối ưu CPU và băng thông RPC
            sleep_time = 0.05 - (time.perf_counter() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    # -----------------------------------------------------------------------
    # LUỒNG THU THẬP & HIỂN THỊ LIDAR 3D OUSTER OS1-64 (ĐỘC LẬP 10Hz)
    def lidar_thread(self):
        """
        Luồng đọc cảm biến LiDAR Ouster OS1-64 độc lập (10Hz).
        Hỗ trợ 3 chế độ trực quan hóa:
        1. BEV Cao Độ (Địa hình Turbo): Bản đồ nhiệt độ cao 2D với mảng màu liền mạch + thước đo mét.
        2. Phối Cảnh 3D (Isometric): Nhìn xiên 3D thấy cây cối, địa hình dựng đứng trong không gian.
        3. Cảnh Báo Cự Ly (Proximity): Tô màu cảnh báo theo khoảng cách vật cản.
        """
        client_lidar = airsim.MultirotorClient(timeout_value=5)
        client_lidar.confirmConnection()
        print(">> [LIDAR OUSTER OS1-64] Luồng quét mây điểm 3D đã sẵn sàng!", flush=True)

        disp_w, disp_h = 436, 348
        kernel = np.ones((3, 3), np.uint8)

        last_lidar_ts = 0
        last_t = time.perf_counter()

        while self.running:
            loop_start = time.perf_counter()
            try:
                ldata = client_lidar.getLidarData(lidar_name="Lidar1", vehicle_name=self.vehicle_name)
                if len(ldata.point_cloud) >= 3 and ldata.time_stamp != last_lidar_ts:
                    last_lidar_ts = ldata.time_stamp
                    raw_pts = np.asarray(ldata.point_cloud, dtype=np.float32).reshape(-1, 3)

                    with self.lock:
                        uav_pos = np.array([self.pos_x, self.pos_y, self.pos_z], dtype=np.float32)
                        uav_quat = self.uav_quat
                        cur_lidar_mode = LIDAR_MODES[self.lidar_mode_idx][1]
                        cur_dens_cfg = LIDAR_DENSITY_LEVELS[self.lidar_density_idx]

                    # Điều chỉnh mật độ điểm theo tỷ lệ để giảm tải CPU/GPU và chống lag
                    ratio = cur_dens_cfg["ratio"]
                    if ratio < 1.0:
                        step = max(1, int(round(1.0 / ratio)))
                        raw_pts = raw_pts[::step]

                    n_pts = len(raw_pts)

                    R_uav = quat_to_R(uav_quat)
                    body_pts = raw_pts @ LIDAR_MOUNT_R.T + LIDAR_MOUNT_POS
                    world_pts = body_pts @ R_uav.T + uav_pos

                    with self.lock:
                        self.latest_world_pts = world_pts

                    bev_img = np.zeros((disp_h, disp_w, 3), dtype=np.uint8)

                    # -------------------------------------------------------
                    # CHẾ ĐỘ 1: BEV CAO ĐỘ (BẢN ĐỒ NHIỆT ĐỘ CAO ĐỊA HÌNH 2D)
                    # -------------------------------------------------------
                    if cur_lidar_mode == "bev_elevation":
                        cx, cy = disp_w // 2, disp_h // 2
                        scale = min(disp_w, disp_h) / 100.0  # 100m range (±50m)

                        px = np.clip((cx + raw_pts[:, 1] * scale).astype(int), 0, disp_w - 1)
                        py = np.clip((cy - raw_pts[:, 0] * scale).astype(int), 0, disp_h - 1)

                        # Trong NED, Z hướng xuống -> độ cao h = -world_pts[:, 2]
                        h = -world_pts[:, 2]
                        h_min = float(np.percentile(h, 2))
                        h_max = float(np.percentile(h, 98))
                        if h_max - h_min < 1.0:
                            h_max = h_min + 1.0

                        h_norm = np.clip(((h - h_min) / (h_max - h_min) * 255.0), 0, 255).astype(np.uint8)
                        # Bảng màu TURBO: Tím/Xanh (thấp/đất) -> Lục -> Vàng -> Đỏ (cây cao/núi)
                        colors = cv2.applyColorMap(h_norm[:, None], cv2.COLORMAP_TURBO)[:, 0, :]

                        bev_img[py, px] = colors
                        # Giãn nở 3x3 để tạo thành các dải mảng cao độ liên tục, không còn chấm li ti!
                        bev_img = cv2.dilate(bev_img, kernel)

                        # Vòng cự ly radar (10m, 25m, 50m)
                        cv2.circle(bev_img, (cx, cy), int(10 * scale), (40, 55, 75), 1)
                        cv2.circle(bev_img, (cx, cy), int(25 * scale), (60, 85, 115), 1)
                        cv2.circle(bev_img, (cx, cy), int(50 * scale), (45, 65, 90), 1)
                        cv2.drawMarker(bev_img, (cx, cy), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=12, thickness=1)

                        cv2.putText(bev_img, "10m", (cx + int(10*scale) + 4, cy - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (120, 160, 200), 1)
                        cv2.putText(bev_img, "25m", (cx + int(25*scale) + 4, cy - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (140, 180, 220), 1)
                        cv2.putText(bev_img, "50m", (cx + int(50*scale) + 4, cy - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (140, 180, 220), 1)

                        # Thước đo độ cao màu (Elevation Legend Bar)
                        bar_h = 110
                        bar_x = disp_w - 24
                        grad = np.linspace(255, 0, bar_h, dtype=np.uint8)
                        bar_col = cv2.applyColorMap(grad[:, None], cv2.COLORMAP_TURBO)[:, 0, :]
                        bev_img[95:95+bar_h, bar_x:bar_x+8] = np.repeat(bar_col[:, None, :], 8, axis=1)
                        cv2.rectangle(bev_img, (bar_x, 95), (bar_x+8, 95+bar_h), (180, 180, 180), 1)
                        cv2.putText(bev_img, f"{h_max:+.0f}m", (bar_x - 36, 93), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 120, 120), 1)
                        cv2.putText(bev_img, f"{h_min:+.0f}m", (bar_x - 36, 95+bar_h+12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 180, 255), 1)

                    # -------------------------------------------------------
                    # CHẾ ĐỘ 2: PHỐI CẢNH 3D (3D ISOMETRIC / PERSPECTIVE VIEW)
                    # -------------------------------------------------------
                    elif cur_lidar_mode == "iso_3d":
                        with self.lock:
                            cur_pitch = self.gimbal_pitch
                        # Góc nhìn 3D thích ứng linh hoạt theo góc gimbal
                        cx, cy = disp_w // 2, int(disp_h * (0.85 + cur_pitch / 400.0))
                        scale_x = 4.3
                        scale_depth = 2.4 * (1.0 - cur_pitch / 150.0)
                        scale_height = 5.2 * (1.0 + cur_pitch / 150.0)

                        fwd_mask = body_pts[:, 0] > 0.5
                        b_pts = body_pts[fwd_mask]
                        w_pts = world_pts[fwd_mask]

                        if len(b_pts) > 0:
                            sx = (cx + b_pts[:, 1] * scale_x).astype(int)
                            sy = (cy - (-b_pts[:, 2] * scale_height + b_pts[:, 0] * scale_depth)).astype(int)

                            valid = (sx >= 0) & (sx < disp_w) & (sy >= 0) & (sy < disp_h)
                            if np.any(valid):
                                val_sx = sx[valid]
                                val_sy = sy[valid]
                                h = -w_pts[valid, 2]
                                h_min = float(np.percentile(h, 2))
                                h_max = float(np.percentile(h, 98))
                                if h_max - h_min < 1.0:
                                    h_max = h_min + 1.0

                                h_norm = np.clip(((h - h_min) / (h_max - h_min) * 255.0), 0, 255).astype(np.uint8)
                                colors = cv2.applyColorMap(h_norm[:, None], cv2.COLORMAP_TURBO)[:, 0, :]

                                bev_img[val_sy, val_sx] = colors
                                bev_img = cv2.dilate(bev_img, kernel)

                        # Biểu tượng UAV ở vị trí đáy giữa
                        cv2.drawMarker(bev_img, (cx, cy), (0, 255, 255), markerType=cv2.MARKER_TRIANGLE_UP, markerSize=14, thickness=2)
                        cv2.putText(bev_img, "UAV", (cx - 12, cy + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1)
                        cv2.putText(bev_img, "GOC NHIN 3D DUNG", (14, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 255), 1)

                    # -------------------------------------------------------
                    # CHẾ ĐỘ 3: CẢNH BÁO CỰ LY VẬT CẢN (PROXIMITY RADAR)
                    # -------------------------------------------------------
                    else:
                        cx, cy = disp_w // 2, disp_h // 2
                        scale = min(disp_w, disp_h) / 100.0

                        px = np.clip((cx + raw_pts[:, 1] * scale).astype(int), 0, disp_w - 1)
                        py = np.clip((cy - raw_pts[:, 0] * scale).astype(int), 0, disp_h - 1)

                        dists = np.linalg.norm(raw_pts, axis=1)
                        # Dưới 10m: Đỏ, 10-25m: Vàng, >25m: Xanh
                        d_norm = np.clip((dists / 50.0 * 255.0), 0, 255).astype(np.uint8)
                        colors = cv2.applyColorMap(255 - d_norm[:, None], cv2.COLORMAP_JET)[:, 0, :]

                        bev_img[py, px] = colors
                        bev_img = cv2.dilate(bev_img, kernel)

                        cv2.circle(bev_img, (cx, cy), int(10 * scale), (0, 0, 255), 1)
                        cv2.circle(bev_img, (cx, cy), int(25 * scale), (0, 200, 255), 1)
                        cv2.circle(bev_img, (cx, cy), int(50 * scale), (50, 70, 90), 1)
                        cv2.drawMarker(bev_img, (cx, cy), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=12, thickness=1)

                        cv2.putText(bev_img, "10m (Nguy hiem)", (cx + int(10*scale) + 4, cy - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 100, 255), 1)
                        cv2.putText(bev_img, "25m (Canh bao)", (cx + int(25*scale) + 4, cy - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 220, 255), 1)

                    surf = pygame.image.frombuffer(bev_img.tobytes(), (disp_w, disp_h), "RGB")

                    now_c = time.perf_counter()
                    dt = max(1e-3, now_c - last_t)
                    last_t = now_c

                    with self.lock:
                        self.lidar_pts_count = n_pts
                        self.lidar_fps = 0.9 * self.lidar_fps + 0.1 * (1.0 / dt)
                        self.lidar_surface = surf

            except Exception:
                pass

            sleep_time = 0.08 - (time.perf_counter() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def trigger_climb_boost(self, duration_sec=2.0):
        """Tự động nâng độ cao mượt mà trong 2s bằng lệnh vận tốc, không dùng task async xung đột."""
        def _climb():
            print("\n>> Đang tự động bay lên độ cao an toàn...", flush=True)
            t_end = time.time() + duration_sec
            while time.time() < t_end and self.running:
                self.cmd_vz = -3.5  # Vận tốc nâng 3.5 m/s
                time.sleep(0.05)
            self.cmd_vz = 0.0
            print(">> Đã bay lên ổn định! Bạn có thể lái tự do.", flush=True)
        threading.Thread(target=_climb, daemon=True).start()

    # -----------------------------------------------------------------------
    # VÒNG LẶP CHÍNH & GIAO DIỆN BÀN PHÍM
    # -----------------------------------------------------------------------
    def run(self):
        enable_anti_reset()
        self.connect_and_init()

        t_heartbeat = threading.Thread(target=self.heartbeat_thread, daemon=True, name="heartbeat")
        t_telemetry = threading.Thread(target=self.telemetry_thread, daemon=True, name="telemetry")
        t_camera = threading.Thread(target=self.thermal_camera_thread, daemon=True, name="cam_stream")
        t_lidar = threading.Thread(target=self.lidar_thread, daemon=True, name="lidar_stream")
        t_heartbeat.start()
        t_telemetry.start()
        t_camera.start()
        t_lidar.start()

        pygame.init()
        pygame.font.init()
        WIN_W, WIN_H = 1200, 640
        VIEW_W, VIEW_H = 436, 348
        COL1_X, COL1_W = 16, 264
        COL2_X, COL2_W = 296, 436
        COL3_X, COL3_W = 748, 436

        screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("UAV TACTICAL GCS - CAMERA NHIỆT & LIDAR 3D OUSTER OS1-64")

        # Đặt cửa sổ bảng điều khiển luôn hiển thị phía trên (Always On Top)
        try:
            pg_hwnd = pygame.display.get_wm_info().get('window')
            if pg_hwnd:
                user32.SetWindowPos(pg_hwnd, -1, 20, 20, 0, 0, 0x0001 | 0x0040)
        except Exception:
            pass

        # Hệ thống phông chữ chuẩn Segoe UI / Consolas
        font_title = pygame.font.SysFont(["Segoe UI", "Tahoma", "Arial"], 15, bold=True)
        font_sec = pygame.font.SysFont(["Segoe UI", "Tahoma", "Arial"], 12, bold=True)
        font_btn_title = pygame.font.SysFont(["Segoe UI", "Tahoma", "Arial"], 11, bold=True)
        font_btn_badge = pygame.font.SysFont(["Segoe UI", "Tahoma", "Arial"], 11, bold=True)
        font_lbl = pygame.font.SysFont(["Segoe UI", "Arial"], 10)
        font_mono = pygame.font.SysFont(["Consolas", "Courier New", "Segoe UI"], 11)
        font_num_lg = pygame.font.SysFont(["Consolas", "Segoe UI", "Arial"], 17, bold=True)
        font_num_sm = pygame.font.SysFont(["Consolas", "Segoe UI", "Arial"], 13, bold=True)

        # Bảng màu sắc thiết kế Tactical GCS Dark Mode
        BG_DARK = (11, 14, 20)
        CARD_BG = (17, 23, 34)
        CARD_HEADER_BG = (23, 31, 46)
        CARD_BORDER = (34, 46, 68)

        CYAN = (0, 229, 255)
        GREEN = (0, 230, 118)
        AMBER = (255, 193, 7)
        RED = (255, 82, 82)
        TEXT_WHITE = (240, 244, 250)
        TEXT_MUTED = (130, 145, 168)

        # Định nghĩa vùng bấm của 6 nút điều khiển tương tác (Mouse Clickable)
        btn_y_start = 290 + 36
        btn_w = COL1_W - 20
        btn_h = 36
        btn_gap = 5
        b_cam_rect = pygame.Rect(COL1_X + 10, btn_y_start + 0 * (btn_h + btn_gap), btn_w, btn_h)
        b_pal_rect = pygame.Rect(COL1_X + 10, btn_y_start + 1 * (btn_h + btn_gap), btn_w, btn_h)
        b_lid_rect = pygame.Rect(COL1_X + 10, btn_y_start + 2 * (btn_h + btn_gap), btn_w, btn_h)
        b_den_rect = pygame.Rect(COL1_X + 10, btn_y_start + 3 * (btn_h + btn_gap), btn_w, btn_h)
        b_fly_rect = pygame.Rect(COL1_X + 10, btn_y_start + 4 * (btn_h + btn_gap), btn_w, btn_h)
        b_hld_rect = pygame.Rect(COL1_X + 10, btn_y_start + 5 * (btn_h + btn_gap), btn_w, btn_h)

        clock = pygame.time.Clock()
        last_t_key = False
        last_h_key = False
        last_v_key = False
        last_c_key = False
        last_l_key = False
        last_p_key = False
        last_lbr_key = False
        last_rbr_key = False
        last_pgup_key = False
        last_pgdn_key = False

        # Bộ lọc chống dội phím / click đúp (Cooldown Timer)
        last_toggle_time = {
            'v': 0.0,
            'c': 0.0,
            'l': 0.0,
            'p': 0.0,
            'pgup': 0.0,
            'pgdn': 0.0,
            't': 0.0,
            'h': 0.0
        }
        COOLDOWN = 0.25

        print("\n" + "=" * 68)
        print(" [SẴN SÀNG] BỘ ĐIỀU KHIỂN UAV TACTICAL GCS:")
        print("  - CHUỘT: Click trực tiếp vào 6 nút trên màn hình để chuyển chế độ!")
        print("  - CUỘN CHUỘT: Điều chỉnh góc cúi/ngửa camera (-90° đến +20°).")
        print("  - ĐIỀU KHIỂN BAY:")
        print("    + [W / S]            : Tiến / Lùi")
        print("    + [A / D]            : Bay Trái / Phải")
        print("    + [Space / Shift]    : Nâng / Hạ độ cao")
        print("    + [Q / E]            : Quay hướng đầu Drone (Yaw)")
        print("    + [Tab]              : Tăng tốc Turbo (14 m/s)")
        print("    + [ESC]              : Thoát an toàn")
        print("=" * 68)

        while self.running:
            v_pressed = False
            c_pressed = False
            l_pressed = False
            p_pressed = False
            dens_dec_pressed = False
            dens_inc_pressed = False
            pgup_pressed = False
            pgdn_pressed = False
            t_pressed = False
            h_pressed = False

            mouse_pos = pygame.mouse.get_pos()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False
                    elif event.key == pygame.K_v:
                        v_pressed = True
                    elif event.key == pygame.K_c:
                        c_pressed = True
                    elif event.key == pygame.K_l:
                        l_pressed = True
                    elif event.key == pygame.K_p:
                        p_pressed = True
                    elif event.key == pygame.K_LEFTBRACKET:
                        dens_dec_pressed = True
                    elif event.key == pygame.K_RIGHTBRACKET:
                        dens_inc_pressed = True
                    elif event.key == pygame.K_PAGEUP:
                        pgup_pressed = True
                    elif event.key == pygame.K_PAGEDOWN:
                        pgdn_pressed = True
                    elif event.key == pygame.K_t:
                        t_pressed = True
                    elif event.key == pygame.K_h:
                        h_pressed = True
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = event.pos
                    if event.button == 1:  # Chuột trái
                        if b_cam_rect.collidepoint((mx, my)):
                            c_pressed = True
                        elif b_pal_rect.collidepoint((mx, my)):
                            v_pressed = True
                        elif b_lid_rect.collidepoint((mx, my)):
                            l_pressed = True
                        elif b_den_rect.collidepoint((mx, my)):
                            p_pressed = True
                        elif b_fly_rect.collidepoint((mx, my)):
                            t_pressed = True
                        elif b_hld_rect.collidepoint((mx, my)):
                            h_pressed = True
                        # Click chuột trái vào khung Camera giữa -> Đổi chế độ Camera
                        elif COL2_X <= mx <= COL2_X + COL2_W and 82 <= my <= 82 + VIEW_H:
                            c_pressed = True
                        # Click chuột trái vào khung LiDAR phải -> Đổi góc quét LiDAR
                        elif COL3_X <= mx <= COL3_X + COL3_W and 82 <= my <= 82 + VIEW_H:
                            l_pressed = True
                    elif event.button == 3:  # Chuột phải
                        if b_den_rect.collidepoint((mx, my)):
                            dens_dec_pressed = True
                        # Click chuột phải vào khung Camera -> Đổi Bảng Màu
                        elif COL2_X <= mx <= COL2_X + COL2_W and 82 <= my <= 82 + VIEW_H:
                            v_pressed = True
                        # Click chuột phải vào khung LiDAR -> Đổi Mật Độ Điểm
                        elif COL3_X <= mx <= COL3_X + COL3_W and 82 <= my <= 82 + VIEW_H:
                            p_pressed = True
                    elif event.button == 4:  # Cuộn chuột lên -> Ngửa camera
                        pgup_pressed = True
                    elif event.button == 5:  # Cuộn chuột xuống -> Cúi camera
                        pgdn_pressed = True

            # Bắt phím toàn cục qua Windows API (khi người dùng đang click vào màn hình Unreal)
            v_down = is_key_down(VK_KEYS['V'])
            if v_down and not last_v_key: v_pressed = True
            last_v_key = v_down

            c_down = is_key_down(VK_KEYS['C'])
            if c_down and not last_c_key: c_pressed = True
            last_c_key = c_down

            l_down = is_key_down(VK_KEYS['L'])
            if l_down and not last_l_key: l_pressed = True
            last_l_key = l_down

            p_down = is_key_down(VK_KEYS['P'])
            if p_down and not last_p_key: p_pressed = True
            last_p_key = p_down

            lbr_down = is_key_down(VK_KEYS['LBRACKET'])
            if lbr_down and not last_lbr_key: dens_dec_pressed = True
            last_lbr_key = lbr_down

            rbr_down = is_key_down(VK_KEYS['RBRACKET'])
            if rbr_down and not last_rbr_key: dens_inc_pressed = True
            last_rbr_key = rbr_down

            pgup_down = is_key_down(VK_KEYS['PGUP'])
            if pgup_down and not last_pgup_key: pgup_pressed = True
            last_pgup_key = pgup_down

            pgdn_down = is_key_down(VK_KEYS['PGDN'])
            if pgdn_down and not last_pgdn_key: pgdn_pressed = True
            last_pgdn_key = pgdn_down

            t_down = is_key_down(VK_KEYS['T'])
            if t_down and not last_t_key: t_pressed = True
            last_t_key = t_down

            h_down = is_key_down(VK_KEYS['H'])
            if h_down and not last_h_key: h_pressed = True
            last_h_key = h_down

            if is_key_down(VK_KEYS['ESC']):
                self.running = False
                break

            # Xử lý sự kiện với bộ lọc Cooldown chống trùng lặp frame
            now_t = time.time()
            if v_pressed and (now_t - last_toggle_time['v'] > COOLDOWN):
                last_toggle_time['v'] = now_t
                with self.lock:
                    self.palette_idx = (self.palette_idx + 1) % len(THERMAL_PALETTES)
                    pal_name = THERMAL_PALETTES[self.palette_idx][0]
                print(f">> [BẢNG MÀU NHIỆT] Đã đổi sang: {pal_name}", flush=True)

            if c_pressed and (now_t - last_toggle_time['c'] > COOLDOWN):
                last_toggle_time['c'] = now_t
                with self.lock:
                    self.cam_mode_idx = (self.cam_mode_idx + 1) % len(CAMERA_MODES)
                    cam_name = CAMERA_MODES[self.cam_mode_idx][0]
                print(f">> [CHẾ ĐỘ CAMERA] Đã đổi sang: {cam_name}", flush=True)

            if l_pressed and (now_t - last_toggle_time['l'] > COOLDOWN):
                last_toggle_time['l'] = now_t
                with self.lock:
                    self.lidar_mode_idx = (self.lidar_mode_idx + 1) % len(LIDAR_MODES)
                    lid_name = LIDAR_MODES[self.lidar_mode_idx][0]
                print(f">> [CHẾ ĐỘ LIDAR] Đã đổi sang: {lid_name}", flush=True)

            if p_pressed and (now_t - last_toggle_time['p'] > COOLDOWN):
                last_toggle_time['p'] = now_t
                with self.lock:
                    self.lidar_density_idx = (self.lidar_density_idx + 1) % len(LIDAR_DENSITY_LEVELS)
                    dens_cfg = LIDAR_DENSITY_LEVELS[self.lidar_density_idx]
                    self.hud_status_msg = dens_cfg["status"]
                print(f">> [MẬT ĐỘ LIDAR] Đổi sang: {dens_cfg['name']} ({dens_cfg['tag']})", flush=True)
                if dens_cfg["warning"]:
                    print(f">> [LIDAR CẢNH BÁO] {dens_cfg['warning']}", flush=True)

            if dens_dec_pressed and (now_t - last_toggle_time['p'] > COOLDOWN):
                last_toggle_time['p'] = now_t
                with self.lock:
                    self.lidar_density_idx = min(len(LIDAR_DENSITY_LEVELS) - 1, self.lidar_density_idx + 1)
                    dens_cfg = LIDAR_DENSITY_LEVELS[self.lidar_density_idx]
                    self.hud_status_msg = dens_cfg["status"]
                print(f">> [MẬT ĐỘ LIDAR] Giảm xuống: {dens_cfg['name']}", flush=True)
                if dens_cfg["warning"]:
                    print(f">> [LIDAR CẢNH BÁO] {dens_cfg['warning']}", flush=True)

            if dens_inc_pressed and (now_t - last_toggle_time['p'] > COOLDOWN):
                last_toggle_time['p'] = now_t
                with self.lock:
                    self.lidar_density_idx = max(0, self.lidar_density_idx - 1)
                    dens_cfg = LIDAR_DENSITY_LEVELS[self.lidar_density_idx]
                    self.hud_status_msg = dens_cfg["status"]
                print(f">> [MẬT ĐỘ LIDAR] Tăng lên: {dens_cfg['name']}", flush=True)

            if pgup_pressed and (now_t - last_toggle_time['pgup'] > 0.08):
                last_toggle_time['pgup'] = now_t
                self.update_gimbal_pitch(+10.0)

            if pgdn_pressed and (now_t - last_toggle_time['pgdn'] > 0.08):
                last_toggle_time['pgdn'] = now_t
                self.update_gimbal_pitch(-10.0)

            if t_pressed and (now_t - last_toggle_time['t'] > 1.0):
                last_toggle_time['t'] = now_t
                self.trigger_climb_boost()

            if h_pressed and (now_t - last_toggle_time['h'] > 0.3):
                last_toggle_time['h'] = now_t
                self.is_hover = not self.is_hover
                print(f">> [CHẾ ĐỘ HOVER] {'ĐÃ BẬT' if self.is_hover else 'ĐÃ TẮT'}", flush=True)

            pg_keys = pygame.key.get_pressed()

            w_down = pg_keys[pygame.K_w] or pg_keys[pygame.K_UP] or is_key_down(VK_KEYS['W']) or is_key_down(VK_KEYS['UP'])
            s_down = pg_keys[pygame.K_s] or pg_keys[pygame.K_DOWN] or is_key_down(VK_KEYS['S']) or is_key_down(VK_KEYS['DOWN'])
            a_down = pg_keys[pygame.K_a] or pg_keys[pygame.K_LEFT] or is_key_down(VK_KEYS['A']) or is_key_down(VK_KEYS['LEFT'])
            d_down = pg_keys[pygame.K_d] or pg_keys[pygame.K_RIGHT] or is_key_down(VK_KEYS['D']) or is_key_down(VK_KEYS['RIGHT'])
            sp_down = pg_keys[pygame.K_SPACE] or is_key_down(VK_KEYS['SPACE'])
            sh_down = pg_keys[pygame.K_LSHIFT] or pg_keys[pygame.K_RSHIFT] or is_key_down(VK_KEYS['SHIFT'])
            q_down = pg_keys[pygame.K_q] or is_key_down(VK_KEYS['Q'])
            e_down = pg_keys[pygame.K_e] or is_key_down(VK_KEYS['E'])
            boost_down = pg_keys[pygame.K_TAB] or pg_keys[pygame.K_LCTRL] or is_key_down(VK_KEYS['TAB']) or is_key_down(VK_KEYS['CTRL'])

            self.is_boost = boost_down
            curr_speed = BOOST_SPEED if self.is_boost else NORMAL_SPEED

            vx = 0.0
            vy = 0.0
            vz = 0.0
            yr = 0.0

            if w_down: vx = curr_speed
            elif s_down: vx = -curr_speed

            if a_down: vy = -curr_speed
            elif d_down: vy = curr_speed

            if sp_down: vz = -4.0
            elif sh_down: vz = +3.5
            else: vz = 0.0

            if q_down: yr = -50.0
            elif e_down: yr = +50.0

            self.cmd_vx = vx
            self.cmd_vy = vy
            self.cmd_vz = vz
            self.cmd_yaw_rate = yr

            # ---------------------------------------------------------------
            # VẼ GIAO DIỆN TACTICAL GCS DARK THEME
            # ---------------------------------------------------------------
            screen.fill(BG_DARK)

            # Đọc an toàn thông số từ các luồng
            with self.lock:
                alt = self.alt_agl
                spd = self.speed
                yw = self.yaw
                px, py, pz = self.pos_x, self.pos_y, self.pos_z
                th_surf = self.thermal_surface
                cur_cam_name, cur_cam_type = CAMERA_MODES[self.cam_mode_idx]
                pal_name, _ = THERMAL_PALETTES[self.palette_idx]
                cur_lid_mode_name, _ = LIDAR_MODES[self.lidar_mode_idx]
                cur_dens = LIDAR_DENSITY_LEVELS[self.lidar_density_idx]
                gim_p = self.gimbal_pitch
                lid_pts = self.lidar_pts_count
                lid_fps_val = self.lidar_fps
                th_fps_val = self.thermal_fps

            # ---------------------------------------------------------------
            # 1. TOP HEADER (y=0, h=42)
            # ---------------------------------------------------------------
            pygame.draw.rect(screen, (15, 20, 30), (0, 0, WIN_W, 42))
            pygame.draw.line(screen, (0, 180, 220), (0, 42), (WIN_W, 42), 2)

            pygame.draw.circle(screen, GREEN, (26, 21), 6)
            pygame.draw.circle(screen, (0, 255, 150), (26, 21), 3)
            screen.blit(font_title.render("UAV TACTICAL GCS", True, TEXT_WHITE), (40, 11))
            screen.blit(font_sec.render("• FLIR THERMAL & OUSTER OS1-64 LIDAR", True, (0, 200, 255)), (190, 13))

            # Status Pill 1: RPC API
            rpc_x = WIN_W - 370
            pygame.draw.rect(screen, (18, 38, 32), (rpc_x, 8, 160, 26), border_radius=13)
            pygame.draw.rect(screen, (0, 200, 100), (rpc_x, 8, 160, 26), 1, border_radius=13)
            pygame.draw.circle(screen, GREEN, (rpc_x + 14, 21), 4)
            screen.blit(font_btn_title.render("RPC : 41451 ACTIVE", True, GREEN), (rpc_x + 26, 13))

            # Status Pill 2: SYSTEM ONLINE
            sys_x = WIN_W - 195
            pygame.draw.rect(screen, (16, 32, 50), (sys_x, 8, 180, 26), border_radius=13)
            pygame.draw.rect(screen, CYAN, (sys_x, 8, 180, 26), 1, border_radius=13)
            screen.blit(font_btn_title.render("SYS: AIRSIM SIMULATOR", True, CYAN), (sys_x + 14, 13))

            # ---------------------------------------------------------------
            # 2. CỘT 1: THÔNG SỐ BAY (TELEMETRY) & BẢNG ĐIỀU KHIỂN NHANH
            # ---------------------------------------------------------------
            # Thẻ 1: Thông số bay
            t_y, t_h = 52, 230
            pygame.draw.rect(screen, CARD_BG, (COL1_X, t_y, COL1_W, t_h), border_radius=8)
            pygame.draw.rect(screen, CARD_BORDER, (COL1_X, t_y, COL1_W, t_h), 1, border_radius=8)
            pygame.draw.rect(screen, CARD_HEADER_BG, (COL1_X, t_y, COL1_W, 30), border_top_left_radius=8, border_top_right_radius=8)
            screen.blit(font_sec.render("THÔNG SỐ BAY (TELEMETRY)", True, CYAN), (COL1_X + 12, t_y + 7))

            # Hàng 1: Độ cao & Vận tốc
            screen.blit(font_lbl.render("ĐỘ CAO (AGL)", True, TEXT_MUTED), (COL1_X + 14, t_y + 36))
            screen.blit(font_num_lg.render(f"{alt:5.1f} m", True, AMBER), (COL1_X + 14, t_y + 50))

            screen.blit(font_lbl.render("VẬN TỐC", True, TEXT_MUTED), (COL1_X + 140, t_y + 36))
            screen.blit(font_num_lg.render(f"{spd:5.1f} m/s", True, GREEN), (COL1_X + 140, t_y + 50))

            # Hàng 2: Góc Yaw & Chế độ bay
            screen.blit(font_lbl.render("HƯỚNG (YAW)", True, TEXT_MUTED), (COL1_X + 14, t_y + 78))
            screen.blit(font_num_lg.render(f"{yw:5.0f}°", True, (180, 210, 255)), (COL1_X + 14, t_y + 92))

            mode_txt = "TURBO (14m/s)" if self.is_boost else ("HOVER (Dừng)" if self.is_hover else "BAY (6m/s)")
            mode_col = RED if self.is_boost else (AMBER if self.is_hover else CYAN)
            screen.blit(font_lbl.render("CHẾ ĐỘ BAY", True, TEXT_MUTED), (COL1_X + 140, t_y + 78))
            pygame.draw.rect(screen, (16, 40, 56), (COL1_X + 138, t_y + 94, 110, 22), border_radius=4)
            pygame.draw.rect(screen, mode_col, (COL1_X + 138, t_y + 94, 110, 22), 1, border_radius=4)
            screen.blit(font_btn_title.render(mode_txt, True, mode_col), (COL1_X + 144, t_y + 97))

            # Đường kẻ ngang phân cách
            pygame.draw.line(screen, CARD_BORDER, (COL1_X + 12, t_y + 126), (COL1_X + COL1_W - 12, t_y + 126), 1)

            # Hàng 3: Tọa độ NED
            screen.blit(font_lbl.render("TỌA ĐỘ THỰC NED (X, Y, Z mét)", True, TEXT_MUTED), (COL1_X + 14, t_y + 132))
            pygame.draw.rect(screen, (12, 16, 24), (COL1_X + 14, t_y + 148, COL1_W - 28, 26), border_radius=4)
            pygame.draw.rect(screen, CARD_BORDER, (COL1_X + 14, t_y + 148, COL1_W - 28, 26), 1, border_radius=4)
            screen.blit(font_mono.render(f"X:{px:5.1f} Y:{py:5.1f} Z:{pz:5.1f}", True, TEXT_WHITE), (COL1_X + 20, t_y + 153))

            # Hàng 4: Góc Pitch Camera & Mây điểm
            screen.blit(font_lbl.render("GÓC PITCH CAMERA", True, TEXT_MUTED), (COL1_X + 14, t_y + 180))
            screen.blit(font_num_sm.render(f"{gim_p:+5.0f}° ({'Cúi' if gim_p < 0 else 'Ngửa'})", True, AMBER), (COL1_X + 14, t_y + 196))

            screen.blit(font_lbl.render("MÂY ĐIỂM QUÉT", True, TEXT_MUTED), (COL1_X + 140, t_y + 180))
            screen.blit(font_num_sm.render(f"{lid_pts:,} pts", True, GREEN), (COL1_X + 140, t_y + 196))

            # Thẻ 2: Bảng Điều Khiển Nhanh Tương Tác
            b_y, b_h = 290, 288
            pygame.draw.rect(screen, CARD_BG, (COL1_X, b_y, COL1_W, b_h), border_radius=8)
            pygame.draw.rect(screen, CARD_BORDER, (COL1_X, b_y, COL1_W, b_h), 1, border_radius=8)
            pygame.draw.rect(screen, CARD_HEADER_BG, (COL1_X, b_y, COL1_W, 30), border_top_left_radius=8, border_top_right_radius=8)
            screen.blit(font_sec.render("BẢNG ĐIỀU KHIỂN NHANH", True, AMBER), (COL1_X + 12, b_y + 7))

            # Danh sách 6 nút bấm trực quan
            btn_specs = [
                (b_cam_rect, "[CAM] CAMERA", cur_cam_name.split(" ")[0], CYAN, (20, 36, 52), (0, 180, 220)),
                (b_pal_rect, "[PAL] BẢNG MÀU", pal_name.split(" ")[0], (255, 220, 100), (45, 40, 20), (220, 180, 50)),
                (b_lid_rect, "[LID] QUÉT LIDAR", cur_lid_mode_name.split(" ")[0], GREEN, (18, 42, 28), (0, 200, 100)),
                (b_den_rect, "[DEN] MẬT ĐỘ", cur_dens["tag"], cur_dens["color"], (18, 42, 28) if not cur_dens["warning"] else (50, 18, 20), (0, 200, 100) if not cur_dens["warning"] else (220, 60, 60)),
                (b_fly_rect, "[FLY] CẤT CÁNH", "Tự bay lên 3m", (140, 200, 255), (20, 32, 48), (60, 120, 200)),
                (b_hld_rect, "[HLD] HOVER PHANH", "Đang giữ chỗ" if self.is_hover else "Bay tự do", (0, 255, 180) if self.is_hover else (255, 120, 120), (20, 45, 35) if self.is_hover else (45, 20, 25), (0, 200, 140) if self.is_hover else (220, 60, 80))
            ]

            for brect, btitle, bbadge, bcol, bbg, bborder in btn_specs:
                is_hover = brect.collidepoint(mouse_pos)
                draw_bg = (min(255, bbg[0] + 16), min(255, bbg[1] + 16), min(255, bbg[2] + 20)) if is_hover else bbg
                draw_border = (min(255, bborder[0] + 50), min(255, bborder[1] + 50), min(255, bborder[2] + 50)) if is_hover else bborder
                pygame.draw.rect(screen, draw_bg, brect, border_radius=6)
                pygame.draw.rect(screen, draw_border, brect, 1, border_radius=6)
                screen.blit(font_btn_title.render(btitle, True, TEXT_WHITE), (brect.x + 8, brect.y + 10))
                badge_surf = font_btn_badge.render(bbadge, True, bcol)
                screen.blit(badge_surf, (brect.right - badge_surf.get_width() - 8, brect.y + 10))

            # ---------------------------------------------------------------
            # 3. CỘT 2: KHUNG CAMERA NHIỆT / CHIỀU SÂU (GIỮA)
            # ---------------------------------------------------------------
            c_y = 52
            pygame.draw.rect(screen, CARD_HEADER_BG, (COL2_X, c_y, COL2_W, 30), border_top_left_radius=8, border_top_right_radius=8)
            pygame.draw.rect(screen, CARD_BORDER, (COL2_X, c_y, COL2_W, 30), 1, border_top_left_radius=8, border_top_right_radius=8)
            screen.blit(font_sec.render("CAMERA NHIỆT FLIR", True, (255, 200, 80)), (COL2_X + 12, c_y + 7))
            screen.blit(font_mono.render(f"{pal_name.split(' ')[0]} | Pitch {gim_p:+.0f}°", True, TEXT_MUTED), (COL2_X + 165, c_y + 8))

            # FPS pill Camera
            fps_w = 60
            pygame.draw.rect(screen, (16, 40, 30), (COL2_X + COL2_W - fps_w - 10, c_y + 5, fps_w, 20), border_radius=10)
            pygame.draw.rect(screen, GREEN, (COL2_X + COL2_W - fps_w - 10, c_y + 5, fps_w, 20), 1, border_radius=10)
            screen.blit(font_mono.render(f"{th_fps_val:.0f} FPS", True, GREEN), (COL2_X + COL2_W - fps_w + 8, c_y + 7))

            # Khung hiển thị Camera
            pygame.draw.rect(screen, (8, 10, 14), (COL2_X, c_y + 30, COL2_W, VIEW_H))
            pygame.draw.rect(screen, (0, 180, 220), (COL2_X, c_y + 30, COL2_W, VIEW_H), 2)
            if th_surf is not None:
                screen.blit(th_surf, (COL2_X, c_y + 30))
            else:
                screen.blit(font_mono.render("Đang nạp tín hiệu Camera...", True, TEXT_MUTED), (COL2_X + 110, c_y + 30 + VIEW_H // 2 - 10))

            # Thẻ phụ: Thông tin phân tích Camera
            ci_y, ci_h = 436, 142
            pygame.draw.rect(screen, CARD_BG, (COL2_X, ci_y, COL2_W, ci_h), border_bottom_left_radius=8, border_bottom_right_radius=8)
            pygame.draw.rect(screen, CARD_BORDER, (COL2_X, ci_y, COL2_W, ci_h), 1, border_bottom_left_radius=8, border_bottom_right_radius=8)

            screen.blit(font_sec.render("PHÂN TÍCH NHẬN DIỆN MỤC TIÊU NHIỆT (FLIR SAR)", True, (255, 200, 80)), (COL2_X + 12, ci_y + 10))
            screen.blit(font_mono.render("• Cảm biến: FLIR Long-Wave IR (8-14 um) | FOV: 70° | 640x512", True, TEXT_WHITE), (COL2_X + 12, ci_y + 32))
            screen.blit(font_mono.render("• Trạng thái: Sẵn sàng phát hiện mục tiêu sinh nhiệt (20°C - 38°C)", True, GREEN), (COL2_X + 12, ci_y + 52))
            screen.blit(font_mono.render(f"• Bảng màu: {pal_name} (Tương phản cao tìm kiếm cứu hộ)", True, TEXT_MUTED), (COL2_X + 12, ci_y + 72))

            pygame.draw.rect(screen, (14, 20, 30), (COL2_X + 10, ci_y + 98, COL2_W - 20, 34), border_radius=6)
            pygame.draw.rect(screen, (30, 42, 60), (COL2_X + 10, ci_y + 98, COL2_W - 20, 34), 1, border_radius=6)
            screen.blit(font_lbl.render("MẸO NHANH:", True, CYAN), (COL2_X + 18, ci_y + 103))
            screen.blit(font_mono.render("Click chuột trái vào khung để đổi Camera | Phải để đổi màu", True, TEXT_WHITE), (COL2_X + 18, ci_y + 116))

            # ---------------------------------------------------------------
            # 4. CỘT 3: KHUNG RADAR LIDAR 3D OUSTER OS1-64 (PHẢI)
            # ---------------------------------------------------------------
            pygame.draw.rect(screen, CARD_HEADER_BG, (COL3_X, c_y, COL3_W, 30), border_top_left_radius=8, border_top_right_radius=8)
            pygame.draw.rect(screen, CARD_BORDER, (COL3_X, c_y, COL3_W, 30), 1, border_top_left_radius=8, border_top_right_radius=8)
            screen.blit(font_sec.render("LIDAR 3D OUSTER OS1-64", True, GREEN), (COL3_X + 12, c_y + 7))

            # Badge mật độ điểm trên thanh tiêu đề LiDAR
            pygame.draw.rect(screen, (16, 40, 26) if not cur_dens["warning"] else (50, 18, 20), (COL3_X + 195, c_y + 5, 125, 20), border_radius=4)
            pygame.draw.rect(screen, cur_dens["color"], (COL3_X + 195, c_y + 5, 125, 20), 1, border_radius=4)
            screen.blit(font_btn_badge.render(cur_dens["tag"], True, cur_dens["color"]), (COL3_X + 205, c_y + 7))

            # FPS pill LiDAR
            pygame.draw.rect(screen, (16, 40, 30), (COL3_X + COL3_W - fps_w - 10, c_y + 5, fps_w, 20), border_radius=10)
            pygame.draw.rect(screen, GREEN, (COL3_X + COL3_W - fps_w - 10, c_y + 5, fps_w, 20), 1, border_radius=10)
            screen.blit(font_mono.render(f"{lid_fps_val:.0f} FPS", True, GREEN), (COL3_X + COL3_W - fps_w + 8, c_y + 7))

            # Khung hiển thị LiDAR
            pygame.draw.rect(screen, (8, 10, 14), (COL3_X, c_y + 30, COL3_W, VIEW_H))
            pygame.draw.rect(screen, cur_dens["color"] if cur_dens["warning"] else GREEN, (COL3_X, c_y + 30, COL3_W, VIEW_H), 2)
            if self.lidar_surface is not None:
                screen.blit(self.lidar_surface, (COL3_X, c_y + 30))
            else:
                screen.blit(font_mono.render("Đang quét LiDAR...", True, TEXT_MUTED), (COL3_X + 130, c_y + 30 + VIEW_H // 2 - 10))

            # Banner cảnh báo nếu mật độ điểm quá thưa (<10%)
            if cur_dens["warning"]:
                warn_box = pygame.Surface((COL3_W - 20, 28), pygame.SRCALPHA)
                warn_box.fill((180, 20, 20, 220))
                screen.blit(warn_box, (COL3_X + 10, c_y + 30 + VIEW_H - 36))
                pygame.draw.rect(screen, (255, 80, 80), (COL3_X + 10, c_y + 30 + VIEW_H - 36, COL3_W - 20, 28), 1, border_radius=4)
                screen.blit(font_sec.render("! CẢNH BÁO: ĐIỂM QUÁ THƯA (<10%) KHÓ QUAN SÁT", True, (255, 240, 240)), (COL3_X + 18, c_y + 30 + VIEW_H - 32))

            # Thẻ phụ: Thông tin phân tích LiDAR
            pygame.draw.rect(screen, CARD_BG, (COL3_X, ci_y, COL3_W, ci_h), border_bottom_left_radius=8, border_bottom_right_radius=8)
            pygame.draw.rect(screen, CARD_BORDER, (COL3_X, ci_y, COL3_W, ci_h), 1, border_bottom_left_radius=8, border_bottom_right_radius=8)

            screen.blit(font_sec.render("THÔNG SỐ QUÉT ĐỊA HÌNH & VẬT CẢN (OUSTER)", True, GREEN), (COL3_X + 12, ci_y + 10))
            screen.blit(font_mono.render("• Cảm biến: Ouster OS1-64 (64 Channels, 100m, Pitch -15°)", True, TEXT_WHITE), (COL3_X + 12, ci_y + 32))
            screen.blit(font_mono.render(f"• Dữ liệu: {lid_pts:,} điểm/khung (Tỷ lệ {cur_dens['tag']} - {cur_dens['name']})", True, (0, 255, 200)), (COL3_X + 12, ci_y + 52))
            screen.blit(font_mono.render(f"• Chế độ: {cur_lid_mode_name} (Nhận diện vật cản & mô phỏng 3D)", True, TEXT_MUTED), (COL3_X + 12, ci_y + 72))

            pygame.draw.rect(screen, (14, 20, 30), (COL3_X + 10, ci_y + 98, COL3_W - 20, 34), border_radius=6)
            pygame.draw.rect(screen, (30, 42, 60), (COL3_X + 10, ci_y + 98, COL3_W - 20, 34), 1, border_radius=6)
            screen.blit(font_lbl.render("MẸO NHANH:", True, GREEN), (COL3_X + 18, ci_y + 103))
            screen.blit(font_mono.render("Click chuột trái để đổi góc quét 3D | Phải để chỉnh mật độ điểm", True, TEXT_WHITE), (COL3_X + 18, ci_y + 116))

            # ---------------------------------------------------------------
            # 5. BOTTOM FLIGHT DOCK (y=588, h=52)
            # ---------------------------------------------------------------
            foot_y = 588
            pygame.draw.rect(screen, (14, 18, 26), (0, foot_y, WIN_W, WIN_H - foot_y))
            pygame.draw.line(screen, (34, 46, 68), (0, foot_y), (WIN_W, foot_y), 1)

            screen.blit(font_sec.render("ĐIỀU KHIỂN BAY:", True, CYAN), (16, foot_y + 15))

            key_guide = [
                ("WASD", "Bay Tiến/Lùi/Trái/Phải"),
                ("Space / Shift", "Nâng / Hạ"),
                ("Q / E", "Quay hướng"),
                ("Cuộn Chuột / PgUp-Dn", "Góc Camera"),
                ("Tab", "Turbo 14m/s")
            ]

            kx = 150
            for k_btn, k_desc in key_guide:
                kw = font_btn_title.size(k_btn)[0] + 12
                pygame.draw.rect(screen, (24, 34, 50), (kx, foot_y + 10, kw, 24), border_radius=4)
                pygame.draw.rect(screen, (60, 80, 110), (kx, foot_y + 10, kw, 24), 1, border_radius=4)
                screen.blit(font_btn_title.render(k_btn, True, (255, 230, 150)), (kx + 6, foot_y + 13))
                kx += kw + 6
                desc_surf = font_lbl.render(k_desc, True, TEXT_MUTED)
                screen.blit(desc_surf, (kx, foot_y + 16))
                kx += desc_surf.get_width() + 16

            status_x = WIN_W - 180
            pygame.draw.circle(screen, GREEN, (status_x, foot_y + 22), 5)
            screen.blit(font_btn_title.render("HỆ THỐNG SẴN SÀNG", True, GREEN), (status_x + 12, foot_y + 14))

            pygame.display.flip()
            clock.tick(60)

        self.running = False
        t_heartbeat.join(timeout=1.0)
        disable_anti_reset()
        pygame.quit()
        print("\n>> Đã dừng bộ điều khiển an toàn.", flush=True)

def main():
    ctrl = DroneController()
    ctrl.run()

if __name__ == "__main__":
    main()
