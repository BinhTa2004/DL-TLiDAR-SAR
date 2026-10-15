# -*- coding: utf-8 -*-
"""
=============================================================================
HE THONG DIEU KHIEN UAV & TRAM GCS DA CAM BIEN TIM KIEM CUU HO
AIRSIM 1.8.1 + UNREAL ENGINE 4.27 (BAN DO AFRICA)
BO DIEU KHIEN TAY CAM XBOX 360 (XINPUT / SDL2) & BAN PHIM TRUC QUAN
=============================================================================
GIAO DIEN DASHBOARD MOT CUA SO 16:9 (canvas logic 1600x900, tu scale theo cua so):
    +--------------------------+--------------------------+-----------+
    |  CAMERA NHIET FLIR       |  CAMERA RGB (FPV / CHASE)|           |
    |  640x512 | FOV 50 deg    |  chuyen bang phim V      |  SIDEBAR  |
    +--------------------------+--------------------------+  DIEU KHIEN|
    |  RADAR LIDAR BEV         |  3D ROI POINT CLOUD      |  & TELEMETRY|
    |  OUSTER OS1-64           |  quanh muc tieu          |           |
    +--------------------------+--------------------------+-----------+
    (4 o luoi moi o 40% x 50%, Sidebar 20% x 100%)

Nang cap:
1. Camera Nhiet FLIR (ImageType 7, 640x512, FOV 50 deg) dong tam LiDAR & RGB:
   - White-Hot / Black-Hot / Ironbow / Rainbow, tam ngam, nhiet do Max/Min
   - Tu dong phat hien nguoi (diem nong nhat) -> ve Bounding Box do
2. Camera RGB: chuyen muot (cross-fade) giua FPV bung drone va Chase Cam bam duoi
3. LiDAR Ouster OS1-64 (64 kenh, 360 x 45 deg, tam 120m, 1.31M diem/s, 10Hz):
   - Radar BEV + hinh non FOV cua camera nhiet
   - Cat vung Point Cloud ban kinh 3-5m quanh nguoi -> hien thi Isometric 3D
4. Dieu khien: luong heartbeat rieng (20Hz) tach khoi doc anh/LiDAR de drone
   khong roi vao hover mode "API call was not received".
5. Dong bo goc Gimbal (0 / -45 / -90) -> simSetCameraPose ngay khi khoi dong.
   LUU Y: AirSim KHONG co API doi tu the LiDAR luc chay -> LiDAR gan co dinh theo
   settings.json (Pitch -45); ma tran gan duoc doc tu settings de chieu diem dung.
=============================================================================
"""

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
import ctypes
import csv

try:
    import win32gui
    import win32con
    HAS_WIN32 = True
except Exception:
    HAS_WIN32 = False

def auto_arrange_windows():
    """
    Su dung win32gui de tu dong tim cua so Unreal Engine (Africa_001) va thu nho hoac ha xuong duoi,
    dong thoi dua cua so GCS Dashboard len tren cung de nguoi lai chi tap trung vao 1 man hinh duy nhat.
    """
    if not HAS_WIN32:
        return
    def enum_handler(hwnd, _):
        if not win32gui.IsWindowVisible(hwnd):
            return True
        title = win32gui.GetWindowText(hwnd)
        cls = win32gui.GetClassName(hwnd)
        # Cua so Unreal Engine Africa
        if ("Africa" in title or "UnrealWindow" in cls) and "UAV GCS" not in title:
            try:
                win32gui.ShowWindow(hwnd, win32con.SW_MINIMIZE)
                win32gui.SetWindowPos(hwnd, win32con.HWND_BOTTOM, 0, 0, 0, 0,
                                      win32con.SWP_NOMOVE | win32con.SWP_NOSIZE | win32con.SWP_NOACTIVATE)
            except Exception:
                pass
        # Cua so Pygame GCS Dashboard
        elif "UAV GCS TIM KIEM CUU HO" in title:
            try:
                win32gui.ShowWindow(hwnd, win32con.SW_RESTORE)
                win32gui.BringWindowToTop(hwnd)
                win32gui.SetForegroundWindow(hwnd)
            except Exception:
                pass
        return True
    try:
        win32gui.EnumWindows(enum_handler, None)
    except Exception:
        pass

import airsim
import numpy as np
import cv2

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
import pygame
import pygame._sdl2.controller as sdl2_controller

# ---------------------------------------------------------------------------
# 1. CAU HINH MAN HINH & HANG SO
# ---------------------------------------------------------------------------
try:
    user32 = ctypes.windll.user32
    try:
        user32.SetProcessDPIAware()
    except Exception:
        pass
    SCREEN_W = user32.GetSystemMetrics(0)
    SCREEN_H = user32.GetSystemMetrics(1)
except Exception:
    SCREEN_W, SCREEN_H = 1920, 1080

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Canvas logic 16:9 - toan bo layout ve tren canvas nay roi scale ra cua so (letterbox)
CANVAS_W, CANVAS_H = 1600, 900
HEADER_H = 36
FOOTER_H = 28
DECK_Y = HEADER_H
DECK_H = CANVAS_H - HEADER_H - FOOTER_H          # 836
PANE_W = int(CANVAS_W * 0.40)                    # 640  (40% rong)
PANE_H = DECK_H // 2                             # 418  (50% cao)
SIDE_X = PANE_W * 2                              # 1280
SIDE_W = CANVAS_W - SIDE_X                       # 320  (20% rong)
PANE_HDR = 30

# Kich thuoc cua so mac dinh: 1600x900 hoac nho hon neu man hinh khong du (giu ti le 16:9)
_fit = min((SCREEN_W - 40) / CANVAS_W, (SCREEN_H - 100) / CANVAS_H, 1.0)
DEFAULT_GCS_W = max(960, int(CANVAS_W * _fit))
DEFAULT_GCS_H = int(DEFAULT_GCS_W * 9 / 16)

os.environ["SDL_VIDEO_CENTERED"] = "1"

GROUND_Z_DATUM = 3.15       # Z (NED) mat dat/mat ho tai diem xuat phat de tinh do cao AGL
THERMAL_FOV_DEG = 50.0      # FLIR Boson ong kinh tam xa
DEFAULT_GIMBAL_PITCH = -45.0  # Goc nghieng tim kiem mac dinh (khop settings.json moi)
CMD_PERIOD = 0.05           # Heartbeat lenh bay: 50ms (20Hz)
CMD_DURATION = 0.25         # Thoi gian hieu luc moi lenh van toc (giu cho drone khong tu hover)
RAW_IMAGE_IS_BGR = True     # AirSim tra anh uncompressed theo thu tu BGR

# ---------------------------------------------------------------------------
# 2. DOC CAU HINH CAM BIEN TU settings.json
# ---------------------------------------------------------------------------
def load_hw_config():
    """Doc toa do/goc gan LiDAR & camera tu settings.json (uu tien ban Unreal dang nap)."""
    cfg = {
        "lidar": {"X": 0.35, "Y": 0.0, "Z": 0.15, "Roll": 0.0, "Pitch": -45.0, "Yaw": 0.0},
        "cam": {"X": 0.35, "Y": 0.0, "Z": 0.15, "Pitch": -45.0, "W": 640, "H": 512},
        "source": "mac dinh",
    }
    for path in (r"C:\AirSim\settings.json", os.path.join(SCRIPT_DIR, "settings.json")):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            veh = next(iter(data.get("Vehicles", {}).values()))
            lid = next(iter(veh.get("Sensors", {}).values()))
            for k in ("X", "Y", "Z", "Roll", "Pitch", "Yaw"):
                if k in lid:
                    cfg["lidar"][k] = float(lid[k])
            cam = veh.get("Cameras", {}).get("front_camera", {})
            for k in ("X", "Y", "Z", "Pitch"):
                if k in cam:
                    cfg["cam"][k] = float(cam[k])
            for cs in cam.get("CaptureSettings", []):
                if cs.get("ImageType") == 7:
                    cfg["cam"]["W"] = int(cs.get("Width", cfg["cam"]["W"]))
                    cfg["cam"]["H"] = int(cs.get("Height", cfg["cam"]["H"]))
            cfg["source"] = path
            break
        except Exception:
            continue
    return cfg


def quat_to_R(q):
    """Quaternion (w, x, y, z) -> ma tran quay 3x3."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)


def euler_to_R(pitch_deg, roll_deg=0.0, yaw_deg=0.0):
    q = airsim.to_quaternion(math.radians(pitch_deg), math.radians(roll_deg), math.radians(yaw_deg))
    return quat_to_R((q.w_val, q.x_val, q.y_val, q.z_val))


# ---------------------------------------------------------------------------
# 3. CLASS HE THONG GCS DA CAM BIEN & DIEU KHIEN UAV
# ---------------------------------------------------------------------------
class UAVUnifiedSystem:
    def __init__(self):
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

        # Telemetry
        self.telemetry = {
            "connected": False,
            "status": "ĐANG KẾT NỐI...",
            "x": 0.0, "y": 0.0, "z": 0.0,
            "alt": 0.0,
            "vx": 0.0, "vy": 0.0, "vz": 0.0,
            "speed": 0.0,
            "pitch": 0.0, "roll": 0.0, "yaw": 0.0,
            "quat": (1.0, 0.0, 0.0, 0.0),
            "battery": 100.0,
            "flight_time": 0.0,
            "points_count": 0
        }

        # Gimbal Camera Angle: 0.0 = Ngang chan troi, -45 = Nghieng tim kiem, -90.0 = Tu tren xuong mat dat
        self.gimbal_pitch = DEFAULT_GIMBAL_PITCH    # Gia tri nguoi dung chon tren UI ("Goc Cam")
        self.gimbal_applied = None                  # Gia tri da gui toi AirSim (simSetCameraPose)

        # Dieu khien van toc
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.cmd_yaw_rate = 0.0

        self.normal_speed = 6.0
        self.boost_speed = 14.0
        self.is_boost = False
        self.is_hover = False
        self.searchlight_on = False

        # Thong ke toc do cac luong
        self.cmd_hz = 0.0
        self.img_hz = 0.0
        self.lidar_hz = 0.0
        self.ui_fps = 0.0

        # Du lieu cam bien
        self.rgb_img = None
        self.ir_img = None
        self.chase_img = None
        self.chase_available = True
        self.detections = []
        self.lidar_pts = np.empty((0, 3), dtype=np.float32)   # Point cloud thua (BEV), toa do the gioi NED
        self.lidar_frames_full = []                           # Khung day du cho cat ROI
        self.lidar_buffer = []
        # LiDAR UAV: Ouster OS1-64 (360 deg ngang x 45 deg doc, tam 120m, 131k diem/s, 10Hz)
        self.lidar_model = "OUSTER OS1-64"
        self.lidar_name = "Lidar1"
        self.lidar_frames_kept = 2                            # Toi uu nhe cho RTX 4060
        self.lidar_roi_frames = 3
        self.lidar_bev_max_pts = 10000
        self.lidar_enabled = True
        self.lidar_ranges = [30.0, 60.0, 120.0]
        self.lidar_range_idx = 2  # 120m = tam toi da cua OS1-64

        # Dai mau RGB cao do cho LiDAR
        self.lidar_colormaps = [
            ("JET (RGB CAO ĐỘ)", cv2.COLORMAP_JET),
            ("TURBO (QUANG PHỔ MỊN)", cv2.COLORMAP_TURBO),
            ("RAINBOW (CẦU VỒNG)", cv2.COLORMAP_RAINBOW)
        ]
        self.lidar_cmap_idx = 1

        # Bang mau camera nhiet FLIR (White-Hot mac dinh chuan cuu ho)
        self.thermal_palettes = [
            ("WHITE-HOT", "white_hot"),
            ("BLACK-HOT", "black_hot"),
            ("IRONBOW", "ironbow"),
            ("RAINBOW HC", "rainbow")
        ]
        self.palette_idx = 0
        self.person_thresh = 245   # Digital count >= 245 ~ 37 do C (than nhiet nguoi)

        # Camera RGB: 0 = FPV bung drone, 1 = Chase cam
        self.rgb_view_target = 0
        self.rgb_blend = 0.0
        self.chase_fail = 0

        # Muc tieu & ROI 3D
        self.target_world = None       # Vi tri 3D (NED) nguoi phat hien duoc tren anh nhiet
        self.target_time = 0.0
        self.target_locked = False
        self.locked_pos = None
        self.roi_radius = 4.0          # 3m <-> 5m
        self.roi_pts = np.empty((0, 3), dtype=np.float32)
        self.roi_center = None
        self.roi_az = 45.0
        self.roi_autorotate = False

        # Che do xem
        self.show_help = False
        self.flight_trail = []
        self.marked_persons = []
        self.last_ping_msg = ""
        self.last_ping_time = 0.0

        # Controller
        self.controller = None
        self.joystick = None
        self.controller_name = "BÀN PHÍM (CHƯA CẮM TAY CẦM)"

        # Thu muc luu anh & CSV
        self.output_dir = os.path.join(SCRIPT_DIR, "rescue_mission_logs")
        os.makedirs(self.output_dir, exist_ok=True)
        self.markers_dir = os.path.join(self.output_dir, "targets")
        os.makedirs(self.markers_dir, exist_ok=True)
        self.csv_path = os.path.join(self.output_dir, "marked_targets.csv")
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["ID", "TIME", "UAV_X", "UAV_Y", "UAV_Z", "ALT", "TARGET_X", "TARGET_Y", "DIST", "GIMBAL_PITCH", "PHOTO_RGB", "PHOTO_THERMAL"])

        # Tiep tuc danh so tu cac phien truoc de khong ghi de anh/CSV cu
        self.marker_counter = 0
        try:
            with open(self.csv_path, 'r', newline='', encoding='utf-8') as f:
                self.marker_counter = max(0, sum(1 for _ in f) - 1)
        except Exception:
            pass

        # Giao dien
        self._fonts = {}
        self.panes = {}
        self.canvas = None
        self.screen = None

    # -----------------------------------------------------------------------
    # TIEN ICH TOAN HOC
    # -----------------------------------------------------------------------
    @staticmethod
    def body_to_world(pts, quat):
        """Quay diem tu khung than UAV (x truoc, y phai, z xuong) sang khung NED the gioi bang quaternion (w, x, y, z)."""
        return pts @ quat_to_R(quat).T

    def sensor_to_world(self, pts, quat, uav_pos):
        """Diem LiDAR (SensorLocalFrame) -> khung than UAV (ma tran gan tu settings) -> NED the gioi."""
        body_pts = pts @ self.lidar_mount_R.T + self.lidar_mount_pos
        return self.body_to_world(body_pts, quat) + uav_pos

    # -----------------------------------------------------------------------
    # TAY CAM
    # -----------------------------------------------------------------------
    def init_controller(self):
        """Khoi tao tay cam qua SDL2 Controller hoac Pygame Joystick fallback"""
        if self.controller is not None:
            return
        try:
            sdl2_controller.init()
            if sdl2_controller.get_count() > 0:
                for idx in range(sdl2_controller.get_count()):
                    if sdl2_controller.is_controller(idx):
                        c = sdl2_controller.Controller(idx)
                        c.init()
                        self.controller = c
                        self.controller_name = c.name
                        print(f">> [TAY CAM XBOX] Da ket noi SDL2: {self.controller_name} (Thiet bi #{idx})", flush=True)
                        return
        except Exception:
            pass

        try:
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                for idx in range(pygame.joystick.get_count()):
                    j = pygame.joystick.Joystick(idx)
                    j.init()
                    j_name = j.get_name()
                    if "keyboard" not in j_name.lower() and "huntsman" not in j_name.lower():
                        self.joystick = j
                        self.controller_name = j_name
                        print(f">> [TAY CAM JOYSTICK] Da ket noi fallback: {self.controller_name} (Thiet bi #{idx})", flush=True)
                        return
        except Exception:
            pass

    # -----------------------------------------------------------------------
    # GIMBAL / CAMERA / MUC TIEU
    # -----------------------------------------------------------------------
    def set_gimbal(self, deg):
        self.gimbal_pitch = max(-90.0, min(0.0, float(deg)))

    def cycle_gimbal(self):
        """Doi nhanh 0 -> -45 -> -90 -> 0"""
        if self.gimbal_pitch > -22.0:
            self.set_gimbal(-45.0)
        elif self.gimbal_pitch > -67.0:
            self.set_gimbal(-90.0)
        else:
            self.set_gimbal(0.0)

    def toggle_rgb_view(self):
        self.rgb_view_target = 1 - self.rgb_view_target

    def toggle_target_lock(self):
        """Khoa/Mo khoa muc tieu: ROI 3D dung yen quanh toa do muc tieu da khoa."""
        if self.target_locked:
            self.target_locked = False
            self.locked_pos = None
            self.notify("[ROI] ĐÃ MỞ KHÓA MỤC TIÊU - TỰ ĐỘNG BÁM THEO ẢNH NHIỆT")
            return
        pos = None
        with self.lock:
            if self.target_world is not None and (time.time() - self.target_time) < 2.0:
                pos = self.target_world.copy()
            elif self.marked_persons:
                m = self.marked_persons[-1]
                pos = np.array([m["tgt_x"], m["tgt_y"], m.get("tgt_z", GROUND_Z_DATUM)], dtype=np.float32)
        if pos is None:
            self.notify("[ROI] KHÔNG CÓ MỤC TIÊU ĐỂ KHÓA (CHƯA PHÁT HIỆN NGƯỜI)")
            return
        self.locked_pos = pos
        self.target_locked = True
        self.notify(f"[ROI] ĐÃ KHÓA MỤC TIÊU TẠI (X:{pos[0]:.1f}, Y:{pos[1]:.1f}, Z:{pos[2]:.1f})")

    def current_roi_target(self):
        if self.target_locked and self.locked_pos is not None:
            return self.locked_pos
        if self.target_world is not None and (time.time() - self.target_time) < 2.0:
            return self.target_world
        return None

    def notify(self, msg):
        self.last_ping_msg = msg
        self.last_ping_time = time.time()
        print(f">> {msg}", flush=True)

    def detect_persons(self, th_gray):
        """Tim cac diem nong nhat (than nhiet nguoi) tren anh nhiet -> danh sach bounding box."""
        if not np.any(th_gray >= self.person_thresh):
            return []
        mask = (th_gray >= self.person_thresh).astype(np.uint8)
        frac = float(mask.mean())
        if frac > 0.25:
            return []
        mask = cv2.dilate(mask, np.ones((3, 3), np.uint8))
        n, _, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
        h_im, w_im = th_gray.shape[:2]
        dets = []
        for i in range(1, n):
            x, y, bw, bh, area = [int(v) for v in stats[i]]
            if area < 12:
                continue
            pad = 4
            peak = float(th_gray[y:y + bh, x:x + bw].max())
            dets.append({
                "bbox": (max(0, x - pad), max(0, y - pad), min(w_im - 1, x + bw + pad), min(h_im - 1, y + bh + pad)),
                "cx": float(cents[i][0]), "cy": float(cents[i][1]),
                "area": area, "temp": 16.0 + peak / 255.0 * 22.5,
            })
        dets.sort(key=lambda d: -d["area"])
        return dets[:6]

    def locate_target_world(self, det, w_im, h_im):
        """Chieu pixel (box nguoi) -> tia 3D -> giao mat dat (tinh chinh bang point cloud LiDAR) => toa do NED."""
        with self.lock:
            quat = self.telemetry["quat"]
            pos = np.array([self.telemetry["x"], self.telemetry["y"], self.telemetry["z"]], dtype=np.float32)
            lidar_pts = self.lidar_pts
        cam_pitch = self.gimbal_applied if self.gimbal_applied is not None else self.gimbal_pitch
        R_body = quat_to_R(quat)
        R_cam = euler_to_R(cam_pitch)
        f = (w_im / 2.0) / math.tan(math.radians(THERMAL_FOV_DEG / 2.0))
        x0, y0, x1, y1 = det["bbox"]
        u = det["cx"]
        v = y1 - 0.25 * (y1 - y0)       # gan chan nguoi
        d_cam = np.array([1.0, (u - w_im / 2.0) / f, (v - h_im / 2.0) / f], dtype=np.float32)
        d_w = R_body @ (R_cam @ d_cam)
        d_w = d_w / (np.linalg.norm(d_w) + 1e-9)
        if d_w[2] <= 0.02:
            return None        # tia khong huong xuong dat
        origin = pos + R_body @ self.cam_mount_np
        gz = GROUND_Z_DATUM
        for _ in range(2):
            t = (gz - origin[2]) / d_w[2]
            if t <= 0:
                return None
            p = origin + t * d_w
            if len(lidar_pts) > 0:
                m = ((lidar_pts[:, 0] - p[0]) ** 2 + (lidar_pts[:, 1] - p[1]) ** 2) < 9.0
                sel = lidar_pts[m, 2]
                if len(sel) >= 20:
                    gz = float(np.percentile(sel, 90))
        t = (gz - origin[2]) / d_w[2]
        if t <= 0 or t > 400:
            return None
        return (origin + t * d_w).astype(np.float32)

    def mark_person_waypoint(self):
        now_str = time.strftime("%Y-%m-%d %H:%M:%S")
        with self.lock:
            px = self.telemetry["x"]
            py = self.telemetry["y"]
            pz = self.telemetry["z"]
            alt = self.telemetry["alt"]
            yaw_deg = self.telemetry["yaw"]
            rgb_copy = self.rgb_img
            ir_copy = self.ir_img
            dets = list(self.detections)
            tgt3d = self.target_world.copy() if (self.target_world is not None and (time.time() - self.target_time) < 2.0) else None
        if self.target_locked and self.locked_pos is not None:
            tgt3d = self.locked_pos.copy()

        if tgt3d is not None:
            tgt_x, tgt_y, tgt_z = float(tgt3d[0]), float(tgt3d[1]), float(tgt3d[2])
        else:
            yaw_rad = math.radians(yaw_deg)
            pitch_rad = math.radians(abs(self.gimbal_pitch))
            if pitch_rad > 0.15:
                d_forward = alt / math.tan(pitch_rad)
            else:
                d_forward = 15.0
            d_forward = min(d_forward, 120.0)
            tgt_x = px + d_forward * math.cos(yaw_rad)
            tgt_y = py + d_forward * math.sin(yaw_rad)
            tgt_z = GROUND_Z_DATUM
        dist_from_uav = math.hypot(tgt_x - px, tgt_y - py)

        self.marker_counter += 1
        wp_id = self.marker_counter
        photo_rgb_name = f"person_{wp_id:03d}_RGB.jpg"
        photo_thermal_name = f"person_{wp_id:03d}_THERMAL.jpg"

        box = dets[0]["bbox"] if dets else (self.cam_w // 4, self.cam_h // 4, self.cam_w * 3 // 4, self.cam_h * 3 // 4)

        if rgb_copy is not None:
            ann_rgb = rgb_copy.copy()
            cv2.rectangle(ann_rgb, (box[0], box[1]), (box[2], box[3]), (255, 0, 0), 2)
            cv2.putText(ann_rgb, f"TARGET PERSON #{wp_id}", (box[0], max(14, box[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            cv2.putText(ann_rgb, f"X={tgt_x:.1f} Y={tgt_y:.1f} ALT={alt:.1f}m", (8, ann_rgb.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imwrite(os.path.join(self.markers_dir, photo_rgb_name), cv2.cvtColor(ann_rgb, cv2.COLOR_RGB2BGR))

        if ir_copy is not None:
            th_gray = ir_copy[:, :, 0] if ir_copy.ndim == 3 else ir_copy
            ann_th = cv2.cvtColor(self.colorize_thermal(th_gray, self.thermal_palettes[self.palette_idx][1]), cv2.COLOR_RGB2BGR)
            cv2.rectangle(ann_th, (box[0], box[1]), (box[2], box[3]), (0, 0, 255), 2)
            cv2.putText(ann_th, f"FLIR TARGET #{wp_id}", (box[0], max(14, box[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imwrite(os.path.join(self.markers_dir, photo_thermal_name), ann_th)

        with open(self.csv_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([wp_id, now_str, f"{px:.2f}", f"{py:.2f}", f"{pz:.2f}", f"{alt:.2f}", f"{tgt_x:.2f}", f"{tgt_y:.2f}", f"{dist_from_uav:.2f}", f"{self.gimbal_pitch:.1f}", photo_rgb_name, photo_thermal_name])

        marker = {
            "id": wp_id,
            "time": time.strftime("%H:%M:%S"),
            "uav_x": px, "uav_y": py,
            "tgt_x": tgt_x, "tgt_y": tgt_y, "tgt_z": tgt_z,
            "alt": alt, "dist": dist_from_uav,
            "color": (255, 50, 50)
        }
        self.marked_persons.append(marker)
        self.notify(f"[PING] ĐÃ ĐÁNH DẤU MỤC TIÊU #{wp_id} TẠI (X:{tgt_x:.1f}m, Y:{tgt_y:.1f}m, CAO:{alt:.1f}m)!")

    # -----------------------------------------------------------------------
    # 4. CAC LUONG NEN (CONTROL HEARTBEAT / ANH / LIDAR) - MOI LUONG 1 CLIENT RIENG
    # -----------------------------------------------------------------------
    def _connect_client(self):
        while self.running:
            try:
                client = airsim.MultirotorClient(timeout_value=5)
                client.confirmConnection()
                return client
            except Exception:
                time.sleep(1.0)
        return None

    def auto_arrange_windows(self):
        """Thu nho Unreal Engine va dua cua so GCS Dashboard len Foreground."""
        auto_arrange_windows()
        self.notify("[WINDOW] ĐÃ THU NHỎ UNREAL VÀ ĐƯA GCS DASHBOARD LÊN TRÊN CÙNG!")

    def window_management_loop(self):
        """Tu dong quet va can chinh cua so Unreal Engine va GCS sau khi khoi dong."""
        delays = [1.0, 2.5, 5.0, 10.0]
        for d in delays:
            if not self.running:
                break
            time.sleep(d)
            auto_arrange_windows()

    def heartbeat_control_loop(self):
        """
        Luong DIEU KHIEN HEARTBEAT RIENG BIET (20Hz - chu ky 50ms).
        Luong nay SU DUNG 1 CLIENT RIENG va CHỈ GỬI LỆNH VẬN TỐC (moveByVelocityBodyFrameAsync).
        Tuyet doi KHONG goi getMultirotorState hay bat ky ham RPC dong bo nao khac.
        -> Thoi gian thuc thi moi vong < 0.5ms, heartbeat khong bao gio bi tre nhip,
        -> GIAI QUYET TRIET DE LOI 'API call was not received, entering hover mode for safety'!
        """
        client = self._connect_client()
        if client is None:
            return
        print(f">> [CONTROL HEARTBEAT] Luong dieu khien 20Hz da ket noi toi AirSim '{self.vehicle_name}'!", flush=True)

        next_t = time.perf_counter()
        last_cmd_t = time.perf_counter()

        while self.running:
            try:
                if self.is_hover:
                    vx = vy = vz = yr = 0.0
                else:
                    vx, vy, vz, yr = self.cmd_vx, self.cmd_vy, self.cmd_vz, self.cmd_yaw_rate

                client.moveByVelocityBodyFrameAsync(
                    vx, vy, vz,
                    duration=CMD_DURATION,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yr),
                    vehicle_name=self.vehicle_name
                )
                now_c = time.perf_counter()
                inst = 1.0 / max(1e-3, now_c - last_cmd_t)
                self.cmd_hz = self.cmd_hz * 0.9 + inst * 0.1
                last_cmd_t = now_c
            except Exception:
                time.sleep(0.05)

            next_t += CMD_PERIOD
            sleep_time = next_t - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif sleep_time < -0.2:
                next_t = time.perf_counter()

        # Khi thoat: dua UAV ve hover an toan
        try:
            client.hoverAsync(vehicle_name=self.vehicle_name)
        except Exception:
            pass

    def telemetry_gimbal_loop(self):
        """
        Luong DOC TELEMETRY & DONG BO GIMBAL CAMERA (20Hz).
        Luong nay SU DUNG 1 CLIENT RIENG:
        - Doc getMultirotorState (vi tri, do cao AGL, van toc, goc Euler, quaternion).
        - Ngay luc khoi dong: goi simSetCameraPose va simSetCameraFov theo 'Goc Cam' tren UI.
        - Khi nguoi dung bam doi goc cam (1: 0°, 2: -45°, 3: -90°): goi simSetCameraPose
          dong bo goc nghieng cua ca Camera Nhiet FLIR va Camera RGB!
        """
        client = self._connect_client()
        if client is None:
            return
        try:
            vehicles = client.listVehicles()
            if vehicles:
                self.vehicle_name = vehicles[0]
        except Exception:
            pass
        with self.lock:
            self.telemetry["connected"] = True
            self.telemetry["status"] = "TRỰC TUYẾN"
        print(f">> [TELEMETRY & GIMBAL] Da ket noi toi '{self.vehicle_name}'!", flush=True)

        try:
            client.enableApiControl(True, vehicle_name=self.vehicle_name)
            client.armDisarm(True, vehicle_name=self.vehicle_name)
            print(">> [API CONTROL] Da kich hoat quyen dieu khien bay tu GCS!", flush=True)
        except Exception:
            pass

        # Dong bo FOV 50 do & goc Gimbal ngay luc khoi dong theo bien "Goc Cam" tren UI
        try:
            client.simSetCameraFov("front_camera", THERMAL_FOV_DEG, vehicle_name=self.vehicle_name)
        except Exception:
            pass
        self._apply_gimbal(client)
        print(f">> [GIMBAL] Da dong bo goc Camera Nhiet/RGB = {self.gimbal_pitch:.0f} do (LiDAR gan co dinh Pitch {self.hw['lidar']['Pitch']:.0f} do)", flush=True)

        start_time = time.time()
        last_enable = time.time()

        while self.running:
            loop_start = time.perf_counter()

            # Duy tri quyen API control moi 3s
            if time.time() - last_enable > 3.0:
                try:
                    client.enableApiControl(True, vehicle_name=self.vehicle_name)
                    last_enable = time.time()
                except Exception:
                    pass

            # (1) Telemetry
            try:
                st = client.getMultirotorState(vehicle_name=self.vehicle_name)
                pos = st.kinematics_estimated.position
                vel = st.kinematics_estimated.linear_velocity
                ori = st.kinematics_estimated.orientation
                pitch_val, roll_val, yaw_val = airsim.to_eularian_angles(ori)

                elapsed = time.time() - start_time
                battery_sim = max(5.0, 100.0 - (elapsed / 30.0))

                with self.lock:
                    self.telemetry["connected"] = True
                    self.telemetry["status"] = "TRỰC TUYẾN"
                    self.telemetry["x"] = pos.x_val
                    self.telemetry["y"] = pos.y_val
                    self.telemetry["z"] = pos.z_val
                    # Tinh do cao thuc te (AGL) so voi mat ho/mat dat (Z datum ~ 3.15m)
                    self.telemetry["alt"] = max(0.0, GROUND_Z_DATUM - pos.z_val)
                    self.telemetry["vx"] = vel.x_val
                    self.telemetry["vy"] = vel.y_val
                    self.telemetry["vz"] = vel.z_val
                    self.telemetry["speed"] = math.sqrt(vel.x_val**2 + vel.y_val**2 + vel.z_val**2)
                    self.telemetry["pitch"] = math.degrees(pitch_val)
                    self.telemetry["roll"] = math.degrees(roll_val)
                    self.telemetry["yaw"] = math.degrees(yaw_val)
                    # Quaternion tu the (w, x, y, z) de chuyen diem LiDAR sang khung the gioi
                    self.telemetry["quat"] = (ori.w_val, ori.x_val, ori.y_val, ori.z_val)
                    self.telemetry["battery"] = battery_sim
                    self.telemetry["flight_time"] = elapsed

                    if len(self.flight_trail) == 0 or math.hypot(pos.x_val - self.flight_trail[-1][0], pos.y_val - self.flight_trail[-1][1]) > 0.5:
                        self.flight_trail.append((pos.x_val, pos.y_val))
                        if len(self.flight_trail) > 500:
                            self.flight_trail.pop(0)
            except Exception:
                with self.lock:
                    self.telemetry["connected"] = False
                    self.telemetry["status"] = "MẤT KẾT NỐI"

            # (2) Dong bo goc Gimbal Camera Nhiet + RGB khi nguoi dung thay doi "Goc Cam"
            if self.gimbal_applied is None or abs(self.gimbal_pitch - self.gimbal_applied) > 0.2:
                self._apply_gimbal(client)

            sleep_time = 0.05 - (time.perf_counter() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _apply_gimbal(self, client):
        """simSetCameraPose cho Camera Nhiet + RGB (cung 1 camera 'front_camera' dong tam LiDAR)."""
        target = self.gimbal_pitch
        try:
            q = airsim.to_quaternion(math.radians(target), 0.0, 0.0)
            client.simSetCameraPose("front_camera", airsim.Pose(self.cam_mount_pos, q), vehicle_name=self.vehicle_name)
            self.gimbal_applied = target
        except Exception:
            self.gimbal_applied = target if self.gimbal_applied is None else self.gimbal_applied

    @staticmethod
    def parse_img(resp):
        if resp is None:
            return None
        raw = resp.image_data_uint8
        if len(raw) == resp.width * resp.height * 3:
            arr = np.frombuffer(raw, dtype=np.uint8).reshape((resp.height, resp.width, 3))
            return arr[:, :, ::-1].copy() if RAW_IMAGE_IS_BGR else arr.copy()
        elif len(raw) == resp.width * resp.height * 4:
            arr = np.frombuffer(raw, dtype=np.uint8).reshape((resp.height, resp.width, 4))
            return cv2.cvtColor(arr, cv2.COLOR_BGRA2RGB)
        elif len(raw) > 0:
            arr = np.frombuffer(raw, dtype=np.uint8)
            dec = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if dec is not None:
                return cv2.cvtColor(dec, cv2.COLOR_BGR2RGB)
        if len(resp.image_data_float) > 0:
            farr = np.array(resp.image_data_float, dtype=np.float32)
            if farr.size == resp.width * resp.height:
                farr = farr.reshape(resp.height, resp.width)
                farr = np.clip(farr, 0.0, 100.0)
                norm = (farr / 100.0 * 255.0).astype(np.uint8)
                return cv2.cvtColor(norm, cv2.COLOR_GRAY2RGB)
        return None

    def image_loop(self):
        """Luong thu thap anh Nhiet / RGB FPV / Chase cam + phat hien nguoi tren anh nhiet."""
        client = self._connect_client()
        if client is None:
            return

        # Hieu chuan dai nhiet do Planck cho Camera Hong Ngoai (Thermal IR)
        try:
            client.simSetSegmentationObjectID(".*", 0, True)
            client.simSetSegmentationObjectID(".*Base_Terrain.*", 65, True)
            client.simSetSegmentationObjectID(".*landscape.*", 65, True)
            client.simSetSegmentationObjectID(".*soil.*", 65, True)
            client.simSetSegmentationObjectID(".*InstancedFoliageActor.*", 35, True)
            client.simSetSegmentationObjectID(".*tree.*", 35, True)
            client.simSetSegmentationObjectID(".*bush.*", 35, True)
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
            print(">> [FLIR THERMAL] Da hieu chuan dai nhiet do Planck (37.4°C White-Hot cho Nguoi)!", flush=True)
        except Exception:
            pass

        last_t = time.perf_counter()
        while self.running:
            loop_start = time.perf_counter()
            want_chase = self.chase_available and (self.rgb_view_target == 1 or self.rgb_blend > 0.01)
            try:
                reqs = [
                    airsim.ImageRequest("front_camera", airsim.ImageType.Scene, False, False),
                    airsim.ImageRequest("front_camera", airsim.ImageType.Infrared, False, False),
                ]
                if want_chase:
                    reqs.append(airsim.ImageRequest("chase_camera", airsim.ImageType.Scene, False, False))
                try:
                    responses = client.simGetImages(reqs, vehicle_name=self.vehicle_name)
                except Exception:
                    responses = None
                    if want_chase:
                        # Chase camera chua ton tai (settings.json cu) -> loai bo, van lay FPV/Nhiet
                        self.chase_fail += 1
                        if self.chase_fail >= 3:
                            self.chase_available = False
                            print(">> [CHASE CAM] Khong tim thay 'chase_camera' - hay khoi dong lai Unreal de nap settings.json moi.", flush=True)
                        responses = client.simGetImages(reqs[:2], vehicle_name=self.vehicle_name)

                im_rgb = self.parse_img(responses[0]) if responses and len(responses) >= 1 else None
                im_ir = self.parse_img(responses[1]) if responses and len(responses) >= 2 else None
                im_chase = self.parse_img(responses[2]) if responses and len(responses) >= 3 else None

                dets = None
                if im_ir is not None:
                    th_gray = im_ir[:, :, 0] if im_ir.ndim == 3 else im_ir
                    dets = self.detect_persons(th_gray)
                    now = time.time()
                    if dets:
                        p = self.locate_target_world(dets[0], th_gray.shape[1], th_gray.shape[0])
                        if p is not None:
                            with self.lock:
                                old = self.target_world
                                if old is not None and (now - self.target_time) < 2.0 and np.linalg.norm(old - p) < 5.0:
                                    p = 0.6 * old + 0.4 * p
                                self.target_world = p.astype(np.float32)
                                self.target_time = now

                with self.lock:
                    if im_rgb is not None:
                        self.rgb_img = im_rgb
                    if im_ir is not None:
                        self.ir_img = im_ir
                        self.detections = dets or []
                    if im_chase is not None:
                        self.chase_img = im_chase

                now_c = time.perf_counter()
                self.img_hz = self.img_hz * 0.9 + (1.0 / max(1e-3, now_c - last_t)) * 0.1
                last_t = now_c
            except Exception:
                time.sleep(0.1)

            # Gioi han 20 FPS cho camera sensor de tranh nghen bang thong GPU-CPU
            sleep_time = 0.05 - (time.perf_counter() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def lidar_loop(self):
        """Luong doc LiDAR Ouster OS1-64 (10Hz), cong don, va cat ROI 3D quanh muc tieu."""
        client = self._connect_client()
        if client is None:
            return
        last_lidar_ts = 0
        last_t = time.perf_counter()
        printed_pose = False
        while self.running:
            loop_start = time.perf_counter()
            if not self.lidar_enabled:
                time.sleep(0.1)
                continue
            try:
                ldata = client.getLidarData(lidar_name=self.lidar_name, vehicle_name=self.vehicle_name)
                # LiDAR cap nhat 10Hz tu settings.json
                if len(ldata.point_cloud) >= 3 and ldata.time_stamp != last_lidar_ts:
                    last_lidar_ts = ldata.time_stamp
                    if not printed_pose:
                        printed_pose = True
                        lp = ldata.pose
                        print(f">> [LIDAR] {self.lidar_model}: pose gan (tu AirSim) = ({lp.position.x_val:.2f},{lp.position.y_val:.2f},{lp.position.z_val:.2f}); "
                              f"su dung ma tran gan tu {self.hw['source']}", flush=True)
                    pts = np.asarray(ldata.point_cloud, dtype=np.float32).reshape(-1, 3)
                    with self.lock:
                        uav_pos = np.array([self.telemetry["x"], self.telemetry["y"], self.telemetry["z"]], dtype=np.float32)
                        quat = self.telemetry["quat"]
                    # settings.json: DataFrame = SensorLocalFrame -> diem theo khung cam bien
                    world_pts = self.sensor_to_world(pts, quat, uav_pos)

                    step = max(1, int(math.ceil(len(world_pts) / float(self.lidar_bev_max_pts))))
                    sparse = world_pts[::step]
                    self.lidar_frames_full.append(world_pts)
                    if len(self.lidar_frames_full) > self.lidar_roi_frames:
                        self.lidar_frames_full.pop(0)
                    self.lidar_buffer.append(sparse)
                    if len(self.lidar_buffer) > self.lidar_frames_kept:
                        self.lidar_buffer.pop(0)
                    merged = np.vstack(self.lidar_buffer)

                    # Cat vung ROI ban kinh 3-5m quanh muc tieu tu cac khung day du
                    tgt = self.current_roi_target()
                    roi = np.empty((0, 3), dtype=np.float32)
                    center = None
                    if tgt is not None:
                        center = np.array(tgt, dtype=np.float32)
                        R = self.roi_radius
                        parts = []
                        for fr in self.lidar_frames_full:
                            d = fr - center
                            m = (d[:, 0] ** 2 + d[:, 1] ** 2 <= R * R) & (np.abs(d[:, 2]) <= R)
                            if m.any():
                                parts.append(fr[m])
                        if parts:
                            roi = np.vstack(parts)

                    with self.lock:
                        self.lidar_pts = merged
                        self.telemetry["points_count"] = len(merged)
                        self.roi_pts = roi
                        self.roi_center = center

                    now_c = time.perf_counter()
                    self.lidar_hz = self.lidar_hz * 0.8 + (1.0 / max(1e-3, now_c - last_t)) * 0.2
                    last_t = now_c
                else:
                    time.sleep(0.03)
            except Exception:
                time.sleep(0.1)

            # Dong bo chu ky ~12Hz phu hop voi 10 RPS cua Ouster OS1-64
            sleep_time = 0.08 - (time.perf_counter() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    # -----------------------------------------------------------------------
    # 5. TIEN ICH VE
    # -----------------------------------------------------------------------
    def font(self, size, bold=False, mono=False):
        key = (size, bold, mono)
        f = self._fonts.get(key)
        if f is None:
            f = pygame.font.SysFont("Consolas" if mono else "Segoe UI", size, bold=bold)
            self._fonts[key] = f
        return f

    def text(self, surf, s, pos, size=12, color=(235, 240, 245), bold=False, mono=False, anchor="topleft"):
        img = self.font(size, bold, mono).render(s, True, color)
        r = img.get_rect()
        setattr(r, anchor, pos)
        surf.blit(img, r)
        return r

    def draw_header(self, surf, title, accent, subtitle=None, sub_color=(200, 215, 230)):
        w = surf.get_width()
        pygame.draw.rect(surf, (17, 23, 33), (0, 0, w, PANE_HDR))
        pygame.draw.rect(surf, accent, (0, PANE_HDR - 2, w, 2))
        r = self.text(surf, title, (10, 6), 12, accent, bold=True)
        if subtitle:
            self.text(surf, subtitle, (r.right + 12, 8), 11, sub_color, bold=False)

    @staticmethod
    def fit_array(rgb, area_w, area_h):
        """Dat anh vao khung area_w x area_h giu ti le (letterbox). Tra ve (canvas, ox, oy, dw, dh)."""
        h, w = rgb.shape[:2]
        s = min(area_w / float(w), area_h / float(h))
        dw, dh = max(1, int(w * s)), max(1, int(h * s))
        resized = cv2.resize(rgb, (dw, dh), interpolation=cv2.INTER_LINEAR if s > 1 else cv2.INTER_AREA)
        canvas = np.zeros((area_h, area_w, 3), dtype=np.uint8)
        ox, oy = (area_w - dw) // 2, (area_h - dh) // 2
        canvas[oy:oy + dh, ox:ox + dw] = resized
        return canvas, ox, oy, dw, dh

    @staticmethod
    def array_to_surface(arr):
        arr = np.ascontiguousarray(arr)
        h, w = arr.shape[:2]
        return pygame.image.frombuffer(arr.tobytes(), (w, h), "RGB")

    @staticmethod
    def colorize_thermal(th_gray, pal_mode):
        if pal_mode == "white_hot":
            return cv2.cvtColor(th_gray, cv2.COLOR_GRAY2RGB)
        if pal_mode == "black_hot":
            return cv2.cvtColor(255 - th_gray, cv2.COLOR_GRAY2RGB)
        if pal_mode == "ironbow":
            return cv2.cvtColor(cv2.applyColorMap(th_gray, cv2.COLORMAP_INFERNO), cv2.COLOR_BGR2RGB)
        if pal_mode == "rainbow":
            return cv2.cvtColor(cv2.applyColorMap(th_gray, cv2.COLORMAP_JET), cv2.COLOR_BGR2RGB)
        return cv2.cvtColor(th_gray, cv2.COLOR_GRAY2RGB)

    @staticmethod
    def temp_of(v):
        return 16.0 + (float(v) / 255.0) * 22.5

    # -----------------------------------------------------------------------
    # 6. RENDERING CAC PANE
    # -----------------------------------------------------------------------
    def render_thermal_pane(self):
        surf = self.panes["thermal"]
        w, h = surf.get_size()
        surf.fill((10, 14, 20))

        with self.lock:
            ir = self.ir_img
            dets = list(self.detections)

        palette_name, pal_mode = self.thermal_palettes[self.palette_idx]
        accent = (255, 170, 50)
        self.draw_header(surf, "CAMERA NHIỆT FLIR", accent,
                         f"({self.cam_w}x{self.cam_h} | FOV {THERMAL_FOV_DEG:.0f}°)  {palette_name}")
        n_det = len(dets)
        self.text(surf, f"{'● ' if n_det else '○ '}{n_det} NGƯỜI", (w - 10, 7), 12,
                  (255, 70, 70) if n_det else (130, 140, 150), bold=True, anchor="topright")

        strip_h = 22
        area = pygame.Rect(0, PANE_HDR, w, h - PANE_HDR - strip_h)
        pygame.draw.rect(surf, (14, 18, 26), (0, h - strip_h, w, strip_h))

        if ir is not None:
            th_gray = ir[:, :, 0] if ir.ndim == 3 else ir
            h_im, w_im = th_gray.shape[:2]
            rgb_th = self.colorize_thermal(th_gray, pal_mode)
            canvas, ox, oy, dw, dh = self.fit_array(rgb_th, area.w, area.h)
            surf.blit(self.array_to_surface(canvas), (area.x, area.y))
            ox += area.x
            oy += area.y
            sc = dw / float(w_im)

            ret_col = (0, 255, 180) if pal_mode in ("white_hot", "ironbow") else (255, 50, 50)
            cx, cy = ox + dw // 2, oy + dh // 2
            pygame.draw.line(surf, ret_col, (cx - 18, cy), (cx - 6, cy), 1)
            pygame.draw.line(surf, ret_col, (cx + 6, cy), (cx + 18, cy), 1)
            pygame.draw.line(surf, ret_col, (cx, cy - 18), (cx, cy - 6), 1)
            pygame.draw.line(surf, ret_col, (cx, cy + 6), (cx, cy + 18), 1)
            pygame.draw.circle(surf, ret_col, (cx, cy), 2, 1)

            center_val = float(th_gray[max(0, h_im // 2 - 2):h_im // 2 + 3, max(0, w_im // 2 - 2):w_im // 2 + 3].mean())
            self.text(surf, f"{self.temp_of(center_val):.1f}°C", (cx + 12, cy + 8), 12, ret_col, bold=True, mono=True)

            # Bounding Box do quanh nguoi phat hien duoc
            for i, d in enumerate(dets):
                x0, y0, x1, y1 = d["bbox"]
                rx0, ry0 = ox + int(x0 * sc), oy + int(y0 * sc)
                rx1, ry1 = ox + int(x1 * sc), oy + int(y1 * sc)
                pygame.draw.rect(surf, (255, 40, 40), (rx0, ry0, max(6, rx1 - rx0), max(6, ry1 - ry0)), 2)
                lbl = f"NGƯỜI {d['temp']:.1f}°C" + ("  [TARGET]" if i == 0 else "")
                self.text(surf, lbl, (rx0, max(area.y + 2, ry0 - 15)), 11, (255, 90, 90), bold=True, mono=True)

            # Thang mau nhiet ben trai
            if ox >= 34:
                grad = np.linspace(255, 0, dh, dtype=np.uint8).reshape(-1, 1)
                bar = self.colorize_thermal(np.repeat(grad, 10, axis=1), pal_mode)
                surf.blit(self.array_to_surface(bar), (10, oy))
                pygame.draw.rect(surf, (150, 170, 190), (10, oy, 10, dh), 1)
                self.text(surf, f"{self.temp_of(255):.0f}°", (23, oy - 2), 10, (220, 230, 240), mono=True)
                self.text(surf, f"{self.temp_of(0):.0f}°", (23, oy + dh - 12), 10, (220, 230, 240), mono=True)

            th_min, th_max = self.temp_of(th_gray.min()), self.temp_of(th_gray.max())
            self.text(surf, f"MAX: {th_max:.1f}°C   MIN: {th_min:.1f}°C   CENTER: {self.temp_of(center_val):.1f}°C   GÓC CAM: {self.gimbal_pitch:.0f}°",
                      (10, h - strip_h + 4), 11, (220, 240, 255), mono=True)
        else:
            self.text(surf, "[CHỜ TÍN HIỆU CAMERA NHIỆT...]", (w // 2, h // 2), 14, (255, 120, 50), anchor="center")

        pygame.draw.rect(surf, accent, (0, 0, w, h), 1)

    def render_rgb_pane(self):
        surf = self.panes["rgb"]
        w, h = surf.get_size()
        surf.fill((10, 14, 20))

        with self.lock:
            fpv = self.rgb_img
            chase = self.chase_img
            dets = list(self.detections)
            alt = self.telemetry["alt"]
            speed = self.telemetry["speed"]

        accent = (0, 220, 140)
        self.draw_header(surf, "CAMERA RGB & GÓC NHÌN TOÀN CẢNH", accent)
        # Tab chuyen doi FPV / CHASE
        tabs = [("FPV BỤNG", 0), ("CHASE CAM", 1)]
        tx = w - 10
        for label, idx in reversed(tabs):
            active = (self.rgb_view_target == idx)
            tw = self.font(11, True).size(label)[0] + 18
            tx -= tw
            pygame.draw.rect(surf, accent if active else (35, 45, 58), (tx, 5, tw, 20), border_radius=4)
            self.text(surf, label, (tx + tw // 2, 15), 11, (5, 20, 15) if active else (170, 185, 200), bold=True, anchor="center")
            tx -= 6
        self.text(surf, "[V]", (tx - 2, 8), 11, (130, 150, 170), bold=True, anchor="topright")

        strip_h = 22
        area = pygame.Rect(0, PANE_HDR, w, h - PANE_HDR - strip_h)
        pygame.draw.rect(surf, (14, 18, 26), (0, h - strip_h, w, strip_h))

        def prep(img):
            if img is None:
                return None
            return self.fit_array(img, area.w, area.h)

        a = prep(fpv)
        b = prep(chase) if self.chase_available else None
        blend = self.rgb_blend
        shown = None
        if blend <= 0.01 or b is None:
            shown = a
            mode_fpv = True
        elif blend >= 0.99 or a is None:
            shown = b
            mode_fpv = False
        else:
            mixed = cv2.addWeighted(a[0], 1.0 - blend, b[0], blend, 0)
            shown = (mixed, b[1], b[2], b[3], b[4])
            mode_fpv = blend < 0.5
        if self.rgb_view_target == 1 and not self.chase_available:
            mode_fpv = True

        if shown is not None:
            canvas, ox, oy, dw, dh = shown
            surf.blit(self.array_to_surface(canvas), (area.x, area.y))
            ox += area.x
            oy += area.y
            cx, cy = ox + dw // 2, oy + dh // 2
            if mode_fpv:
                # Bounding box cua nguoi (cung he quy chieu voi camera nhiet)
                if fpv is not None and dets:
                    sc = dw / float(fpv.shape[1])
                    for d in dets[:3]:
                        x0, y0, x1, y1 = d["bbox"]
                        pygame.draw.rect(surf, (255, 40, 40), (ox + int(x0 * sc), oy + int(y0 * sc), max(6, int((x1 - x0) * sc)), max(6, int((y1 - y0) * sc))), 2)
                pygame.draw.circle(surf, (0, 255, 120), (cx, cy), 22, 1)
                pygame.draw.line(surf, (0, 255, 120), (cx - 30, cy), (cx + 30, cy), 1)
                pygame.draw.line(surf, (0, 255, 120), (cx, cy - 30), (cx, cy + 30), 1)
                self.text(surf, f"H:{alt:4.1f}m", (ox + dw - 8, cy - 8), 12, (0, 255, 120), mono=True, anchor="topright")
                self.text(surf, f"V:{speed:3.1f}m/s", (ox + 8, cy - 8), 12, (0, 255, 120), mono=True)
            else:
                # Marker UAV trung tam khung chase
                self.text(surf, "CHASE CAM - BÁM ĐUÔI UAV", (ox + 10, oy + 8), 12, (255, 220, 100), bold=True)
                self.text(surf, f"H:{alt:4.1f}m  V:{speed:3.1f}m/s", (ox + dw - 10, oy + 8), 12, (0, 255, 120), mono=True, anchor="topright")
        else:
            self.text(surf, "[CHỜ TÍN HIỆU CAMERA RGB...]", (w // 2, h // 2), 14, (0, 220, 120), anchor="center")

        tilt = abs(self.gimbal_pitch)
        if tilt < 15:
            cam_mode_str = "NGANG (0°)"
        elif tilt > 75:
            cam_mode_str = "TỪ TRÊN XUỐNG (-90°)"
        else:
            cam_mode_str = f"CHÉO ({self.gimbal_pitch:.0f}°)"
        if self.rgb_view_target == 1 and not self.chase_available:
            msg = "CHASE CAM KHÔNG KHẢ DỤNG - KHỞI ĐỘNG LẠI UNREAL ĐỂ NẠP settings.json MỚI"
            col = (255, 140, 60)
        elif mode_fpv:
            msg = f"FPV BỤNG DRONE | GÓC NHÌN: {cam_mode_str} | FOV {THERMAL_FOV_DEG:.0f}°"
            col = (170, 255, 215)
        else:
            msg = "NGOẠI CẢNH: camera 5m sau đuôi, nghiêng -15° (cố định theo thân UAV)"
            col = (255, 230, 150)
        self.text(surf, msg, (10, h - strip_h + 4), 11, col, mono=False)
        pygame.draw.rect(surf, accent, (0, 0, w, h), 1)

    def draw_fov_cone(self, surf, cx, cy, scale, yaw_rad, alt, gimbal_pitch, max_px_range):
        """Hinh non (hinh thang tren mat dat) mo ta vung nhin cua Camera Nhiet tren ban do BEV."""
        alt_eff = max(alt, 2.0)
        hfov = THERMAL_FOV_DEG
        vfov = 2.0 * math.degrees(math.atan(math.tan(math.radians(hfov / 2.0)) * self.cam_h / float(self.cam_w)))
        depress = -gimbal_pitch
        a_near = min(89.9, depress + vfov / 2.0)
        a_far = depress - vfov / 2.0
        d_near = alt_eff / math.tan(math.radians(max(a_near, 0.5)))
        d_far = alt_eff / math.tan(math.radians(a_far)) if a_far > 1.0 else max_px_range * 1.5
        d_far = min(d_far, max_px_range * 1.5)
        d_near = min(d_near, d_far)
        tan_h = math.tan(math.radians(hfov / 2.0))
        hn = math.hypot(alt_eff, d_near) * tan_h
        hf = math.hypot(alt_eff, d_far) * tan_h
        fwd = (math.cos(yaw_rad), math.sin(yaw_rad))
        rgt = (-math.sin(yaw_rad), math.cos(yaw_rad))

        def to_px(f, r):
            dx = f * fwd[0] + r * rgt[0]
            dy = f * fwd[1] + r * rgt[1]
            return (cx + dy * scale, cy - dx * scale)

        poly = [to_px(d_near, -hn), to_px(d_near, hn), to_px(d_far, hf), to_px(d_far, -hf)]
        ov = pygame.Surface(surf.get_size(), pygame.SRCALPHA)
        pygame.draw.polygon(ov, (255, 170, 50, 46), poly)
        pygame.draw.polygon(ov, (255, 190, 80, 200), poly, 2)
        pygame.draw.line(ov, (255, 190, 80, 120), (cx, cy), to_px(d_far, 0), 1)
        surf.blit(ov, (0, 0))

    def render_lidar_radar_pane(self):
        surf = self.panes["lidar"]
        w, h = surf.get_size()
        surf.fill((8, 12, 18))

        with self.lock:
            pts = self.lidar_pts
            pts_count = self.telemetry["points_count"]
            uav_x = self.telemetry["x"]
            uav_y = self.telemetry["y"]
            uav_z = self.telemetry["z"]
            alt = self.telemetry["alt"]
            yaw_deg = self.telemetry["yaw"]
            trail_copy = list(self.flight_trail)
            markers_copy = list(self.marked_persons)
            tgt = self.current_roi_target()

        max_range = self.lidar_ranges[self.lidar_range_idx]
        cmap_name, colormap_id = self.lidar_colormaps[self.lidar_cmap_idx]
        accent = (0, 200, 240)

        cbar_w = 52
        plot_w = w - cbar_w
        plot_h = h - PANE_HDR
        cx = plot_w // 2
        cy = PANE_HDR + plot_h // 2
        radius = min(cx - 12, plot_h // 2 - 10)
        scale = radius / max_range

        # Luoi dong tam
        for frac in (0.25, 0.5, 0.75, 1.0):
            pygame.draw.circle(surf, (22, 32, 44) if frac < 1 else (34, 50, 68), (cx, cy), int(radius * frac), 1)
        pygame.draw.line(surf, (22, 32, 44), (cx - radius, cy), (cx + radius, cy), 1)
        pygame.draw.line(surf, (22, 32, 44), (cx, cy - radius), (cx, cy + radius), 1)
        for frac in (0.5, 1.0):
            self.text(surf, f"{max_range * frac:.0f}m", (cx + int(radius * frac) - 4, cy + 3), 10, (80, 115, 150), mono=True, anchor="topright")
        self.text(surf, "B", (cx, cy - radius - 2), 10, (0, 200, 255), bold=True, anchor="midbottom")

        # Diem LiDAR (the gioi tuyet doi NED, Bac o tren) -> toa do tuong doi so voi UAV
        max_elev_disp = 10.0
        if len(pts) > 0 and self.lidar_enabled:
            rel = pts - np.array([uav_x, uav_y, uav_z], dtype=np.float32)
            dists = np.hypot(rel[:, 0], rel[:, 1])
            in_range = dists <= max_range
            filtered = rel[in_range]

            if len(filtered) > 0:
                # Gioi han toi da 8.000 diem tren Radar BEV de ve 60 FPS cuc muot
                if len(filtered) > 8000:
                    sub_step = len(filtered) // 8000
                    filtered = filtered[::sub_step]

                pxs = (cx + filtered[:, 1] * scale).astype(int)
                pys = (cy - filtered[:, 0] * scale).astype(int)

                valid = (pxs >= 1) & (pxs < plot_w - 2) & (pys >= PANE_HDR + 1) & (pys < h - 3)
                v_pxs = pxs[valid]
                v_pys = pys[valid]

                if len(v_pxs) > 0:
                    z_vals = filtered[valid, 2]
                    z_ground = np.percentile(z_vals, 95)
                    elev = np.clip(z_ground - z_vals, 0.0, 60.0)
                    max_elev_disp = max(4.0, float(np.percentile(elev, 98)))
                    norm_elev = np.clip(elev / max_elev_disp, 0.0, 1.0)

                    color_indices = (norm_elev * 255.0).astype(np.uint8)
                    bgr_pts = cv2.applyColorMap(color_indices, colormap_id)
                    v_cols = cv2.cvtColor(bgr_pts, cv2.COLOR_BGR2RGB).reshape(-1, 3)

                    # Ve vector hoa bang numpy (moi diem 2x2 pixel)
                    pix = pygame.surfarray.pixels3d(surf)
                    for ox, oy in ((0, 0), (1, 0), (0, 1), (1, 1)):
                        pix[v_pxs + ox, v_pys + oy] = v_cols
                    del pix

        # Hinh non FOV cua Camera Nhiet
        self.draw_fov_cone(surf, cx, cy, scale, math.radians(yaw_deg), alt, self.gimbal_pitch, max_range)

        if len(trail_copy) > 1:
            trail_px = []
            for tx, ty in trail_copy[-120:]:
                px_t = int(cx + (ty - uav_y) * scale)
                py_t = int(cy - (tx - uav_x) * scale)
                if 1 <= px_t < plot_w - 1 and PANE_HDR + 1 <= py_t < h - 1:
                    trail_px.append((px_t, py_t))
            if len(trail_px) > 1:
                pygame.draw.lines(surf, (0, 160, 220), False, trail_px, 1)

        for m in markers_copy:
            mx = int(cx + (m["tgt_y"] - uav_y) * scale)
            my = int(cy - (m["tgt_x"] - uav_x) * scale)
            if 0 <= mx < plot_w and PANE_HDR <= my < h:
                pygame.draw.circle(surf, (255, 30, 30), (mx, my), 6, 2)
                self.text(surf, f"#{m['id']}", (mx + 8, my - 6), 10, (255, 90, 90), mono=True)

        if tgt is not None:
            mx = int(cx + (tgt[1] - uav_y) * scale)
            my = int(cy - (tgt[0] - uav_x) * scale)
            if 0 <= mx < plot_w and PANE_HDR <= my < h:
                rr = max(6, int(self.roi_radius * scale))
                pygame.draw.circle(surf, (0, 255, 255), (mx, my), rr + 4, 1)
                pygame.draw.line(surf, (0, 255, 255), (mx - 10, my), (mx + 10, my), 1)
                pygame.draw.line(surf, (0, 255, 255), (mx, my - 10), (mx, my + 10), 1)
                self.text(surf, "ROI", (mx + rr + 6, my - 6), 10, (0, 255, 255), bold=True, mono=True)

        yaw_rad = math.radians(yaw_deg)
        head = (cx + int(11 * math.sin(yaw_rad)), cy - int(11 * math.cos(yaw_rad)))
        left = (cx + int(7 * math.sin(yaw_rad + 2.5)), cy - int(7 * math.cos(yaw_rad + 2.5)))
        right = (cx + int(7 * math.sin(yaw_rad - 2.5)), cy - int(7 * math.cos(yaw_rad - 2.5)))
        pygame.draw.polygon(surf, (0, 255, 200), [head, left, right])

        # Thanh mau cao do
        cb_x = w - cbar_w + 6
        cb_y = PANE_HDR + 12
        cb_h = h - cb_y - 12
        cb_w = 12
        grad_vals = np.linspace(255, 0, cb_h, dtype=np.uint8).reshape(-1, 1)
        grad_rgb = cv2.cvtColor(cv2.applyColorMap(grad_vals, colormap_id), cv2.COLOR_BGR2RGB)
        grad_bar = cv2.resize(grad_rgb, (cb_w, cb_h))
        surf.blit(pygame.surfarray.make_surface(grad_bar.swapaxes(0, 1)), (cb_x, cb_y))
        pygame.draw.rect(surf, (150, 180, 210), (cb_x, cb_y, cb_w, cb_h), 1)
        self.text(surf, f"{max_elev_disp:2.0f}m", (cb_x + cb_w + 3, cb_y - 2), 10, (255, 90, 90), bold=True, mono=True)
        self.text(surf, f"{max_elev_disp * 0.5:2.0f}m", (cb_x + cb_w + 3, cb_y + cb_h // 2 - 6), 10, (255, 230, 80), bold=True, mono=True)
        self.text(surf, "0m", (cb_x + cb_w + 3, cb_y + cb_h - 11), 10, (80, 160, 255), bold=True, mono=True)

        self.draw_header(surf, "BẢN ĐỒ RADAR LIDAR TOÀN CẢNH", accent,
                         f"{self.lidar_model} | 360°x45° | THANG: {max_range:.0f}m")
        self.text(surf, f"{pts_count:,} ĐIỂM | {cmap_name.split(' ')[0]} | {self.lidar_hz:.0f}Hz", (10, h - 8), 10, (110, 150, 185), mono=True, anchor="bottomleft")
        self.text(surf, "VÙNG CAM NHIỆT (FOV)", (plot_w - 8, h - 8), 10, (255, 190, 80), bold=True, anchor="bottomright")
        pygame.draw.rect(surf, accent, (0, 0, w, h), 1)

    def render_roi_pane(self):
        """O 3D ROI: point cloud ban kinh 3-5m quanh nan nhan, hien thi Isometric (splat = upsampling truc quan)."""
        surf = self.panes["roi"]
        w, h = surf.get_size()
        surf.fill((9, 11, 17))
        accent = (200, 120, 255)

        with self.lock:
            roi = self.roi_pts
            center = None if self.roi_center is None else self.roi_center.copy()
            uav = np.array([self.telemetry["x"], self.telemetry["y"], self.telemetry["z"]], dtype=np.float32)

        R = self.roi_radius
        status = "LOCK" if self.target_locked else "AUTO"
        self.draw_header(surf, "KHÔNG GIAN 3D XUNG QUANH MỤC TIÊU", accent, f"(3D ROI POINT CLOUD) | R={R:.0f}m")
        self.text(surf, status, (w - 10, 7), 12, (255, 90, 90) if self.target_locked else (0, 230, 160), bold=True, anchor="topright")

        plot_h = h - PANE_HDR
        cx = w // 2
        cy = PANE_HDR + int(plot_h * 0.60)
        S = min(0.46 * w / (1.732 * R), 0.537 * plot_h / R, 0.60 * plot_h / (0.707 * R + 4.0))
        c30, s30 = 0.8660254, 0.5
        az = math.radians(self.roi_az)
        ca, sa = math.cos(az), math.sin(az)

        def project(xr, yr, hh):
            return cx + (xr - yr) * c30 * S, cy + (xr + yr) * s30 * S - hh * S

        def rot(dx, dy):
            return dx * ca - dy * sa, dx * sa + dy * ca

        if center is None:
            self.text(surf, "CHƯA CÓ MỤC TIÊU", (w // 2, h // 2 - 22), 18, (160, 140, 190), bold=True, anchor="center")
            self.text(surf, "Chờ phát hiện người trên ảnh nhiệt, hoặc bấm [T] / Back để khóa mục tiêu", (w // 2, h // 2 + 6), 12, (120, 125, 150), anchor="center")
            self.text(surf, "Vùng Point Cloud bán kính 3m-5m sẽ tự động được cắt & hiển thị tại đây", (w // 2, h // 2 + 26), 12, (120, 125, 150), anchor="center")
            pygame.draw.rect(surf, accent, (0, 0, w, h), 1)
            return

        # Luoi mat dat: vong tron R, R/2 va truc
        theta = np.linspace(0, 2 * math.pi, 64)
        for frac, col in ((1.0, (60, 70, 100)), (0.5, (40, 48, 70))):
            ring = [project(*rot(R * frac * math.cos(t), R * frac * math.sin(t)), 0.0) for t in theta]
            pygame.draw.lines(surf, col, True, ring, 1)
        for ang in range(0, 360, 90):
            a = math.radians(ang)
            p0 = project(*rot(0.0, 0.0), 0.0)
            p1 = project(*rot(R * math.cos(a), R * math.sin(a)), 0.0)
            pygame.draw.line(surf, (38, 46, 68), p0, p1, 1)
        nx, ny = rot(R * 1.12, 0.0)           # Huong Bac (+x NED)
        nn = project(nx, ny, 0.0)
        self.text(surf, "B", (int(nn[0]), int(nn[1])), 11, (0, 200, 255), bold=True, anchor="center")

        n_pts = len(roi)
        if n_pts > 0:
            pts_draw = roi
            if n_pts > 3500:
                pts_draw = roi[::(n_pts // 3500)]
            d = pts_draw - center
            xr, yr = rot(d[:, 0], d[:, 1])
            hh = -d[:, 2]                      # chieu cao so voi mat dat tai muc tieu (NED: z xuong)
            ground = float(np.percentile(hh, 10))
            hrel = np.clip(hh - ground, 0.0, 4.0)
            depth = xr + yr
            order = np.argsort(depth)          # xa truoc -> gan sau (painter)
            xr, yr, hh, hrel = xr[order], yr[order], hh[order], hrel[order]
            sx = (cx + (xr - yr) * c30 * S).astype(int)
            sy = (cy + (xr + yr) * s30 * S - hh * S).astype(int)
            cidx = (np.clip(hrel / 3.0, 0.0, 1.0) * 255.0).astype(np.uint8)
            cols = cv2.cvtColor(cv2.applyColorMap(cidx, self.lidar_colormaps[self.lidar_cmap_idx][1]), cv2.COLOR_BGR2RGB).reshape(-1, 3)
            offs = [(0, 0), (1, 0), (0, 1), (1, 1)]
            pix = pygame.surfarray.pixels3d(surf)
            for ox, oy in offs:
                xx, yy = sx + ox - 1, sy + oy - 1
                ok = (xx >= 1) & (xx < w - 1) & (yy >= PANE_HDR + 1) & (yy < h - 1)
                pix[xx[ok], yy[ok]] = cols[ok]
            del pix

        # Marker muc tieu: coc do cao 1.8m (nguoi) + vong tron
        base = project(0.0, 0.0, 0.0)
        top = project(0.0, 0.0, 1.8)
        pygame.draw.line(surf, (255, 50, 50), base, top, 3)
        pygame.draw.circle(surf, (255, 50, 50), (int(top[0]), int(top[1])), 5, 2)
        pygame.draw.circle(surf, (255, 255, 255), (int(base[0]), int(base[1])), 3)
        self.text(surf, "MỤC TIÊU", (int(top[0]) + 10, int(top[1]) - 8), 11, (255, 110, 110), bold=True)

        dist = float(np.linalg.norm(center[:2] - uav[:2]))
        self.text(surf, f"X:{center[0]:.1f} Y:{center[1]:.1f} Z:{center[2]:.1f}  |  CỰ LY {dist:.0f}m", (10, h - 26), 11, (210, 200, 235), mono=True, anchor="bottomleft")
        self.text(surf, f"{n_pts:,} ĐIỂM | ISOMETRIC {self.roi_az:.0f}° | SPLAT x{'3' if n_pts < 1500 else '2'}", (10, h - 8), 10, (140, 130, 170), mono=True, anchor="bottomleft")
        self.text(surf, "[Z] 3/5m  [,][.] xoay  [O] tự quay", (w - 8, h - 8), 10, (120, 125, 150), anchor="bottomright")
        if n_pts == 0:
            self.text(surf, "CHƯA CÓ ĐIỂM LIDAR TRONG VÙNG ROI", (w // 2, PANE_HDR + 22), 12, (255, 170, 80), bold=True, anchor="center")
        pygame.draw.rect(surf, accent, (0, 0, w, h), 1)

    def render_sidebar(self):
        surf = self.panes["side"]
        w, h = surf.get_size()
        surf.fill((12, 16, 24))

        with self.lock:
            conn = self.telemetry["connected"]
            status_str = self.telemetry["status"]
            alt = self.telemetry["alt"]
            spd = self.telemetry["speed"]
            bat = self.telemetry["battery"]
            yaw = self.telemetry["yaw"]
            pitch = self.telemetry["pitch"]
            roll = self.telemetry["roll"]
            px = self.telemetry["x"]
            py = self.telemetry["y"]
            pz = self.telemetry["z"]
            ftime = self.telemetry["flight_time"]
            markers_copy = list(self.marked_persons)

        accent = (255, 200, 50)
        pygame.draw.rect(surf, (17, 23, 33), (0, 0, w, PANE_HDR))
        pygame.draw.rect(surf, accent, (0, PANE_HDR - 2, w, 2))
        self.text(surf, "BẢNG ĐIỀU KHIỂN & TELEMETRY", (10, 6), 12, accent, bold=True)

        def card(y, ch, title, col):
            pygame.draw.rect(surf, (18, 24, 35), (8, y, w - 16, ch), border_radius=6)
            pygame.draw.rect(surf, col, (8, y + 8, 3, 14), border_radius=2)
            self.text(surf, title, (18, y + 6), 11, col, bold=True)
            return y + 28

        # ---- 1. KET NOI & DIEU KHIEN ----
        y = PANE_HDR + 8
        ch = 158
        iy = card(y, ch, "KẾT NỐI & ĐIỀU KHIỂN", (100, 200, 255))
        has_ctrl = (self.controller is not None or self.joystick is not None)
        pygame.draw.circle(surf, (0, 255, 150) if conn else (255, 70, 70), (24, iy + 8), 5)
        self.text(surf, f"AirSim: {status_str}", (36, iy), 12, (0, 255, 170) if conn else (255, 100, 100), bold=True)
        pygame.draw.circle(surf, (0, 255, 150) if has_ctrl else (255, 150, 50), (24, iy + 28), 5)
        self.text(surf, ("Tay cầm: " if has_ctrl else "Bàn phím: ") + self.controller_name[:26], (36, iy + 20), 12,
                  (0, 255, 170) if has_ctrl else (255, 170, 80), bold=True)

        btn_states = [
            ("RT↑", self.cmd_vz < -0.2, (50, 255, 50)),
            ("LT↓", self.cmd_vz > 0.2, (255, 120, 50)),
            ("A:Boost", self.is_boost, (255, 70, 70)),
            ("B:Đèn", self.searchlight_on, (255, 255, 0)),
            ("X:LiDAR", self.lidar_enabled, (0, 200, 255)),
            ("Y:Ping", (time.time() - self.last_ping_time) < 1.0, (255, 40, 40)),
            ("Hover", self.is_hover, (255, 180, 0)),
            ("Khóa", self.target_locked, (255, 90, 160)),
        ]
        bx, by = 16, iy + 44
        for name, active, col in btn_states:
            tw = self.font(10, True).size(name)[0] + 14
            if bx + tw > w - 12:
                bx = 16
                by += 24
            pygame.draw.rect(surf, col if active else (36, 44, 58), (bx, by, tw, 20), border_radius=4)
            self.text(surf, name, (bx + tw // 2, by + 10), 10, (0, 0, 0) if active else (175, 188, 202), bold=True, anchor="center")
            bx += tw + 5
        self.text(surf, f"Vx {self.cmd_vx:+5.1f}  Vy {self.cmd_vy:+5.1f}  Vz {self.cmd_vz:+5.1f}  Yaw {self.cmd_yaw_rate:+4.0f}",
                  (16, iy + 96), 11, (200, 220, 240), mono=True)
        hb_col = (0, 255, 170) if self.cmd_hz > 12 else (255, 120, 60)
        self.text(surf, f"HEARTBEAT LỆNH: {self.cmd_hz:4.1f} Hz", (16, iy + 114), 11, hb_col, bold=True, mono=True)

        # ---- 2. GIMBAL ----
        y += ch + 8
        ch = 146
        iy = card(y, ch, "GIMBAL - GÓC CAMERA (NHIỆT + RGB)", (255, 220, 100))
        self.text(surf, f"{self.gimbal_pitch:5.1f}°", (16, iy - 2), 24, (255, 230, 120), bold=True, mono=True)
        self.text(surf, "Góc Cam", (110, iy + 12), 11, (170, 180, 195))
        bar_x, bar_w = 18, w - 52
        bar_y = iy + 38
        pygame.draw.rect(surf, (32, 40, 54), (bar_x, bar_y, bar_w, 8), border_radius=4)
        frac = (self.gimbal_pitch + 90.0) / 90.0          # -90 -> 0, 0 -> 1
        pygame.draw.rect(surf, (255, 200, 60), (bar_x, bar_y, int(bar_w * frac), 8), border_radius=4)
        pygame.draw.circle(surf, (255, 255, 255), (bar_x + int(bar_w * frac), bar_y + 4), 8)
        for val, lab in ((-90, "-90°"), (-45, "-45°"), (0, "0°")):
            tx = bar_x + int(bar_w * (val + 90.0) / 90.0)
            pygame.draw.line(surf, (110, 120, 140), (tx, bar_y + 10), (tx, bar_y + 15), 1)
            self.text(surf, lab, (tx, bar_y + 17), 10, (140, 150, 170), mono=True, anchor="midtop")
        presets = [("1: LÊN 0°", 0.0), ("2: NGHIÊNG -45°", -45.0), ("3: XUỐNG -90°", -90.0)]
        bx = 14
        for lab, val in presets:
            tw = self.font(9, True).size(lab)[0] + 10
            active = abs(self.gimbal_pitch - val) < 4.0
            pygame.draw.rect(surf, (255, 200, 60) if active else (36, 44, 58), (bx, iy + 84, tw, 18), border_radius=4)
            self.text(surf, lab, (bx + tw // 2, iy + 93), 9, (0, 0, 0) if active else (175, 188, 202), bold=True, anchor="center")
            bx += tw + 4
        synced = (self.gimbal_applied is not None and abs(self.gimbal_applied - self.gimbal_pitch) < 0.3)
        self.text(surf, f"{'ĐÃ ĐỒNG BỘ' if synced else 'ĐANG ĐỒNG BỘ...'} AirSim | LiDAR cố định {self.hw['lidar']['Pitch']:.0f}°",
                  (16, iy + 108), 10, (0, 230, 160) if synced else (255, 170, 80))

        # ---- 3. THONG SO BAY ----
        y += ch + 8
        ch = 196
        iy = card(y, ch, "THÔNG SỐ BAY (TELEMETRY)", (0, 220, 170))
        c1, c2 = 16, w // 2 + 4
        rows = [
            (c1, "ĐỘ CAO", f"{alt:6.1f} m", (0, 255, 180)), (c2, "TỐC ĐỘ", f"{spd:5.1f} m/s", (0, 255, 180)),
            (c1, "YAW", f"{yaw:6.1f}°", (0, 255, 180)), (c2, "PITCH", f"{pitch:5.1f}°", (0, 255, 180)),
            (c1, "GPS X", f"{px:6.1f} m", (225, 225, 225)), (c2, "GPS Y", f"{py:5.1f} m", (225, 225, 225)),
            (c1, "GPS Z", f"{pz:6.1f} m", (225, 225, 225)), (c2, "ROLL", f"{roll:5.1f}°", (225, 225, 225)),
            (c1, "PIN", f"{bat:5.0f} %", (255, 220, 90) if bat > 25 else (255, 80, 80)),
            (c2, "BAY", f"{int(ftime // 60):02d}:{int(ftime % 60):02d}", (225, 225, 225)),
            (c1, "FPS UI", f"{self.ui_fps:5.0f}", (190, 200, 255)), (c2, "ẢNH", f"{self.img_hz:4.0f} Hz", (190, 200, 255)),
        ]
        for i, (cx_, lab, val, col) in enumerate(rows):
            ry = iy + (i // 2) * 24
            self.text(surf, lab, (cx_, ry + 2), 10, (125, 140, 160), bold=True)
            self.text(surf, val, (cx_ + 58 if cx_ == c1 else cx_ + 50, ry), 13, col, bold=True, mono=True)

        # ---- 4. MUC TIEU ----
        y += ch + 8
        ch = h - y - 8
        iy = card(y, ch, f"MỤC TIÊU ĐÃ ĐÁNH DẤU ({len(markers_copy)} NGƯỜI)", (255, 110, 110))
        if not markers_copy:
            self.text(surf, "Chưa có mục tiêu.", (16, iy), 12, (140, 145, 155))
            self.text(surf, "Bấm [Y] để đánh dấu vị trí!", (16, iy + 18), 12, (140, 145, 155))
        else:
            max_rows = max(1, (ch - 40) // 38)
            ry = iy
            for m in reversed(markers_copy[-max_rows:]):
                pygame.draw.rect(surf, (28, 32, 44), (14, ry, w - 28, 34), border_radius=4)
                self.text(surf, f"#{m['id']}", (20, ry + 3), 13, (255, 110, 110), bold=True, mono=True)
                self.text(surf, m["time"], (w - 20, ry + 4), 10, (140, 150, 165), mono=True, anchor="topright")
                self.text(surf, f"X:{m['tgt_x']:6.1f} Y:{m['tgt_y']:6.1f}  {m['dist']:4.0f}m", (20, ry + 19), 11, (255, 175, 175), mono=True)
                ry += 38

        pygame.draw.rect(surf, accent, (0, 0, w, h), 1)

    # -----------------------------------------------------------------------
    # 7. CUA SO / VONG LAP CHINH CUA GCS
    # -----------------------------------------------------------------------
    def set_display(self, fullscreen):
        self.fullscreen = fullscreen
        if fullscreen:
            self.screen = pygame.display.set_mode((SCREEN_W, SCREEN_H), pygame.FULLSCREEN)
        else:
            self.screen = pygame.display.set_mode((self.win_w, self.win_h), pygame.RESIZABLE)

    def present(self):
        screen = pygame.display.get_surface()
        w, h = screen.get_size()
        if (w, h) == (CANVAS_W, CANVAS_H):
            screen.blit(self.canvas, (0, 0))
        else:
            s = min(w / float(CANVAS_W), h / float(CANVAS_H))
            dw, dh = max(1, int(CANVAS_W * s)), max(1, int(CANVAS_H * s))
            scaled = pygame.transform.smoothscale(self.canvas, (dw, dh))
            screen.fill((0, 0, 0))
            screen.blit(scaled, ((w - dw) // 2, (h - dh) // 2))
        pygame.display.flip()

    def build_surfaces(self):
        self.canvas = pygame.Surface((CANVAS_W, CANVAS_H)).convert()
        self.panes = {
            "thermal": pygame.Surface((PANE_W, PANE_H)).convert(),
            "rgb": pygame.Surface((PANE_W, PANE_H)).convert(),
            "lidar": pygame.Surface((PANE_W, PANE_H)).convert(),
            "roi": pygame.Surface((PANE_W, PANE_H)).convert(),
            "side": pygame.Surface((SIDE_W, DECK_H)).convert(),
        }

    def draw_dashboard(self, clock):
        cv = self.canvas
        w, h = CANVAS_W, CANVAS_H
        cv.fill((8, 11, 16))

        # --- Header ---
        pygame.draw.rect(cv, (16, 22, 32), (0, 0, w, HEADER_H))
        pygame.draw.rect(cv, (255, 190, 60), (0, HEADER_H - 2, w, 2))
        self.text(cv, "HỆ THỐNG UAV GCS TÌM KIẾM CỨU HỘ", (12, 8), 14, (255, 255, 255), bold=True)
        self.text(cv, "[UE 4.27 AFRICA]", (322, 10), 11, (255, 200, 80), bold=True)

        bx = 470
        has_ctrl = (self.controller is not None or self.joystick is not None)
        js_color = (0, 255, 180) if has_ctrl else (255, 140, 60)
        r = self.text(cv, f"[{self.controller_name[:22]}]", (bx, 11), 11, js_color, bold=True)
        bx = r.right + 16
        mode_str = "TURBO BOOST" if self.is_boost else ("HOVER (TẠM DỪNG)" if self.is_hover else "TÌM KIẾM CỨU HỘ")
        mode_col = (255, 70, 70) if self.is_boost else ((255, 180, 0) if self.is_hover else (0, 220, 255))
        r = self.text(cv, f"[{mode_str}]", (bx, 11), 11, mode_col, bold=True)
        bx = r.right + 16
        light_col = (255, 255, 0) if self.searchlight_on else (120, 120, 120)
        self.text(cv, f"[ĐÈN: {'BẬT' if self.searchlight_on else 'TẮT'}]", (bx, 11), 11, light_col, bold=True)

        with self.lock:
            h_alt = self.telemetry["alt"]
            h_spd = self.telemetry["speed"]
            h_bat = self.telemetry["battery"]
        self.text(cv, f"ĐỘ CAO: {h_alt:4.1f}m | TỐC ĐỘ: {h_spd:4.1f}m/s | PIN: {h_bat:3.0f}% | MỤC TIÊU: {len(self.marked_persons)} | FPS: {self.ui_fps:2.0f}  [F11] Toàn màn hình",
                  (w - 12, 11), 11, (200, 220, 240), bold=True, anchor="topright")

        # --- Luoi 2x2 + Sidebar ---
        cv.blit(self.panes["thermal"], (0, DECK_Y))
        cv.blit(self.panes["rgb"], (PANE_W, DECK_Y))
        cv.blit(self.panes["lidar"], (0, DECK_Y + PANE_H))
        cv.blit(self.panes["roi"], (PANE_W, DECK_Y + PANE_H))
        cv.blit(self.panes["side"], (SIDE_X, DECK_Y))

        if (time.time() - self.last_ping_time) < 4.5:
            bw = 760
            banner_y = DECK_Y + PANE_HDR + 8
            bxx = (PANE_W * 2 - bw) // 2
            pygame.draw.rect(cv, (180, 20, 20), (bxx, banner_y, bw, 34), border_radius=6)
            pygame.draw.rect(cv, (255, 255, 0), (bxx, banner_y, bw, 34), 2, border_radius=6)
            self.text(cv, self.last_ping_msg, (bxx + bw // 2, banner_y + 17), 12, (255, 255, 255), bold=True, anchor="center")

        footer_y = h - FOOTER_H
        pygame.draw.rect(cv, (12, 18, 26), (0, footer_y, w, FOOTER_H))
        help_str = ("[WASD] Bay  [Q/E] Yaw  [Space/Shift] Độ cao  [↑↓ | 1·2·3] Góc cam  [V] FPV/Chase  [R] Màu nhiệt  "
                    "[T] Khóa mục tiêu  [Y] Đánh dấu  [Z] ROI 3/5m  [F10] Ẩn Unreal  [F11] Toàn màn hình  [F1] Trợ giúp")
        self.text(cv, help_str, (14, footer_y + 7), 11, (160, 210, 240), bold=True)

        if self.show_help:
            self.draw_help_overlay()

    def draw_help_overlay(self):
        w, h = CANVAS_W, CANVAS_H
        ov = pygame.Surface((w, h), pygame.SRCALPHA)
        ov.fill((0, 0, 0, 215))
        box = pygame.Rect(w // 2 - 480, 30, 960, h - 60)
        pygame.draw.rect(ov, (25, 35, 50), box, border_radius=8)
        pygame.draw.rect(ov, (0, 220, 255), box, 2, border_radius=8)
        t = self.font(18, True).render("BẢNG ÁNH XẠ NÚT BẤM TAY CẦM XBOX VÀ BÀN PHÍM", True, (0, 255, 220))
        ov.blit(t, (w // 2 - t.get_width() // 2, 44))
        rows = [
            ("Cần Phải (Lên / Xuống) | ↑ ↓", "Chỉnh góc camera liên tục (0° ngang ... -90° nhìn thẳng xuống)"),
            ("R3 / G  |  phím 1 · 2 · 3", "Góc cam: R3/G xoay vòng 0° → -45° → -90°; 1/2/3 chọn thẳng 0° / -45° / -90°"),
            ("LB / V", "Chuyển mượt giữa Cam FPV bụng drone và Chase Cam bám đuôi"),
            ("RB / R", "Đổi bảng màu nhiệt FLIR (White-Hot / Black-Hot / Ironbow / Rainbow)"),
            ("Back / T", "KHÓA / MỞ KHÓA mục tiêu cho ô 3D ROI (mặc định tự bám người phát hiện được)"),
            ("Z  |  , .  |  O", "Bán kính ROI 3m ↔ 5m | xoay góc Isometric | tự quay"),
            ("Nút Y / Y", "ĐÁNH DẤU VỊ TRÍ NGƯỜI (lưu tọa độ 3D, ảnh RGB + nhiệt, CSV)"),
            ("Cần Trái / WASD", "Điều khiển hướng bay (Lên: tiến, Xuống: lùi, Trái/Phải: sang ngang)"),
            ("Cần Phải (Trái/Phải) / Q,E", "Xoay hướng đầu UAV (Yaw)"),
            ("RT / Space   |   LT / Shift", "Tăng độ cao   |   Giảm độ cao"),
            ("Nút A / Tab", "Turbo Boost 14 m/s"),
            ("Nút X / X   |   Phím C", "Thang LiDAR 30m → 60m → 120m   |   Đổi dải màu LiDAR"),
            ("Nút B / B", "Bật / Tắt đèn hỗ trợ tìm kiếm"),
            ("D-Pad / H", "Tạm dừng / tiếp tục (Hover)"),
            ("F10  |  F11", "Thu nhỏ Unreal xuống Taskbar  |  Bật/tắt toàn màn hình (Fullscreen)"),
            ("F1 / START  |  ESC", "Bật/tắt bảng trợ giúp này  |  Thoát (ESC thoát toàn màn hình trước)"),
        ]
        ry = 86
        for c1, c2 in rows:
            ov.blit(self.font(14, True).render(f"• {c1}", True, (255, 220, 100)), (w // 2 - 450, ry))
            ov.blit(self.font(12).render(c2, True, (230, 230, 230)), (w // 2 - 430, ry + 19))
            ry += 46
        self.canvas.blit(ov, (0, 0))

    def run(self):
        pygame.init()
        pygame.font.init()
        self.init_controller()
        if self.controller is None and self.joystick is None:
            print(">> [TAY CAM] Khong tim thay tay cam Xbox, dung ban phim WASD / Space / Shift / Y / B / X...", flush=True)

        self.set_display(False)
        pygame.display.set_caption("HE THONG UAV GCS TIM KIEM CUU HO [UE 4.27 AFRICA]")
        self.build_surfaces()
        print(f">> [GCS] Dashboard 16:9 {CANVAS_W}x{CANVAS_H} | cua so {self.win_w}x{self.win_h} | cau hinh cam bien: {self.hw['source']}", flush=True)

        threads = [
            threading.Thread(target=self.heartbeat_control_loop, daemon=True, name="heartbeat_ctrl"),
            threading.Thread(target=self.telemetry_gimbal_loop, daemon=True, name="telemetry_gimbal"),
            threading.Thread(target=self.image_loop, daemon=True, name="images"),
            threading.Thread(target=self.lidar_loop, daemon=True, name="lidar"),
            threading.Thread(target=self.window_management_loop, daemon=True, name="window_mgr"),
        ]
        for t in threads:
            t.start()

        clock = pygame.time.Clock()

        last_b = last_x = last_y = last_lb = last_rb = last_start = last_dpad = last_r3 = last_back = False

        print(">> [GCS STATION] He thong GCS da san sang!", flush=True)

        while self.running:
            dt = clock.get_time() / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.VIDEORESIZE and not self.fullscreen:
                    self.win_w, self.win_h = max(640, event.w), max(360, event.h)
                elif event.type in (pygame.JOYDEVICEADDED, getattr(pygame, "CONTROLLERDEVICEADDED", 0x650)):
                    self.init_controller()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        if self.fullscreen:
                            self.set_display(False)
                        else:
                            self.running = False
                    elif event.key == pygame.K_F10:
                        self.auto_arrange_windows()
                    elif event.key == pygame.K_F11:
                        self.set_display(not self.fullscreen)
                    elif event.key == pygame.K_y:
                        self.mark_person_waypoint()
                    elif event.key == pygame.K_b:
                        self.searchlight_on = not self.searchlight_on
                    elif event.key == pygame.K_x:
                        self.lidar_range_idx = (self.lidar_range_idx + 1) % len(self.lidar_ranges)
                    elif event.key == pygame.K_c:
                        self.lidar_cmap_idx = (self.lidar_cmap_idx + 1) % len(self.lidar_colormaps)
                    elif event.key == pygame.K_v:
                        self.toggle_rgb_view()
                    elif event.key == pygame.K_r:
                        self.palette_idx = (self.palette_idx + 1) % len(self.thermal_palettes)
                    elif event.key == pygame.K_h:
                        self.is_hover = not self.is_hover
                    elif event.key == pygame.K_t:
                        self.toggle_target_lock()
                    elif event.key == pygame.K_z:
                        self.roi_radius = 5.0 if self.roi_radius < 4.5 else 3.0
                    elif event.key == pygame.K_o:
                        self.roi_autorotate = not self.roi_autorotate
                    elif event.key == pygame.K_COMMA:
                        self.roi_az = (self.roi_az - 15.0) % 360.0
                    elif event.key == pygame.K_PERIOD:
                        self.roi_az = (self.roi_az + 15.0) % 360.0
                    elif event.key == pygame.K_F1:
                        self.show_help = not self.show_help
                    elif event.key == pygame.K_g:
                        self.cycle_gimbal()
                    elif event.key == pygame.K_1:
                        self.set_gimbal(0.0)
                    elif event.key == pygame.K_2:
                        self.set_gimbal(-45.0)
                    elif event.key == pygame.K_3:
                        self.set_gimbal(-90.0)

            vx = 0.0
            vy = 0.0
            vz = 0.0
            yaw_rate = 0.0

            # 1. Doc lenh dieu khien tu Tay cam (SDL2 Controller)
            if self.controller is not None:
                btn_a = self.controller.get_button(pygame.CONTROLLER_BUTTON_A)
                self.is_boost = bool(btn_a)
                curr_speed = self.boost_speed if self.is_boost else self.normal_speed

                raw_lx = self.controller.get_axis(pygame.CONTROLLER_AXIS_LEFTX) / 32767.0
                raw_ly = self.controller.get_axis(pygame.CONTROLLER_AXIS_LEFTY) / 32767.0
                if abs(raw_ly) > 0.15: vx = -raw_ly * curr_speed
                if abs(raw_lx) > 0.15: vy = raw_lx * curr_speed

                raw_rx = self.controller.get_axis(pygame.CONTROLLER_AXIS_RIGHTX) / 32767.0
                raw_ry = self.controller.get_axis(pygame.CONTROLLER_AXIS_RIGHTY) / 32767.0
                if abs(raw_rx) > 0.15: yaw_rate = raw_rx * 50.0

                if raw_ry < -0.15: self.set_gimbal(min(0.0, self.gimbal_pitch + abs(raw_ry) * 1.8))
                elif raw_ry > 0.15: self.set_gimbal(max(-90.0, self.gimbal_pitch - abs(raw_ry) * 1.8))

                btn_r3 = self.controller.get_button(pygame.CONTROLLER_BUTTON_RIGHTSTICK)
                if btn_r3 and not last_r3: self.cycle_gimbal()
                last_r3 = btn_r3

                raw_rt = max(0.0, self.controller.get_axis(pygame.CONTROLLER_AXIS_TRIGGERRIGHT) / 32767.0)
                raw_lt = max(0.0, self.controller.get_axis(pygame.CONTROLLER_AXIS_TRIGGERLEFT) / 32767.0)
                if raw_rt > 0.10: vz -= raw_rt * 4.0
                if raw_lt > 0.10: vz += raw_lt * 3.5

                btn_b = self.controller.get_button(pygame.CONTROLLER_BUTTON_B)
                btn_x = self.controller.get_button(pygame.CONTROLLER_BUTTON_X)
                btn_y = self.controller.get_button(pygame.CONTROLLER_BUTTON_Y)
                btn_lb = self.controller.get_button(pygame.CONTROLLER_BUTTON_LEFTSHOULDER)
                btn_rb = self.controller.get_button(pygame.CONTROLLER_BUTTON_RIGHTSHOULDER)
                btn_start = self.controller.get_button(pygame.CONTROLLER_BUTTON_START)
                btn_back = self.controller.get_button(pygame.CONTROLLER_BUTTON_BACK)
                btn_dpad = (self.controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_UP) or
                            self.controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_DOWN) or
                            self.controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_LEFT) or
                            self.controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_RIGHT))

                if btn_b and not last_b: self.searchlight_on = not self.searchlight_on
                last_b = btn_b
                if btn_x and not last_x: self.lidar_range_idx = (self.lidar_range_idx + 1) % len(self.lidar_ranges)
                last_x = btn_x
                if btn_y and not last_y: self.mark_person_waypoint()
                last_y = btn_y
                if btn_lb and not last_lb: self.toggle_rgb_view()
                last_lb = btn_lb
                if btn_rb and not last_rb: self.palette_idx = (self.palette_idx + 1) % len(self.thermal_palettes)
                last_rb = btn_rb
                if btn_dpad and not last_dpad: self.is_hover = not self.is_hover
                last_dpad = btn_dpad
                if btn_start and not last_start: self.show_help = not self.show_help
                last_start = btn_start
                if btn_back and not last_back: self.toggle_target_lock()
                last_back = btn_back

            # 2. Fallback tay cam qua Pygame Joystick
            elif self.joystick is not None:
                try:
                    btn_a = self.joystick.get_button(0)
                    self.is_boost = bool(btn_a)
                    curr_speed = self.boost_speed if self.is_boost else self.normal_speed

                    raw_lx = self.joystick.get_axis(0)
                    raw_ly = self.joystick.get_axis(1)
                    if abs(raw_ly) > 0.15: vx = -raw_ly * curr_speed
                    if abs(raw_lx) > 0.15: vy = raw_lx * curr_speed

                    num_axes = self.joystick.get_numaxes()
                    raw_rx = self.joystick.get_axis(3 if num_axes > 3 else 0)
                    raw_ry = self.joystick.get_axis(4 if num_axes > 4 else 1)
                    if abs(raw_rx) > 0.15: yaw_rate = raw_rx * 50.0
                    if raw_ry < -0.15: self.set_gimbal(min(0.0, self.gimbal_pitch + abs(raw_ry) * 1.8))
                    elif raw_ry > 0.15: self.set_gimbal(max(-90.0, self.gimbal_pitch - abs(raw_ry) * 1.8))

                    btn_r3 = self.joystick.get_button(9 if self.joystick.get_numbuttons() > 9 else 0)
                    if btn_r3 and not last_r3: self.cycle_gimbal()
                    last_r3 = btn_r3

                    raw_rt = max(0.0, self.joystick.get_axis(5 if num_axes > 5 else 2))
                    raw_lt = max(0.0, self.joystick.get_axis(2 if num_axes > 5 else 2))
                    if raw_rt > 0.10: vz -= raw_rt * 4.0
                    if raw_lt > 0.10: vz += raw_lt * 3.5

                    btn_b = self.joystick.get_button(1)
                    btn_x = self.joystick.get_button(2)
                    btn_y = self.joystick.get_button(3)
                    btn_lb = self.joystick.get_button(4)
                    btn_rb = self.joystick.get_button(5)
                    btn_back = self.joystick.get_button(6) if self.joystick.get_numbuttons() > 6 else 0
                    btn_start = self.joystick.get_button(7)

                    if btn_b and not last_b: self.searchlight_on = not self.searchlight_on
                    last_b = btn_b
                    if btn_x and not last_x: self.lidar_range_idx = (self.lidar_range_idx + 1) % len(self.lidar_ranges)
                    last_x = btn_x
                    if btn_y and not last_y: self.mark_person_waypoint()
                    last_y = btn_y
                    if btn_lb and not last_lb: self.toggle_rgb_view()
                    last_lb = btn_lb
                    if btn_rb and not last_rb: self.palette_idx = (self.palette_idx + 1) % len(self.thermal_palettes)
                    last_rb = btn_rb
                    if btn_back and not last_back: self.toggle_target_lock()
                    last_back = btn_back
                    if btn_start and not last_start: self.show_help = not self.show_help
                    last_start = btn_start
                except Exception:
                    pass

            # 3. Ban phim dong bo
            keys = pygame.key.get_pressed()
            curr_speed = self.boost_speed if (keys[pygame.K_TAB] or keys[pygame.K_LCTRL] or self.is_boost) else self.normal_speed

            if keys[pygame.K_w]: vx = curr_speed
            elif keys[pygame.K_s]: vx = -curr_speed
            if keys[pygame.K_a]: vy = -curr_speed
            elif keys[pygame.K_d]: vy = curr_speed
            if keys[pygame.K_SPACE]: vz = -4.0
            elif keys[pygame.K_LSHIFT]: vz = +3.5
            if keys[pygame.K_q]: yaw_rate = -50.0
            elif keys[pygame.K_e]: yaw_rate = +50.0
            if keys[pygame.K_UP]: self.set_gimbal(self.gimbal_pitch + 1.5)
            elif keys[pygame.K_DOWN]: self.set_gimbal(self.gimbal_pitch - 1.5)

            # Luong dieu khien tu doc cac bien nay moi 50ms (heartbeat) - khong phu thuoc toc do ve UI
            self.cmd_vx = vx
            self.cmd_vy = vy
            self.cmd_vz = vz
            self.cmd_yaw_rate = yaw_rate

            # Chuyen muot FPV <-> Chase (cross-fade ~0.3s)
            step = max(0.0, dt) * 3.5
            if self.rgb_blend < self.rgb_view_target:
                self.rgb_blend = min(float(self.rgb_view_target), self.rgb_blend + step)
            elif self.rgb_blend > self.rgb_view_target:
                self.rgb_blend = max(float(self.rgb_view_target), self.rgb_blend - step)
            if self.roi_autorotate:
                self.roi_az = (self.roi_az + 25.0 * max(0.0, dt)) % 360.0

            # Ve
            self.ui_fps = clock.get_fps()
            self.render_thermal_pane()
            self.render_rgb_pane()
            self.render_lidar_radar_pane()
            self.render_roi_pane()
            self.render_sidebar()
            self.draw_dashboard(clock)
            self.present()
            clock.tick(60)

        self.running = False
        threads[0].join(timeout=1.0)
        pygame.quit()
        print(">> [GCS STATION] Da dung phien lam viec.", flush=True)

def main():
    system = UAVUnifiedSystem()
    system.run()

if __name__ == "__main__":
    main()
