"""Stage 1 & 2: Sensor Data Acquisition and Temporal Scan Windowing (Report Sections 3.2.1–3.2.2)."""
import cv2
import numpy as np
from configs.config import RUNTIME


def parse_image_response(resp):
    """Report Sections 3.2.1–3.2.2: decode captured RGB/IR sensor response."""
    if resp is None: return None
    w = getattr(resp, "width", 0)
    h = getattr(resp, "height", 0)
    if w <= 0 or h <= 0:
        return None
    raw = getattr(resp, "image_data_uint8", None)
    if not raw or len(raw) == 0:
        return None
    if len(raw) == w * h * 3:
        arr = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 3))
        return arr[:, :, ::-1] if RUNTIME.raw_image_is_bgr else arr
    elif len(raw) == w * h * 4:
        arr = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 4))
        return cv2.cvtColor(arr, cv2.COLOR_BGRA2RGB)
    return None


def recent_lidar_points(scans, now, fallback):
    """Report Section 3.2.2: aggregate synchronized 0.9 s LiDAR scan window in world NED."""
    recent = [scan for timestamp, scan in scans if now - timestamp <= RUNTIME.recent_scan_age_s]
    return np.concatenate(recent, axis=0) if recent else fallback
