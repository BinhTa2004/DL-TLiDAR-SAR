"""Stage 3c & 4: Cross-Modal Projection Eq. (2) and 4D Spatio-Thermal Frustum Alignment Eq. (3) (Report Sections 3.2.3–3.2.4)."""
import math
import numpy as np
from configs.config import FUSION


def quat_to_rotation(q):
    """Report Section 3.2.3, Eq. (2): convert quaternion to 3x3 rotation matrix."""
    if hasattr(q, "w_val"):
        w, x, y, z = float(q.w_val), float(q.x_val), float(q.y_val), float(q.z_val)
    elif hasattr(q, "w"):
        w, x, y, z = float(q.w), float(q.x), float(q.y), float(q.z)
    else:
        w, x, y, z = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)


def euler_to_rotation(pitch_deg, roll_deg=0.0, yaw_deg=0.0):
    """Report Section 3.2.3, Eq. (2): convert gimbal Euler angles (pitch, roll, yaw) to 3x3 rotation matrix."""
    pitch = math.radians(float(pitch_deg)) * 0.5
    roll = math.radians(float(roll_deg)) * 0.5
    yaw = math.radians(float(yaw_deg)) * 0.5
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(yaw), math.sin(yaw)
    q = (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )
    return quat_to_rotation(q)


def project_cluster_bbox(cluster_ned, origin_ned, cam_rotation, focal, w_im, h_im):
    """Report Section 3.2.4, Eq. (2): project 3D LiDAR cluster returns onto the 2D thermal image plane."""
    points = (np.asarray(cluster_ned, dtype=np.float32) - origin_ned) @ cam_rotation
    points = points[np.all(np.isfinite(points), axis=1) & (points[:, 0] > 0.5)]
    if len(points) == 0:
        return None
    u = focal * points[:, 1] / points[:, 0] + w_im * 0.5
    v = focal * points[:, 2] / points[:, 0] + h_im * 0.5
    x0 = int(np.clip(np.floor(u.min()) - 2, 0, w_im - 1))
    y0 = int(np.clip(np.floor(v.min()) - 2, 0, h_im - 1))
    x1 = int(np.clip(np.ceil(u.max()) + 2, 0, w_im - 1))
    y1 = int(np.clip(np.ceil(v.max()) + 2, 0, h_im - 1))
    return (x0, y0, x1, y1) if x1 > x0 and y1 > y0 else None


def project_lidar_frustum(points, origin, cam_rotation, focal, w_im, h_im, bbox_2d):
    """Report Section 3.2.4, Eq. (2): project 3D LiDAR points into camera coordinates and gate by padded 2D ROI frustum."""
    x0, y0, x1, y1 = [float(value) for value in bbox_2d]
    bw, bh = max(1.0, x1 - x0), max(1.0, y1 - y0)
    rel_world = points - origin
    cam_points = rel_world @ cam_rotation
    depth = cam_points[:, 0]
    front = (depth > FUSION.min_camera_depth_m) & (depth < FUSION.max_camera_depth_m)
    safe_depth = np.maximum(depth, 1e-3)
    u = focal * (cam_points[:, 1] / safe_depth) + float(w_im) * 0.5
    v = focal * (cam_points[:, 2] / safe_depth) + float(h_im) * 0.5
    pad_x, pad_y = FUSION.bbox_padding_fraction * bw, FUSION.bbox_padding_fraction * bh
    in_bbox = front & (u >= x0 - pad_x) & (u <= x1 + pad_x) & (v >= y0 - pad_y) & (v <= y1 + pad_y)
    return in_bbox, u, v


def sample_projected_temperatures(associated, projected_u, projected_v, thermal_temp_c, thermal_image):
    """Report Section 3.2.4, Eq. (3): sample 3x3 neighborhood temperatures to form 4D spatio-thermal points Q = (X, Y, Z, T)."""
    associated_u = np.rint(projected_u).astype(np.int32)
    associated_v = np.rint(projected_v).astype(np.int32)
    associated_temps = np.full(len(associated), float(thermal_temp_c), dtype=np.float32)
    if thermal_image is not None and len(associated):
        thermal = np.asarray(thermal_image)
        if thermal.ndim == 3:
            thermal = thermal[:, :, 0]
        if thermal.ndim == 2 and thermal.size:
            th, tw = thermal.shape[:2]
            associated_u = np.clip(associated_u, 0, tw - 1)
            associated_v = np.clip(associated_v, 0, th - 1)
            # One pixel of camera/LiDAR reprojection error is common at range.
            # Sample a 3x3 neighbourhood so a return on the body boundary can
            # still inherit the adjacent hot person pixel.
            raw_temperature = thermal[associated_v, associated_u].astype(np.float32)
            for dv in (-1, 0, 1):
                vv = np.clip(associated_v + dv, 0, th - 1)
                for du in (-1, 0, 1):
                    uu = np.clip(associated_u + du, 0, tw - 1)
                    raw_temperature = np.maximum(raw_temperature, thermal[vv, uu])
            associated_temps = FUSION.intensity_base_c + (raw_temperature / FUSION.intensity_max) * FUSION.intensity_span_c
    return associated_temps
