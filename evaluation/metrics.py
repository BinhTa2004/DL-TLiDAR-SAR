"""Match estimated rescue positions to valid simulator ground truth."""

import csv
import json
import math
import os
import tempfile
from functools import lru_cache
from statistics import mean, median


CSV_COLUMNS = [
    "ID", "TIME", "UAV_X", "UAV_Y", "UAV_Z", "ALT", "TARGET_X", "TARGET_Y",
    "DIST", "GIMBAL_PITCH", "PHOTO_RGB", "PHOTO_THERMAL",
    "GT_NAME", "GT_X", "GT_Y", "GT_Z", "ERROR_2D_M", "ERROR_3D_M", "GT_STATUS",
    "POSITION_SOURCE", "LIDAR_VERIFIED",
    "HORIZONTAL_ERROR_M", "VERTICAL_ERROR_M",
]

SUMMARY_COLUMNS = [
    "GROUND_TRUTH_COUNT", "MARKED_COUNT", "LIDAR_VERIFIED_COUNT", "VALID_3D_COUNT",
    "MEAN_3D_ERROR_M", "MEDIAN_3D_ERROR_M", "P90_LOCALIZATION_ERROR_M",
    "SUCCESS_3D_0_5M_PCT", "SUCCESS_3D_1M_PCT", "SUCCESS_DENOMINATOR",
]


def load_ground_truth(path):
    'Report Sections 3.5.3 and 4.2: Read validated evaluation-only ground truth.'
    try:
        with open(path, encoding="utf-8") as stream:
            data = json.load(stream)
    except (OSError, ValueError):
        return []
    if not isinstance(data, list):
        return []
    valid = []
    for item in data:
        try:
            xyz = tuple(float(item[key]) for key in ("x_ned", "y_ned", "z_ned"))
            name = str(item["name"])
        except (KeyError, TypeError, ValueError):
            continue
        if name and all(math.isfinite(value) for value in xyz):
            valid.append({"name": name, "xyz": xyz})
    return valid


def match_ground_truth(positions, truth, used_names=(), max_distance_m=25.0):
    'Report Sections 3.5.3 and 4.2: Assign XY pairs one-to-one with the original 25 m gate.'
    result = [{"status": "NO_GROUND_TRUTH" if not truth else "NO_MATCH"}
              for _ in positions]
    available = [target for target in truth if target["name"] not in set(used_names)]
    distances = [
        [math.hypot(float(pos[0]) - target["xyz"][0],
                    float(pos[1]) - target["xyz"][1]) for target in available]
        for pos in positions
    ]

    @lru_cache(None)
    def assign(i, used_mask):
        if i == len(positions):
            return (0, 0.0, ())
        count, total, choices = assign(i + 1, used_mask)
        best = (count, total, (-1,) + choices)
        for j, distance in enumerate(distances[i]):
            if used_mask & (1 << j) or distance > max_distance_m:
                continue
            next_count, next_total, next_choices = assign(i + 1, used_mask | (1 << j))
            option = (next_count + 1, next_total + distance, (j,) + next_choices)
            if option[0] > best[0] or (option[0] == best[0] and option[1] < best[1]):
                best = option
        return best

    _, _, choices = assign(0, 0)
    for i, j in enumerate(choices):
        if j < 0:
            continue
        target = available[j]
        distance = distances[i][j]
        gx, gy, gz = target["xyz"]
        x, y, z = (float(v) for v in positions[i])
        result[i] = {
            "status": "MATCHED", "name": target["name"],
            "xyz": target["xyz"], "error_2d_m": distance,
            "vertical_error_m": abs(z - gz),
            "error_3d_m": math.dist((x, y, z), (gx, gy, gz)),
        }
    return result


def calculate_localization_metrics(markers, truth):
    'Report Sections 3.5.3 and 4.2: Compute errors and all-GT success rates.'
    verified = [marker for marker in markers if marker.get("lidar_verified", False)]
    positions = [(marker["tgt_x"], marker["tgt_y"], marker["tgt_z"])
                 for marker in verified]
    matches = match_ground_truth(positions, truth)
    errors = sorted(match["error_3d_m"] for match in matches if match["status"] == "MATCHED")
    gt_count = len(truth)
    p90 = None
    if errors:
        index = 0.9 * (len(errors) - 1)
        lower = int(math.floor(index))
        upper = int(math.ceil(index))
        p90 = errors[lower] + (errors[upper] - errors[lower]) * (index - lower)
    return {
        "GROUND_TRUTH_COUNT": gt_count,
        "MARKED_COUNT": len(markers),
        "LIDAR_VERIFIED_COUNT": len(verified),
        "VALID_3D_COUNT": len(errors),
        "MEAN_3D_ERROR_M": mean(errors) if errors else None,
        "MEDIAN_3D_ERROR_M": median(errors) if errors else None,
        "P90_LOCALIZATION_ERROR_M": p90,
        "SUCCESS_3D_0_5M_PCT": (100.0 * sum(e <= 0.5 + 1e-6 for e in errors) / gt_count)
                               if gt_count else None,
        "SUCCESS_3D_1M_PCT": (100.0 * sum(e <= 1.0 + 1e-6 for e in errors) / gt_count)
                             if gt_count else None,
        "SUCCESS_DENOMINATOR": "ALL_VALID_GT_ACTORS" if gt_count else "NO_GROUND_TRUTH",
    }


def write_summary_csv(path, metrics):
    'Report Sections 3.5.3 and 4.2: Atomically export the unchanged summary CSV schema.'
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    handle, temp_path = tempfile.mkstemp(prefix="localization_metrics_", suffix=".csv", dir=directory)
    try:
        with os.fdopen(handle, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=SUMMARY_COLUMNS)
            writer.writeheader()
            writer.writerow({key: f"{value:.3f}" if isinstance(value, float) else
                             "" if value is None else value
                             for key, value in metrics.items()})
        os.replace(temp_path, path)
    finally:
        if os.path.exists(temp_path):
            os.remove(temp_path)


def ensure_csv_columns(path):
    'Report Sections 3.5.3 and 4.2: Extend the rescue log without modifying historical rows.'
    if not os.path.exists(path):
        return
    with open(path, newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.reader(stream))
    if not rows or rows[0] == CSV_COLUMNS:
        return
    old_header = rows[0]
    if not old_header or not set(old_header).issubset(CSV_COLUMNS):
        raise ValueError("Unknown rescue log CSV schema")
    directory = os.path.dirname(os.path.abspath(path))
    handle, temp_path = tempfile.mkstemp(prefix="rescue_log_", suffix=".csv", dir=directory)
    try:
        with os.fdopen(handle, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(CSV_COLUMNS)
            for row in rows[1:]:
                old = dict(zip(old_header, row))
                writer.writerow([old.get(column, "") for column in CSV_COLUMNS])
        os.replace(temp_path, path)
    finally:
        if os.path.exists(temp_path):
            os.remove(temp_path)
