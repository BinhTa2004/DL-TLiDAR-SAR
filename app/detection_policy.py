"""Range-gated YOLO labels and rescue marking rules shared by the UI and export."""
import math
from configs.config import DETECTOR, RUNTIME

CLASSIFICATION_RANGE_M = RUNTIME.classification_range_m
SEARCH_CLASS_IDS = DETECTOR.class_ids  # Active checkpoint: person (class 0).
LABELS = {"person": "NGƯỜI", "animal": "ĐỘNG VẬT", "thermal": "TÍN HIỆU NHIỆT"}
COLORS = {"person": (0, 255, 120), "animal": (255, 190, 60), "thermal": (255, 145, 45)}


def target_presentation(det, uav_pos_ned):
    """Classify within 40 m slant range; unknown range/class stays a heat signal.

    YOLO determines the species. LiDAR confirms geometry for marking people.
    Recompute from positions at display/mark time so UAV motion cannot leave a
    stale near-range label or allow a far-away target to be marked.
    """
    distance = None
    position = det.get("_fusion_pos_ned")
    if position is not None and uav_pos_ned is not None:
        try:
            if len(position) == len(uav_pos_ned) == 3:
                value = math.dist(position, uav_pos_ned)
                if math.isfinite(value):
                    distance = value
        except (TypeError, ValueError):
            pass
    classification = "thermal"
    if distance is not None and distance <= CLASSIFICATION_RANGE_M and det.get("is_yolo", False):
        classification = {0: "person", 1: "animal"}.get(det.get("class_id"), "thermal")
    fusion_verdict = det.get("fusion_verdict", det.get("verdict", ""))
    verified = bool(det.get("lidar_verified", False))
    can_mark = classification == "person" and verified and fusion_verdict == "CONFIRMED_HUMAN"
    if classification == "person":
        verdict = "CONFIRMED_HUMAN" if can_mark else "PERSON_CANDIDATE"
    elif classification == "animal":
        verdict = "CONFIRMED_ANIMAL" if verified and fusion_verdict == "CONFIRMED_ANIMAL" else "ANIMAL_CANDIDATE"
    else:
        verdict = "THERMAL_SIGNAL"
    return {
        "classification": classification,
        "distance_m": distance,
        "label": LABELS[classification],
        "verdict": verdict,
        "is_human": classification == "person",
        "can_mark": can_mark,
    }


def detection_caption(presentation):
    distance = presentation.get("distance_m")
    range_text = f"{distance:.1f}m" if distance is not None else "CHƯA RÕ CỰ LY"
    caption = f"{presentation['label']} | {range_text}"
    if presentation.get("verdict") in ("CONFIRMED_HUMAN", "CONFIRMED_ANIMAL"):
        caption += f" | LiDAR XÁC NHẬN: {int(presentation.get('lidar_points', 0))} điểm"
    elif presentation.get("classification") in ("person", "animal"):
        caption += " | CHỜ LiDAR"
    return caption

