"""System Configuration, Paths, Thermal Profiles, and Thresholds (Report Section 3.2 & Table 2)."""
from dataclasses import dataclass
import os
from pathlib import Path
import re
import sys
import tempfile

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "outputs"
SIMULATOR = ROOT / "simulation" / "WindowsNoEditor"

for _directory in (ROOT / "simulation", ROOT):
    _value = str(_directory)
    if _value in sys.path:
        sys.path.remove(_value)
    sys.path.insert(0, _value)


def local_path(path):
    """Resolve a project-relative path and reject paths escaping the project."""
    resolved = (ROOT / path).resolve()
    if not resolved.is_relative_to(ROOT):
        raise ValueError(f"Path must stay inside the project: {path}")
    return resolved


def configure_runtime():
    os.chdir(ROOT)
    locations = {
        "TEMP": "outputs/tmp", "TMP": "outputs/tmp", "TMPDIR": "outputs/tmp",
        "YOLO_CONFIG_DIR": "outputs/config/ultralytics",
        "MPLCONFIGDIR": "outputs/config/matplotlib",
        "TORCH_HOME": "outputs/cache/torch",
        "HF_HOME": "outputs/cache/huggingface",
        "XDG_CACHE_HOME": "outputs/cache",
        "NUMBA_CACHE_DIR": "outputs/cache/numba",
        "PIP_CACHE_DIR": "outputs/cache/pip",
        "PYTHONPYCACHEPREFIX": "outputs/cache/pycache",
    }
    for name, relative in locations.items():
        directory = local_path(relative)
        directory.mkdir(parents=True, exist_ok=True)
        os.environ[name] = str(directory)
    tempfile.tempdir = os.environ["TEMP"]
    os.environ["PYTHONNOUSERSITE"] = "1"
    os.environ["PYTHONUTF8"] = "1"
    os.environ["PYTHONIOENCODING"] = "utf-8"
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")


configure_runtime()

# ---------------------------------------------------------------------------
# THERMAL PROFILES & ACTOR TOKENS (Report Section 3.5.1)
# ---------------------------------------------------------------------------
HUMAN_ID = 255
ANIMAL_ID = 230
ANIMAL_IR_VALUE = 243
ANIMAL_DISPLAY_VALUE = 238
HUMAN_TOKENS = ("mixamo", "male_laying", "sitting_anim", "thoughtful_head", "looking_behind")
EXCLUDED_TOKENS = ("flag", "cloth", "banner", "tarp", "rope", "scarecrow",
                   "mannequin", "nathan", "poseasset", "audio", "aud_", "amb_",
                   "sound", "trigger", "manager", "hitching", "manure", "birdhouse")
ANIMAL_PATTERN = re.compile(
    r"(?:^|_)(?:animal|buffalo|bison|horse|cow|cattle|calf|bull|sheep|goat|"
    r"chicken|rooster|hen|pig|boar|deer|stag|dog|wolf|fox|cat|rabbit|hare|"
    r"bear|bird|crow|raven|eagle|duck|goose|pigeon)(?:s|\d)*(?:_|$)", re.I)


def target_kind(name):
    lowered = name.lower()
    if any(token in lowered for token in EXCLUDED_TOKENS):
        return None
    if any(token in lowered for token in HUMAN_TOKENS):
        return "human"
    if ANIMAL_PATTERN.search(name):
        return "animal"
    return None


# ---------------------------------------------------------------------------
# SYSTEM CONFIGURATION DATACLASSES (Report Section 3.2 & Table 2)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class DetectorConfig:
    model_path: str = "Yolo/Yolo26m_trained(1).pt"
    class_ids: tuple[int, ...] = (0,)
    confidence: float = 0.25
    runtime_imgsz: int = 960
    runtime_confidence: float = 0.28
    person_threshold: int = 245
    max_regions: int = 8
    fallback_max_image_fraction: float = 0.08
    fallback_min_area: int = 8
    fallback_max_area: int = 12000
    fallback_padding: int = 3
    fallback_confidence: float = 0.50


@dataclass(frozen=True)
class FusionConfig:
    intensity_base_c: float = 20.0
    intensity_span_c: float = 18.5
    intensity_max: float = 255.0
    warm_threshold_c: float = 34.0
    thermal_origin_c: float = 20.0
    thermal_span_c: float = 20.0
    thermal_axis_scale: float = 0.9
    default_cluster_eps_m: float = 0.65
    cluster_eps_m: float = 0.85
    min_cluster_points: int = 5
    min_feature_points: int = 4
    default_thermal_weight: float = 0.25
    cluster_thermal_weight: float = 0.35
    voxel_size_m: float = 0.10
    min_height_m: float = 0.03
    max_height_m: float = 3.20
    ground_percentile: float = 85.0
    bounds_lower_percentile: float = 5.0
    bounds_upper_percentile: float = 95.0
    completeness_points: float = 28.0
    completeness_min: float = 0.10
    min_camera_depth_m: float = 0.5
    max_camera_depth_m: float = 150.0
    bbox_padding_fraction: float = 0.12
    min_cloud_points: int = 15
    min_local_points: int = 8
    refine_min_radius_m: float = 2.0
    refine_max_radius_m: float = 4.5
    refine_range_fraction: float = 0.08
    ray_min_down: float = 0.015
    ray_min_cloud_points: int = 12
    ray_min_distance_m: float = 1.2
    ray_max_distance_m: float = 135.0
    ray_min_candidates: int = 5
    ray_min_beam_radius_m: float = 0.55
    ray_beam_range_fraction: float = 0.03
    ray_min_group_points: int = 4
    ray_group_gap_m: float = 1.35
    ray_ground_radius_sq_m: float = 9.0
    ray_min_ground_height_m: float = 0.18
    ray_max_ground_height_m: float = 2.25
    ray_surface_percentile: float = 35.0
    max_ground_intersection_m: float = 150.0
    flat_min_height_m: float = 0.12
    flat_planarity: float = 0.78
    low_flat_planarity: float = 0.68
    low_flat_height_m: float = 0.22
    trunk_height_m: float = 2.45
    trunk_width_m: float = 0.45
    human_min_dimension_m: float = 0.22
    human_max_dimension_m: float = 2.40
    human_min_height_m: float = 0.15
    human_max_height_m: float = 2.20
    human_min_volume_m3: float = 0.008
    human_max_volume_m3: float = 2.80
    relaxed_min_dimension_m: float = 0.18
    relaxed_max_dimension_m: float = 2.80
    relaxed_min_height_m: float = 0.12
    relaxed_max_height_m: float = 2.45
    nonplanarity_scale: float = 0.85
    confirmed_shape_weight: float = 0.70
    confirmed_shape_bias: float = 0.25
    confirmed_score_min: float = 0.55
    confirmed_score_max: float = 0.96
    relaxed_shape_weight: float = 0.55
    relaxed_shape_bias: float = 0.20
    relaxed_score_min: float = 0.45
    relaxed_score_max: float = 0.82
    geometry_weight_base: float = 0.25
    geometry_completeness_weight: float = 0.45
    completeness_bonus: float = 0.10
    final_score_min: float = 0.05
    final_score_max: float = 0.99


@dataclass(frozen=True)
class RuntimeConfig:
    ground_z_ned_m: float = 1.92
    thermal_fov_deg: float = 50.0
    default_gimbal_pitch_deg: float = 0.0
    command_period_s: float = 0.03
    command_duration_s: float = 0.08
    capture_period_s: float = 0.08
    raw_image_is_bgr: bool = True
    classification_range_m: float = 40.0
    recent_scan_count: int = 8
    recent_scan_age_s: float = 0.90
    display_point_budget: int = 8000
    sensor_min_range_sq_m: float = 2.25
    sensor_max_range_sq_m: float = 25600.0


@dataclass(frozen=True)
class MappingConfig:
    scan_voxel_m: float = 0.22
    global_voxel_m: float = 0.30
    max_points: int = 450_000
    voxel_size_m: float = 0.25
    fine_voxel_size_m: float = 0.12
    max_global_points: int = 850_000
    display_max_points: int = 400_000


DETECTOR = DetectorConfig()
FUSION = FusionConfig()
RUNTIME = RuntimeConfig()
MAPPING = MappingConfig()
