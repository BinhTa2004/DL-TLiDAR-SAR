"""7-Stage Thermal-LiDAR SAR Fusion Pipeline (Report Section 3.2 & Figure 7)."""
from configs import config as _config
from .acquisition import parse_image_response, recent_lidar_points
from .thermal_detector import (
    SEARCH_CLASS_IDS, synthesize_radiometric_flir,
    temperature_from_intensity, detect_thermal_targets,
)
from .lidar_processing import (
    filter_sensor_ranges, filter_body_returns, compute_geometric_features,
    cluster_subcanopy_points, cluster_subcanopy_points_4d,
)
from .frustum_alignment import (
    quat_to_rotation, euler_to_rotation, project_lidar_frustum,
    project_cluster_bbox, sample_projected_temperatures,
)
from .occlusion_fusion import (
    ACCEPTED_VERDICTS, REJECTED_VERDICTS,
    intersect_ray_lidar_terrain, verify_and_fuse_target,
)
from .localization_3d import locate_target_sensor_fusion

__all__ = [
    "parse_image_response", "recent_lidar_points",
    "SEARCH_CLASS_IDS", "synthesize_radiometric_flir", "temperature_from_intensity", "detect_thermal_targets",
    "filter_sensor_ranges", "filter_body_returns", "compute_geometric_features",
    "cluster_subcanopy_points", "cluster_subcanopy_points_4d",
    "quat_to_rotation", "euler_to_rotation", "project_lidar_frustum",
    "project_cluster_bbox", "sample_projected_temperatures",
    "ACCEPTED_VERDICTS", "REJECTED_VERDICTS", "intersect_ray_lidar_terrain", "verify_and_fuse_target",
    "locate_target_sensor_fusion",
]
