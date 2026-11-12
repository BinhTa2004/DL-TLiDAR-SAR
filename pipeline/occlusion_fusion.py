"""Stage 5: Occlusion Reasoning and Adaptive Multimodal Fusion Eq. (4) (Report Section 3.2.5)."""
import numpy as np
from configs.config import FUSION, RUNTIME

ACCEPTED_VERDICTS = {
    "CONFIRMED_HUMAN",
    "CONFIRMED_ANIMAL",
    "UNVERIFIED_FAR_TARGET",
}
REJECTED_VERDICTS = {"REJECTED_HOT_ROCK", "REJECTED_TREE_TRUNK", "REJECTED_GEOMETRY"}


def verify_and_fuse_target(geom, s_thermal=0.85, thermal_temp_c=37.0, target_class_id=0):
    """Report Section 3.2.5, Eq. (4): evaluate 3D geometric priors and compute adaptive fusion score S(c)."""
    if geom is None:
        return {
            "verdict": "UNVERIFIED_FAR_TARGET",
            "is_human": False,
            "s_geo": 0.0,
            "s_final": float(np.clip(float(s_thermal) * 0.75, 0.35, 0.70)),
            "reason": "Anh nhiet da phat hien; chua du diem LiDAR trong bbox de kiem chung",
        }

    dx, dy, dz = geom["dx"], geom["dy"], geom["dz"]
    max_dim = geom["max_dim"]
    volume = geom["volume_m3"]
    planarity = geom["planarity"]
    completeness = geom["completeness_r"]

    if dz < FUSION.flat_min_height_m or planarity > FUSION.flat_planarity or (planarity > FUSION.low_flat_planarity and dz < FUSION.low_flat_height_m):
        return {
            "verdict": "REJECTED_HOT_ROCK",
            "is_human": False,
            "s_geo": 0.05,
            "s_final": 0.05,
            "reason": f"Vat the mong/phang (dz={dz:.2f}m, P={planarity:.2f})",
        }
    if dz > FUSION.trunk_height_m and dx < FUSION.trunk_width_m and dy < FUSION.trunk_width_m:
        return {
            "verdict": "REJECTED_TREE_TRUNK",
            "is_human": False,
            "s_geo": 0.10,
            "s_final": 0.10,
            "reason": f"Than cay thang dung (cao={dz:.2f}m)",
        }

    human_dims = (
        FUSION.human_min_dimension_m <= max_dim <= FUSION.human_max_dimension_m
        and FUSION.human_min_height_m <= dz <= FUSION.human_max_height_m
        and FUSION.human_min_volume_m3 <= volume <= FUSION.human_max_volume_m3
    )
    warm_enough = float(thermal_temp_c) >= FUSION.warm_threshold_c
    non_planar = max(0.05, 1.0 - planarity / FUSION.nonplanarity_scale)

    if target_class_id == 1 and warm_enough:
        # The species comes from YOLO. Do not force an animal cluster through
        # human body dimensions; the clutter rejection above still applies.
        verdict = "CONFIRMED_ANIMAL"
        s_geo = float(np.clip(FUSION.confirmed_shape_weight * non_planar + FUSION.confirmed_shape_bias,
                              FUSION.confirmed_score_min, FUSION.confirmed_score_max))
        reason = f"YOLO dong vat; cum LiDAR nong {dx:.2f}x{dy:.2f}x{dz:.2f}m"
        is_human = False
    elif target_class_id != 1 and human_dims and warm_enough:
        verdict = "CONFIRMED_HUMAN"
        s_geo = float(np.clip(FUSION.confirmed_shape_weight * non_planar + FUSION.confirmed_shape_bias,
                              FUSION.confirmed_score_min, FUSION.confirmed_score_max))
        reason = f"Hinh hoc nguoi {dx:.2f}x{dy:.2f}x{dz:.2f}m, V={volume:.3f}m3"
        is_human = True
    elif target_class_id != 1 and FUSION.relaxed_min_dimension_m <= max_dim <= FUSION.relaxed_max_dimension_m and FUSION.relaxed_min_height_m <= dz <= FUSION.relaxed_max_height_m and warm_enough:
        verdict = "CONFIRMED_HUMAN"
        s_geo = float(np.clip(FUSION.relaxed_shape_weight * non_planar + FUSION.relaxed_shape_bias,
                              FUSION.relaxed_score_min, FUSION.relaxed_score_max))
        reason = f"Hinh hoc nguoi mo rong (cao={dz:.2f}m, V={volume:.3f}m3)"
        is_human = True
    else:
        verdict = "REJECTED_GEOMETRY"
        s_geo = 0.10
        reason = f"Hinh hoc/nhiet khong phu hop nguoi (T={thermal_temp_c:.1f}C)"
        is_human = False

    if verdict in REJECTED_VERDICTS:
        s_final = 0.05
    else:
        w_geo = float(FUSION.geometry_weight_base + FUSION.geometry_completeness_weight * completeness)
        w_thermal = 1.0 - w_geo
        s_final = float(np.clip(
            w_thermal * float(s_thermal) + w_geo * s_geo + FUSION.completeness_bonus * completeness,
            FUSION.final_score_min,
            FUSION.final_score_max,
        ))
    return {
        "verdict": verdict,
        "is_human": is_human,
        "s_geo": s_geo,
        "s_final": s_final,
        "reason": reason,
    }


def intersect_ray_lidar_terrain(origin, direction, lidar_pts, terrain_agl=None,
                                ground_z_datum=RUNTIME.ground_z_ned_m):
    """Report Section 3.2.5: ray-casting intersection along camera line-of-sight through sub-canopy LiDAR returns."""
    origin = np.asarray(origin, dtype=np.float32)
    direction = np.asarray(direction, dtype=np.float32)
    points = np.asarray(lidar_pts, dtype=np.float32) if lidar_pts is not None else np.empty((0, 3), dtype=np.float32)
    if direction[2] <= FUSION.ray_min_down:
        return None

    if len(points) >= FUSION.ray_min_cloud_points:
        rel = points - origin
        distance_along = rel @ direction
        valid = (distance_along >= FUSION.ray_min_distance_m) & (distance_along <= FUSION.ray_max_distance_m)
        if np.count_nonzero(valid) >= FUSION.ray_min_candidates:
            cand = points[valid]
            cand_t = distance_along[valid]
            projected = origin + cand_t[:, None] * direction
            perp_sq = np.sum((cand - projected) ** 2, axis=1)
            beam_radius = np.maximum(FUSION.ray_min_beam_radius_m, FUSION.ray_beam_range_fraction * cand_t)
            in_beam = perp_sq <= beam_radius ** 2
            if np.count_nonzero(in_beam) >= FUSION.ray_min_group_points:
                beam_t = cand_t[in_beam]
                beam_pts = cand[in_beam]
                order = np.argsort(beam_t)
                beam_t, beam_pts = beam_t[order], beam_pts[order]
                splits = np.where(np.diff(beam_t) > FUSION.ray_group_gap_m)[0] + 1
                for group_t, group_pts in zip(np.split(beam_t, splits), np.split(beam_pts, splits)):
                    if len(group_t) < FUSION.ray_min_group_points:
                        continue
                    candidate = origin + float(np.median(group_t)) * direction
                    dxy = np.sum((points[:, :2] - candidate[:2]) ** 2, axis=1)
                    local = points[dxy <= FUSION.ray_ground_radius_sq_m]
                    if len(local) >= FUSION.ray_min_candidates:
                        ground_z = float(np.percentile(local[:, 2], FUSION.ground_percentile))
                        height = ground_z - candidate[2]
                        if FUSION.ray_min_ground_height_m <= height <= FUSION.ray_max_ground_height_m:
                            return (origin + float(np.percentile(group_t, FUSION.ray_surface_percentile)) * direction).astype(np.float32)

    ground_z = (origin[2] + float(terrain_agl)) if terrain_agl is not None and terrain_agl > 0.5 else float(ground_z_datum)
    t = (ground_z - origin[2]) / direction[2]
    if 0 < t <= FUSION.max_ground_intersection_m:
        return (origin + t * direction).astype(np.float32)
    return None
