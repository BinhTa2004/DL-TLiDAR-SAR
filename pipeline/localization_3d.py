"""Stage 6 & 7: 3D Target Localization Eq. (5)–(7) and Final System Outputs (Report Sections 3.2.6–3.2.7)."""
import math
import numpy as np
from configs.config import FUSION, RUNTIME
from .lidar_processing import filter_body_returns, cluster_subcanopy_points_4d, compute_geometric_features
from .frustum_alignment import (
    quat_to_rotation, euler_to_rotation,
    project_lidar_frustum, sample_projected_temperatures, project_cluster_bbox,
)
from .occlusion_fusion import ACCEPTED_VERDICTS, intersect_ray_lidar_terrain, verify_and_fuse_target


def locate_target_sensor_fusion(
    *, u_center, v_bottom, bbox_2d, w_im, h_im, fov_deg,
    uav_pos_ned, uav_quat, cam_pitch_deg, yaw_deg,
    lidar_points_ned, ground_z_datum=RUNTIME.ground_z_ned_m, terrain_agl=None,
    cam_mount_frd=None, gimbal_stabilized=True,
    thermal_confidence=0.85, thermal_temp_c=37.0, thermal_image=None,
    camera_position_ned=None, camera_quat=None,
    target_class_id=0,
):
    """Report Sections 3.2.2–3.2.7: select optimal 4D candidate cluster c* and estimate 3D world position p_hat."""
    points = np.asarray(lidar_points_ned, dtype=np.float32) if lidar_points_ned is not None else np.empty((0, 3), dtype=np.float32)
    if cam_mount_frd is None:
        cam_mount_frd = np.array([0.35, 0.0, 0.15], dtype=np.float32)
    else:
        cam_mount_frd = np.asarray(cam_mount_frd, dtype=np.float32)

    body_rotation = quat_to_rotation(uav_quat)
    if gimbal_stabilized:
        cam_rotation = euler_to_rotation(cam_pitch_deg, 0.0, yaw_deg)
    else:
        cam_rotation = body_rotation @ euler_to_rotation(cam_pitch_deg, 0.0, 0.0)
    origin = np.asarray(uav_pos_ned, dtype=np.float32) + body_rotation @ cam_mount_frd
    if camera_position_ned is not None and camera_quat is not None:
        # ImageResponse carries the camera's actual world pose at capture time.
        origin = np.asarray(camera_position_ned, dtype=np.float32)
        cam_rotation = quat_to_rotation(camera_quat)

    x0, y0, x1, y1 = [float(v) for v in bbox_2d]
    bw, bh = max(1.0, x1 - x0), max(1.0, y1 - y0)
    v_eval = (y0 + y1) * 0.5 if bw >= bh * 0.85 or abs(float(cam_pitch_deg)) >= 15.0 else float(v_bottom)
    focal = (float(w_im) * 0.5) / math.tan(math.radians(float(fov_deg) * 0.5))
    ray_cam = np.array([
        1.0,
        (float(u_center) - float(w_im) * 0.5) / focal,
        (v_eval - float(h_im) * 0.5) / focal,
    ], dtype=np.float32)
    ray_world = cam_rotation @ ray_cam
    ray_world /= np.linalg.norm(ray_world) + 1e-9
    if ray_world[2] <= FUSION.ray_min_down:
        return None

    rough_hit = intersect_ray_lidar_terrain(
        origin, ray_world, points, terrain_agl=terrain_agl,
        ground_z_datum=ground_z_datum,
    )
    if rough_hit is None:
        return None

    # Direct camera/LiDAR association: transform world points into the camera
    # FRD frame and retain only those whose projections fall inside the bbox.
    if len(points) < FUSION.min_cloud_points:
        verification = verify_and_fuse_target(None, thermal_confidence, thermal_temp_c, target_class_id)
        return {
            "pos_ned": rough_hit,
            "verdict": verification["verdict"],
            "is_human": False,
            "s_final": verification["s_final"],
            "s_geo": verification["s_geo"],
            "planarity": 0.0,
            "points_count": 0,
            "reason": verification["reason"],
            "lidar_verified": False,
        }

    in_bbox, u, v = project_lidar_frustum(points, origin, cam_rotation, focal, w_im, h_im, bbox_2d)

    range_to_hit = float(np.linalg.norm(rough_hit - origin))
    refine_radius = max(FUSION.refine_min_radius_m, min(FUSION.refine_max_radius_m, FUSION.refine_range_fraction * range_to_hit))
    near_hit = np.sum((points[:, :2] - rough_hit[:2]) ** 2, axis=1) <= refine_radius ** 2
    local_all = points[near_hit]
    if len(local_all) < FUSION.min_local_points:
        verification = verify_and_fuse_target(None, thermal_confidence, thermal_temp_c, target_class_id)
        return {
            "pos_ned": rough_hit,
            "verdict": verification["verdict"],
            "is_human": False,
            "s_final": verification["s_final"],
            "s_geo": verification["s_geo"],
            "planarity": 0.0,
            "points_count": 0,
            "reason": verification["reason"],
            "lidar_verified": False,
        }

    ground_z = float(np.percentile(local_all[:, 2], FUSION.ground_percentile))
    associated_mask = in_bbox & near_hit
    associated = points[associated_mask]
    associated_temps = sample_projected_temperatures(
        associated, u[associated_mask], v[associated_mask], thermal_temp_c, thermal_image
    )
    body_points, body_temps = filter_body_returns(associated, associated_temps, ground_z)

    # A sparse return in the thermal bbox is not evidence that the whole
    # nearby terrain cylinder belongs to the person. In rocky scenes that
    # cylinder can be several metres wide and incorrectly reject a real
    # thermal detection as non-human. Keep the ray/surface position as an
    # unverified candidate until body returns actually overlap the bbox.
    if len(body_points) < FUSION.min_cluster_points:
        verification = verify_and_fuse_target(None, thermal_confidence, thermal_temp_c, target_class_id)
        return {
            "pos_ned": rough_hit,
            "verdict": verification["verdict"],
            "is_human": False,
            "s_final": verification["s_final"],
            "s_geo": verification["s_geo"],
            "planarity": 0.0,
            "points_count": int(len(body_points)),
            "reason": "Thieu diem LiDAR nong phan biet trong khung nhiet; cho xac minh hinh hoc",
            "lidar_verified": False,
        }

    clusters = cluster_subcanopy_points_4d(
        body_points, body_temps, eps=FUSION.cluster_eps_m, min_samples=FUSION.min_cluster_points, thermal_weight=FUSION.cluster_thermal_weight
    )

    best = None
    verdict_rank = {"CONFIRMED_HUMAN": 3, "CONFIRMED_ANIMAL": 3}
    for cluster, cluster_temps in clusters:
        geom = compute_geometric_features(cluster, ground_z)
        mean_temp = float(np.mean(cluster_temps)) if len(cluster_temps) else float(thermal_temp_c)
        verification = verify_and_fuse_target(geom, thermal_confidence, mean_temp, target_class_id)
        center = geom["centroid_ned"] if geom is not None else np.median(cluster, axis=0)
        rel_center = center - origin
        t_center = float(rel_center @ ray_world)
        ray_distance = float(np.linalg.norm(rel_center - t_center * ray_world))
        rank = verdict_rank.get(verification["verdict"], 0)
        score = (rank, verification["s_final"], -ray_distance, len(cluster))
        if best is None or score > best[0]:
            best = (score, cluster, geom, verification, center, mean_temp)

    if best is None:
        verification = verify_and_fuse_target(None, thermal_confidence, thermal_temp_c, target_class_id)
        return {
            "pos_ned": np.array([rough_hit[0], rough_hit[1], ground_z - 0.9], dtype=np.float32),
            "verdict": verification["verdict"],
            "is_human": False,
            "s_final": verification["s_final"],
            "s_geo": verification["s_geo"],
            "planarity": 0.0,
            "points_count": int(len(body_points)),
            "reason": verification["reason"],
            "lidar_verified": False,
        }

    _, cluster, geom, verification, center, mean_temp = best
    return {
        "pos_ned": np.asarray(center, dtype=np.float32),
        "verdict": verification["verdict"],
        "is_human": verification["is_human"],
        "s_final": verification["s_final"],
        "s_geo": verification["s_geo"],
        "planarity": float(geom["planarity"]),
        "points_count": int(len(cluster)),
        "dimensions_m": {
            "dx": float(geom["dx"]),
            "dy": float(geom["dy"]),
            "dz": float(geom["dz"]),
        },
        "lidar_bbox_2d": project_cluster_bbox(cluster, origin, cam_rotation, focal, w_im, h_im)
                         if verification["verdict"] in ACCEPTED_VERDICTS else None,
        "ground_z_ned": ground_z,
        "mean_temp_c": mean_temp,
        "reason": verification["reason"],
        "lidar_verified": verification["verdict"] in ACCEPTED_VERDICTS,
    }
