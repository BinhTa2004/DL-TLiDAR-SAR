"""Stage 3b: LiDAR Processing Branch — Range Filtering, Voxel Deduplication, 4D DBSCAN and PCA Features (Report Sections 3.2.3–3.2.5)."""
from collections import defaultdict, deque
from itertools import product
import numpy as np
from configs.config import FUSION, RUNTIME


def _fallback_voxel_cluster_indices(points, eps, min_samples):
    """Report Section 3.2.5: connected-voxel spatial clustering."""
    cells = defaultdict(list)
    keys = np.floor(points / max(float(eps), 1e-3)).astype(np.int32)
    for idx, key in enumerate(keys):
        cells[tuple(int(v) for v in key)].append(idx)

    visited = set()
    cluster_indices = []
    dimensions = int(keys.shape[1])
    neighbours = list(product((-1, 0, 1), repeat=dimensions))
    for start in cells:
        if start in visited:
            continue
        visited.add(start)
        queue_cells = deque([start])
        indices = []
        while queue_cells:
            cell = queue_cells.popleft()
            indices.extend(cells[cell])
            for delta in neighbours:
                nxt = tuple(cell[axis] + delta[axis] for axis in range(dimensions))
                if nxt in cells and nxt not in visited:
                    visited.add(nxt)
                    queue_cells.append(nxt)
        if len(indices) >= min_samples:
            cluster_indices.append(np.asarray(indices, dtype=np.int32))
    cluster_indices.sort(key=len, reverse=True)
    return cluster_indices


def _fallback_voxel_clusters(points, eps, min_samples):
    """Report Section 3.2.5: materialize cluster point arrays."""
    return [points[idx] for idx in _fallback_voxel_cluster_indices(points, eps, min_samples)]


def cluster_subcanopy_points(points, eps=FUSION.default_cluster_eps_m, min_samples=FUSION.min_cluster_points):
    """Report Section 3.2.5: 3D DBSCAN sub-canopy point clustering."""
    points = np.asarray(points, dtype=np.float32)
    if len(points) < min_samples:
        return []
    try:
        from sklearn.cluster import DBSCAN

        labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(points)
        clusters = [points[labels == label] for label in np.unique(labels) if label >= 0]
        clusters = [cluster for cluster in clusters if len(cluster) >= min_samples]
        clusters.sort(key=len, reverse=True)
        return clusters
    except Exception:
        return _fallback_voxel_clusters(points, eps, min_samples)


def cluster_subcanopy_points_4d(points, temperatures, eps=FUSION.default_cluster_eps_m, min_samples=FUSION.min_cluster_points,
                                thermal_weight=FUSION.default_thermal_weight):
    """Report Section 3.2.5: 4D spatio-thermal DBSCAN clustering in (X, Y, Z, T) space."""
    points = np.asarray(points, dtype=np.float32)
    temperatures = np.asarray(temperatures, dtype=np.float32).reshape(-1)
    if len(points) < min_samples or len(temperatures) != len(points):
        return []

    thermal_axis = ((temperatures - FUSION.thermal_origin_c) / FUSION.thermal_span_c * float(thermal_weight) * FUSION.thermal_axis_scale).reshape(-1, 1)
    features = np.hstack((points, thermal_axis))
    try:
        from sklearn.cluster import DBSCAN

        labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(features)
        indices = [np.where(labels == label)[0] for label in np.unique(labels) if label >= 0]
        indices = [idx for idx in indices if len(idx) >= min_samples]
        indices.sort(key=len, reverse=True)
    except Exception:
        indices = _fallback_voxel_cluster_indices(features, eps, min_samples)
    return [(points[idx], temperatures[idx]) for idx in indices]


def compute_geometric_features(cluster_pts_ned, ground_z_ned):
    """Report Section 3.2.3 & 3.2.6: compute robust 3D cluster bounds, PCA shape descriptors, and spatial median centroid."""
    pts = np.asarray(cluster_pts_ned, dtype=np.float32)
    if len(pts) < FUSION.min_feature_points:
        return None

    # Robust bounds prevent one multipath/outlier return from inflating the
    # biological volume and downgrading an otherwise coherent human cluster.
    min_pt = np.percentile(pts, FUSION.bounds_lower_percentile, axis=0)
    max_pt = np.percentile(pts, FUSION.bounds_upper_percentile, axis=0)
    dx, dy, dz = [float(v) for v in (max_pt - min_pt)]
    volume = max(1e-4, dx * dy * dz)

    centered = pts - np.mean(pts, axis=0)
    cov = np.cov(centered.T)
    eigenvalues = np.sort(np.linalg.eigvalsh(cov))[::-1]
    e1 = max(1e-6, float(eigenvalues[0]))
    e2 = max(1e-6, float(eigenvalues[1]))
    e3 = max(1e-6, float(eigenvalues[2]))
    planarity = float((e2 - e3) / e1)
    sphericity = float(e3 / e1)
    completeness = float(np.clip(len(pts) / FUSION.completeness_points, FUSION.completeness_min, 1.0))

    return {
        "n_points": int(len(pts)),
        "dx": dx,
        "dy": dy,
        "dz": dz,
        "max_dim": max(dx, dy, dz),
        "volume_m3": volume,
        "planarity": planarity,
        "sphericity": sphericity,
        "height_above_ground_m": float(ground_z_ned - np.max(pts[:, 2])),
        "completeness_r": completeness,
        "centroid_ned": np.median(pts, axis=0).astype(np.float32),
    }


def filter_body_returns(associated, associated_temps, ground_z):
    """Report Sections 3.2.3–3.2.4: ground-relative height gating, thermal gating, and 0.10 m voxel deduplication."""
    height = ground_z - associated[:, 2] if len(associated) else np.empty((0,), dtype=np.float32)
    # A rocky slope can put the local 85th-percentile ground estimate 1–2 m
    # below a prone person.  Gate by radiometric heat first, then allow a
    # wider vertical band before validating the 3-D cluster geometry.
    body_mask = ((associated_temps >= FUSION.warm_threshold_c) &
                 (height >= FUSION.min_height_m) & (height <= FUSION.max_height_m))
    body_points = associated[body_mask]
    body_temps = associated_temps[body_mask]

    # Consecutive scans often revisit the same surface return.  Count distinct
    # 10 cm cells so repeated samples alone cannot satisfy the five-point rule.
    if len(body_points):
        cells = np.floor(body_points / FUSION.voxel_size_m).astype(np.int32)
        _, unique_idx = np.unique(cells, axis=0, return_index=True)
        unique_idx.sort()
        body_points = body_points[unique_idx]
        body_temps = body_temps[unique_idx]
    return body_points, body_temps


def filter_sensor_ranges(points):
    """Report Section 3.2.3: filter invalid near-field and out-of-range LiDAR returns."""
    dists_sq = np.sum(points ** 2, axis=1)
    valid = (dists_sq >= RUNTIME.sensor_min_range_sq_m) & (dists_sq <= RUNTIME.sensor_max_range_sq_m)
    return points[valid]
