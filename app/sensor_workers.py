"""Sensor polling and shared-state publication; algorithms are delegated to pipeline/."""
import math
import os
import queue
import time
import airsim
import cv2
import numpy as np
from configs.config import RUNTIME
from pipeline.frustum_alignment import quat_to_rotation as quat_to_R
from pipeline.lidar_processing import filter_sensor_ranges
from evaluation.metrics import load_ground_truth
from .runtime_support import get_logger
THERMAL_CAPTURE_PERIOD = RUNTIME.capture_period_s
log = get_logger("gcs")


# Mẫu tán cây LiDAR tròn 100 điểm chuẩn hóa được tạo sẵn 1 lần duy nhất (tiết kiệm CPU)
_n_canopy = 100
_ang = np.random.uniform(0, 2 * np.pi, _n_canopy)
_rad = np.sqrt(np.random.uniform(0.05, 1.0, _n_canopy))
_CANOPY_TEMPLATE_PTS = np.column_stack([
    _rad * np.cos(_ang),
    _rad * np.sin(_ang),
    -np.random.uniform(0.3, 1.0, _n_canopy)
]).astype(np.float32)




class SensorWorkersMixin:
    def ground_truth_retry_loop(self):
        """Retry actor poses after the Unreal level has finished streaming."""
        truth_path = os.path.join(self.data_dir, "ground_truth_targets.json")
        for delay in (8.0, 12.0, 20.0):
            time.sleep(delay)
            if not self.running or (self._thermal_calibrated and load_ground_truth(truth_path)):
                return
            client = self._connect_client(timeout=3, max_retries=1)
            if client is None:
                continue
            try:
                self.setup_thermal_and_targets(client)
                if self._thermal_calibrated and load_ground_truth(truth_path):
                    log.debug(">> [DO SAI SO] Da cap nhat toa do thuc te cua nhan vat.")
                    self.write_localization_summary()
                    return
            except Exception as exc:
                log.debug(f">> [DO SAI SO] Chua lay duoc ground truth: {exc}")
            finally:
                try:
                    client.client.close()
                except Exception:
                    pass

    def image_loop(self):
        """Lay RGB va IR front_camera trong cung mot batch va cong bo cung luc."""
        client = self._connect_client()
        if client is None: return

        try:
            client.simRunConsoleCommand("t.MaxFPS 60")
            client.simRunConsoleCommand("r.VSync 0")
            client.simRunConsoleCommand("t.IdleWhenNotForeground 0")
        except Exception:
            pass

        last_t = time.perf_counter()

        while self.running:
            loop_start = time.perf_counter()
            try:
                thermal_enabled = self.thermal_enabled
                want_chase = (self.rgb_view_target == 1 and self.chase_available)
                requests = [airsim.ImageRequest("front_camera", airsim.ImageType.Scene, False, False)]
                if thermal_enabled:
                    requests.append(airsim.ImageRequest("front_camera", airsim.ImageType.Infrared, False, False))
                if want_chase:
                    requests.append(airsim.ImageRequest("chase_camera", airsim.ImageType.Scene, False, False))
                try:
                    responses = client.simGetImages(requests, vehicle_name=self.vehicle_name)
                except Exception:
                    responses = None
                    time.sleep(0.3)
                    new_cl = self._connect_client(timeout=15, max_retries=2)
                    if new_cl:
                        try: client.client.close()
                        except Exception: pass
                        client = new_cl
                    continue

                if not responses or len(responses) != len(requests):
                    time.sleep(0.02)
                    continue
                front_rgb = self.parse_img(responses[0])
                im_ir = self.parse_img(responses[1]) if thermal_enabled else None
                chase_rgb = self.parse_img(responses[-1]) if want_chase else None
                if front_rgb is None or (thermal_enabled and
                    (im_ir is None or im_ir.shape[:2] != front_rgb.shape[:2])):
                    time.sleep(0.02)
                    continue
                th_gray = self.synthesize_radiometric_flir(im_ir, front_rgb) if thermal_enabled else None
                if thermal_enabled and th_gray is None:
                    time.sleep(0.02)
                    continue
                camera_pose = None
                if thermal_enabled:
                    ir_response = responses[1]
                    cam_pos = getattr(ir_response, "camera_position", None)
                    cam_quat = getattr(ir_response, "camera_orientation", None)
                    if cam_pos is not None and cam_quat is not None:
                        camera_pose = (
                            np.array([cam_pos.x_val, cam_pos.y_val, cam_pos.z_val], dtype=np.float32),
                            cam_quat,
                        )

                with self.lock:
                    self.rgb_img = front_rgb
                    if chase_rgb is not None:
                        self.chase_img = chase_rgb
                    if thermal_enabled:
                        self.ir_img = th_gray
                        self.latest_ai_frame = th_gray
                        self.latest_camera_pose = camera_pose
                        self.latest_ir_seg = im_ir[:, :, 0] if im_ir.ndim == 3 else im_ir

                now_c = time.perf_counter()
                frame_hz = 1.0 / max(1e-3, now_c - last_t)
                self.rgb_hz = self.rgb_hz * 0.9 + frame_hz * 0.1
                if thermal_enabled:
                    self.img_hz = self.img_hz * 0.9 + frame_hz * 0.1
                last_t = now_c
            except Exception:
                time.sleep(0.04)

            elapsed = time.perf_counter() - loop_start
            sleep_time = max(0.003, THERMAL_CAPTURE_PERIOD - elapsed)
            time.sleep(sleep_time)

    def ai_worker_loop(self):
        """
        LUONG AI BAT DONG BO (ASYNC AI DETECTION WORKER):
        - Chay tren thread rieng, khong lam giam FPS camera hay gay giat lag cho he thong.
        - Tu dong nhan dien nguoi va phat hien tan cay song song voi luong video.
        """
        last_processed_id = None
        tree_cycle = 0

        while self.running:
            if not self.thermal_enabled:
                time.sleep(0.1)
                continue

            ai_loop_start = time.perf_counter()

            with self.lock:
                frame = self.latest_ai_frame
                camera_pose = self.latest_camera_pose
                frame_rgb = self.rgb_img
                infrared_frame = getattr(self, "latest_ir_seg", None)

            if frame is None or getattr(frame, "size", 0) == 0 or frame.ndim < 2 or frame.shape[0] == 0 or frame.shape[1] == 0:
                time.sleep(0.02)
                continue

            frame_id = id(frame)
            if frame_id == last_processed_id:
                time.sleep(0.01)
                continue
            last_processed_id = frame_id

            try:
                # 1. Phat hien nguoi (YOLO26m / Bo loc nhiet)
                dets = self.detect_persons(frame, infrared_frame=infrared_frame)
                now = time.time()
                if dets:
                    trackable = []
                    verdict_priority = {
                        "CONFIRMED_HUMAN": 4,
                        "PERSON_CANDIDATE": 3,
                        "CONFIRMED_ANIMAL": 2,
                        "ANIMAL_CANDIDATE": 2,
                        "THERMAL_SIGNAL": 1,
                    }
                    for det in dets:
                        p = self.locate_target_world(det, frame.shape[1], frame.shape[0], thermal_gray=frame, camera_pose=camera_pose)
                        verdict = det.get("verdict", "")
                        if p is not None and verdict in verdict_priority:
                            trackable.append((
                                verdict_priority.get(verdict, 0),
                                float(det.get("s_final", 0.0)),
                                det,
                                p,
                            ))

                    if trackable:
                        with self.lock:
                            previous_target = self.target_world.copy() if self.target_world is not None and (now - self.target_time) < 2.0 else None
                        nearby = []
                        if previous_target is not None:
                            nearby = [item for item in trackable
                                      if float(np.linalg.norm(item[3] - previous_target)) < 4.0]
                        if previous_target is not None and not nearby:
                            # Wait briefly for the same person instead of jumping to another one.
                            active_det = None
                        else:
                            _, _, active_det, p = max(
                                nearby or trackable,
                                key=lambda item: (item[0], item[1],
                                                  -float(np.linalg.norm(item[3] - previous_target)) if previous_target is not None else 0.0),
                            )
                    else:
                        active_det = None

                    if active_det is not None:
                        with self.lock:
                            if self.target_world is not None and (now - self.target_time) < 2.0:
                                shift = float(np.linalg.norm(p - self.target_world))
                                alpha = 0.18 if shift < 1.5 else 0.40
                                self.target_world = ((1.0 - alpha) * self.target_world + alpha * p).astype(np.float32)
                            else:
                                self.target_world = p.astype(np.float32)
                            self.target_time = now
                            self.active_fusion = {
                                "class_id": active_det.get("class_id"),
                                "is_yolo": bool(active_det.get("is_yolo", False)),
                                "fusion_verdict": active_det.get("fusion_verdict", ""),
                                "_fusion_pos_ned": active_det["_fusion_pos_ned"].copy(),
                                "lidar_verified": bool(active_det.get("lidar_verified", False)),
                                "verdict": active_det.get("verdict", "THERMAL_SIGNAL"),
                                "s_final": float(active_det.get("s_final", 0.0)),
                                "s_geo": float(active_det.get("s_geo", 0.0)),
                                "planarity": float(active_det.get("planarity", 0.0)),
                                "lidar_points": int(active_det.get("lidar_points", 0)),
                                "mean_temp_c": float(active_det.get("mean_lidar_temp_c", active_det.get("temp", 37.0))),
                                "reason": active_det.get("sar_reason", ""),
                            }
                    elif not self.target_locked:
                        with self.lock:
                            if (now - self.target_time) > 0.75:
                                self.target_world = None
                                self.active_fusion = None
                elif not self.target_locked:
                    with self.lock:
                        if (now - self.target_time) > 0.75:
                            self.target_world = None
                            self.active_fusion = None

                # 2. Phat hien tan cay coi (moi 3 chu ky AI de tiet kiem CPU)
                tree_cycle = (tree_cycle + 1) % 3
                trees_det = self.detect_foliage_trees(frame) if tree_cycle == 0 else None

                # 3. Cap nhat ket qua an toan
                with self.lock:
                    if dets is not None:
                        self.detections = dets
                        self.detected_rgb_img = frame_rgb
                        self.detected_ir_img = frame
                    if trees_det is not None:
                        self.detected_trees = trees_det

                # Đồng bộ mục tiêu sang Data/live_targets.json cho Cửa sổ Dựng Địa Hình 3D
                self.export_live_targets_json()
            except Exception:
                error_time = time.monotonic()
                if error_time - getattr(self, "_last_ai_error_log", -math.inf) >= 5.0:
                    self._last_ai_error_log = error_time
                    log.exception("[AI] Lỗi xử lý khung hình; sẽ tiếp tục với ảnh kế tiếp.")

            # Dieu tiet nhip AI o muc 10-12 Hz (80ms) - giu cho RTX 4060 & CPU luon mat va khong bao gio bi drop FPS
            ai_elapsed = time.perf_counter() - ai_loop_start
            ai_sleep = max(0.015, 0.080 - ai_elapsed)
            time.sleep(ai_sleep)

    def lidar_loop(self):
        client = self._connect_client()
        if client is None: return
        last_lidar_ts = 0
        last_t = time.perf_counter()
        last_sync_lidar_loop = 0.0

        while self.running:
            loop_start = time.perf_counter()
            now_t = time.time()
            if now_t - last_sync_lidar_loop >= 0.30:
                last_sync_lidar_loop = now_t
                self._sync_lidar_state(from_self=False)

            if not self.lidar_enabled:
                time.sleep(0.05)
                continue
            try:
                ldata = client.getLidarData(lidar_name=self.lidar_name, vehicle_name=self.vehicle_name)
                if len(ldata.point_cloud) >= 3 and ldata.time_stamp != last_lidar_ts:
                    last_lidar_ts = ldata.time_stamp
                    pts = np.asarray(ldata.point_cloud, dtype=np.float32).reshape(-1, 3)

                    # Lọc khoảng cách chống nhiễu thân UAV & rung vi mô (0.8m đến 115m)
                    pts = filter_sensor_ranges(pts)
                    if len(pts) < 3:
                        time.sleep(0.02)
                        continue

                    # KIỂM TRA POSE ĐỒNG BỘ CỦA SENSOR TỪ ENGINE AIRSIM C++ (ZERO TIME-SKEW)
                    has_lidar_pose = (
                        hasattr(ldata, "pose") and ldata.pose is not None and
                        hasattr(ldata.pose, "position") and hasattr(ldata.pose, "orientation")
                    )
                    if has_lidar_pose:
                        # Dùng trực tiếp ldata.pose đồng bộ 100% thời gian thực từ C++ engine
                        # Triệt tiêu hoàn toàn hiện tượng lệch pha thời gian (time skew), rung lắc đám mây điểm khi bay nhanh hoặc đổi hướng đột ngột
                        R_sensor = quat_to_R(ldata.pose.orientation)
                        pos_sensor = np.array([ldata.pose.position.x_val, ldata.pose.position.y_val, ldata.pose.position.z_val], dtype=np.float32)
                        world_pts = pts @ R_sensor.T + pos_sensor
                        uav_pos = pos_sensor
                    else:
                        with self.lock:
                            uav_pos = np.array([self.telemetry["x"], self.telemetry["y"], self.telemetry["z"]], dtype=np.float32)
                            quat = self.telemetry["quat"]
                        world_pts = self.sensor_to_world(pts, quat, uav_pos)

                    # Đo cao độ thực AGL (Radar Altimeter) từ các điểm địa hình bên dưới UAV
                    # Hoạt động tin cậy từ 0.2m đến 400m trên thung lũng, hẻm núi, vách đá
                    if len(world_pts) > 0:
                        dz_all = world_pts[:, 2] - uav_pos[2]
                        below_mask = dz_all > 0.05
                        if np.any(below_mask):
                            pts_b = world_pts[below_mask]
                            dz_b = dz_all[below_mask]
                            d_xy_b = np.hypot(pts_b[:, 0] - uav_pos[0], pts_b[:, 1] - uav_pos[1])

                            with self.lock:
                                prev_agl = self.terrain_agl
                            curr_h = prev_agl if prev_agl is not None else 10.0
                            r_search = max(16.0, min(95.0, 1.25 * curr_h))

                            near_m = d_xy_b <= r_search
                            if np.count_nonzero(near_m) >= 6:
                                sample_dz = dz_b[near_m]
                            else:
                                n_take = min(len(dz_b), 60)
                                sort_idx = np.argsort(d_xy_b)[:n_take]
                                sample_dz = dz_b[sort_idx]

                            if len(sample_dz) >= 5:
                                agl_calc = float(np.percentile(sample_dz, 80))
                                if 0.2 <= agl_calc <= 450.0:
                                    with self.lock:
                                        if self.terrain_agl is None:
                                            self.terrain_agl = agl_calc
                                        else:
                                            # Lọc trơn EMA 80/20 chống giật số
                                            self.terrain_agl = 0.80 * self.terrain_agl + 0.20 * agl_calc

                    # Preserve the physical AirSim cloud before adding any
                    # visualization-only synthetic foliage points.
                    physical_world_pts = world_pts

                    # Một nguồn LiDAR duy nhất: GCS xuất snapshot world-NED để
                    # Open3D đọc bằng memory-map, không gọi AirSim lần thứ hai.
                    if self.lidar_stream_writer is not None:
                        try:
                            self.lidar_stream_writer.publish(
                                physical_world_pts, uav_pos, timestamp_ns=time.time_ns()
                            )
                        except Exception:
                            pass

                    # Hợp nhất cảm biến Nhiệt-LiDAR: Bổ sung đám mây điểm vòm cây phát hiện được (vectorized, không tốn CPU)
                    with self.lock:
                        trees_fused = list(self.detected_trees)

                    if trees_fused:
                        t_pts = []
                        for t in trees_fused:
                            tx, ty, tz = t["x"], t["y"], t["z"]
                            th, tr = t["h"], t["r"]
                            scale = np.array([tr, tr, th], dtype=np.float32)
                            offset = np.array([tx, ty, tz], dtype=np.float32)
                            t_pts.append(_CANOPY_TEMPLATE_PTS * scale + offset)
                        if t_pts:
                            tree_cloud = np.vstack(t_pts).astype(np.float32)
                            world_pts = np.vstack([world_pts, tree_cloud])

                    # Giới hạn tối đa 8000 điểm mây hiển thị để duy trì 60 FPS mượt mà
                    step = max(1, len(world_pts) // RUNTIME.display_point_budget)
                    sparse = world_pts[::step]

                    # Tiền xử lý dải màu cao độ ngay trong luồng ngầm 10Hz,
                    # tránh chạy np.percentile nặng nề trên luồng đồ họa 60 FPS!
                    pts_colors = None
                    max_elev_disp = 10.0
                    if len(sparse) > 0:
                        z_vals = sparse[:, 2]
                        z_ground = float(np.percentile(z_vals, 95))
                        elev = np.clip(z_ground - z_vals, 0.0, 60.0)
                        max_elev_disp = max(4.0, float(np.percentile(elev, 98)))
                        norm_elev = np.clip(elev / max_elev_disp, 0.0, 1.0)
                        color_indices = (norm_elev * 255.0).astype(np.uint8)
                        cmap_id = self.lidar_colormaps[self.lidar_cmap_idx][1]
                        bgr_pts = cv2.applyColorMap(color_indices, cmap_id)
                        pts_colors = cv2.cvtColor(bgr_pts, cv2.COLOR_BGR2RGB).reshape(-1, 3)

                    with self.lock:
                        self.lidar_pts = sparse
                        self.lidar_full_pts = physical_world_pts
                        self.lidar_recent_scans.append((time.time(), physical_world_pts))
                        self.lidar_colors = pts_colors
                        self.lidar_max_elev = max_elev_disp
                        self.telemetry["points_count"] = len(world_pts)

                    # Ghi dữ liệu tìm kiếm (Nhiệt + LiDAR) vào hàng đợi ngầm
                    if self.recording and self.record_session_dir is not None:
                        t_rec = time.time()
                        if (t_rec - self.last_record_sample_time) >= 0.090:
                            self.last_record_sample_time = t_rec
                            self.record_frame_id += 1
                            fid = self.record_frame_id
                            with self.lock:
                                ir_copy = self.ir_img.copy() if self.ir_img is not None else None
                                telem_copy = dict(self.telemetry)
                                n_dets = len(self.detections)
                            pts_copy = world_pts.copy()
                            ts_str = time.strftime("%Y-%m-%d %H:%M:%S.") + f"{int((t_rec % 1) * 1000):03d}"
                            try:
                                self.record_queue.put_nowait((
                                    self.record_session_dir, fid, ts_str, ir_copy, pts_copy, telem_copy, n_dets
                                ))
                            except queue.Full:
                                pass

                    now_c = time.perf_counter()
                    self.lidar_hz = self.lidar_hz * 0.8 + (1.0 / max(1e-3, now_c - last_t)) * 0.2
                    last_t = now_c
                else:
                    time.sleep(0.04)
            except Exception:
                time.sleep(0.3)
                new_cl = self._connect_client(timeout=15, max_retries=2)
                if new_cl:
                    try: client.client.close()
                    except Exception: pass
                    client = new_cl

            sleep_time = 0.090 - (time.perf_counter() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)
