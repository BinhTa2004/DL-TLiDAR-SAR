"""Mission persistence and evaluation adapters (Report Sections 3.2.7, 3.5.3)."""
import csv
import json
import math
import os
import queue
import time
import cv2
import numpy as np
from configs.config import ROOT, RUNTIME
from .runtime_support import get_logger
from .detection_policy import target_presentation
from evaluation.metrics import (CSV_COLUMNS, calculate_localization_metrics, ensure_csv_columns,
                               load_ground_truth, match_ground_truth, write_summary_csv)
SCRIPT_DIR = str(ROOT)
THERMAL_FOV_DEG = RUNTIME.thermal_fov_deg
log = get_logger("gcs")


class MissionRecordsMixin:
    def mark_person_waypoint(self):
        now_str = time.strftime("%Y-%m-%d %H:%M:%S")
        now = time.time()
        with self.lock:
            px = self.telemetry["x"]
            py = self.telemetry["y"]
            pz = self.telemetry["z"]
            alt = self.telemetry["alt"]
            dets = [dict(d) for d in self.detections]
            rgb_copy = self.detected_rgb_img
            ir_copy = self.detected_ir_img
            marked_positions = [(float(m["tgt_x"]), float(m["tgt_y"])) for m in self.marked_persons]
            used_gt_names = [m.get("gt_name") for m in self.marked_persons if m.get("gt_name")]

        candidates = []
        skipped = 0
        for det in dets:
            pos = det.get("_fusion_pos_ned")
            det.update(target_presentation(det, (px, py, pz)))
            if (not det["can_mark"] or pos is None or
                    now - float(det.get("_fusion_time", 0.0)) > 1.5):
                continue
            pos = np.asarray(pos, dtype=np.float32)
            if pos.shape != (3,) or not np.all(np.isfinite(pos)):
                continue
            tx, ty, tz = (float(pos[0]), float(pos[1]), float(pos[2]))
            if any(math.hypot(tx - mx, ty - my) < 1.0 for mx, my in marked_positions):
                skipped += 1
                continue
            candidates.append((det, tx, ty, tz))
            marked_positions.append((tx, ty))

        if not candidates:
            self.notify(f"[MỤC TIÊU] Chỉ đánh dấu NGƯỜI trong 40m có LiDAR xác nhận "
                        f"({skipped} vị trí đã đánh dấu).")
            return

        ground_truth = load_ground_truth(os.path.join(self.data_dir, "ground_truth_targets.json"))
        verified_indices = [i for i, (det, _, _, _) in enumerate(candidates)
                            if det.get("lidar_verified", False)]
        verified_matches = match_ground_truth(
            [candidates[i][1:] for i in verified_indices], ground_truth, used_gt_names
        )
        matches = [{"status": "NO_LIDAR_VERIFICATION"} for _ in candidates]
        for i, match in zip(verified_indices, verified_matches):
            matches[i] = match
        os.makedirs(self.markers_dir, exist_ok=True)
        ensure_csv_columns(self.csv_path)
        is_new_csv = not os.path.exists(self.csv_path)
        th_color = None
        if ir_copy is not None and getattr(ir_copy, "size", 0) > 0:
            th_gray = ir_copy[:, :, 0] if ir_copy.ndim == 3 else ir_copy
            th_color = self.colorize_thermal(th_gray, self.thermal_palettes[self.palette_idx][1])

        new_markers = []
        with open(self.csv_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            if is_new_csv:
                writer.writerow(CSV_COLUMNS)
            for (det, tgt_x, tgt_y, tgt_z), match in zip(candidates, matches):
                self.marker_counter += 1
                wp_id = self.marker_counter
                box = tuple(int(v) for v in det["bbox"])
                photo_rgb_name = f"person_{wp_id:03d}_RGB.jpg" if rgb_copy is not None else ""
                photo_thermal_name = f"person_{wp_id:03d}_THERMAL.jpg" if th_color is not None else ""
                if rgb_copy is not None:
                    ann_rgb = rgb_copy.copy()
                    cv2.rectangle(ann_rgb, box[:2], box[2:], (255, 0, 0), 2)
                    cv2.putText(ann_rgb, f"TARGET PERSON #{wp_id}", (box[0], max(14, box[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    cv2.imwrite(os.path.join(self.markers_dir, photo_rgb_name), cv2.cvtColor(ann_rgb, cv2.COLOR_RGB2BGR))
                if th_color is not None:
                    ann_th = cv2.cvtColor(th_color.copy(), cv2.COLOR_RGB2BGR)
                    cv2.rectangle(ann_th, box[:2], box[2:], (0, 0, 255), 2)
                    cv2.putText(ann_th, f"FLIR TARGET #{wp_id}", (box[0], max(14, box[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.imwrite(os.path.join(self.markers_dir, photo_thermal_name), ann_th)

                dist_from_uav = math.hypot(tgt_x - px, tgt_y - py)
                gt_xyz = match.get("xyz", (None, None, None))
                writer.writerow([
                    wp_id, now_str, f"{px:.2f}", f"{py:.2f}", f"{pz:.2f}",
                    f"{alt:.2f}", f"{tgt_x:.2f}", f"{tgt_y:.2f}",
                    f"{dist_from_uav:.2f}", f"{self.gimbal_pitch:.1f}",
                    photo_rgb_name, photo_thermal_name,
                    match.get("name", ""),
                    *[f"{value:.2f}" if value is not None else "" for value in gt_xyz],
                    f"{match['error_2d_m']:.2f}" if "error_2d_m" in match else "",
                    f"{match['error_3d_m']:.2f}" if "error_3d_m" in match else "",
                    match["status"],
                    "THERMAL_LIDAR" if det.get("lidar_verified", False) else "THERMAL_RAY_TERRAIN",
                    int(bool(det.get("lidar_verified", False))),
                    f"{match['error_2d_m']:.2f}" if "error_2d_m" in match else "",
                    f"{match['vertical_error_m']:.2f}" if "vertical_error_m" in match else "",
                ])
                new_markers.append({
                    "id": wp_id, "time": time.strftime("%H:%M:%S"),
                    "uav_x": px, "uav_y": py,
                    "tgt_x": tgt_x, "tgt_y": tgt_y, "tgt_z": tgt_z,
                    "alt": alt, "dist": dist_from_uav,
                    "verdict": det.get("verdict", "UNVERIFIED_FAR_TARGET"),
                    "s_final": float(det.get("s_final", 0.0)),
                    "lidar_points": int(det.get("lidar_points", 0)),
                    "reason": det.get("sar_reason", ""),
                    "gt_name": match.get("name", ""),
                    "error_2d_m": match.get("error_2d_m"),
                    "error_3d_m": match.get("error_3d_m"),
                    "vertical_error_m": match.get("vertical_error_m"),
                    "position_source": "THERMAL_LIDAR" if det.get("lidar_verified", False) else "THERMAL_RAY_TERRAIN",
                    "lidar_verified": bool(det.get("lidar_verified", False)),
                    "classification": "person",
                    "distance_3d_m": det["distance_m"],
                })

        with self.lock:
            self.marked_persons.extend(new_markers)
        self.export_live_targets_json(force=True)
        self.write_localization_summary()
        matched_count = sum(match["status"] == "MATCHED" for match in matches)
        error_summary = ", ".join(
            f"#{marker['id']}: {marker['error_2d_m']:.2f}m"
            for marker in new_markers if marker["error_2d_m"] is not None
        )
        self.notify(f"[ĐÁNH DẤU] Thêm {len(new_markers)} người, bỏ qua {skipped} trùng; "
                    f"sai số nhiệt+LiDAR 2D ({matched_count} người): {error_summary or 'chưa có dữ liệu hợp lệ'}.")

    def write_localization_summary(self):
        with self.lock:
            markers = [dict(marker) for marker in self.marked_persons]
        truth = load_ground_truth(os.path.join(self.data_dir, "ground_truth_targets.json"))
        metrics = calculate_localization_metrics(markers, truth)
        write_summary_csv(os.path.join(self.data_dir, "rescue_localization_metrics.csv"), metrics)

    def clear_marked_persons(self):
        """Xóa sạch các mốc đánh dấu nạn nhân cũ (Phím F8)."""
        with self.lock:
            self.marked_persons.clear()
            self.marker_counter = 0
            self.target_locked = False
            self.locked_pos = None
        self.export_live_targets_json(force=True)
        self.write_localization_summary()
        self.notify("[MỤC TIÊU] ĐÃ XÓA TOÀN BỘ CÁC MỐC ĐÁNH DẤU NẠN NHÂN [F8]!")

    def toggle_rescue_routes(self):
        """Bật/tắt đường thẳng từ vị trí khởi hành tới mọi người đã đánh dấu."""
        if not self._debounce("toggle_rescue_routes", 0.35):
            return
        with self.lock:
            self.route_lines_enabled = not self.route_lines_enabled
            enabled = self.route_lines_enabled
            count = len(self.marked_persons)
        self.export_live_targets_json(force=True)
        state = "HIỆN" if enabled else "ẨN"
        self.notify(f"[BẢN ĐỒ 3D] {state} ĐƯỜNG TỪ ĐIỂM XUẤT PHÁT ĐẾN {count} VỊ TRÍ ĐÃ ĐÁNH DẤU [D-PAD LÊN]")

    def export_live_targets_json(self, force=False):
        """
        Xuất danh sách mục tiêu phát hiện và nạn nhân đã đánh dấu sang Data/live_targets.json
        để cửa sổ Dựng Địa Hình 3D (uav_lidar_3d_pointr.py) hiển thị khung đánh dấu chuẩn xác thời gian thực.
        """
        now = time.time()
        if not force and now - getattr(self, "_last_export_t", 0.0) < 0.20:
            return
        self._last_export_t = now

        try:
            with self.lock:
                px = float(self.telemetry.get("x", 0.0))
                py = float(self.telemetry.get("y", 0.0))
                pz = float(self.telemetry.get("z", 0.0))
                markers = list(self.marked_persons)
                start_pos = self.start_pos_ned
                routes_enabled = self.route_lines_enabled
                dets = [dict(d) for d in self.detections]
                tgt3d = self.target_world.copy() if (self.target_world is not None and (time.time() - self.target_time) < 3.0) else None
                active_fusion = dict(self.active_fusion) if self.active_fusion else None
                if self.target_locked and self.locked_pos is not None:
                    tgt3d = self.locked_pos.copy()
                    active_fusion = dict(self.locked_fusion) if self.locked_fusion else None

            out_data = {
                "session_id": getattr(self, "session_id", None),
                "timestamp": now,
                "uav_pos_ned": [px, py, pz],
                "uav_pos_enu": [py, px, -pz],
                "start_pos_ned": list(start_pos) if start_pos is not None else None,
                "start_pos_enu": [start_pos[1], start_pos[0], -start_pos[2]] if start_pos is not None else None,
                "route_lines_enabled": routes_enabled,
                "lidar_enabled": bool(self.lidar_enabled),
                "active_target": None,
                "marked_persons": [],
                "live_detections": []
            }

            if tgt3d is not None:
                tx, ty, tz = float(tgt3d[0]), float(tgt3d[1]), float(tgt3d[2])
                active = dict(active_fusion or {})
                active.setdefault("_fusion_pos_ned", tgt3d)
                if self.target_locked:
                    active["_fusion_pos_ned"] = tgt3d
                presentation = target_presentation(active, (px, py, pz))
                out_data["active_target"] = {
                    "x_ned": tx, "y_ned": ty, "z_ned": tz,
                    "x_enu": ty, "y_enu": tx, "z_enu": -tz,
                    "x": ty, "y": tx, "z": -tz,
                    "dx": 1.2, "dy": 1.2, "dz": 1.8,
                    "locked": bool(self.target_locked),
                    **presentation,
                    "s_final": float((active_fusion or {}).get("s_final", 1.0)),
                    "s_geo": float((active_fusion or {}).get("s_geo", 0.0)),
                    "planarity": float((active_fusion or {}).get("planarity", 0.0)),
                    "lidar_points": int((active_fusion or {}).get("lidar_points", 0)),
                    "mean_temp_c": float((active_fusion or {}).get("mean_temp_c", 37.0)),
                    "reason": (active_fusion or {}).get("reason", "")
                }

            for m in markers:
                mx = float(m["tgt_x"])
                my = float(m["tgt_y"])
                mz = float(m.get("tgt_z", self.ground_z_datum))
                out_data["marked_persons"].append({
                    "id": m["id"],
                    "time": m.get("time", ""),
                    "x_ned": mx, "y_ned": my, "z_ned": mz,
                    "x_enu": my, "y_enu": mx, "z_enu": -mz,
                    "x": my, "y": mx, "z": -mz,
                    "dx": 1.2, "dy": 1.2, "dz": 1.8,
                    "dist": float(m.get("dist", 0.0)),
                    "verdict": m.get("verdict", "UNVERIFIED_FAR_TARGET"),
                    "s_final": float(m.get("s_final", 0.0)),
                    "lidar_points": int(m.get("lidar_points", 0)),
                    "reason": m.get("reason", ""),
                    "label": f"NAN NHAN #{m['id']}"
                })

            for idx, d in enumerate(dets):
                det_3d = d.get("_fusion_pos_ned")
                if det_3d is None or (now - float(d.get("_fusion_time", 0.0))) > 1.0:
                    continue
                if det_3d is not None:
                    dx, dy, dz = float(det_3d[0]), float(det_3d[1]), float(det_3d[2])
                    if not all(math.isfinite(v) for v in (dx, dy, dz)):
                        continue
                    presentation = target_presentation(d, (px, py, pz))
                    out_data["live_detections"].append({
                        "id": idx + 1,
                        "x_ned": dx, "y_ned": dy, "z_ned": dz,
                        "x_enu": dy, "y_enu": dx, "z_enu": -dz,
                        "x": dy, "y": dx, "z": -dz,
                        "dx": 1.2, "dy": 1.2, "dz": 1.8,
                        "conf": float(d.get("conf", 0.8)),
                        "s_final": float(d.get("s_final", 0.0)),
                        "s_geo": float(d.get("s_geo", 0.0)),
                        **presentation,
                        "planarity": float(d.get("planarity", 0.0)),
                        "lidar_points": int(d.get("lidar_points", 0)),
                        "mean_temp_c": float(d.get("mean_lidar_temp_c", d.get("temp", 37.0))),
                        "reason": d.get("sar_reason", ""),
                        "temp": float(d.get("temp", 37.0)),
                        "label": f"{presentation['label']} #{idx+1}"
                    })

            data_dir = getattr(self, "data_dir", os.path.join(SCRIPT_DIR, "outputs"))
            os.makedirs(data_dir, exist_ok=True)
            target_path = os.path.join(data_dir, "live_targets.json")
            tmp_path = f"{target_path}.{os.getpid()}.tmp"
            try:
                with open(tmp_path, "w", encoding="utf-8") as f:
                    json.dump(out_data, f, indent=2)
                for _ in range(5):
                    try:
                        os.replace(tmp_path, target_path)
                        break
                    except PermissionError:
                        time.sleep(0.01)
                else:
                    with open(target_path, "w", encoding="utf-8") as f:
                        json.dump(out_data, f, indent=2)
                    try:
                        os.remove(tmp_path)
                    except OSError:
                        pass
            except (PermissionError, OSError):
                pass
        except Exception:
            pass

    def start_recording(self):
        """Bật chế độ ghi dữ liệu tìm kiếm (Lần ấn 1)."""
        now_str = time.strftime("%Y%m%d_%H%M%S")
        self.record_session_dir = os.path.join(self.data_dir, f"session_{now_str}")
        os.makedirs(os.path.join(self.record_session_dir, "thermal"), exist_ok=True)
        os.makedirs(os.path.join(self.record_session_dir, "lidar"), exist_ok=True)

        csv_path = os.path.join(self.record_session_dir, "telemetry_log.csv")
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow([
                "FRAME_ID", "TIMESTAMP", "FLIGHT_TIME_S",
                "UAV_X", "UAV_Y", "UAV_Z", "ALT_AGL", "SPEED",
                "PITCH", "ROLL", "YAW", "GIMBAL_PITCH", "LIDAR_PTS",
                "PERSONS_DETECTED", "THERMAL_FILE", "LIDAR_FILE"
            ])

        meta_path = os.path.join(self.record_session_dir, "session_info.json")
        with open(meta_path, 'w', encoding='utf-8') as f:
            json.dump({
                "status": "recording",
                "start_time": time.strftime("%Y-%m-%d %H:%M:%S"),
                "vehicle": self.vehicle_name,
                "flir_camera": f"FLIR Boson {self.cam_w}x{self.cam_h} FOV {THERMAL_FOV_DEG}",
                "lidar": "Ouster OS1-64 (64 Channels, 120m)",
                "save_directory": self.record_session_dir,
                "readme": "Ảnh nhiệt lưu trong thermal/ (.jpg). Mây điểm LiDAR lưu trong lidar/ (.npy - mảng float32 [X,Y,Z] trong tọa độ thế giới)."
            }, f, indent=2, ensure_ascii=False)

        readme_path = os.path.join(self.data_dir, "HUONG_DAN_DOC_DU_LIEU.txt")
        if not os.path.exists(readme_path):
            try:
                with open(readme_path, 'w', encoding='utf-8') as f:
                    f.write(
                        "===================================================================\n"
                        "HƯỚNG DẪN ĐỌC DỮ LIỆU TÌM KIẾM CỨU HỘ (NHIỆT + LIDAR)\n"
                        "===================================================================\n\n"
                        "1. CẤU TRÚC THƯ MỤC:\n"
                        "   - Data/session_YYYYMMDD_HHMMSS/thermal/ : Ảnh nhiệt FLIR (.jpg)\n"
                        "   - Data/session_YYYYMMDD_HHMMSS/lidar/   : Mây điểm LiDAR (.npy)\n"
                        "   - Data/session_YYYYMMDD_HHMMSS/telemetry_log.csv : Tọa độ & tư thế đồng bộ\n"
                        "   - Data/session_YYYYMMDD_HHMMSS/session_info.json : Thông số phiên ghi\n\n"
                        "2. ĐỌC MÂY ĐIỂM LIDAR BẰNG PYTHON:\n"
                        "   import numpy as np\n"
                        "   points = np.load('Data/session_.../lidar/lidar_000001.npy')\n"
                        "   print('Kích thước mây điểm:', points.shape)  # (N, 3) dạng float32\n"
                        "   # X, Y, Z đã được chuyển đổi sang hệ tọa độ thế giới (mét)\n\n"
                        "3. ĐỌC ẢNH NHIỆT:\n"
                        "   Ảnh .jpg trong thermal/ có thể mở trực tiếp bằng bất kỳ trình xem ảnh nào.\n"
                    )
            except Exception:
                pass

        self.record_frame_id = 0
        self.record_start_time = time.time()
        self.recording = True
        self.notify("[GHI DỮ LIỆU] ĐÃ BẬT GHI! Bấm D-Pad Xuống (↓) hoặc phím K để TẮT & LƯU")
        log.info(f">> [RECORDER] Bắt đầu ghi dữ liệu tìm kiếm (Nhiệt + LiDAR) vào: {self.record_session_dir}")

    def stop_recording(self):
        """Tắt chế độ ghi dữ liệu tìm kiếm và hoàn tất lưu (Lần ấn 2)."""
        if not self.recording:
            return
        self.recording = False
        folder_name = os.path.basename(self.record_session_dir) if self.record_session_dir else "Data"
        dur = time.time() - self.record_start_time if self.record_start_time > 0 else 0.0

        # Cập nhật session_info.json trạng thái hoàn thành
        if self.record_session_dir:
            try:
                meta_path = os.path.join(self.record_session_dir, "session_info.json")
                if os.path.exists(meta_path):
                    with open(meta_path, 'r', encoding='utf-8') as f:
                        meta = json.load(f)
                    meta["status"] = "completed"
                    meta["end_time"] = time.strftime("%Y-%m-%d %H:%M:%S")
                    meta["duration_sec"] = round(dur, 2)
                    meta["total_frames"] = self.record_frame_id
                    with open(meta_path, 'w', encoding='utf-8') as f:
                        json.dump(meta, f, indent=2, ensure_ascii=False)
            except Exception:
                pass

        self.notify(f"[GHI DỮ LIỆU] ĐÃ TẮT & LƯU XONG: {self.record_frame_id} KHUNG HÌNH ({dur:.1f}s) VÀO Data/{folder_name}")
        log.info(f">> [RECORDER] ĐÃ TẮT GHI (D-PAD ↓ / LẦN 2). Đã lưu {self.record_frame_id} khung hình ({dur:.1f}s) vào Data/{folder_name}")

    def toggle_recording(self):
        """Chuyển đổi trạng thái Ghi: Ấn lần 1 để BẬT, ấn lần 2 để TẮT & LƯU."""
        with self.record_lock:
            now = time.time()
            if now - self._last_record_toggle_time < 0.30:
                return  # Chống dội phím hoặc double-click
            self._last_record_toggle_time = now

            if not self.recording:
                self.start_recording()
            else:
                self.stop_recording()

    def data_recorder_worker(self):
        """Luồng ngầm ghi dữ liệu ra ổ đĩa (0ms độ trễ cho rendering và bay)."""
        while self.running:
            try:
                item = self.record_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:
                break
            session_dir, fid, ts_str, ir_img, pts, telem, n_dets = item
            try:
                # 1. Lưu ảnh nhiệt FLIR
                th_fname = f"thermal_{fid:06d}.jpg"
                th_path = os.path.join(session_dir, "thermal", th_fname)
                if ir_img is not None and getattr(ir_img, "size", 0) > 0:
                    th_gray = ir_img[:, :, 0] if ir_img.ndim == 3 else ir_img
                    pal_name, pal_mode = self.thermal_palettes[self.palette_idx]
                    rgb_th = self.colorize_thermal(th_gray, pal_mode)
                    bgr_th = cv2.cvtColor(rgb_th, cv2.COLOR_RGB2BGR)
                    cv2.putText(bgr_th, f"UAV X:{telem.get('x',0.0):+.1f} Y:{telem.get('y',0.0):+.1f} Alt:{telem.get('alt',0.0):.1f}m G:{self.gimbal_pitch:.0f}d",
                                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                    cv2.imwrite(th_path, bgr_th, [int(cv2.IMWRITE_JPEG_QUALITY), 90])

                # 2. Lưu mây điểm LiDAR Ouster OS1-64 (.npy)
                lid_fname = f"lidar_{fid:06d}.npy"
                lid_path = os.path.join(session_dir, "lidar", lid_fname)
                if pts is not None and len(pts) > 0:
                    np.save(lid_path, pts.astype(np.float32))

                # 3. Ghi dòng dữ liệu đồng bộ vào CSV
                csv_path = os.path.join(session_dir, "telemetry_log.csv")
                with open(csv_path, 'a', newline='', encoding='utf-8') as f:
                    w = csv.writer(f)
                    w.writerow([
                        fid, ts_str, f"{telem.get('flight_time', 0.0):.2f}",
                        f"{telem.get('x', 0.0):.2f}", f"{telem.get('y', 0.0):.2f}", f"{telem.get('z', 0.0):.2f}",
                        f"{telem.get('alt', 0.0):.2f}", f"{telem.get('speed', 0.0):.2f}",
                        f"{telem.get('pitch', 0.0):.1f}", f"{telem.get('roll', 0.0):.1f}", f"{telem.get('yaw', 0.0):.1f}",
                        f"{self.gimbal_pitch:.1f}", len(pts) if pts is not None else 0,
                        n_dets, th_fname, lid_fname
                    ])
            except Exception:
                pass
            finally:
                self.record_queue.task_done()
