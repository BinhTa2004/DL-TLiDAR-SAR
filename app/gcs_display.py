"""Pygame HUD, RGB/thermal panes, radar and display interaction."""
import math
import time
import cv2
import numpy as np
import pygame
from .runtime_support import get_logger
from .detection_policy import COLORS, target_presentation, detection_caption
from configs.config import RUNTIME
from .layout import CANVAS_H, CANVAS_W, DECK_H, DECK_Y, FOOTER_H, HEADER_H, PANE_H, PANE_HDR, PANE_W, SIDE_W, SIDE_X

log = get_logger("gcs")
THERMAL_FOV_DEG = RUNTIME.thermal_fov_deg


class GCSDisplayMixin:
    """Render application state; never selects LiDAR clusters or commands flight."""

    def font(self, size, bold=False, mono=False):
        key = (size, bold, mono)
        f = self._fonts.get(key)
        if f is None:
            f = pygame.font.SysFont("Consolas" if mono else "Segoe UI", size, bold=bold)
            self._fonts[key] = f
        return f

    def text(self, surf, s, pos, size=12, color=(235, 240, 245), bold=False, mono=False, anchor="topleft"):
        cache_key = (s, size, color, bold, mono)
        img = self._text_cache.get(cache_key)
        if img is None:
            if len(self._text_cache) > 400:
                self._text_cache.clear()
            img = self.font(size, bold, mono).render(s, True, color)
            self._text_cache[cache_key] = img
        r = img.get_rect()
        setattr(r, anchor, pos)
        surf.blit(img, r)
        return r

    def draw_header(self, surf, title, accent, subtitle=None, sub_color=(200, 215, 230)):
        w = surf.get_width()
        pygame.draw.rect(surf, (17, 23, 33), (0, 0, w, PANE_HDR))
        pygame.draw.rect(surf, accent, (0, PANE_HDR - 2, w, 2))
        r = self.text(surf, title, (10, 6), 12, accent, bold=True)
        if subtitle:
            self.text(surf, subtitle, (r.right + 12, 8), 11, sub_color, bold=False)

    @classmethod
    def fit_array(cls, rgb, area_w, area_h):
        h, w = rgb.shape[:2]
        s = min(area_w / float(w), area_h / float(h))
        dw, dh = max(1, int(w * s)), max(1, int(h * s))
        resized = cv2.resize(rgb, (dw, dh), interpolation=cv2.INTER_LINEAR)
        key = (area_h, area_w)
        if key not in cls._CANVAS_CACHE:
            cls._CANVAS_CACHE[key] = np.zeros((area_h, area_w, 3), dtype=np.uint8)
        canvas = cls._CANVAS_CACHE[key].copy()
        ox, oy = (area_w - dw) // 2, (area_h - dh) // 2
        canvas[oy:oy + dh, ox:ox + dw] = resized
        return canvas, ox, oy, dw, dh

    @staticmethod
    def array_to_surface(arr):
        if not arr.flags['C_CONTIGUOUS']:
            arr = np.ascontiguousarray(arr)
        surf = pygame.image.frombuffer(arr, (arr.shape[1], arr.shape[0]), "RGB")
        try:
            return surf.convert()
        except Exception:
            return surf

    @staticmethod
    def colorize_thermal(th_gray, pal_mode):
        if th_gray is None or getattr(th_gray, "size", 0) == 0:
            return np.zeros((10, 10, 3), dtype=np.uint8)
        if th_gray.ndim == 3:
            th_gray = th_gray[:, :, 0]
        if th_gray.shape[0] == 0 or th_gray.shape[1] == 0:
            return np.zeros((10, 10, 3), dtype=np.uint8)
        try:
            if pal_mode == "white_hot": return cv2.cvtColor(th_gray, cv2.COLOR_GRAY2RGB)
            if pal_mode == "black_hot": return cv2.cvtColor(255 - th_gray, cv2.COLOR_GRAY2RGB)
            if pal_mode == "ironbow": return cv2.cvtColor(cv2.applyColorMap(th_gray, cv2.COLORMAP_INFERNO), cv2.COLOR_BGR2RGB)
            if pal_mode == "rainbow": return cv2.cvtColor(cv2.applyColorMap(th_gray, cv2.COLORMAP_JET), cv2.COLOR_BGR2RGB)
            return cv2.cvtColor(th_gray, cv2.COLOR_GRAY2RGB)
        except Exception:
            return np.zeros((max(1, th_gray.shape[0]), max(1, th_gray.shape[1]), 3), dtype=np.uint8)

    def render_thermal_pane(self):
        surf = self.panes["thermal"]
        w, h = surf.get_size()
        surf.fill((10, 14, 20))
        if not self.thermal_enabled:
            accent = (255, 170, 50)
            self.draw_header(surf, "CAMERA NHIỆT FLIR", accent, "ĐÃ TẮT [D-PAD ◀ / PHÍM U ĐỂ BẬT]")
            self.text(surf, "○ TẮT", (w - 10, 7), 12, (130, 140, 150), bold=True, anchor="topright")
            self.text(surf, "[CAMERA NHIỆT FLIR: ĐANG TẮT]", (w // 2, h // 2 - 12), 14, (255, 170, 50), bold=True, anchor="center")
            self.text(surf, "Bấm D-Pad Trái ◀ hoặc phím [U] để BẬT", (w // 2, h // 2 + 14), 11, (170, 185, 205), anchor="center")
            pygame.draw.rect(surf, accent, (0, 0, w, h), 1)
            return

        with self.lock:
            ir = getattr(self, "_display_ir", self.ir_img)
            uav_pos = (self.telemetry["x"], self.telemetry["y"], self.telemetry["z"])
            dets = [dict(d) for d in self.detections]
        for d in dets:
            d.update(target_presentation(d, uav_pos))

        palette_name, pal_mode = self.thermal_palettes[self.palette_idx]
        accent = (255, 170, 50)
        ai_badge = f" [AI YOLO {self.yolo_fps:.0f}FPS]" if (self.yolo_enabled and self.yolo_model) else ""
        self.draw_header(surf, "CAMERA NHIỆT FLIR", accent,
                         f"({self.cam_w}x{self.cam_h} | FOV {THERMAL_FOV_DEG:.0f}°)  {palette_name}{ai_badge}")
        n_yolo = sum(1 for d in dets if d.get("is_yolo", False))
        n_verified = sum(1 for d in dets if d.get("can_mark", False))
        n_animals = sum(1 for d in dets if d.get("classification") == "animal")
        n_heat = sum(1 for d in dets if d.get("classification") == "thermal")
        if getattr(self, "show_bbox", True):
            if self.yolo_enabled:
                tag_col = (0, 255, 140) if n_yolo else (120, 200, 160)
                tag_txt = (f"● NGƯỜI XN: {n_verified} | ĐV: {n_animals} | NHIỆT: {n_heat}"
                           if dets else "● YOLO+LIDAR: ĐANG QUÉT...")
            else:
                tag_col = (130, 140, 150)
                tag_txt = f"● YOLO TẮT | LIDAR XN: {n_verified}"
            self.text(surf, tag_txt, (w - 10, 7), 12, tag_col, bold=True, anchor="topright")
        else:
            self.text(surf, "○ KHUNG [O]: TẮT", (w - 10, 7), 11, (130, 140, 150), bold=True, anchor="topright")

        strip_h = 22
        area = pygame.Rect(0, PANE_HDR, w, h - PANE_HDR - strip_h)
        pygame.draw.rect(surf, (14, 18, 26), (0, h - strip_h, w, strip_h))

        if ir is not None and getattr(ir, "size", 0) > 0 and ir.ndim >= 2 and ir.shape[0] > 0 and ir.shape[1] > 0:
            th_gray = ir[:, :, 0] if ir.ndim == 3 else ir
            h_im, w_im = th_gray.shape[:2]

            if ir is not getattr(self, "_last_ir_arr", None) or pal_mode != getattr(self, "_last_pal_mode", None) or getattr(self, "_cached_th_surf", None) is None:


                rgb_th = self.colorize_thermal(th_gray, pal_mode)


                canvas, ox, oy, dw, dh = self.fit_array(rgb_th, area.w, area.h)


                self._cached_th_surf = self.array_to_surface(canvas)


                self._cached_th_ox = ox


                self._cached_th_oy = oy


                self._cached_th_dw = dw


                self._cached_th_dh = dh


                self._last_ir_arr = ir


                self._last_pal_mode = pal_mode


            surf.blit(self._cached_th_surf, (area.x, area.y))


            ox = self._cached_th_ox + area.x


            oy = self._cached_th_oy + area.y


            dw = self._cached_th_dw


            dh = self._cached_th_dh


            sc = dw / float(w_im)

            ret_col = (0, 255, 180) if pal_mode in ("white_hot", "ironbow") else (255, 50, 50)
            cx, cy = ox + dw // 2, oy + dh // 2
            pygame.draw.line(surf, ret_col, (cx - 18, cy), (cx - 6, cy), 1)
            pygame.draw.line(surf, ret_col, (cx + 6, cy), (cx + 18, cy), 1)
            pygame.draw.line(surf, ret_col, (cx, cy - 18), (cx, cy - 6), 1)
            pygame.draw.line(surf, ret_col, (cx, cy + 6), (cx, cy + 18), 1)
            pygame.draw.circle(surf, ret_col, (cx, cy), 2, 1)

            center_val = float(th_gray[h_im // 2, w_im // 2])
            self.text(surf, f"{self.temp_of(center_val):.1f}°C", (cx + 12, cy + 8), 12, ret_col, bold=True, mono=True)

            if getattr(self, "show_bbox", True):
                try:
                    for i, d in enumerate(dets):
                        x0, y0, x1, y1 = d["bbox"]
                        rx0, ry0 = ox + int(x0 * sc), oy + int(y0 * sc)
                        rx1, ry1 = ox + int(x1 * sc), oy + int(y1 * sc)
                        bw_box = max(8, rx1 - rx0)
                        bh_box = max(8, ry1 - ry0)
                        box_col = COLORS[d["classification"]]

                        # Khung nhận diện chính
                        pygame.draw.rect(surf, box_col, (rx0, ry0, bw_box, bh_box), 2)
                        corner_len = min(12, min(bw_box, bh_box) // 3)
                        for cx_i, cy_i, dx, dy in [
                            (rx0, ry0, 1, 1), (rx0 + bw_box, ry0, -1, 1),
                            (rx0, ry0 + bh_box, 1, -1), (rx0 + bw_box, ry0 + bh_box, -1, -1)
                        ]:
                            pygame.draw.line(surf, (255, 255, 255), (cx_i, cy_i), (cx_i + dx * corner_len, cy_i), 2)
                            pygame.draw.line(surf, (255, 255, 255), (cx_i, cy_i), (cx_i, cy_i + dy * corner_len), 2)

                        # Tâm ngắm mục tiêu
                        mcx, mcy = rx0 + bw_box // 2, ry0 + bh_box // 2
                        pygame.draw.circle(surf, box_col, (mcx, mcy), 3)

                        # Nhãn thông tin gọn gàng & nền trong suốt (không che khuất cảnh)
                        lbl = f"[{detection_caption(d)}]"
                        f_mono = self.font(11, bold=True, mono=True)
                        lbl_surf = f_mono.render(lbl, True, box_col)
                        shadow_surf = f_mono.render(lbl, True, (0, 0, 0))
                        lw, lh = lbl_surf.get_size()
                        lbl_y = max(area.y + 2, ry0 - lh - 4)

                        # Nền trong suốt mờ nhẹ (translucent 70% see-through)
                        bg_trans = pygame.Surface((lw + 6, lh + 2), pygame.SRCALPHA)
                        bg_trans.fill((0, 0, 0, 75))
                        surf.blit(bg_trans, (rx0 - 1, lbl_y - 1))

                        # Đổ bóng chữ đen mảnh để chữ luôn rõ nét trên mọi nền
                        for sx, sy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                            surf.blit(shadow_surf, (rx0 + 2 + sx, lbl_y + sy))
                        surf.blit(lbl_surf, (rx0 + 2, lbl_y))
                except Exception:
                    pass

            if ox >= 34:


                if pal_mode != getattr(self, "_last_cbar_pal", None) or dh != getattr(self, "_last_cbar_dh", None) or getattr(self, "_cached_th_cbar", None) is None:


                    grad = np.linspace(255, 0, dh, dtype=np.uint8).reshape(-1, 1)


                    bar = self.colorize_thermal(np.repeat(grad, 8, axis=1), pal_mode)


                    self._cached_th_cbar = self.array_to_surface(bar)


                    self._last_cbar_pal = pal_mode


                    self._last_cbar_dh = dh


                surf.blit(self._cached_th_cbar, (10, oy))
                pygame.draw.rect(surf, (150, 170, 190), (10, oy, 8, dh), 1)
                self.text(surf, f"{self.temp_of(255):.0f}°", (21, oy - 2), 10, (220, 230, 240), mono=True)
                self.text(surf, f"{self.temp_of(0):.0f}°", (21, oy + dh - 12), 10, (220, 230, 240), mono=True)

            th_min, th_max = self.temp_of(th_gray.min()), self.temp_of(th_gray.max())
            self.text(surf, f"MAX: {th_max:.1f}°C   MIN: {th_min:.1f}°C   CENTER: {self.temp_of(center_val):.1f}°C   GÓC CAM: {self.gimbal_pitch:.0f}°",
                      (10, h - strip_h + 4), 11, (220, 240, 255), mono=True)
        else:
            self.text(surf, "[ĐANG KẾT NỐI CAMERA NHIỆT...]", (w // 2, h // 2), 14, (255, 120, 50), anchor="center")

        pygame.draw.rect(surf, accent, (0, 0, w, h), 1)

    def render_rgb_pane(self):
        surf = self.panes["rgb"]
        w, h = surf.get_size()
        surf.fill((10, 14, 20))

        with self.lock:
            fpv = getattr(self, "_display_rgb", self.rgb_img)
            chase = getattr(self, "_display_chase", self.chase_img)
            alt = self.telemetry["alt"]
            speed = self.telemetry["speed"]

        accent = (0, 220, 140)
        self.draw_header(surf, "CAMERA RGB & GÓC NHÌN TOÀN CẢNH", accent)

        tabs = [("FPV BỤNG", 0), ("CHASE CAM", 1)]
        tx = w - 10
        for label, idx in reversed(tabs):
            active = (self.rgb_view_target == idx)
            tw = self.font(11, True).size(label)[0] + 18
            tx -= tw
            r_tab = pygame.Rect(tx, 5, tw, 20)
            pygame.draw.rect(surf, accent if active else (35, 45, 58), r_tab, border_radius=4)
            self.text(surf, label, (tx + tw // 2, 15), 11, (5, 20, 15) if active else (170, 185, 200), bold=True, anchor="center")
            tx -= 6
        self.text(surf, "[V]", (tx - 2, 8), 11, (130, 150, 170), bold=True, anchor="topright")

        strip_h = 22
        area = pygame.Rect(0, PANE_HDR, w, h - PANE_HDR - strip_h)
        pygame.draw.rect(surf, (14, 18, 26), (0, h - strip_h, w, strip_h))

        img_show = chase if (self.rgb_view_target == 1 and chase is not None) else fpv


        if img_show is not None and getattr(img_show, "size", 0) > 0 and img_show.ndim >= 2 and img_show.shape[0] > 0 and img_show.shape[1] > 0:
            if img_show is not getattr(self, "_last_rgb_arr", None) or getattr(self, "_cached_rgb_surf", None) is None:
                canvas, ox, oy, dw, dh = self.fit_array(img_show, area.w, area.h)
                self._cached_rgb_surf = self.array_to_surface(canvas)
                self._cached_rgb_ox = ox
                self._cached_rgb_oy = oy
                self._cached_rgb_dw = dw
                self._cached_rgb_dh = dh
                self._last_rgb_arr = img_show

            surf.blit(self._cached_rgb_surf, (area.x, area.y))

            # RGB: ứng viên YOLO chờ LiDAR màu cam; LiDAR xác nhận màu cyan.
            if self.rgb_view_target == 0 and getattr(self, "show_bbox", True):
                try:
                    ox_rgb = getattr(self, "_cached_rgb_ox", 0) + area.x
                    oy_rgb = getattr(self, "_cached_rgb_oy", 0) + area.y
                    dw_rgb = getattr(self, "_cached_rgb_dw", area.w)
                    w_im_rgb = img_show.shape[1]
                    sc_rgb = dw_rgb / float(w_im_rgb) if w_im_rgb > 0 else 1.0

                    with self.lock:
                        uav_pos = (self.telemetry["x"], self.telemetry["y"], self.telemetry["z"])
                        dets_rgb = [dict(d) for d in self.detections]

                    for i, d in enumerate(dets_rgb):
                        d.update(target_presentation(d, uav_pos))
                        lidar_confirmed = (d["verdict"] in ("CONFIRMED_HUMAN", "CONFIRMED_ANIMAL")
                                           and d.get("lidar_verified", False))
                        x0, y0, x1, y1 = d["bbox"]
                        rx0, ry0 = ox_rgb + int(x0 * sc_rgb), oy_rgb + int(y0 * sc_rgb)
                        rx1, ry1 = ox_rgb + int(x1 * sc_rgb), oy_rgb + int(y1 * sc_rgb)
                        bw_box = max(8, rx1 - rx0)
                        bh_box = max(8, ry1 - ry0)
                        box_col = (0, 220, 255) if lidar_confirmed else COLORS[d["classification"]]
                        if d["classification"] == "person" and not lidar_confirmed:
                            box_col = (255, 145, 45)

                        pygame.draw.rect(surf, box_col, (rx0, ry0, bw_box, bh_box), 2)
                        # A second frame encloses the projected LiDAR returns,
                        # rather than claiming the full YOLO box is a measured body.
                        lidar_box = d.get("lidar_bbox_2d") if lidar_confirmed else None
                        if lidar_box is not None:
                            lx0, ly0, lx1, ly1 = lidar_box
                            measured_rect = (ox_rgb + int(lx0 * sc_rgb), oy_rgb + int(ly0 * sc_rgb),
                                             max(3, int((lx1 - lx0) * sc_rgb)),
                                             max(3, int((ly1 - ly0) * sc_rgb)))
                            pygame.draw.rect(surf, (0, 220, 255), measured_rect, 2)
                        corner_len = min(12, min(bw_box, bh_box) // 3)
                        for cx_i, cy_i, dx, dy in [
                            (rx0, ry0, 1, 1), (rx0 + bw_box, ry0, -1, 1),
                            (rx0, ry0 + bh_box, 1, -1), (rx0 + bw_box, ry0 + bh_box, -1, -1)
                        ]:
                            pygame.draw.line(surf, (255, 255, 255), (cx_i, cy_i), (cx_i + dx * corner_len, cy_i), 2)
                            pygame.draw.line(surf, (255, 255, 255), (cx_i, cy_i), (cx_i, cy_i + dy * corner_len), 2)

                        lbl = f"[{detection_caption(d)}]"
                        f_mono = self.font(11, bold=True, mono=True)
                        lbl_surf = f_mono.render(lbl, True, box_col)
                        shadow_surf = f_mono.render(lbl, True, (0, 0, 0))
                        lw, lh = lbl_surf.get_size()
                        lbl_y = max(area.y + 2, ry0 - lh - 4)

                        # Nền trong suốt mờ nhẹ (translucent 70% see-through)
                        bg_trans = pygame.Surface((lw + 6, lh + 2), pygame.SRCALPHA)
                        bg_trans.fill((0, 0, 0, 75))
                        surf.blit(bg_trans, (rx0 - 1, lbl_y - 1))

                        # Đổ bóng chữ đen mảnh để chữ luôn rõ nét trên mọi nền
                        for sx, sy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                            surf.blit(shadow_surf, (rx0 + 2 + sx, lbl_y + sy))
                        surf.blit(lbl_surf, (rx0 + 2, lbl_y))
                except Exception:
                    pass
        else:
            self.text(surf, "[ĐANG KẾT NỐI CAMERA RGB...]", (w // 2, h // 2), 14, (0, 220, 120), anchor="center")

        tilt = abs(self.gimbal_pitch)
        cam_mode_str = "NGANG (0°)" if tilt < 15 else ("TỪ TRÊN XUỐNG (-90°)" if tilt > 75 else f"CHÉO ({self.gimbal_pitch:.0f}°)")
        view_label = "CHASE CAM" if self.rgb_view_target == 1 else "CAM MŨI UAV (HƯỚNG UAV)"
        msg = f"{view_label}  |  GÓC CAM: {cam_mode_str}  |  ĐỘ CAO: {alt:.1f}m  |  VẬN TỐC: {speed:.1f}m/s"
        self.text(surf, msg, (10, h - strip_h + 4), 11, (200, 240, 220), mono=False)
        pygame.draw.rect(surf, accent, (0, 0, w, h), 1)

    def draw_fov_cone(self, surf, cx, cy, scale, yaw_rad, alt, gimbal_pitch, max_px_range):
        alt_eff = max(alt, 2.0)
        hfov = THERMAL_FOV_DEG
        vfov = 2.0 * math.degrees(math.atan(math.tan(math.radians(hfov / 2.0)) * self.cam_h / float(self.cam_w)))
        depress = -gimbal_pitch
        a_near = min(89.9, depress + vfov / 2.0)
        a_far = depress - vfov / 2.0
        d_near = max(0.5, min(alt_eff / math.tan(math.radians(max(a_near, 0.5))), max_px_range / scale))
        d_far = max(d_near + 1.0, min((alt_eff / math.tan(math.radians(max(a_far, 0.5)))) if a_far > 1.0 else (max_px_range / scale), max_px_range / scale))

        th_half = math.radians(hfov / 2.0)
        hn = d_near * math.tan(th_half)
        hf = d_far * math.tan(th_half)
        cyaw, syaw = math.cos(yaw_rad), math.sin(yaw_rad)

        def to_px(fwd, right):
            x = fwd * cyaw - right * syaw
            y = fwd * syaw + right * cyaw
            return cx + int(y * scale), cy - int(x * scale)

        poly = [to_px(d_near, -hn), to_px(d_near, hn), to_px(d_far, hf), to_px(d_far, -hf)]
        ov = pygame.Surface(surf.get_size(), pygame.SRCALPHA)
        pygame.draw.polygon(ov, (255, 170, 50, 46), poly)
        pygame.draw.polygon(ov, (255, 190, 80, 200), poly, 2)
        pygame.draw.line(ov, (255, 190, 80, 120), (cx, cy), to_px(d_far, 0), 1)
        surf.blit(ov, (0, 0))

    def render_lidar_radar_pane(self):
        surf = self.panes["lidar"]
        w, h = surf.get_size()
        surf.fill((8, 12, 18))

        with self.lock:
            pts = self.lidar_pts
            pts_count = self.telemetry["points_count"]
            uav_x = self.telemetry["x"]
            uav_y = self.telemetry["y"]
            uav_z = self.telemetry["z"]
            alt = self.telemetry["alt"]
            yaw_deg = self.telemetry["yaw"]
            trail_copy = list(self.flight_trail)
            markers_copy = list(self.marked_persons)
            trees_copy = list(self.detected_trees)
            tgt = self.current_target()

        max_range = self.lidar_ranges[self.lidar_range_idx]
        cmap_name, colormap_id = self.lidar_colormaps[self.lidar_cmap_idx]
        accent = (0, 200, 240)

        cbar_w = 48
        plot_w = w - cbar_w
        plot_h = h - PANE_HDR
        cx = plot_w // 2
        cy = PANE_HDR + plot_h // 2
        radius = min(cx - 12, plot_h // 2 - 10)
        scale = radius / max_range

        is_hu = self.radar_heading_up
        if is_hu:
            # Làm trơn góc xoay la bàn Heading-Up, triệt tiêu hiện tượng vặn xoắn giật cục khi quay đầu UAV
            if not hasattr(self, "smooth_radar_yaw"):
                self.smooth_radar_yaw = yaw_deg
            diff_y = (yaw_deg - self.smooth_radar_yaw + 180.0) % 360.0 - 180.0
            self.smooth_radar_yaw = (self.smooth_radar_yaw + 0.30 * diff_y) % 360.0
            yaw_rad = math.radians(self.smooth_radar_yaw)
        else:
            yaw_rad = math.radians(yaw_deg)
        cyaw, syaw = math.cos(yaw_rad), math.sin(yaw_rad)

        def to_bev_px(dx, dy):
            if is_hu:
                fwd = dx * cyaw + dy * syaw
                right = -dx * syaw + dy * cyaw
                return cx + int(right * scale), cy - int(fwd * scale)
            else:
                return cx + int(dy * scale), cy - int(dx * scale)

        # Vòng cự ly radar
        for frac in (0.25, 0.5, 0.75, 1.0):
            pygame.draw.circle(surf, (22, 32, 44) if frac < 1 else (34, 50, 68), (cx, cy), int(radius * frac), 1)
        pygame.draw.line(surf, (22, 32, 44), (cx - radius, cy), (cx + radius, cy), 1)
        pygame.draw.line(surf, (22, 32, 44), (cx, cy - radius), (cx, cy + radius), 1)
        for frac in (0.5, 1.0):
            self.text(surf, f"{max_range * frac:.0f}m", (cx + int(radius * frac) - 4, cy + 3), 10, (80, 115, 150), mono=True, anchor="topright")

        # Kim la bàn / Chỉ hướng Bắc ("B")
        if is_hu:
            # Heading-Up: Mũi drone luôn thẳng lên trên (12H), Hướng Bắc "B" xoay theo góc thực tế
            bx = cx - int((radius + 1) * syaw)
            by = cy - int((radius + 1) * cyaw)
            bx_c = max(12, min(plot_w - 12, bx))
            by_c = max(PANE_HDR + 12, min(h - 12, by))
            pygame.draw.circle(surf, (0, 160, 200), (bx_c, by_c), 8, 1)
            self.text(surf, "B", (bx_c, by_c), 10, (0, 240, 255), bold=True, anchor="center")
            # Mũi nhọn chỉ hướng thẳng phía trước (12H / FWD)
            fwd_tip = (cx, cy - radius - 2)
            pygame.draw.polygon(surf, (0, 255, 200), [fwd_tip, (cx - 4, cy - radius + 6), (cx + 4, cy - radius + 6)])
        else:
            # North-Up: Bắc luôn ở đỉnh
            self.text(surf, "B", (cx, cy - radius - 2), 10, (0, 200, 255), bold=True, anchor="midbottom")

        # Vẽ đám mây điểm LiDAR
        max_elev_disp = getattr(self, "lidar_max_elev", 10.0)
        pts_colors = getattr(self, "lidar_colors", None)
        if len(pts) > 0 and self.lidar_enabled:
            rel = pts - np.array([uav_x, uav_y, uav_z], dtype=np.float32)
            dists = np.hypot(rel[:, 0], rel[:, 1])
            in_range = dists <= max_range
            filtered = rel[in_range]

            if len(filtered) > 0:
                if is_hu:
                    fwd = filtered[:, 0] * cyaw + filtered[:, 1] * syaw
                    right = -filtered[:, 0] * syaw + filtered[:, 1] * cyaw
                    pxs = (cx + right * scale).astype(int)
                    pys = (cy - fwd * scale).astype(int)
                else:
                    pxs = (cx + filtered[:, 1] * scale).astype(int)
                    pys = (cy - filtered[:, 0] * scale).astype(int)

                valid = (pxs >= 1) & (pxs < plot_w - 2) & (pys >= PANE_HDR + 1) & (pys < h - 3)
                v_pxs = pxs[valid]
                v_pys = pys[valid]

                if len(v_pxs) > 0:
                    if pts_colors is not None and len(pts_colors) == len(pts):
                        filtered_colors = pts_colors[in_range]
                        v_cols = filtered_colors[valid]
                    else:
                        z_vals = filtered[valid, 2]
                        elev = np.clip(uav_z - z_vals, 0.0, 60.0)
                        norm_elev = np.clip(elev / max(4.0, max_elev_disp), 0.0, 1.0)
                        color_indices = (norm_elev * 255.0).astype(np.uint8)
                        bgr_pts = cv2.applyColorMap(color_indices, colormap_id)
                        v_cols = cv2.cvtColor(bgr_pts, cv2.COLOR_BGR2RGB).reshape(-1, 3)

                    pix = pygame.surfarray.pixels3d(surf)
                    pix[v_pxs, v_pys] = v_cols
                    del pix

        # Nón quan sát camera FOV cone
        cone_yaw = 0.0 if is_hu else yaw_rad
        self.draw_fov_cone(surf, cx, cy, scale, cone_yaw, alt, self.gimbal_pitch, max_range)

        # Vết đường bay
        if len(trail_copy) > 1:
            trail_px = []
            for tx, ty in trail_copy[-100:]:
                px_t, py_t = to_bev_px(tx - uav_x, ty - uav_y)
                if 1 <= px_t < plot_w - 1 and PANE_HDR + 1 <= py_t < h - 1:
                    trail_px.append((px_t, py_t))
            if len(trail_px) > 1:
                pygame.draw.lines(surf, (0, 160, 220), False, trail_px, 1)

        # Các điểm đánh dấu nạn nhân
        for m in markers_copy:
            mx, my = to_bev_px(m["tgt_x"] - uav_x, m["tgt_y"] - uav_y)
            if 0 <= mx < plot_w and PANE_HDR <= my < h:
                pygame.draw.circle(surf, (255, 30, 30), (mx, my), 5, 2)
                self.text(surf, f"#{m['id']}", (mx + 7, my - 6), 10, (255, 90, 90), mono=True)

        # Mục tiêu hiện tại
        if tgt is not None:
            mx, my = to_bev_px(tgt[0] - uav_x, tgt[1] - uav_y)
            if 0 <= mx < plot_w and PANE_HDR <= my < h:
                pygame.draw.circle(surf, (0, 255, 255), (mx, my), 8, 1)
                self.text(surf, "TARGET", (mx + 10, my - 6), 10, (0, 255, 255), bold=True, mono=True)

        # Chướng ngại vật Cây cối phát hiện được qua Camera Nhiệt
        for t in trees_copy:
            tx, ty = to_bev_px(t["x"] - uav_x, t["y"] - uav_y)
            if 0 <= tx < plot_w and PANE_HDR <= ty < h:
                r_px = max(4, int(t["r"] * scale))
                pygame.draw.circle(surf, (40, 220, 100), (tx, ty), r_px, 1)
                pygame.draw.circle(surf, (20, 180, 70), (tx, ty), 2)
                self.text(surf, f"CÂY {t['dist']:.0f}m", (tx + r_px + 2, ty - 6), 9, (100, 255, 160), mono=True)

        # Cảnh báo chướng ngại vật gần (< 15m)
        close_trees = [t for t in trees_copy if t["dist"] <= 15.0]
        if close_trees:
            warn_msg = f"⚠️ CẢNH BÁO: CÂY CÁCH {close_trees[0]['dist']:.1f}m"
            self.text(surf, warn_msg, (cx, cy + radius - 20), 12, (255, 200, 50), bold=True, anchor="center")

        # Biểu tượng UAV ở tâm radar
        if is_hu:
            # Mũi UAV luôn chỉ thẳng lên trên (12H)
            head = (cx, cy - 11)
            left = (cx - 7, cy + 6)
            right = (cx + 7, cy + 6)
            pygame.draw.polygon(surf, (0, 255, 200), [head, left, right])
        else:
            # Mũi UAV xoay theo góc yaw_rad
            head = (cx + int(11 * math.sin(yaw_rad)), cy - int(11 * math.cos(yaw_rad)))
            left = (cx + int(7 * math.sin(yaw_rad + 2.5)), cy - int(7 * math.cos(yaw_rad + 2.5)))
            right = (cx + int(7 * math.sin(yaw_rad - 2.5)), cy - int(7 * math.cos(yaw_rad - 2.5)))
            pygame.draw.polygon(surf, (0, 255, 200), [head, left, right])

        # Thanh màu cao độ
        cb_x = w - cbar_w + 6
        cb_y = PANE_HDR + 12
        cb_h = h - cb_y - 12
        cb_w = 10
        if self.cached_colorbar_surf is None or self.cached_colorbar_cmap != colormap_id:
            grad_vals = np.linspace(255, 0, cb_h, dtype=np.uint8).reshape(-1, 1)
            grad_rgb = cv2.cvtColor(cv2.applyColorMap(grad_vals, colormap_id), cv2.COLOR_BGR2RGB)
            grad_bar = cv2.resize(grad_rgb, (cb_w, cb_h))
            self.cached_colorbar_surf = pygame.surfarray.make_surface(grad_bar.swapaxes(0, 1))
            self.cached_colorbar_cmap = colormap_id

        surf.blit(self.cached_colorbar_surf, (cb_x, cb_y))
        pygame.draw.rect(surf, (150, 180, 210), (cb_x, cb_y, cb_w, cb_h), 1)
        self.text(surf, f"{max_elev_disp:2.0f}m", (cb_x + cb_w + 3, cb_y - 2), 10, (255, 90, 90), bold=True, mono=True)
        self.text(surf, " 0m", (cb_x + cb_w + 3, cb_y + cb_h - 10), 10, (0, 220, 255), bold=True, mono=True)

        mode_str = "H-UP" if is_hu else "N-UP"
        tree_hdr = f" | 🌲 {len(trees_copy)} cây" if trees_copy else ""
        lid_status = f"{pts_count:,} điểm" if self.lidar_enabled else "TẮT [J]"
        self.draw_header(surf, "RADAR BEV LIDAR OUSTER OS1-64", accent,
                         f"[{mode_str}] Thang {max_range:.0f}m | {lid_status}{tree_hdr} | {cmap_name}")
        if not self.lidar_enabled:
            self.text(surf, "[QUÉT LIDAR: ĐANG TẮT]", (cx, cy + 32), 12, (255, 180, 60), bold=True, anchor="center")
            self.text(surf, "Bấm nút [J] BẬT QUÉT hoặc D-Pad Phải ▶ để BẬT", (cx, cy + 50), 10, (160, 180, 200), anchor="center")

        # 1. Nút bấm đổi chế độ [N] trên góc phải thanh tiêu đề
        btn_n_w = 75
        btn_n_r = pygame.Rect(plot_w - btn_n_w - 6, 4, btn_n_w, 22)
        pygame.draw.rect(surf, (14, 32, 44), btn_n_r, border_radius=3)
        pygame.draw.rect(surf, (0, 200, 240), btn_n_r, 1, border_radius=3)
        self.text(surf, f"[N] {mode_str}", (btn_n_r.centerx, btn_n_r.centery), 10, (0, 240, 255), bold=True, anchor="center")

        # 2. Nút XEM MAP 3D [F2]
        btn_m3d_w = 100
        btn_m3d_r = pygame.Rect(btn_n_r.left - btn_m3d_w - 6, 4, btn_m3d_w, 22)
        pygame.draw.rect(surf, (0, 60, 80), btn_m3d_r, border_radius=3)
        pygame.draw.rect(surf, (0, 255, 200), btn_m3d_r, 1, border_radius=3)
        self.text(surf, "🌐 [F2] MAP 3D", (btn_m3d_r.centerx, btn_m3d_r.centery), 10, (0, 255, 220), bold=True, anchor="center")

        # 3. Nút BẬT / TẮT LiDAR [J]
        btn_j_w = 80
        btn_j_r = pygame.Rect(btn_m3d_r.left - btn_j_w - 6, 4, btn_j_w, 22)
        j_bg = (0, 80, 50) if self.lidar_enabled else (100, 30, 20)
        j_border = (0, 255, 120) if self.lidar_enabled else (255, 80, 80)
        j_txt = "[J] QUÉT: BẬT" if self.lidar_enabled else "[J] BẬT QUÉT"
        pygame.draw.rect(surf, j_bg, btn_j_r, border_radius=3)
        pygame.draw.rect(surf, j_border, btn_j_r, 1, border_radius=3)
        self.text(surf, j_txt, (btn_j_r.centerx, btn_j_r.centery), 9, (255, 255, 255), bold=True, anchor="center")

        pygame.draw.rect(surf, accent, (0, 0, w, h), 1)

    def render_roi_pane(self):
        """Ô DƯỚI-PHẢI SIÊU GỌN: ĐIỀU HƯỚNG 3D & TỌA ĐỘ MỤC TIÊU CỨU HỘ."""
        surf = self.panes["roi"]
        w, h = surf.get_size()
        surf.fill((10, 14, 22))

        with self.lock:
            tgt = self.current_target()
            uav_x = self.telemetry["x"]
            uav_y = self.telemetry["y"]
            alt = self.telemetry["alt"]
            yaw = self.telemetry["yaw"]
            spd = self.telemetry["speed"]
            n_marked = len(self.marked_persons)
            latest_m = self.marked_persons[-1] if self.marked_persons else None

        accent = (160, 100, 255)
        self.draw_header(surf, "ĐIỀU HƯỚNG 3D & TỌA ĐỘ MỤC TIÊU CỨU HỘ", accent,
                         "Tự động tính cự ly & góc phương vị mục tiêu")

        card_w = w - 24
        pygame.draw.rect(surf, (16, 22, 34), (12, PANE_HDR + 10, card_w, 140), border_radius=6)
        pygame.draw.rect(surf, (160, 100, 255), (12, PANE_HDR + 10, card_w, 140), 1, border_radius=6)

        if tgt is not None:
            dist = math.hypot(tgt[0] - uav_x, tgt[1] - uav_y)
            bearing_rad = math.atan2(tgt[1] - uav_y, tgt[0] - uav_x)
            bearing_deg = (math.degrees(bearing_rad) + 360.0) % 360.0
            rel_angle = ((bearing_deg - yaw + 180.0) % 360.0) - 180.0

            lock_status = "ĐÃ KHÓA MỤC TIÊU [T]" if self.target_locked else "TỰ ĐỘNG BÁM THEO CAMERA NHIỆT"
            self.text(surf, f"● {lock_status}", (24, PANE_HDR + 18), 12, (0, 255, 180) if self.target_locked else (255, 200, 60), bold=True)
            self.text(surf, "TỌA ĐỘ MỤC TIÊU 3D:", (24, PANE_HDR + 40), 11, (170, 185, 210))
            self.text(surf, f"X = {tgt[0]:+7.1f}m    Y = {tgt[1]:+7.1f}m    Z = {tgt[2]:+6.1f}m", (24, PANE_HDR + 58), 16, (255, 230, 100), bold=True, mono=True)
            self.text(surf, f"CỰ LY TRỰC TIẾP: {dist:5.1f}m    |    GÓC PHƯƠNG VỊ: {bearing_deg:03.0f}°    |    GÓC LỆCH MŨI: {rel_angle:+03.0f}°", (24, PANE_HDR + 90), 12, (200, 230, 255), mono=True)

            if abs(rel_angle) < 10: nav_guide = "ĐANG BAY THẲNG ĐẾN MỤC TIÊU"
            elif rel_angle > 0: nav_guide = f"LƯỢN PHẢI {abs(rel_angle):.0f}° ĐỂ TIẾP CẬN MỤC TIÊU"
            else: nav_guide = f"LƯỢN TRÁI {abs(rel_angle):.0f}° ĐỂ TIẾP CẬN MỤC TIÊU"
            self.text(surf, f"CHỈ DẪN BAY: {nav_guide}", (24, PANE_HDR + 118), 12, (0, 240, 160), bold=True)
        else:
            self.text(surf, "○ CHƯA PHÁT HIỆN MỤC TIÊU TRONG TẦM NHÌN", (24, PANE_HDR + 22), 14, (255, 140, 70), bold=True)
            self.text(surf, "Tìm tín hiệu nhiệt; trong 40m phân loại người / động vật.", (24, PANE_HDR + 52), 12, (180, 195, 215))
            self.text(surf, "sẽ tự động được tính toán và hiển thị tại đây theo thời gian thực.", (24, PANE_HDR + 72), 12, (180, 195, 215))
            self.text(surf, "Bấm [T] / nút Back để khóa mục tiêu cố định.", (24, PANE_HDR + 102), 11, (130, 150, 180))

        y_sec = PANE_HDR + 162
        pygame.draw.rect(surf, (16, 22, 34), (12, y_sec, card_w, h - y_sec - 12), border_radius=6)
        self.text(surf, "TIẾN TRÌNH NHIỆM VỤ CỨU HỘ & TỔNG HỢP VỊ TRÍ", (24, y_sec + 10), 12, (255, 200, 80), bold=True)
        self.text(surf, f"• Số nạn nhân đã đánh dấu: {n_marked} người", (24, y_sec + 34), 12, (255, 255, 255), bold=True)

        if latest_m is not None:
            self.text(surf, f"• Nạn nhân gần nhất #{latest_m['id']}: X={latest_m['tgt_x']:.1f}m, Y={latest_m['tgt_y']:.1f}m (Lúc {latest_m['time']})",
                      (24, y_sec + 56), 11, (220, 235, 250), mono=True)
        else:
            self.text(surf, "• [Y]: chỉ đánh dấu người trong 40m đã có LiDAR xác nhận", (24, y_sec + 56), 11, (160, 175, 195))

        self.text(surf, f"• Tọa độ UAV hiện tại: X={uav_x:+6.1f}m, Y={uav_y:+6.1f}m, Cao={alt:4.1f}m AGL | Tốc độ={spd:3.1f}m/s",
                  (24, y_sec + 82), 11, (180, 210, 235), mono=True)
        self.text(surf, f"• Cảm biến hoạt động: Camera Nhiệt FLIR ({self.img_hz:.0f} FPS) + Ouster LiDAR OS1-64 ({self.lidar_hz:.0f} Hz)",
                  (24, y_sec + 104), 11, (0, 230, 160), mono=True)

        pygame.draw.rect(surf, accent, (0, 0, w, h), 1)

    def render_sidebar(self):
        surf = self.panes["side"]
        w, h = surf.get_size()
        surf.fill((12, 16, 24))
        self.sidebar_buttons = []

        with self.lock:
            conn = self.telemetry["connected"]
            status_str = self.telemetry["status"]
            alt = self.telemetry["alt"]
            spd = self.telemetry["speed"]
            px = self.telemetry["x"]
            py = self.telemetry["y"]
            pz = self.telemetry["z"]
            ftime = self.telemetry["flight_time"]

        accent = (255, 200, 50)
        pygame.draw.rect(surf, (17, 23, 33), (0, 0, w, PANE_HDR))
        pygame.draw.rect(surf, accent, (0, PANE_HDR - 2, w, 2))
        self.text(surf, "BẢNG ĐIỀU KHIỂN & TELEMETRY", (10, 6), 12, accent, bold=True)

        def card(y, ch, title, col):
            pygame.draw.rect(surf, (18, 24, 35), (8, y, w - 16, ch), border_radius=6)
            pygame.draw.rect(surf, col, (8, y + 8, 3, 14), border_radius=2)
            self.text(surf, title, (18, y + 6), 11, col, bold=True)
            return y + 28

        # 1. KẾT NỐI & BAY
        y = PANE_HDR + 8
        ch = 216
        iy = card(y, ch, "KẾT NỐI & ĐIỀU KHIỂN", (100, 200, 255))
        has_ctrl = (self.xinput.connected or self.controller is not None or self.joystick is not None)
        ctrl_label = "Xbox Gamepad" if self.xinput.connected else (self.controller_name[:20] if has_ctrl else "Bàn phím WASD")
        conn_col = (0, 255, 150) if conn else ((255, 200, 50) if "KẾT NỐI" in status_str else (255, 70, 70))
        pygame.draw.circle(surf, conn_col, (24, iy + 8), 5)
        self.text(surf, f"AirSim: {status_str}", (36, iy), 12, conn_col, bold=True)
        self.text(surf, f"Thiết bị: {ctrl_label}", (16, iy + 22), 11, (210, 225, 240))

        r_flight_btn = pygame.Rect(16, iy + 44, w - 32, 26)
        btn_bg = (200, 60, 50) if self.flight_armed else (0, 160, 110)
        btn_txt = ("■ HẠ CÁNH [L]" if self.is_airborne else
                   "■ TẮT ĐỘNG CƠ [L]" if self.flight_armed else "▲ ARM / BAY THỦ CÔNG [L]")
        pygame.draw.rect(surf, btn_bg, r_flight_btn, border_radius=4)
        self.text(surf, btn_txt, (w // 2, iy + 56), 11, (255, 255, 255), bold=True, anchor="center")
        self.sidebar_buttons.append(("takeoff_land", r_flight_btn))

        # NÚT BẬT / TẮT GHI DỮ LIỆU TÌM KIẾM (NHIỆT + LIDAR) - ẤN LẦN 1 BẬT, LẦN 2 TẮT
        r_rec_btn = pygame.Rect(16, iy + 76, w - 32, 38)
        rec_flash = self.recording and (int(time.time() * 2) % 2 == 0)
        if self.recording:
            rec_dur = int(time.time() - self.record_start_time)
            rec_bg = (220, 35, 35) if rec_flash else (160, 20, 20)
            pygame.draw.rect(surf, rec_bg, r_rec_btn, border_radius=5)
            pygame.draw.rect(surf, (255, 255, 255), r_rec_btn, 2, border_radius=5)
            self.text(surf, f"■ ĐANG GHI: {self.record_frame_id}F ({rec_dur}s)", (w // 2, iy + 87), 11, (255, 255, 255), bold=True, anchor="center")
            hint_col = (255, 235, 80) if rec_flash else (255, 255, 255)
            self.text(surf, "▶ BẤM D-PAD ↓ / LẦN 2 ĐỂ TẮT [K] ◀", (w // 2, iy + 101), 9, hint_col, bold=True, anchor="center")
        else:
            rec_bg = (34, 44, 60)
            pygame.draw.rect(surf, rec_bg, r_rec_btn, border_radius=5)
            pygame.draw.rect(surf, (255, 80, 80), r_rec_btn, 1, border_radius=5)
            self.text(surf, "● BẬT GHI NHIỆT + LIDAR [D-PAD ↓ / K]", (w // 2, iy + 87), 11, (255, 255, 255), bold=True, anchor="center")
            self.text(surf, "(Ấn lần 1: Bật  •  Ấn lần 2: Tắt)", (w // 2, iy + 101), 9, (180, 200, 225), anchor="center")
        self.sidebar_buttons.append(("toggle_record", r_rec_btn))

        r_win_btn = pygame.Rect(16, iy + 120, w - 32, 22)
        pygame.draw.rect(surf, (35, 45, 60), r_win_btn, border_radius=4)
        self.text(surf, "Căn chỉnh cửa sổ Unreal & GCS [F10]", (w // 2, iy + 130), 10, (180, 210, 235), anchor="center")
        self.sidebar_buttons.append(("arrange_windows", r_win_btn))

        r_map_btn = pygame.Rect(16, iy + 148, w - 32, 24)
        pygame.draw.rect(surf, (0, 70, 95), r_map_btn, border_radius=4)
        pygame.draw.rect(surf, (0, 220, 255), r_map_btn, 1, border_radius=4)
        self.text(surf, "🌐 MỞ / XEM MAP 3D OPEN3D [F2]", (w // 2, iy + 160), 10, (0, 240, 255), bold=True, anchor="center")
        self.sidebar_buttons.append(("open_3d_map", r_map_btn))

        # 2. GÓC CAMERA (GIMBAL)
        y += ch + 8
        ch = 160
        iy = card(y, ch, "GIMBAL - GÓC CAMERA (NHIỆT + RGB)", (255, 220, 100))
        self.text(surf, f"{self.gimbal_pitch:5.1f}°", (16, iy - 2), 24, (255, 230, 120), bold=True, mono=True)
        self.text(surf, "Góc Cam", (110, iy + 12), 11, (170, 180, 195))

        bar_x, bar_w = 18, w - 52
        bar_y = iy + 36
        pygame.draw.rect(surf, (32, 40, 54), (bar_x, bar_y, bar_w, 8), border_radius=4)
        frac = (self.gimbal_pitch + 90.0) / 90.0
        pygame.draw.rect(surf, (255, 200, 60), (bar_x, bar_y, int(bar_w * frac), 8), border_radius=4)
        pygame.draw.circle(surf, (255, 255, 255), (bar_x + int(bar_w * frac), bar_y + 4), 8)
        self.sidebar_buttons.append(("gimbal_slider", pygame.Rect(bar_x - 4, bar_y - 6, bar_w + 8, 20)))

        for val, lab in ((-90, "-90°"), (-45, "-45°"), (0, "0°")):
            vx = bar_x + int(bar_w * ((val + 90.0) / 90.0))
            self.text(surf, lab, (vx, bar_y + 14), 10, (140, 155, 175), mono=True, anchor="midtop")

        presets = [("0° Ngang", 0.0), ("-45° Nghiêng", -45.0), ("-90° Đất", -90.0)]
        bx = 14
        for lab, val in presets:
            tw = self.font(9, True).size(lab)[0] + 10
            active = abs(self.gimbal_pitch - val) < 3.0
            r_pre = pygame.Rect(bx, iy + 80, tw, 20)
            pygame.draw.rect(surf, (255, 200, 60) if active else (36, 44, 58), r_pre, border_radius=4)
            self.text(surf, lab, (bx + tw // 2, iy + 90), 9, (0, 0, 0) if active else (175, 188, 202), bold=True, anchor="center")
            self.sidebar_buttons.append((f"preset_{int(val)}", r_pre))
            bx += tw + 4

        r_inv_btn = pygame.Rect(14, iy + 106, (w - 32) // 2 - 2, 22)
        inv_active = self.invert_cam_pitch
        pygame.draw.rect(surf, (200, 100, 40) if inv_active else (34, 44, 60), r_inv_btn, border_radius=4)
        inv_label = f"Đảo Cam [I]: {'BẬT' if inv_active else 'TẮT'}"
        self.text(surf, inv_label, (r_inv_btn.centerx, iy + 117), 9, (255, 255, 255) if inv_active else (180, 200, 220), bold=True, anchor="center")
        self.sidebar_buttons.append(("toggle_invert", r_inv_btn))

        r_stab_btn = pygame.Rect(14 + (w - 32) // 2 + 2, iy + 106, (w - 32) // 2 - 2, 22)
        stab_active = self.gimbal_stabilization_enabled
        pygame.draw.rect(surf, (0, 140, 90) if stab_active else (50, 40, 40), r_stab_btn, border_radius=4)
        stab_label = f"Chống rung [F1]: {'BẬT' if stab_active else 'TẮT'}"
        self.text(surf, stab_label, (r_stab_btn.centerx, iy + 117), 9, (255, 255, 255) if stab_active else (210, 150, 150), bold=True, anchor="center")
        self.sidebar_buttons.append(("toggle_stabilization", r_stab_btn))

        # 3. BAY & ĐỘ CAO (ALTITUDE HOLD)
        y += ch + 8
        ch = 148
        iy = card(y, ch, "BAY & TỰ GIỮ ĐỘ CAO (ALT HOLD)", (0, 230, 160))
        self.text(surf, f"{alt:5.1f}m", (16, iy - 2), 26, (0, 240, 170), bold=True, mono=True)
        agl_label = "Độ cao AGL (LiDAR Altimeter)" if self.terrain_agl is not None else "Độ cao AGL (Ước tính)"
        self.text(surf, agl_label, (110, iy + 12), 11, (170, 180, 195))

        target_alt_disp = (self.ground_z_datum - self.target_z) if self.target_z is not None else alt
        alt_asl_val = self.ground_z_datum - pz
        self.text(surf, f"Mục tiêu giữ: {target_alt_disp:4.1f}m (Mốc cất cánh)", (16, iy + 36), 11, (100, 230, 210), mono=True)
        self.text(surf, f"Vận tốc tổng: {spd:4.1f} m/s", (16, iy + 54), 11, (210, 225, 240), mono=True)
        asl_hint = " (Dưới bãi cất cánh)" if alt_asl_val < -1.0 else ""
        self.text(surf, f"Mốc xuất phát: {alt_asl_val:4.1f}m ASL{asl_hint}", (16, iy + 72), 10, (140, 210, 255), mono=True)
        self.text(surf, f"Tọa độ NED: X={px:5.1f}  Y={py:5.1f}  Z={pz:5.1f}", (16, iy + 88), 10, (160, 175, 195), mono=True)

        # 4. THỐNG KÊ CỨU HỘ & AI YOLO
        y += ch + 8
        ch = 152
        iy = card(y, ch, "NHIỆM VỤ CỨU HỘ & AI YOLO", (255, 120, 180))
        self.text(surf, f"Đã đánh dấu: {len(self.marked_persons)} người", (16, iy), 12, (255, 140, 190), bold=True)
        self.text(surf, f"Thời gian bay: {int(ftime // 60):02d}:{int(ftime % 60):02d}", (16, iy + 20), 11, (210, 225, 240), mono=True)
        self.text(surf, f"HB:{self.cmd_hz:2.0f} | Nhiệt:{self.img_hz:2.0f} | RGB:{self.rgb_hz:2.0f} FPS", (16, iy + 40), 10, (160, 180, 200), mono=True)

        # Nút chuyển đổi AI YOLO26m [M]
        r_yolo_btn = pygame.Rect(14, iy + 60, w - 28, 24)
        yolo_on = (self.yolo_enabled and self.yolo_model is not None)
        yolo_bg = (0, 100, 60) if yolo_on else (40, 48, 62)
        pygame.draw.rect(surf, yolo_bg, r_yolo_btn, border_radius=4)
        pygame.draw.rect(surf, (0, 255, 150) if yolo_on else (130, 140, 155), r_yolo_btn, 1, border_radius=4)
        yolo_btn_txt = f"AI YOLO26m [M]: {'BẬT (' + ('GPU' if self.yolo_device.startswith('cuda') else 'CPU') + ')' if yolo_on else 'TẮT (NGƯỠNG)'}"
        self.text(surf, yolo_btn_txt, (w // 2, iy + 72), 10, (255, 255, 255) if yolo_on else (180, 195, 210), bold=True, anchor="center")
        self.sidebar_buttons.append(("toggle_yolo", r_yolo_btn))

        yolo_fps_str = f"{self.yolo_fps:.0f} FPS" if yolo_on else "N/A"
        self.text(surf, f"AI Tốc độ: {yolo_fps_str}  |  UI: {self.ui_fps:2.0f} FPS", (16, iy + 92), 10, (0, 230, 160) if yolo_on else (160, 175, 195), mono=True)
        rec_status_str = f"ĐANG GHI: {self.record_frame_id}F (Ấn lần 2 để tắt)" if self.recording else "ĐÃ TẮT [Bấm nút / K để bật]"
        rec_col = (255, 80, 80) if self.recording else (140, 160, 180)
        self.text(surf, f"Lưu Nhiệt + LiDAR: {rec_status_str}", (16, iy + 108), 10, rec_col, mono=True)

        pygame.draw.rect(surf, accent, (0, 0, w, h), 1)

    def draw_dashboard(self, clock):
        cv = self.canvas
        w, h = CANVAS_W, CANVAS_H
        cv.fill((8, 11, 16))

        # Header
        pygame.draw.rect(cv, (16, 22, 32), (0, 0, w, HEADER_H))
        pygame.draw.rect(cv, (255, 190, 60), (0, HEADER_H - 2, w, 2))
        self.text(cv, "HỆ THỐNG UAV GCS TÌM KIẾM CỨU HỘ", (12, 8), 14, (255, 255, 255), bold=True)
        self.text(cv, "[UE 4.27 MEDIEVAL VILLAGE]", (322, 10), 11, (100, 240, 255), bold=True)

        bx = 540
        has_ctrl = (self.xinput.connected or self.controller is not None or self.joystick is not None)
        js_color = (0, 255, 180) if has_ctrl else (255, 140, 60)
        r = self.text(cv, f"[{'Xbox Gamepad' if self.xinput.connected else self.controller_name[:18]}]", (bx, 11), 11, js_color, bold=True)
        bx = r.right + 14

        if self.is_taking_off: mode_str, mode_col = "ĐANG ARM...", (255, 200, 50)
        elif self.is_landing: mode_str, mode_col = "ĐANG HẠ CÁNH...", (255, 120, 50)
        elif not self.is_airborne: mode_str, mode_col = "MẶT ĐẤT (GIỮ SPACE / RT ĐỂ BAY LÊN)", (255, 150, 50)
        elif self.is_boost: mode_str, mode_col = f"TURBO BOOST ({self.boost_speed:.0f} m/s)", (255, 70, 70)
        elif self.is_hover: mode_str, mode_col = "HOVER (TỰ GIỮ ĐỘ CAO)", (255, 180, 0)
        else: mode_str, mode_col = "TÌM KIẾM CỨU HỘ", (0, 220, 255)
        r = self.text(cv, f"[{mode_str}]", (bx, 11), 11, mode_col, bold=True)
        bx = r.right + 14

        yolo_on = (self.yolo_enabled and self.yolo_model is not None)
        ai_col = (0, 255, 160) if yolo_on else (150, 160, 175)
        ai_tag = f"[AI YOLO: {'GPU' if self.yolo_device.startswith('cuda') else 'CPU'}]" if yolo_on else "[AI: TẮT]"
        r = self.text(cv, ai_tag, (bx, 11), 11, ai_col, bold=True)
        bx = r.right + 14

        light_col = (255, 255, 0) if self.searchlight_on else (120, 120, 120)
        r = self.text(cv, f"[ĐÈN: {'BẬT' if self.searchlight_on else 'TẮT'}]", (bx, 11), 11, light_col, bold=True)
        bx = r.right + 14
        lid_col = (0, 200, 255) if self.lidar_enabled else (120, 130, 140)
        r = self.text(cv, f"[LiDAR: {'BẬT' if self.lidar_enabled else 'TẮT'}]", (bx, 11), 11, lid_col, bold=True)
        bx = r.right + 14

        th_col = (255, 170, 50) if self.thermal_enabled else (120, 130, 140)
        r = self.text(cv, f"[NHIỆT: {'BẬT' if self.thermal_enabled else 'TẮT'}]", (bx, 11), 11, th_col, bold=True)
        bx = r.right + 14
        inv_str = " (ĐẢO)" if self.invert_cam_pitch else ""
        r = self.text(cv, f"[GÓC CAM: {self.gimbal_pitch:+.0f}°{inv_str}]", (bx, 11), 11, (255, 220, 80), bold=True)
        bx = r.right + 14

        if self.recording:
            rec_flash = (int(time.time() * 2) % 2 == 0)
            rec_dur = int(time.time() - self.record_start_time)
            rec_col = (255, 60, 60) if rec_flash else (255, 235, 80)
            r = self.text(cv, f"[● REC: {self.record_frame_id}F | {rec_dur}s (BẤM D-PAD ↓ ĐỂ TẮT)]", (bx, 11), 11, rec_col, bold=True)
            bx = r.right + 14

        with self.lock:
            h_alt = self.telemetry["alt"]
            h_spd = self.telemetry["speed"]
            h_bat = self.telemetry["battery"]
        self.text(cv, f"AGL: {h_alt:4.1f}m | V: {h_spd:4.1f}m/s | PIN: {h_bat:3.0f}% | ĐÃ TÌM: {len(self.marked_persons)} | {self.ui_fps:2.0f} FPS",
                  (w - 12, 11), 11, (200, 220, 240), bold=True, anchor="topright")

        # Panes
        cv.blit(self.panes["thermal"], (0, DECK_Y))
        cv.blit(self.panes["rgb"], (PANE_W, DECK_Y))
        cv.blit(self.panes["lidar"], (0, DECK_Y + PANE_H))
        cv.blit(self.panes["roi"], (PANE_W, DECK_Y + PANE_H))
        cv.blit(self.panes["side"], (SIDE_X, DECK_Y))

        # Banner thông báo
        if (time.time() - self.last_ping_time) < 2.2:
            bw = 760
            banner_y = DECK_Y + PANE_HDR + 8
            bxx = (PANE_W * 2 - bw) // 2
            pygame.draw.rect(cv, (180, 20, 20), (bxx, banner_y, bw, 34), border_radius=6)
            pygame.draw.rect(cv, (255, 255, 0), (bxx, banner_y, bw, 34), 2, border_radius=6)
            self.text(cv, self.last_ping_msg, (bxx + bw // 2, banner_y + 17), 12, (255, 255, 255), bold=True, anchor="center")

        footer_y = h - FOOTER_H
        pygame.draw.rect(cv, (12, 18, 26), (0, footer_y, w, FOOTER_H))
        help_str = ("[WASD] Bay  [Space/Shift] Cao độ  [D-Pad ▶/J] LiDAR  [D-Pad ▲/F2] 3D Map  [D-Pad ◀/U] Nhiệt FLIR  [M] YOLO  [K] Ghi Data")
        self.text(cv, help_str, (14, footer_y + 7), 11, (160, 210, 240), bold=True)

        if self.show_help:
            self.draw_help_overlay()

    def draw_help_overlay(self):
        w, h = CANVAS_W, CANVAS_H
        ov = pygame.Surface((w, h), pygame.SRCALPHA)
        ov.fill((0, 0, 0, 215))
        box = pygame.Rect(w // 2 - 480, 30, 960, h - 60)
        pygame.draw.rect(ov, (25, 35, 50), box, border_radius=8)
        pygame.draw.rect(ov, (0, 220, 255), box, 2, border_radius=8)
        t = self.font(18, True).render("BẢNG HƯỚNG DẪN ĐIỀU KHIỂN UAV GCS (RESILIENT)", True, (0, 255, 220))
        ov.blit(t, (w // 2 - t.get_width() // 2, 44))
        rows = [
            ("Cần Phải (Lên / Xuống) | Phím ↑ / ↓", "Chỉnh góc camera: Đẩy Lên -> Ngẩng Lên (0°); Kéo Xuống -> Chúc Xuống (-90°)"),
            ("Phím [I] | Nút trên Sidebar", "ĐẢO CHIỀU góc camera (Invert Pitch) tức thời theo thói quen"),
            ("R3 (Tay cầm) | G | Phím 1 · 2 · 3", "Góc cam nhanh: 1: 0° (ngang), 2: -45° (tìm kiếm), 3: -90° (thẳng xuống)"),
            ("Cò RT / Space   |   Cò LT / Shift", "Bay LÊN (tự động giữ độ cao)   |   Hạ XUỐNG"),
            ("L: arm / tắt động cơ / hạ cánh", "Giữ Space / RT để bay lên; thả để giữ độ cao hiện tại"),
            ("Cần Trái / Phím W A S D", "Điều khiển tiến / lùi / sang trái / sang phải (Auto Altitude Hold vĩnh viễn)"),
            ("Cần Phải (Trái / Phải) / Q, E", "Xoay đầu UAV (Yaw)"),
            ("LB / Phím V", "Chuyển mượt FPV bụng drone ↔ Chase Cam bám đuôi"),
            ("RB / Phím R", "Đổi bảng màu Camera Nhiệt: White-Hot / Ironbow / Rainbow / Black-Hot"),
            ("Phím M | Nút trên Sidebar", "BẬT / TẮT mô hình AI YOLO26m phát hiện người (GPU CUDA / CPU)"),
            ("Nút Y / Phím Y", "Đánh dấu NGƯỜI trong 40m, LiDAR xác nhận (ảnh RGB, nhiệt, CSV)"),
            ("D-Pad Lên (↑)", "Hiện / Ẩn đường nối điểm xuất phát với mọi người đã đánh dấu trên bản đồ 3D"),
            ("D-Pad Xuống (↓) | Phím K / F9", "BẬT / TẮT GHI DỮ LIỆU TÌM KIẾM (Ấn lần 1: BẬT  |  Ấn lần 2: TẮT & LƯU vào Data/)"),
            ("Back / Phím T", "Khóa / Mở khóa mục tiêu cứu hộ"),
            ("Nút A / Tab / Ctrl", f"Turbo Boost siêu tốc ({int(self.boost_speed)} m/s - ~{int(self.boost_speed * 3.6)} km/h)"),
            ("Phím [+] / [-]", "Tùy chỉnh tốc độ Turbo Boost (Tăng / Giảm từ 15 ~ 80 m/s)"),
            ("F10 | F11", "Căn chỉnh cửa sổ Unreal & GCS  |  Toàn màn hình"),
            ("F2 | Bấm MAP 3D", "Mở / Chuyển tới cửa sổ Dựng Địa Hình 3D (Open3D LiDAR)"),
            ("F1 | START", "Bật / Tắt bảng hướng dẫn phím bấm này"),
        ]
        ry = 86
        for c1, c2 in rows:
            ov.blit(self.font(14, True).render(f"• {c1}", True, (255, 220, 100)), (w // 2 - 450, ry))
            ov.blit(self.font(12).render(c2, True, (230, 230, 230)), (w // 2 - 430, ry + 19))
            ry += 44
        self.canvas.blit(ov, (0, 0))

    def handle_mouse_click(self, cx, cy):
        if cx >= SIDE_X and DECK_Y <= cy < DECK_Y + DECK_H:
            side_x = cx - SIDE_X
            side_y = cy - DECK_Y
            for btn_id, rect in self.sidebar_buttons:
                if rect.collidepoint(side_x, side_y):
                    if btn_id == "takeoff_land":
                        if self.is_airborne or self.flight_armed: self.trigger_land()
                        else: self.trigger_takeoff()
                    elif btn_id == "toggle_yolo":
                        self.toggle_yolo_mode()
                    elif btn_id == "toggle_record":
                        self.toggle_recording()
                    elif btn_id == "arrange_windows":
                        self.auto_arrange_windows()
                    elif btn_id == "open_3d_map":
                        self.open_or_focus_3d_map()
                    elif btn_id == "toggle_invert":
                        self.toggle_invert_pitch()
                    elif btn_id == "toggle_stabilization":
                        self.toggle_gimbal_stabilization()
                    elif btn_id == "preset_0":
                        self.set_gimbal(0.0)
                    elif btn_id == "preset_-45":
                        self.set_gimbal(-45.0)
                    elif btn_id == "preset_-90":
                        self.set_gimbal(-90.0)
                    elif btn_id == "gimbal_slider":
                        bar_x = rect.x + 4
                        bar_w = rect.width - 8
                        frac = max(0.0, min(1.0, (side_x - bar_x) / float(bar_w)))
                        self.set_gimbal(-90.0 + frac * 90.0)
                    return

        # Pane RGB (Góc trên bên phải)
        if PANE_W <= cx < PANE_W * 2 and DECK_Y <= cy < DECK_Y + 30:
            rx = cx - PANE_W
            if rx > PANE_W - 120:
                self.toggle_rgb_view()

        # Pane Thermal FLIR (Góc trên bên trái)
        if 0 <= cx < PANE_W and DECK_Y <= cy < DECK_Y + 30:
            if cx > PANE_W - 120:
                self.toggle_thermal()
            return

        # Pane LiDAR (Góc dưới bên trái: 0 <= cx < PANE_W và (DECK_Y + PANE_H) <= cy < (DECK_Y + PANE_H * 2))
        if 0 <= cx < PANE_W and (DECK_Y + PANE_H) <= cy < (DECK_Y + PANE_H * 2):
            lid_y = cy - (DECK_Y + PANE_H)
            plot_w = PANE_W - 48
            btn_n_left = plot_w - 75 - 6
            btn_m3d_left = btn_n_left - 100 - 6
            btn_j_left = btn_m3d_left - 80 - 6
            if lid_y < 30:
                if cx >= btn_n_left:
                    self.toggle_radar_heading()
                elif cx >= btn_m3d_left:
                    self.open_or_focus_3d_map()
                elif cx >= btn_j_left:
                    self.toggle_lidar()
                else:
                    # Click vào tiêu đề radar: Mở / Xem Map 3D thay vì tắt
                    self.open_or_focus_3d_map()
            else:
                # Click vào khu vực radar dưới: Đổi thang cự ly zoom [X]
                self.cycle_lidar_range()
            return

    def present(self):
        screen = pygame.display.get_surface()
        if screen is None: return
        w, h = screen.get_size()
        if (w, h) == (CANVAS_W, CANVAS_H):
            screen.blit(self.canvas, (0, 0))
        else:
            s = min(w / float(CANVAS_W), h / float(CANVAS_H))
            dw, dh = max(1, int(CANVAS_W * s)), max(1, int(CANVAS_H * s))
            # Bộ lọc SIMD cho chữ, đường radar và khung mục tiêu sắc hơn khi
            # cửa sổ không ở đúng độ phân giải thiết kế 1600x900.
            scaled = pygame.transform.smoothscale(self.canvas, (dw, dh))
            screen.fill((0, 0, 0))
            screen.blit(scaled, ((w - dw) // 2, (h - dh) // 2))
        pygame.display.flip()

    def set_display(self, fullscreen):
        self.fullscreen = fullscreen
        flags = pygame.FULLSCREEN if fullscreen else (pygame.RESIZABLE | pygame.DOUBLEBUF)
        size = (0, 0) if fullscreen else (self.win_w, self.win_h)
        for attempt_flags in [flags, pygame.RESIZABLE | pygame.DOUBLEBUF, pygame.RESIZABLE, 0]:
            try:
                pygame.display.set_mode(size, attempt_flags, vsync=0)
                return
            except TypeError:
                try:
                    pygame.display.set_mode(size, attempt_flags)
                    return
                except Exception:
                    continue
            except Exception:
                continue
        try:
            pygame.display.set_mode((1280, 720))
        except Exception as e:
            log.warning(f">> [GCS LOI] Khong the tao cua so hien thi: {e}")

    def build_surfaces(self):
        self.canvas = pygame.Surface((CANVAS_W, CANVAS_H)).convert()
        self.panes = {
            "thermal": pygame.Surface((PANE_W, PANE_H)).convert(),
            "rgb": pygame.Surface((PANE_W, PANE_H)).convert(),
            "lidar": pygame.Surface((PANE_W, PANE_H)).convert(),
            "roi": pygame.Surface((PANE_W, PANE_H)).convert(),
            "side": pygame.Surface((SIDE_W, DECK_H)).convert(),
        }
