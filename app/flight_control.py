"""Manual flight, altitude hold and gimbal control; no target classification logic."""
import math
import time
import airsim
import pygame
import pygame._sdl2.controller as sdl2_controller
from configs.config import RUNTIME
from .runtime_support import get_logger
from .input_devices import (
    is_key_down, VK_1, VK_2, VK_3, VK_A, VK_ADD, VK_B, VK_C, VK_CTRL, VK_D,
    VK_DOWN, VK_E, VK_F1, VK_F10, VK_F2, VK_F9, VK_G, VK_H, VK_I, VK_J, VK_K,
    VK_L, VK_LBRACKET, VK_M, VK_N, VK_NUMPAD1, VK_NUMPAD2, VK_NUMPAD3, VK_O,
    VK_OEM_MINUS, VK_OEM_PLUS, VK_PGDN, VK_PGUP, VK_Q, VK_R, VK_RBRACKET, VK_S,
    VK_SHIFT, VK_SPACE, VK_SUBTRACT, VK_T, VK_TAB, VK_U, VK_UP, VK_V, VK_W, VK_X, VK_Y,
)
CMD_PERIOD = RUNTIME.command_period_s
CMD_DURATION = RUNTIME.command_duration_s
THERMAL_FOV_DEG = RUNTIME.thermal_fov_deg
log = get_logger("gcs")


def quat_multiply(q1, q2):
    """Multiply AirSim quaternions without changing the original component order."""
    w1, x1, y1, z1 = float(q1.w_val), float(q1.x_val), float(q1.y_val), float(q1.z_val)
    w2, x2, y2, z2 = float(q2.w_val), float(q2.x_val), float(q2.y_val), float(q2.z_val)
    return airsim.Quaternionr(
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    )


def quat_inverse(q):
    return airsim.Quaternionr(-float(q.x_val), -float(q.y_val), -float(q.z_val), float(q.w_val))


class FlightControlMixin:
    def init_controller(self):
        try:
            sdl2_controller.init()
            if sdl2_controller.get_count() > 0:
                for idx in range(sdl2_controller.get_count()):
                    if sdl2_controller.is_controller(idx):
                        c = sdl2_controller.Controller(idx)
                        self.controller = c
                        self.controller_name = c.name
                        if self._last_reported_ctrl != self.controller_name:
                            self._last_reported_ctrl = self.controller_name
                            log.debug(f">> [TAY CAM XBOX/SDL2] Đã kết nối: {self.controller_name}")
                        return
        except Exception:
            pass

        try:
            pygame.joystick.init()
            cnt = pygame.joystick.get_count()
            if cnt > 0:
                for idx in range(cnt):
                    j = pygame.joystick.Joystick(idx)
                    try:
                        if not j.get_init(): j.init()
                        j_name = j.get_name()
                        ignored = ["keyboard", "huntsman", "mouse", "virtual", "touchpad"]
                        if not any(ign in j_name.lower() for ign in ignored):
                            self.joystick = j
                            self.controller_name = j_name
                            if self._last_reported_ctrl != self.controller_name:
                                self._last_reported_ctrl = self.controller_name
                                log.debug(f">> [TAY CAM JOYSTICK] Đã kết nối: {self.controller_name}")
                            return
                    except Exception:
                        pass
        except Exception:
            pass

    def _debounce(self, key, min_interval=0.25):
        now = time.time()
        last = self._last_db.get(key, 0.0)
        if now - last < min_interval:
            return False
        self._last_db[key] = now
        return True

    def set_gimbal(self, deg):
        self.target_gimbal_pitch = max(-90.0, min(0.0, float(deg)))

    def cycle_gimbal(self):
        if not self._debounce("cycle_gimbal", 0.20): return
        if self.target_gimbal_pitch > -22.0: self.set_gimbal(-45.0)
        elif self.target_gimbal_pitch > -67.0: self.set_gimbal(-90.0)
        else: self.set_gimbal(0.0)

    def toggle_invert_pitch(self):
        if not self._debounce("toggle_invert", 0.20): return
        self.invert_cam_pitch = not self.invert_cam_pitch
        state_str = "ĐÃ ĐẢO CHIỀU (INVERTED)" if self.invert_cam_pitch else "CHUẨN (NORMAL)"
        self.notify(f"[GIMBAL] HƯỚNG ĐIỀU KHIỂN GÓC CAM: {state_str}")

    def toggle_gimbal_stabilization(self):
        if not self._debounce("toggle_gimbal_stab", 0.20): return
        self.gimbal_stabilization_enabled = not self.gimbal_stabilization_enabled
        st = "BẬT (CHỐNG RUNG 2 TRỤC)" if self.gimbal_stabilization_enabled else "TẮT (CỐ ĐỊNH THÂN UAV)"
        self.notify(f"[CHỐNG RUNG CAMERA] {st}")

    def trigger_takeoff(self):
        if not self._debounce("takeoff", 0.5):
            return
        if self.is_taking_off or self.flight_armed or self.is_airborne:
            return
        self.req_takeoff = True
        self.cmd_event.set()

    def _perform_takeoff(self, client):
        """Arm for manual vertical control without issuing an ascent command."""
        state = client.getMultirotorState(vehicle_name=self.vehicle_name)
        current_z = float(state.kinematics_estimated.position.z_val)
        if not math.isfinite(current_z):
            raise RuntimeError("Toa do UAV chua san sang")
        if self._has_ground_contact(client, state, self.vehicle_name):
            self.ground_z_datum = current_z
        client.enableApiControl(True, vehicle_name=self.vehicle_name)
        if not client.armDisarm(True, vehicle_name=self.vehicle_name):
            raise RuntimeError("AirSim khong cho phep arm UAV")
        return current_z

    @staticmethod
    def _has_ground_contact(client, state, vehicle_name):
        """Confirm a resting floor contact when SimpleFlight leaves Flying set."""
        if state.landed_state == airsim.LandedState.Landed:
            return True
        kin = state.kinematics_estimated
        if max(abs(kin.linear_velocity.x_val), abs(kin.linear_velocity.y_val),
               abs(kin.linear_velocity.z_val)) > 0.12:
            return False
        collision = client.simGetCollisionInfo(vehicle_name=vehicle_name)
        # has_collided is an edge flag; resting contacts retain the last pose.
        if collision.time_stamp <= 0 or collision.normal.z_val > -0.7:
            return False
        distance_sq = sum((getattr(kin.position, axis) - getattr(collision.position, axis)) ** 2
                          for axis in ("x_val", "y_val", "z_val"))
        return int(state.timestamp) >= int(collision.time_stamp) and distance_sq < 0.02

    def trigger_land(self):
        if not self._debounce("land", 0.5):
            return
        if self.is_landing or not (self.is_airborne or self.flight_armed):
            return
        self.req_land = True
        self.cmd_event.set()

    def poll_flight_inputs(self, dt_ctrl):
        vx = 0.0
        vy = 0.0
        vz = 0.0
        yaw_rate = 0.0
        is_boost = False

        # 1. Tay cầm XInput
        pad = self.xinput.poll()
        if pad is not None:
            if pad["btn_a"]: is_boost = True
            curr_speed = self.boost_speed if is_boost else self.normal_speed

            if abs(pad["ly"]) > 0.0: vx = pad["ly"] * curr_speed
            if abs(pad["lx"]) > 0.0: vy = pad["lx"] * curr_speed
            yaw_max = 90.0 if is_boost else 55.0
            if abs(pad["rx"]) > 0.0: yaw_rate = pad["rx"] * yaw_max

            # Cần Phải Y: Điều chỉnh góc camera mượt mà (framerate-independent)
            if abs(pad["ry"]) > 0.12:
                ry_val = pad["ry"]
                sign = 1.0 if ry_val > 0 else -1.0
                curved_rate = (abs(ry_val) ** 1.3) * 45.0 * sign  # Tối đa 45°/s
                rate = curved_rate * dt_ctrl
                delta = -rate if self.invert_cam_pitch else rate
                self.target_gimbal_pitch = max(-90.0, min(0.0, self.target_gimbal_pitch + delta))

            # D-Pad Up: hiện/ẩn các đường nối điểm xuất phát với người đã đánh dấu.
            if pad["dpad_up"] and not self._last_pad_dpad_up:
                self.toggle_rescue_routes()
            self._last_pad_dpad_up = pad["dpad_up"]

            # D-Pad Down (Nút Hướng Xuống trong 4 nút D-Pad): BẬT (LẦN 1) / TẮT & LƯU (LẦN 2) GHI DỮ LIỆU
            if pad["dpad_down"] and not self._last_pad_dpad_down:
                self.toggle_recording()
            self._last_pad_dpad_down = pad["dpad_down"]

            # Cò RT / LT (Leo cao / Hạ thấp)
            climb_rate = -10.0 if is_boost else -4.0
            descend_rate = +7.5 if is_boost else +3.2
            if pad["rt"] > 0.10: vz = climb_rate * pad["rt"]
            elif pad["lt"] > 0.10: vz = descend_rate * pad["lt"]

            if pad["btn_r3"] and not self._last_pad_r3: self.cycle_gimbal()
            self._last_pad_r3 = pad["btn_r3"]

            if pad["btn_b"] and not self._last_pad_b: self.searchlight_on = not self.searchlight_on
            self._last_pad_b = pad["btn_b"]

            if pad["btn_x"] and not self._last_pad_x:
                self.lidar_range_idx = (self.lidar_range_idx + 1) % len(self.lidar_ranges)
            self._last_pad_x = pad["btn_x"]

            if pad["btn_y"] and not self._last_pad_y: self.mark_person_waypoint()
            self._last_pad_y = pad["btn_y"]

            if pad["btn_lb"] and not self._last_pad_lb: self.toggle_rgb_view()
            self._last_pad_lb = pad["btn_lb"]

            if pad["btn_rb"] and not self._last_pad_rb:
                self.palette_idx = (self.palette_idx + 1) % len(self.thermal_palettes)
            self._last_pad_rb = pad["btn_rb"]

            if pad.get("btn_l3", False) and not self._last_pad_l3: self.toggle_recording()
            self._last_pad_l3 = pad.get("btn_l3", False)

            if pad["btn_back"] and not self._last_pad_back: self.toggle_target_lock()
            self._last_pad_back = pad["btn_back"]

            if pad["btn_start"] and not self._last_pad_start: self.show_help = not self.show_help
            self._last_pad_start = pad["btn_start"]

            # D-Pad Right: Bật / Tắt quét LiDAR

            if pad["dpad_right"] and not self._last_pad_dpad_right:

                self.toggle_lidar()

            self._last_pad_dpad_right = pad["dpad_right"]

            # D-Pad Left: Bật / Tắt camera nhiệt FLIR

            if pad["dpad_left"] and not self._last_pad_dpad_left:

                self.toggle_thermal()

            self._last_pad_dpad_left = pad["dpad_left"]

        # 2. Bàn phím Windows
        boost_key = is_key_down(VK_TAB) or is_key_down(VK_CTRL)
        if boost_key: is_boost = True
        curr_speed = self.boost_speed if is_boost else self.normal_speed

        if is_key_down(VK_W): vx = curr_speed
        elif is_key_down(VK_S): vx = -curr_speed

        if is_key_down(VK_A): vy = -curr_speed
        elif is_key_down(VK_D): vy = curr_speed

        climb_rate = -10.0 if is_boost else -4.0
        descend_rate = +7.5 if is_boost else +3.2
        if is_key_down(VK_SPACE): vz = climb_rate
        elif is_key_down(VK_SHIFT): vz = descend_rate

        yaw_spd = 90.0 if is_boost else 55.0
        if is_key_down(VK_Q): yaw_rate = -yaw_spd
        elif is_key_down(VK_E): yaw_rate = +yaw_spd

        cam_up = is_key_down(VK_UP) or is_key_down(VK_PGUP) or is_key_down(VK_RBRACKET)
        cam_down = is_key_down(VK_DOWN) or is_key_down(VK_PGDN) or is_key_down(VK_LBRACKET)
        if cam_up and not cam_down:
            rate = 35.0 * dt_ctrl
            d = -rate if self.invert_cam_pitch else rate
            self.target_gimbal_pitch = max(-90.0, min(0.0, self.target_gimbal_pitch + d))
        elif cam_down and not cam_up:
            rate = 35.0 * dt_ctrl
            d = rate if self.invert_cam_pitch else -rate
            self.target_gimbal_pitch = max(-90.0, min(0.0, self.target_gimbal_pitch + d))

        def check_edge(vk_code, state_attr):
            down = is_key_down(vk_code)
            was_down = getattr(self, state_attr, False)
            setattr(self, state_attr, down)
            return down and not was_down

        if check_edge(VK_1, "_edge_1") or check_edge(VK_NUMPAD1, "_edge_n1"): self.set_gimbal(0.0)
        elif check_edge(VK_2, "_edge_2") or check_edge(VK_NUMPAD2, "_edge_n2"): self.set_gimbal(-45.0)
        elif check_edge(VK_3, "_edge_3") or check_edge(VK_NUMPAD3, "_edge_n3"): self.set_gimbal(-90.0)
        elif check_edge(VK_G, "_edge_g"): self.cycle_gimbal()
        elif check_edge(VK_I, "_edge_i"): self.toggle_invert_pitch()
        elif check_edge(VK_F1, "_edge_f1"): self.toggle_gimbal_stabilization()

        if check_edge(VK_M, "_edge_m"): self.toggle_yolo_mode()
        if check_edge(VK_J, "_edge_j"): self.toggle_lidar()
        if check_edge(VK_U, "_edge_u"): self.toggle_thermal()
        if check_edge(VK_Y, "_edge_y"): self.mark_person_waypoint()
        if check_edge(VK_V, "_edge_v"): self.toggle_rgb_view()
        if check_edge(VK_R, "_edge_r"): self.palette_idx = (self.palette_idx + 1) % len(self.thermal_palettes)
        if check_edge(VK_X, "_edge_x"): self.lidar_range_idx = (self.lidar_range_idx + 1) % len(self.lidar_ranges)
        if check_edge(VK_C, "_edge_c"):
            self.lidar_cmap_idx = (self.lidar_cmap_idx + 1) % len(self.lidar_colormaps)
            self.cached_colorbar_surf = None
        if check_edge(VK_H, "_edge_h"): self.is_hover = not self.is_hover
        if check_edge(VK_B, "_edge_b"): self.searchlight_on = not self.searchlight_on
        if check_edge(VK_T, "_edge_t"): self.toggle_target_lock()
        if check_edge(VK_N, "_edge_n"): self.toggle_radar_heading()
        if check_edge(VK_O, "_edge_o"):
            self.toggle_bbox()
        if check_edge(VK_K, "_edge_k") or check_edge(VK_F9, "_edge_f9"): self.toggle_recording()
        if check_edge(VK_L, "_edge_l"):
            if self.is_airborne or self.flight_armed: self.trigger_land()
            else: self.trigger_takeoff()
        if check_edge(VK_F10, "_edge_f10"): self.auto_arrange_windows()
        if check_edge(VK_F2, "_edge_f2"): self.open_or_focus_3d_map()
        if check_edge(VK_OEM_PLUS, "_edge_plus") or check_edge(VK_ADD, "_edge_add"):
            self.boost_speed = min(30.0, self.boost_speed + 2.0)
            self.notify(f"[TỐC ĐỘ TURBO] TĂNG LÊN: {self.boost_speed:.0f} m/s (~{self.boost_speed * 3.6:.0f} km/h)")
        if check_edge(VK_OEM_MINUS, "_edge_minus") or check_edge(VK_SUBTRACT, "_edge_sub"):
            self.boost_speed = max(10.0, self.boost_speed - 2.0)
            self.notify(f"[TỐC ĐỘ TURBO] GIẢM XUỐNG: {self.boost_speed:.0f} m/s (~{self.boost_speed * 3.6:.0f} km/h)")

        self.is_boost = is_boost
        return vx, vy, vz, yaw_rate

    def flight_control_telemetry_loop(self):
        """
        Read telemetry and send manual flight commands through one RPC client.

        Preserve the existing altitude-hold, arming and bounded reconnect logic.
        """
        client = self._connect_client()
        if client is None:
            return

        def init_flight_client(cl):
            try:
                cl.enableApiControl(True, vehicle_name=self.vehicle_name)
                if self.flight_armed:
                    cl.armDisarm(True, vehicle_name=self.vehicle_name)
                try: cl.simSetCameraFov("front_camera", THERMAL_FOV_DEG, vehicle_name=self.vehicle_name)
                except Exception: pass
                self._apply_gimbal(cl)
                try:
                    cl.simRunConsoleCommand("t.MaxFPS 60")
                    cl.simRunConsoleCommand("r.VSync 0")
                    cl.simRunConsoleCommand("t.IdleWhenNotForeground 0")
                except Exception:
                    pass
                with self.lock:
                    self.telemetry["connected"] = True
                    self.telemetry["status"] = "TRỰC TUYẾN"
                log.debug(f">> [FLIGHT SYSTEM] Đã kết nối & kích hoạt điều khiển '{self.vehicle_name}'!")
                self.setup_thermal_and_targets(cl)
                return True
            except Exception:
                return False

        init_flight_client(client)

        # Kiểm tra trạng thái mặt đất lúc khởi động
        try:
            st = client.getMultirotorState(vehicle_name=self.vehicle_name)
            init_pos = st.kinematics_estimated.position
            start_xyz = (float(init_pos.x_val), float(init_pos.y_val), float(init_pos.z_val))
            if all(math.isfinite(v) for v in start_xyz):
                with self.lock:
                    if self.start_pos_ned is None:
                        self.start_pos_ned = start_xyz
                self.export_live_targets_json(force=True)
            init_z = float(st.kinematics_estimated.position.z_val)
            grounded = self._has_ground_contact(client, st, self.vehicle_name)
            if grounded:
                self.ground_z_datum = init_z
            agl = max(0.0, self.ground_z_datum - init_z)
            if grounded:
                self.is_airborne = False
                self.target_z = None
                log.debug(f">> [STANDBY] UAV đang ở mặt đất (Datum Z={self.ground_z_datum:.2f}). Bấm [L] hoặc [Space] (Cò RT) để cất cánh!")
                self.notify("[CHỜ CẤT CÁNH] BẤM [L] HOẶC [SPACE] / CÒ RT ĐỂ CẤT CÁNH")
            else:
                self.is_airborne = True
                self.flight_armed = True
                self.target_z = init_z
                log.debug(f">> [HOVER] UAV đang ở trên không (độ cao {agl:.1f}m). Sẵn sàng!")
        except Exception:
            self.is_airborne = False
            self.target_z = None

        start_time = time.time()
        last_cmd_t = time.perf_counter()
        last_enable_t = time.time()
        consecutive_errors = 0
        previous_sim_timestamp = None
        last_reconnect_notice = 0.0
        last_ground_check = 0.0
        grounded = not self.is_airborne

        while self.running:
            t_now = time.perf_counter()
            loop_dt = max(0.005, min(0.10, t_now - last_cmd_t))
            last_cmd_t = t_now

            # Cập nhật tần số thực tế chính xác (33Hz)
            inst = 1.0 / loop_dt
            self.cmd_hz = self.cmd_hz * 0.9 + inst * 0.1

            # Tự động kích hoạt lại API control định kỳ
            if time.time() - last_enable_t > 3.0:
                last_enable_t = time.time()
                try: client.enableApiControl(True, vehicle_name=self.vehicle_name)
                except Exception: pass

            # 1. XỬ LÝ LỆNH CẤT CÁNH
            if self.req_takeoff:
                self.req_takeoff = False
                self.is_taking_off = True
                self.notify("[ARM] CHUẨN BỊ ĐIỀU KHIỂN ĐỘ CAO THỦ CÔNG")
                try:
                    hover_target_z = self._perform_takeoff(client)
                    self.flight_armed = True
                    self.target_z = hover_target_z
                    self.notify("[ARM XONG] GIỮ SPACE / RT ĐỂ BAY LÊN; THẢ ĐỂ GIỮ ĐỘ CAO")
                except Exception as e:
                    try:
                        actual_state = client.getMultirotorState(vehicle_name=self.vehicle_name)
                        actual_z = float(actual_state.kinematics_estimated.position.z_val)
                        still_flying = (math.isfinite(actual_z) and
                                        self.ground_z_datum - actual_z >= 0.8)
                    except Exception:
                        still_flying = False
                    self.is_airborne = still_flying
                    self.flight_armed = still_flying
                    self.target_z = actual_z if still_flying else None
                    self.notify(f"[LỖI CẤT CÁNH] {e}" +
                                (" | UAV da roi dat, dang giu cao do hien tai" if still_flying else ""))
                finally:
                    self.is_taking_off = False
                continue

            # 2. XỬ LÝ LỆNH HẠ CÁNH
            if self.req_land:
                self.req_land = False
                self.is_landing = True
                self.notify("[HẠ CÁNH] ĐANG TỰ ĐỘNG HẠ CÁNH AN TOÀN...")
                try:
                    landing_state = client.getMultirotorState(vehicle_name=self.vehicle_name)
                    if not self._has_ground_contact(client, landing_state, self.vehicle_name):
                        client.landAsync(timeout_sec=30, vehicle_name=self.vehicle_name).join()
                        landed = client.getMultirotorState(vehicle_name=self.vehicle_name)
                        if not self._has_ground_contact(client, landed, self.vehicle_name):
                            raise RuntimeError("UAV chua ha canh; tiep tuc giu do cao")
                    # landAsync may already have disarmed; repeating it returns False.
                    client.armDisarm(False, vehicle_name=self.vehicle_name)
                    self.flight_armed = False
                    self._vertical_input_active = False
                    self.is_airborne = False
                    self.target_z = None
                    self.notify("[HẠ CÁNH] UAV ĐÃ HẠ CÁNH AN TOÀN TRÊN MẶT ĐẤT.")
                except Exception as e:
                    self.notify(f"[LỖI HẠ CÁNH] {e}")
                finally:
                    self.is_landing = False
                continue

            if self.is_taking_off or self.is_landing:
                time.sleep(0.05)
                continue

            # 3. ĐỌC TELEMETRY & TƯ THẾ UAV
            try:
                st = client.getMultirotorState(vehicle_name=self.vehicle_name)
                sim_timestamp = int(st.timestamp)
                vertical_dt = (max(0.0, min(0.10, (sim_timestamp - previous_sim_timestamp) / 1e9))
                               if previous_sim_timestamp is not None else 0.0)
                previous_sim_timestamp = sim_timestamp
                kin = st.kinematics_estimated
                pos = kin.position
                vel = kin.linear_velocity
                ori = kin.orientation
                if time.monotonic() - last_ground_check >= 0.5:
                    last_ground_check = time.monotonic()
                    grounded = self._has_ground_contact(client, st, self.vehicle_name)
                if abs(vel.z_val) > 0.12:
                    grounded = False
                self.is_airborne = self.flight_armed and not grounded
                pitch_r, roll_r, yaw_r = airsim.to_eularian_angles(ori)
                # alt_asl: Cao độ tương đối so với mốc xuất phát ban đầu (âm nếu lượn xuống vực)
                alt_asl = self.ground_z_datum - pos.z_val
                with self.lock:
                    t_agl = self.terrain_agl
                # alt: Ưu tiên cao độ thực AGL từ LiDAR radar altimeter; nếu chưa có thì dùng ASL
                alt = t_agl if t_agl is not None else max(0.0, alt_asl)
                spd = math.sqrt(vel.x_val ** 2 + vel.y_val ** 2 + vel.z_val ** 2)

                with self.lock:
                    if self.start_pos_ned is None:
                        candidate = (float(pos.x_val), float(pos.y_val), float(pos.z_val))
                        if all(math.isfinite(v) for v in candidate):
                            self.start_pos_ned = candidate
                    self.telemetry.update({
                        "connected": True,
                        "status": "TRỰC TUYẾN",
                        "x": float(pos.x_val), "y": float(pos.y_val), "z": float(pos.z_val),
                        "alt": alt,
                        "alt_asl": alt_asl,
                        "alt_agl": t_agl if t_agl is not None else max(0.0, alt_asl),
                        "vx": float(vel.x_val), "vy": float(vel.y_val), "vz": float(vel.z_val),
                        "speed": spd,
                        "pitch": math.degrees(pitch_r), "roll": math.degrees(roll_r), "yaw": math.degrees(yaw_r),
                        "quat": (float(ori.w_val), float(ori.x_val), float(ori.y_val), float(ori.z_val)),
                        "flight_time": time.time() - start_time,
                    })
                    if self.is_airborne and (time.time() - self.last_trail_time) > 0.5:
                        self.flight_trail.append((float(pos.x_val), float(pos.y_val)))
                        self.last_trail_time = time.time()
                        if len(self.flight_trail) > 400: self.flight_trail.pop(0)

                consecutive_errors = 0
            except Exception as e:
                consecutive_errors += 1
                with self.lock:
                    self.telemetry["connected"] = False
                    self.telemetry["status"] = "ĐANG KẾT NỐI LẠI..."
                if consecutive_errors >= 2:
                    if time.monotonic() - last_reconnect_notice >= 10.0:
                        last_reconnect_notice = time.monotonic()
                        log.warning(f">> [AUTO-RECONNECT] Phát hiện gián đoạn ({e}). Đang tự động kết nối lại AirSim...")
                    new_cl = self._connect_client(timeout=15, max_retries=1)
                    if new_cl and init_flight_client(new_cl):
                        try:
                            client.client.close()
                        except Exception:
                            pass
                        client = new_cl
                        consecutive_errors = 0
                        previous_sim_timestamp = None
                        # Resume at the measured height, including after a map reset.
                        try:
                            recovered = client.getMultirotorState(vehicle_name=self.vehicle_name)
                            recovered_z = float(recovered.kinematics_estimated.position.z_val)
                            if not math.isfinite(recovered_z):
                                raise RuntimeError("Toa do UAV chua san sang sau ket noi lai")
                            self.is_airborne = not self._has_ground_contact(client, recovered, self.vehicle_name)
                            self.flight_armed = self.is_airborne
                            self.target_z = recovered_z if self.is_airborne else None
                            self._vertical_input_active = False
                            if self.is_airborne:
                                client.moveByVelocityZBodyFrameAsync(
                                    0.0, 0.0, self.target_z,
                                    duration=2.0,
                                    vehicle_name=self.vehicle_name
                                )
                        except Exception:
                            self.target_z = None
                time.sleep(0.03)
                continue

            # 4. ĐỌC LỆNH TAY CẦM / BÀN PHÍM
            vx, vy, vz, yr = self.poll_flight_inputs(loop_dt)
            self.cmd_vx, self.cmd_vy, self.cmd_vz, self.cmd_yaw_rate = vx, vy, vz, yr

            # Cất cánh nhanh bằng Space / RT khi đang ở mặt đất
            if not self.flight_armed and not self.is_airborne and vz < -0.5:
                self.trigger_takeoff()
                time.sleep(0.1)
                continue

            if not self.flight_armed:
                time.sleep(CMD_PERIOD)
                continue

            if self.target_z is None:
                with self.lock: curr_z = self.telemetry["z"]
                if not math.isfinite(curr_z):
                    time.sleep(CMD_PERIOD)
                    continue
                self.target_z = curr_z

            # Tự động giữ độ cao (Altitude Hold)
            if abs(vz) > 0.1:
                self._vertical_input_active = True
                # Integrate in simulator time so a slow frame cannot queue a climb.
                self.target_z += vz * vertical_dt
                min_z = self.ground_z_datum - 500.0  # Leo cao tới 500m trên đỉnh núi
                max_z = self.ground_z_datum + 250.0  # Hạ xuống vực sâu 250m
                self.target_z = max(min_z, min(max_z, self.target_z))
            elif self._vertical_input_active:
                self.target_z = float(pos.z_val)
                self._vertical_input_active = False
            elif not self.is_airborne:
                self.target_z = float(pos.z_val)

            is_zero = (abs(vx) < 0.05 and abs(vy) < 0.05 and abs(vz) < 0.05 and abs(yr) < 0.05)

            # 5. GỬI LỆNH BAY VÀ TỰ ĐỘNG KHÓA VỊ TRÍ (CHỐNG RƠI & PHẢN HỒI MƯỢT MÀ KHÔNG TRỄ)
            try:
                # Làm mịn tốc độ xoay Yaw (Yaw Rate Slew-Rate Limiter):
                # Giới hạn gia tốc góc ở mức 140°/s² để triệt tiêu hoàn toàn xung mô-men xoắn cánh quạt,
                # ngăn chặn triệt để hiện tượng rung lắc/vặn xoắn thân UAV khi ấn Q/E hoặc bẻ cần đột ngột!
                diff_yr = yr - self.smooth_yr
                max_yr_step = 140.0 * loop_dt
                if abs(diff_yr) <= max_yr_step:
                    self.smooth_yr = yr
                else:
                    self.smooth_yr += math.copysign(max_yr_step, diff_yr)

                # Điều khiển gia tốc mượt mà có giới hạn độ giật (Jerk-Limited S-Curve Accel):
                # Triệt tiêu xung gia tốc tức thời, giúp UAV thay đổi góc nghiêng êm dịu,
                # loại bỏ hoàn toàn hiện tượng rung lắc/chúi giật khi nhấn/nhả phím điều khiển!
                max_accel = 8.5   # m/s² (~0.85g)
                max_jerk = 35.0   # m/s³

                target_vx = 0.0 if (is_zero or self.is_hover) else vx
                target_vy = 0.0 if (is_zero or self.is_hover) else vy

                for axis, t_v in (('x', target_vx), ('y', target_vy)):
                    c_v = self.smooth_vx if axis == 'x' else self.smooth_vy
                    c_a = self.curr_accel_x if axis == 'x' else self.curr_accel_y
                    d_v = t_v - c_v

                    des_a = math.copysign(min(abs(d_v) / 0.22, max_accel), d_v) if abs(d_v) > 0.02 else 0.0
                    d_a = des_a - c_a
                    max_a_step = max_jerk * loop_dt
                    if abs(d_a) <= max_a_step:
                        c_a = des_a
                    else:
                        c_a += math.copysign(max_a_step, d_a)

                    n_v = c_v + c_a * loop_dt
                    if (d_v > 0 and n_v > t_v) or (d_v < 0 and n_v < t_v):
                        n_v = t_v
                        c_a = 0.0

                    if axis == 'x':
                        self.smooth_vx = n_v
                        self.curr_accel_x = c_a
                    else:
                        self.smooth_vy = n_v
                        self.curr_accel_y = c_a

                # Sử dụng trực tiếp target_z để bộ điều khiển PID nội tại của AirSim giữ độ cao vững chắc,
                # loại bỏ hoàn toàn hiện tượng rung lắc/nhấp nhô cao độ do xung đột setpoint
                client.moveByVelocityZBodyFrameAsync(
                    self.smooth_vx, self.smooth_vy, self.target_z,
                    duration=CMD_DURATION,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=self.smooth_yr),
                    vehicle_name=self.vehicle_name
                )
            except Exception:
                consecutive_errors += 1

            sleep_t = CMD_PERIOD - (time.perf_counter() - t_now)
            if sleep_t > 0:
                time.sleep(sleep_t)

        if self.is_airborne:
            try: client.hoverAsync(vehicle_name=self.vehicle_name)
            except Exception: pass

    def _compute_stabilized_cam_body_quat(self, desired_pitch_deg, drone_pitch_deg, drone_roll_deg, drone_yaw_deg):
        """
        Tính toán Quaternion bù góc nghiêng thân UAV trong Body Frame:
        - Giữ phẳng chân trời thế giới: Roll = 0.0° tuyệt đối (Horizon Lock)
        - Giữ góc chúi/ngước thế giới: Pitch = desired_pitch_deg (Pitch Decoupling)
        - Khóa theo hướng bay thế giới: Yaw = drone_yaw_deg
        """
        clamped_pitch = max(-89.5, min(15.0, desired_pitch_deg))
        if not self.gimbal_stabilization_enabled:
            return airsim.to_quaternion(math.radians(clamped_pitch), 0.0, 0.0)

        dp = math.radians(drone_pitch_deg)
        dr = math.radians(drone_roll_deg)
        dy = math.radians(drone_yaw_deg)

        q_cam_world = airsim.to_quaternion(math.radians(clamped_pitch), 0.0, dy)
        q_drone_filt = airsim.to_quaternion(dp, dr, dy)
        return quat_multiply(quat_inverse(q_drone_filt), q_cam_world)

    def _apply_gimbal(self, client):
        target = self.gimbal_pitch
        try:
            with self.lock:
                dp = self.filtered_drone_pitch
                dr = self.filtered_drone_roll
                dy = self.telemetry.get("yaw", 0.0)
            q_front = self._compute_stabilized_cam_body_quat(target, dp, dr, dy)
            client.simSetCameraPose("front_camera", airsim.Pose(self.cam_mount_pos, q_front), vehicle_name=self.vehicle_name)
            self.gimbal_applied = target
            self._last_applied_gimbal_quat = q_front
        except Exception:
            pass

    def gimbal_loop(self):
        """
        LUỒNG ĐIỀU KHIỂN & CHỐNG RUNG GIMBAL 2 TRỤC CHỦ ĐỘNG (~30Hz):
        - Chống rung lắc chân trời (Horizon Lock / Roll Stabilization): Camera luôn giữ phẳng Roll = 0°.
        - Tách rời góc chúi (Pitch Decoupling): Khóa chặt góc nhìn độc lập (0° / -45° / -90°)
          bất kể UAV chúi mũi tăng tốc (-20°) hay phanh ngửa (+12°).
        - Bộ lọc giảm chấn rung động (Vibration Damping): Triệt tiêu rung động cao tần từ cánh quạt.
        - Điều tiết RPC thông minh tránh nghẽn GameThread Unreal Engine.
        """
        client = self._connect_client(timeout=15)
        if client is None:
            return

        last_applied_quat = None
        last_rpc_t = 0.0
        last_t = time.perf_counter()

        while self.running:
            t_now = time.perf_counter()
            dt = max(0.005, min(0.08, t_now - last_t))
            last_t = t_now

            # 1. Nội suy mượt mà gimbal_pitch tiến tới target_gimbal_pitch
            with self.lock:
                target = self.target_gimbal_pitch
                curr = self.gimbal_pitch
                diff = target - curr
                if abs(diff) > 0.05:
                    max_step = 65.0 * dt  # Tốc độ lướt mượt 65°/s khi chuyển preset
                    step = math.copysign(min(abs(diff), max_step), diff)
                    self.gimbal_pitch = curr + step
                else:
                    self.gimbal_pitch = target
                curr_pitch = self.gimbal_pitch

                p_raw = self.telemetry.get("pitch", 0.0)
                r_raw = self.telemetry.get("roll", 0.0)
                y_raw = self.telemetry.get("yaw", 0.0)

            # 2. Bộ lọc thông thấp giảm chấn rung động động cơ (~10 Hz)
            alpha = 0.32
            self.filtered_drone_pitch = (1.0 - alpha) * self.filtered_drone_pitch + alpha * p_raw
            self.filtered_drone_roll = (1.0 - alpha) * self.filtered_drone_roll + alpha * r_raw

            # 3. Tính toán Quaternion chống rung bù góc nghiêng thân UAV
            q_cam_body = self._compute_stabilized_cam_body_quat(
                curr_pitch, self.filtered_drone_pitch, self.filtered_drone_roll, y_raw
            )

            # 4. Kiểm tra góc biến thiên so với lần gửi trước (ngưỡng 0.12° hoặc định kỳ 150ms khi có biến thiên nhỏ)
            need_update = False
            if last_applied_quat is None:
                need_update = True
            else:
                dot = (q_cam_body.w_val * last_applied_quat.w_val +
                       q_cam_body.x_val * last_applied_quat.x_val +
                       q_cam_body.y_val * last_applied_quat.y_val +
                       q_cam_body.z_val * last_applied_quat.z_val)
                ang_diff = 2.0 * math.acos(max(-1.0, min(1.0, abs(dot))))
                ang_deg = math.degrees(ang_diff)
                if ang_deg >= 0.12 or (t_now - last_rpc_t >= 0.15 and ang_deg >= 0.03):
                    need_update = True

            if need_update:
                try:
                    client.simSetCameraPose("front_camera", airsim.Pose(self.cam_mount_pos, q_cam_body), vehicle_name=self.vehicle_name)
                    last_applied_quat = q_cam_body
                    last_rpc_t = t_now
                    self.gimbal_applied = curr_pitch
                except Exception:
                    time.sleep(0.3)
                    new_cl = self._connect_client(timeout=15, max_retries=2)
                    if new_cl:
                        try: client.client.close()
                        except Exception: pass
                        client = new_cl
                    continue

            # Giữ nhịp ~30Hz nhẹ nhàng, không gây nghẽn GameThread
            time.sleep(0.033)
