# HỆ THỐNG GIAO DIỆN UAV GCS ĐIỀU KHIỂN TÌM KIẾM CỨU HỘ
### AirSim 1.8.1 + Unreal Engine 4.27 (Bản đồ Africa)
### Thiết kế Single-Window Dashboard 16:9 (Ouster OS1-64 + FLIR Boson + 3D ROI Point Cloud)

---

## 1. Bố cục Giao diện Thống nhất (Single-Window Unified Dashboard 16:9)
Giao diện trạm điều khiển GCS được thiết kế đồng bộ trên canvas 1600x900 (tự scale theo cửa sổ và hỗ trợ **F11 Fullscreen**), giải quyết triệt để vấn đề 2 cửa sổ Unreal và GCS đè lên nhau nhờ module `win32gui` tự động thu nhỏ/hạ cửa sổ Unreal xuống Taskbar:

```
+------------------------------------+------------------------------------+------------------------+
|                                    |                                    |                        |
|  Ô TRÊN - TRÁI (40% x 50%)         |  Ô TRÊN - PHẢI (40% x 50%)         |                        |
|  CAMERA NHIỆT FLIR                 |  CAMERA RGB & GÓC NHÌN TOÀN CẢNH   |                        |
|  640x512 | FOV 50° | Bounding Box  |  [V] FPV Bụng Drone ↔ Chase Cam   |  SIDEBAR PHẢI          |
|                                    |                                    |  (20% x 100%)          |
+------------------------------------+------------------------------------+                        |
|                                    |                                    |  - KẾT NỐI & TAY CẦM   |
|  Ô DƯỚI - TRÁI (40% x 50%)         |  Ô DƯỚI - PHẢI (40% x 50%) - MỚI   |  - GIMBAL CAMERA       |
|  RADAR LIDAR TOÀN CẢNH             |  KHÔNG GIAN 3D XUNG QUANH MỤC TIÊU |  - THÔNG SỐ BAY        |
|  OUSTER OS1-64 | THANG 120m | BEV  |  (3D ROI POINT CLOUD QUANH NGƯỜI)  |  - DANH SÁCH MỤC TIÊU  |
|  Kèm FOV Cone Camera Nhiệt         |  R = 3m-5m | Isometric 3D Splat    |                        |
+------------------------------------+------------------------------------+------------------------+
```

### Chi tiết 4 Ô Chức năng:
1. **Ô Trên-Trái: Camera Nhiệt FLIR (640x512 | FOV 50°)**
   - Ống kính FLIR Boson tầm xa chuẩn cứu hộ (FOV 50° giúp người dưới mặt đất to rõ, không méo hình).
   - 4 Chế độ bảng màu: White-Hot, Black-Hot, Ironbow (Inferno), Rainbow HC.
   - Tự động phát hiện nhiệt độ cơ thể người (37.4°C) và vẽ Bounding Box màu đỏ quanh nạn nhân.
   - Tâm ngắm crosshair đo nhiệt độ tức thời (°C) và thanh đo nhiệt độ Max/Min.

2. **Ô Trên-Phải: Camera RGB & Góc nhìn Toàn cảnh**
   - Phím tắt `V` (hoặc nút `LB`): Chuyển đổi mượt mà giữa Cam FPV bụng drone và Cam góc nhìn thứ 3 (Chase Cam bám đuôi UAV 5m sau lưng).
   - Người điều khiển không cần phải nhìn sang cửa sổ Unreal Engine nữa.
   - Vẽ Bounding Box người đồng bộ tuyệt đối với Camera Nhiệt khi ở chế độ FPV.

3. **Ô Dưới-Trái: Bản đồ Radar LiDAR Toàn cảnh (Ouster OS1-64)**
   - Nâng cấp cảm biến Ouster OS1-64: 64 kênh, 360° x 45°, tầm quét 120m, 1.31M điểm/giây, tần số 10Hz.
   - Hiển thị mây điểm Bird's-Eye View (BEV) tô màu theo cao độ (dải màu Jet, Turbo, Rainbow).
   - **Hình nón góc nhìn (FOV Cone)** của Camera Nhiệt tự động co giãn và di chuyển theo độ cao UAV và góc chúc Gimbal.
   - Vệt bay (Flight Trail) và các điểm đánh dấu nạn nhân `#[ID]`.

4. **Ô Dưới-Phải: Không gian 3D xung quanh Mục tiêu (3D ROI Point Cloud)**
   - Khi phát hiện người (hoặc bấm `T` khóa mục tiêu), tự động cắt vùng mây điểm bán kính 3m-5m quanh tọa độ 3D của nạn nhân.
   - Hiển thị hình chiếu Isometric 3D với kỹ thuật upsampling splat điểm (vẽ điểm mịn) để thấy rõ cây cối, mô đất, địa hình quanh nạn nhân.
   - Cột mốc mục tiêu màu đỏ (chiều cao 1.8m), vòng bán kính, chỉ hướng Bắc (B), cự ly từ drone.
   - Phím `Z` đổi bán kính 3m ↔ 5m, phím `,` và `.` xoay góc nhìn, phím `O` tự động xoay 360° quanh mục tiêu.

5. **Sidebar Phải: Bảng Điều khiển & Telemetry**
   - Trạng thái kết nối AirSim, Tay cầm / Bàn phím, Tần số Heartbeat lệnh bay (20Hz).
   - Gimbal góc camera (-90° đến 0°), thanh trượt hiển thị trực quan, 3 nút preset Lên 0° / Nghiêng -45° / Xuống -90°.
   - Thông số bay: Độ cao AGL (m), Tốc độ (m/s), Yaw, Pitch, Roll, GPS X, Y, Z, Pin %, Thời gian bay, FPS UI.
   - Danh sách nạn nhân đã đánh dấu: ID, thời gian, tọa độ X/Y, khoảng cách.

---

## 2. Khắc phục Lỗi Điều khiển & Tối ưu Hiệu năng
1. **Sửa lỗi `"API call was not received, entering hover mode for safety"`:**
   - Tách biệt hoàn toàn luồng gửi lệnh bay (`heartbeat_control_loop`) với các luồng đọc telemetry, hình ảnh và LiDAR.
   - Luồng Heartbeat chạy độc lập với 1 client RPC riêng, chu kỳ 50ms (20Hz), liên tục gửi lệnh vận tốc ngay cả khi đứng yên hoặc hover. Nhờ đó, drone không bao giờ bị AirSim ngắt về hover mode an toàn!
2. **Đồng bộ Góc Gimbal Camera & LiDAR:**
   - Ngay khi khởi động, script tự động gọi `simSetCameraPose` và `simSetCameraFov` cập nhật góc cam theo biến `Góc Cam` trên UI (mặc định -45° chúc tìm kiếm trong rừng).
   - Khi bấm phím `1` (0°), `2` (-45°), `3` (-90°) hoặc cần gạt tay cầm, `simSetCameraPose` lập tức cập nhật đồng bộ cho cả Camera Nhiệt FLIR và Camera RGB!
   - Hình nón FOV Cone trên bản đồ Radar LiDAR co giãn tương ứng theo thời gian thực.

---

## 3. Bảng Ánh xạ Nút bấm Tay cầm Xbox và Bàn phím

| Phím Bàn Phím | Nút Tay Cầm Xbox | Chức Năng |
| :--- | :--- | :--- |
| **`W / S`** | Cần Trái (Trục Y) | Bay Tiến / Bay Lùi |
| **`A / D`** | Cần Trái (Trục X) | Bay sang Trái / Bay sang Phải |
| **`Space`** | **RT (Cò Phải)** | Tăng độ cao (Bay lên) |
| **`Shift`** | **LT (Cò Trái)** | Giảm độ cao (Hạ xuống) |
| **`Q / E`** | Cần Phải (Trục X) | Xoay hướng đầu UAV (Yaw Trái / Phải) |
| **`↑ / ↓`** | Cần Phải (Trục Y) | Chỉnh góc Camera liên tục (0° ngang ... -90° nhìn thẳng xuống) |
| **`1 / 2 / 3`** | **R3 (Nhấn Cần Phải) / G** | Chọn nhanh góc Cam: `1` = Lên 0°, `2` = Nghiêng -45°, `3` = Xuống -90° |
| **`V`** | **LB** | Chuyển đổi mượt giữa Cam FPV bụng drone và Chase Cam bám đuôi |
| **`R`** | **RB** | Đổi bảng màu Camera Nhiệt: White-Hot ↔ Black-Hot ↔ Ironbow ↔ Rainbow |
| **`T`** | **Back** | Khóa / Mở khóa mục tiêu cho ô 3D ROI (mặc định tự bám người trên ảnh nhiệt) |
| **`Z`** | - | Đổi bán kính vùng 3D ROI: 3m ↔ 5m |
| **`, / .`** | - | Xoay góc nhìn Isometric của ô 3D ROI |
| **`O`** | - | Bật / Tắt chế độ tự quay 360° quanh mục tiêu 3D |
| **`Y`** | **Nút Y** | **ĐÁNH DẤU VỊ TRÍ NGƯỜI (Lưu tọa độ 3D, ảnh RGB + Nhiệt, CSV)** |
| **`Tab / Ctrl`** | **Nút A** | Giữ nút để kích hoạt Turbo Boost (14 m/s) |
| **`B`** | **Nút B** | Bật / Tắt đèn tìm kiếm cứu hộ (Searchlight) |
| **`X`** | **Nút X** | Đổi thang đo LiDAR Ouster OS1-64 (30m → 60m → 120m) |
| **`C`** | - | Đổi dải màu cao độ LiDAR (JET / TURBO / RAINBOW) |
| **`H`** | **D-Pad** | Tạm dừng tại chỗ (Hover) / Tiếp tục bay |
| **`F10`** | - | Thu nhỏ cửa sổ Unreal xuống Taskbar và đưa GCS Dashboard lên trên |
| **`F11`** | - | Bật / Tắt chế độ Toàn màn hình (Fullscreen 16:9) |
| **`F1`** | **Start** | Bật / Tắt bảng hướng dẫn phím bấm trên màn hình |
| **`Esc`** | - | Thoát toàn màn hình (nếu đang bật) hoặc thoát chương trình |

---

## 4. Dữ liệu Đánh dấu Mục tiêu (Mission Logs)
Mỗi khi bấm **Nút Y** trên tay cầm hoặc bàn phím:
1. **Ảnh chụp hiện trường:** Lưu vào thư mục `rescue_mission_logs/targets/`:
   - `person_001_RGB.jpg` (ảnh RGB kèm khung mục tiêu và thông số vị trí)
   - `person_001_THERMAL.jpg` (ảnh nhiệt FLIR độ tương phản cao kèm khung mục tiêu)
2. **Nhật ký tọa độ:** File CSV `rescue_mission_logs/marked_targets.csv` ghi lại:
   - ID, Thời gian, Tọa độ UAV (X, Y, Z, Alt), Tọa độ 3D mục tiêu mặt đất, Cự ly, Góc Gimbal, Tên file ảnh RGB và Nhiệt.
3. **Hiển thị trực quan:** Radar LiDAR và ô 3D ROI lập tức đánh dấu mốc đỏ `#ID` tại tọa độ nạn nhân.

---

## 5. Khởi động Hệ thống
Nhấp đúp vào file sau để khởi động toàn bộ môi trường mô phỏng và trạm điều khiển:
```cmd
START_UAV_SYSTEM.bat
```
