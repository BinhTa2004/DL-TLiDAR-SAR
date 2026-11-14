# UAV Search-and-Rescue: Thermal–LiDAR 3D Localization & Mapping System

Hệ thống mô phỏng tìm kiếm và cứu nạn (Search and Rescue — SAR) bằng thiết bị bay không người lái (UAV) trong môi trường rừng rậm và địa hình phức tạp, kết hợp **Camera Nhiệt Hồng ngoại (Thermal IR)**, **Camera Quang học (RGB)** và **Cảm biến Quét Laser 3D (LiDAR)** trên nền tảng **Unreal Engine 4.27 & Microsoft AirSim (`MedievalVillageMegascansS`)**.

---

## 1. Bối cảnh Mô phỏng & Bài toán Thực tế

Trong thực tế tìm kiếm người mất tích ở vùng rừng núi hoặc khu vực thiên tai:
- **Camera RGB thông thường** rất khó phát hiện nạn nhân khi bị che khuất bởi bóng râm, tán cây rậm rạp hoặc điều kiện thiếu sáng.
- **Camera Nhiệt 2D** giúp phát hiện nhanh nguồn nhiệt ấm áp của cơ thể sống nhưng chỉ cho biết vị trí trên mặt phẳng ảnh 2D (không đo được khoảng cách/tọa độ 3D chính xác) và dễ bị báo động giả bởi **động vật hoang dã** hoặc **đá/mái nhà hấp thụ nhiệt mặt trời**.
- **Cảm biến LiDAR 3D** có khả năng phóng các tia laser xuyên qua các khe hở của tán lá để đo tọa độ 3D của mặt đất và vật thể bên dưới, nhưng đám mây điểm 3D thuần túy lại không biết đâu là cơ thể người có nhiệt độ sống.

---

## 2. Kiến trúc Mô phỏng Unreal Engine 4.27 & Microsoft AirSim

Hệ thống sử dụng sự kết hợp giữa **Unreal Engine 4.27 (UE4)** và plugin **Microsoft AirSim** để tái tạo một môi trường thử nghiệm vật lý khép kín, sát với điều kiện bay thực tế:

### 2.1. Vai trò của Unreal Engine 4.27 (`MedievalVillageMegascansS`)
- **Đồ họa & Địa hình Photorealistic (Quixel Megascans):** Xây dựng thế giới 3D quy mô lớn gồm ngôi làng cổ, rừng thông nhiều tầng tán lá, địa hình đồi dốc lồi lõm, vách đá, bờ suối và các công trình gỗ/đá với lưới va chạm vật lý (Collision Mesh) chi tiết.
- **Kịch bản 12 Nạn nhân & Tác nhân Gây nhiễu:**
  - Trong bản đồ bố trí **12 mô hình nạn nhân 3D (Ground Truth Targets)** ở nhiều tư thế khác nhau (nằm bất tỉnh, ngồi tựa gốc cây, đứng khuất dưới tán thông, nằm cạnh bờ suối hoặc sát vách nhà).
  - Bổ sung các đàn **động vật hoang dã** (`BUFFALO_DEMO` có thân nhiệt ấm) cùng **đá tảng, thân cây gỗ, mái nhà** làm nguồn gây nhiễu tự nhiên để kiểm chứng độ chính xác của thuật toán.
- **Bản đóng gói Độc lập (`WindowsNoEditor`):** Toàn bộ môi trường Unreal Engine và plugin AirSim đã được biên dịch và đóng gói sẵn thành file thực thi độc lập (`MedievalVillageMegascansS.exe`). **Người dùng KHÔNG CẦN cài đặt bộ công cụ Unreal Engine Editor hay Visual Studio** — chỉ cần tải gói bản đồ về là chạy được ngay.

### 2.2. Vai trò của Microsoft AirSim & Hệ thống Cảm biến trên UAV (`UAV1`)
Plugin **AirSim** chạy bên trong tiến trình Unreal Engine, đảm nhiệm 3 nhiệm vụ cốt lõi:
1. **Mô phỏng Động lực học Bay (`SimpleFlight` 6-DoF):**
   - Tính toán vật lý bay 6 bậc tự do của quadrotor (trọng lực, lực đẩy cánh quạt, quán tính, ma sát không khí) và mô phỏng các cảm biến nội tại gồm **IMU (Gia tốc kế & Con quay hồi chuyển)**, **Từ kế (Magnetometer)**, **Áp kế (Barometer)** và **GPS**.
2. **Mô phỏng Tổ hợp Cảm biến Thực tế gắn trên UAV (Cấu hình tại [`simulation/airsim_settings.json`](simulation/airsim_settings.json)):**
   - **Camera Nhiệt Hồng ngoại FLIR Boson (`640×512`, FOV `90°`):** Trích xuất kênh bức xạ hồng ngoại từ bộ đệm Stencil/Infrared của Unreal Engine (`Human ID = 255`, `Animal ID = 230`), kết hợp mô hình truyền nhiệt lõi cơ thể (Torso Core Heat) và tán xạ thấu kính Germanium.
   - **Camera Quang học RGB (`640×512`, FOV `90°`):** Đặt đồng trục với camera nhiệt trên cụm Gimbal có chống rung (`Phím Z`) và điều chỉnh được góc nghiêng (`Pitch` từ `0°` nhìn thẳng đến `-90°` nhìn thẳng xuống đất).
   - **Cảm biến Quét Laser 3D LiDAR (`Lidar1` — 16 kênh, 10 Hz, 30.000 điểm/giây, tầm xa 75 m, góc quét dọc `+10°` đến `-45°`):** Bắn các tia Ray-casting trực tiếp trong không gian 3D của Unreal Engine, xuyên qua các khe hở tán lá để thu nhận đám mây điểm 3D `(X, Y, Z)` của địa hình và cơ thể nạn nhân bên dưới.
3. **Giao tiếp Thời gian thực qua MsgPack-RPC (Cổng TCP `41451`):**
   - AirSim mở cổng RPC `41451` để trạm điều khiển Python gửi lệnh vận tốc/góc quay ở tần số `33 Hz` và thu thập đồng bộ ảnh RGB + Nhiệt + LiDAR theo thời gian thực.

### 2.3. Hai Chế độ Vận hành Cửa sổ Unreal Engine
Khi chạy hệ thống, bạn có thể lựa chọn 1 trong 2 chế độ hiển thị của Unreal Engine:
- **Chế độ Tối ưu Hiệu năng (`python main.py` hoặc `run.bat` — Mặc định):** Unreal Engine chạy ở chế độ `ViewMode: NoDisplay` (vẫn tính toán đầy đủ vật lý bay, quét tia LiDAR 3D và render hình ảnh cho camera trên mình UAV, nhưng tắt cửa sổ góc nhìn thứ 3 của màn hình chính). Chế độ này giúp **giảm 40%–60% tải GPU/VRAM**, ưu tiên tối đa FPS cho suy luận AI `YOLO26m`, giao diện GCS 2D và Bản đồ 3D.
- **Chế độ Hiển thị Đầy đủ Cửa sổ 3D Unreal Engine (`python main.py all --show-unreal`):** Mở thêm cửa sổ đồ họa 3D trực tiếp của Unreal Engine (`1024×576`, chế độ camera `SpringArmChase` bay bám theo sau lưng chiếc UAV trong rừng) để bạn vừa điều khiển trên trạm GCS vừa quan sát trực quan chiếc drone đang bay lượn trong thế giới 3D.

---

## 3. Khi Chạy Chương trình Hệ thống Hoạt động Như thế nào?

Khi bạn khởi chạy hệ thống, một quy trình tác chiến cứu hộ khép kín gồm **4 bước** sẽ diễn ra như sau:

### Bước 1: Tự động Khởi tạo Thế giới Unreal Engine & Hiệu chuẩn Nhiệt độ
- Hệ thống tự động bật tiến trình Unreal Engine (`MedievalVillageMegascansS.exe`) và đợi máy chủ AirSim sẵn sàng trên cổng `41451`.
- Ngay khi bản đồ tải xong, [`simulation/scene_setup.py`](simulation/scene_setup.py) tự động quét toàn bộ đối tượng 3D trong cảnh để hiệu chuẩn đặc tính bức xạ nhiệt (`outputs/thermal_audit.json`): nền rừng/sông suối/đất đá mang nhiệt độ môi trường mát, động vật (`ID 230`) mang nhiệt độ ấm vừa phải, và 12 nạn nhân (`ID 255`) mang bức xạ nhiệt cơ thể người (`36.5°C – 38.5°C`).

### Bước 2: Mở Đồng thời 2 Cửa sổ Điều hành Thời gian thực
Hệ thống tự động mở và sắp xếp **2 cửa sổ tác chiến hoạt động song song** (cùng cửa sổ Unreal Engine nếu bật `--show-unreal`):

1. **Cửa sổ 1 — Trạm Điều khiển Mặt đất 2D (`GCS Station` - 60 FPS):**
   - **Khung Camera Nhiệt FLIR (Góc trên bên trái):** Hiển thị luồng ảnh nhiệt kèm các khung nhận diện AI **YOLO26m (`960×960`)** tự động bám theo mục tiêu (bật/tắt nhanh bằng phím **`M`**). Trên mỗi khung hiển thị rõ: Phân loại (`NGƯỜI`, `ĐỘNG VẬT`, hoặc `TÍN HIỆU NHIỆT`), khoảng cách 3D tính bằng mét, và số điểm LiDAR xác nhận.
   - **Khung Camera RGB (Góc trên bên phải):** Hiển thị hình ảnh màu thực tế từ camera quang học của UAV (chuyển đổi góc nhìn FPV bụng drone ↔ Chase Cam bám đuôi bằng phím **`V`**), kèm thước ngắm tâm (Crosshair) và góc nghiêng Gimbal.
   - **Khung Radar LiDAR 2D (Góc dưới bên trái):** Hiển thị mặt cắt đám mây điểm xung quanh UAV theo hướng mũi bay (`Heading-Up` hoặc `North-Up` bằng phím **`N`**), giúp phi công quan sát địa hình và tránh vật cản.
   - **Khung Phóng đại Mục tiêu ROI & Bảng Điều khiển Cứu hộ (Góc dưới bên phải & Cột phải):** Phóng to mục tiêu đang theo dõi, hiển thị trạng thái kết nối, tốc độ bay, độ cao AGL, các nút bấm nhanh trên Sidebar, và **Danh sách các nạn nhân đã đánh dấu** kèm tọa độ 3D `(X, Y, Z)`.

2. **Cửa sổ 2 — Bản đồ Đám mây Điểm 3D Thời gian thực (`Open3D Viewer`):**
   - Nhận trực tiếp luồng điểm LiDAR từ trạm GCS thông qua bộ nhớ dùng chung (`Shared Memory mmap`) với độ trễ bằng 0.
   - **UAV bay đến đâu, bản đồ 3D được vẽ thêm đến đó:** Toàn bộ nhà cửa, rừng cây, sườn đồi trong Unreal Engine được quét và tái tạo thành bản đồ đám mây điểm 3D sống động tô màu theo độ cao địa hình, kèm mô hình khung UAV di chuyển theo thời gian thực.
   - Tích hợp bộ khóa phím thông minh ([`app/key_bindings.py`](app/key_bindings.py)): tách biệt hoàn toàn phím tắt của bản đồ 3D (`P`, `F`, `F3`–`F7`, `F12`) với phím lái UAV, đảm bảo khi bạn đang mở cửa sổ 3D mà bấm `WASD`, `Space`, `Shift`, `M`, `Y`... sẽ không bao giờ làm tắt hay nhảy nhầm chức năng của Open3D.

### Bước 3: Bay Tuần tra, Hợp nhất Cảm biến & Đánh dấu Cứu hộ
Trong lúc bạn điều khiển UAV bay tuần tra trên khu vực rừng và ngôi làng:
1. **Phát hiện từ xa ($> 40\text{ m}$):** Khi phát hiện đốm nhiệt ở xa, hệ thống hiển thị khung cảnh báo `TÍN HIỆU NHIỆT` để phi công điều khiển UAV bay lại gần kiểm tra.
2. **Phân loại & Xác minh Hình học 3D ($\le 40\text{ m}$):** Khi UAV tiếp cận trong bán kính `40 m`:
   - Mô hình **YOLO26m** phân loại hình dáng nhiệt trên ảnh 2D.
   - Đồng thời, lõi thuật toán [`pipeline/`](pipeline/) chiếu các điểm LiDAR 3D vào trong khung nhận diện 2D, gom cụm không gian–nhiệt **4D DBSCAN** và tính toán đặc trưng hình học **PCA** (kích thước dài/rộng/cao, thể tích, độ phẳng):
     - Nếu cụm điểm là mặt phẳng ngang (đá nóng, mặt đất) hoặc cột cao hẹp (thân cây) $\to$ **Tự động loại bỏ (`REJECTED`)**.
     - Nếu là động vật $\to$ Hiển thị nhãn **`ĐỘNG VẬT`** (màu vàng), khóa chức năng đánh dấu để tránh báo động giả.
     - Nếu kích thước và nhiệt độ khớp với cơ thể người $\to$ Hiển thị nhãn **`NGƯỜI | LiDAR XÁC NHẬN`** (màu xanh lá) và tính chính xác tọa độ tâm 3D của nạn nhân trên mặt đất.
3. **Đánh dấu Nạn nhân (Phím `Y` trên bàn phím hoặc Nút `Y` trên tay cầm Xbox):**
   - Khi mục tiêu đã được xác nhận là **`NGƯỜI`** trong phạm vi `40 m`, bạn bấm phím **`Y`** (hoặc bấm **`F8`** nếu muốn xóa toàn bộ mốc để đánh dấu lại).
   - **Trên Bản đồ 3D:** Lập tức xuất hiện một **Khung hộp 3D màu đỏ cùng Cột tiêu Cứu hộ (Rescue Beacon)** dựng thẳng đứng ngay tại vị trí nạn nhân trên mặt đất.
   - **Trong thư mục `outputs/`:** Hệ thống tự động chụp lưu 2 bức ảnh bằng chứng (1 ảnh RGB + 1 ảnh Nhiệt có đóng dấu tọa độ vào `outputs/rescue_mission_logs/`), ghi tọa độ 3D vào `outputs/rescue_targets_log.csv` và `outputs/rescue_localization_metrics.csv`, đồng thời đối chiếu ngay với `ground_truth.json` để tính sai số định vị 2D/3D (độ chính xác trung bình thực nghiệm đạt **`0.703 m`** trên toàn bộ `12/12` nạn nhân).

### Bước 4: Lưu Bản đồ 3D & Xem lại Chiến dịch Cứu hộ
- Bạn có thể bấm phím **`K`** (hoặc **`F9`**) trong lúc bay để bật/tắt ghi dữ liệu tìm kiếm (chuỗi ảnh nhiệt `.jpg` + đám mây điểm `.npy` + nhật ký bay `.csv`) vào `outputs/session_YYYYMMDD_HHMMSS/`.
- Trên cửa sổ Bản đồ 3D, bấm phím **`P`** bất cứ lúc nào để lưu toàn bộ bản đồ đám mây điểm 3D ra file `outputs/reconstructed_map_3d.ply` kèm danh sách mục tiêu (`reconstructed_map_3d_targets.json` / `.csv`). Đội cứu hộ có thể mở xem lại bản đồ này bằng lệnh `python main.py map`.

---

## 4. Cấu trúc Thư mục Dự án

```text
DL-TLiDAR-SAR/
│
├── main.py                           # Điểm chạy chính điều phối toàn hệ thống (all, uav, sim, gcs, lidar, map, check)
├── run.bat                           # File khởi chạy nhanh 1-Click trên Windows
├── requirements.txt                  # Danh sách các thư viện Python cần thiết (Python 3.10 - 3.12)
├── pyrightconfig.json                # Cấu hình kiểm tra kiểu tĩnh và đường dẫn import cho IDE
├── README.md                         # Tài liệu mô tả dự án, hướng dẫn cài đặt và vận hành
│
├── configs/                          # 1. CẤU HÌNH HỆ THỐNG
│   ├── config.py                     # Tập trung toàn bộ tham số: YOLO26m, Voxel 0.10m, DBSCAN, ngưỡng lọc hình học & nhiệt độ
│   └── setup_environment.py          # Script hỗ trợ cài đặt nhanh toàn bộ thư viện từ requirements.txt
│
├── pipeline/                         # 2. LÕI THUẬT TOÁN XỬ LÝ & HỢP NHẤT CẢM BIẾN (6 module chuẩn)
│   ├── acquisition.py                # Giải mã khung ảnh RGB/Thermal và gom cửa sổ quét LiDAR 0.9s
│   ├── thermal_detector.py           # Tổng hợp bức xạ nhiệt FLIR và nhận diện người bằng YOLO26m (960x960)
│   ├── lidar_processing.py           # Lọc cự ly, lọc Voxel 0.10m, gom cụm 4D DBSCAN và trích xuất đặc trưng hình học PCA
│   ├── frustum_alignment.py          # Biến đổi tọa độ camera/UAV, chiếu 3D->2D và gán nhiệt độ tạo điểm 4D (X,Y,Z,T)
│   ├── occlusion_fusion.py           # Dò tia (Ray-casting) xuyên tán lá, lọc vật cản giả và chấm điểm tin cậy đa cảm biến
│   └── localization_3d.py            # Chọn cụm mục tiêu tối ưu và tính tọa độ tâm 3D của nạn nhân
│
├── app/                              # 3. ĐIỀU KHIỂN BAY, GIAO DIỆN 2D HUD & BẢN ĐỒ 3D
│   ├── uav_controller.py             # Bộ điều khiển trung tâm Trạm Mặt đất (GCS) và vòng lặp chính
│   ├── flight_control.py             # Điều khiển bay 33Hz, giữ độ cao (Altitude Hold), Gimbal và xử lý phím bấm
│   ├── input_devices.py              # Đọc tín hiệu điều khiển từ Bàn phím (Win32 VK) và Tay cầm Xbox (XInput)
│   ├── key_bindings.py               # Quản lý phím tắt GLFW của Bản đồ 3D (P, F, F3-F7, F12) và khóa chặn phím bay UAV
│   ├── sensor_workers.py             # Các luồng chạy ngầm đọc dữ liệu Camera/LiDAR từ AirSim và gọi vào pipeline/
│   ├── hud_display.py                # Entry module khởi chạy giao diện điều khiển 2D HUD
│   ├── gcs_display.py                # Vẽ chi tiết giao diện HUD 2D, camera nhiệt/RGB, radar, sidebar và bảng hướng dẫn F1
│   ├── layout.py                     # Cấu hình kích thước và bố cục khung hình giao diện GCS
│   ├── viewer_3d.py                  # Entry module cho cửa sổ Bản đồ Đám mây Điểm 3D thời gian thực (Open3D)
│   ├── lidar_mapping.py              # Tích lũy đám mây điểm 3D toàn cục theo thời gian thực và lưu file .ply
│   ├── lidar_display.py              # Vòng lặp hiển thị Open3D 3D, vẽ mô hình UAV và khung đánh dấu nạn nhân
│   ├── shared_lidar_stream.py        # Bộ nhớ dùng chung (mmap) truyền trực tiếp đám mây điểm từ GCS sang cửa sổ 3D
│   ├── mission_records.py            # Chụp ảnh bằng chứng (RGB + Nhiệt), ghi/xóa mốc nạn nhân (Y / F8) và xuất log CSV
│   ├── detection_policy.py           # Quy tắc phân loại trong bán kính 40m và điều kiện xác nhận mục tiêu cứu hộ
│   ├── runtime_support.py            # Quản lý kết nối AirSim, khởi tạo phiên bay an toàn đa tiến trình và ghi log
│   └── view_saved_map.py             # Công cụ mở xem lại file bản đồ 3D (.ply) và mốc cứu hộ sau chuyến bay
│
├── evaluation/                       # 4. ĐÁNH GIÁ ĐỘ CHÍNH XÁC ĐỊNH VỊ
│   └── metrics.py                    # Đối chiếu tọa độ định vị với Ground Truth, tính sai số 2D/3D (Mean, Median, P90)
│
├── simulation/                       # 5. MÔI TRƯỜNG MÔ PHỎNG UNREAL ENGINE & AIRSIM
│   ├── WindowsNoEditor/              # [TẢI TỪ GOOGLE DRIVE] Bộ cài bản đồ 3D Unreal Engine MedievalVillageMegascansS
│   ├── airsim/                       # Thư viện giao tiếp RPC AirSim tích hợp sẵn (tương thích Python 3.10 - 3.12)
│   ├── airsim_settings.json          # Cấu hình chi tiết phương tiện UAV1, Camera RGB/Nhiệt và cảm biến LiDAR 16 kênh
│   ├── simulator_lifecycle.py        # Quản lý khởi động/đóng an toàn tiến trình Unreal Engine và cổng 41451
│   ├── scene_setup.py                # Thiết lập bức xạ nhiệt cho các đối tượng trong map và xuất thermal_audit.json
│   └── ground_truth.json             # Tọa độ thực 3D của 12 mục tiêu nạn nhân trong bản đồ Unreal Engine
│
├── outputs/                          # 6. KẾT QUẢ ĐẦU RA & DỮ LIỆU PHIÊN BAY
│   ├── reconstructed_map_3d.ply      # File bản đồ đám mây điểm 3D đã tái tạo
│   ├── reconstructed_map_3d_targets.csv  # Danh sách tọa độ các nạn nhân đi kèm bản đồ 3D đã lưu
│   ├── rescue_targets_log.csv        # Nhật ký chi tiết các nạn nhân được đánh dấu trong phiên bay
│   ├── rescue_localization_metrics.csv   # Bảng tổng hợp sai số định vị 2D/3D của phiên bay hiện tại
│   ├── localization_results.csv      # Kết quả định vị thực nghiệm chuẩn trên 12 mục tiêu
│   ├── localization_metrics.csv      # Bảng tổng hợp độ chính xác thực nghiệm (12/12 mục tiêu, Mean 3D Error 0.703m)
│   ├── thermal_audit.json            # Kết quả kiểm tra gán mã nhiệt Stencil ID cho 12 người (255) và 2 động vật (230)
│   └── rescue_mission_logs/          # Thư mục chứa ảnh chụp hiện trường (RGB + Nhiệt) khi đánh dấu nạn nhân
│
└── Yolo/                             # 7. TRỌNG SỐ MÔ HÌNH AI
    └── Yolo26m_trained(1).pt         # Trọng số mô hình YOLO26m nhận diện người trên ảnh nhiệt UAV
```

---

## 5. Hướng dẫn Cài đặt & Thiết lập Bản đồ Unreal Engine (Map Setup)

### Bước 1: Cài đặt Thư viện Python
- Yêu cầu **Windows 10/11 (64-bit)**, **DirectX 11/12** và **Python 3.10 – 3.12** (khuyến nghị máy có card đồ họa rời NVIDIA hỗ trợ CUDA để chạy mượt cả Unreal Engine và AI YOLO26m).
- Mở Terminal tại thư mục gốc của dự án và chạy:

```powershell
pip install -r requirements.txt
```

*(Nếu thư mục dự án đã đi kèm gói môi trường `python_env/`, bạn có thể bỏ qua bước này).*

---

### Bước 2: Tải và Giải nén Bản đồ Unreal Engine (`WindowsNoEditor`) từ Google Drive

Do gói môi trường 3D Unreal Engine 4.27 (`WindowsNoEditor` đã tích hợp sẵn plugin AirSim) có dung lượng lớn nên được lưu trữ riêng trên Google Drive:

> 🔗 **Link tải Bản đồ Mô phỏng Unreal Engine (`WindowsNoEditor`):**  
> **[https://drive.google.com/file/d/1LOmgK6vbJ0kVfmGQegFqbfDdbIUB-yZu/view](https://drive.google.com/file/d/1LOmgK6vbJ0kVfmGQegFqbfDdbIUB-yZu/view)**

Sau khi tải về, thực hiện **1 trong 2 cách** sau để gắn bản đồ vào dự án:

- **Cách 1 — Giải nén trực tiếp vào `simulation/` (Khuyên dùng):**  
  Giải nén thư mục `WindowsNoEditor` vào trong thư mục `simulation/` sao cho đường dẫn tới file `.exe` của Unreal Engine đúng như sau:
  ```text
  simulation/WindowsNoEditor/MedievalVillageMegascansS.exe
  ```

- **Cách 2 — Nối đường dẫn nhanh (Junction Link) nếu Map đang để ở ổ đĩa khác:**  
  Nếu bạn đã giải nén `WindowsNoEditor` ở một thư mục khác trên máy (ví dụ `D:\Map\WindowsNoEditor`) và không muốn copy lại tốn dung lượng ổ cứng, hãy mở **Command Prompt (CMD)** tại thư mục dự án và chạy:
  ```cmd
  mklink /J "simulation\WindowsNoEditor" "D:\Map\WindowsNoEditor"
  ```

Kiểm tra nhanh xem dự án đã nhận đủ Map Unreal Engine, Model YOLO và thư viện chưa bằng lệnh:

```powershell
python main.py check
```

---

## 6. Cách Chạy Chương trình

### Cách 1: Chạy nhanh bằng `run.bat` (1-Click)
Bấm đúp chuột trực tiếp vào file **`run.bat`** ở thư mục gốc. Hệ thống sẽ tự động mở bản đồ Unreal Engine, hiệu chuẩn 12 mục tiêu nhiệt, và bật đồng thời **Trạm Điều khiển Mặt đất 2D (GCS HUD)** cùng **Cửa sổ Bản đồ 3D Thời gian thực (Open3D)**.

### Cách 2: Chạy bằng lệnh Python theo từng chế độ

```powershell
# 1. Chạy toàn bộ hệ thống: Simulator + Trạm GCS 2D + Bản đồ 3D (Mặc định tắt viewport chính của Unreal để tối ưu GPU)
python main.py

# 2. Chạy toàn bộ hệ thống và HIỂN THỊ CẢ CỬA SỔ 3D CỦA UNREAL ENGINE (góc nhìn bay sau lưng UAV)
python main.py all --show-unreal

# 3. Chạy Simulator + Trạm GCS 2D (chưa mở ngay cửa sổ Bản đồ 3D, có thể bấm F2 để mở bất kỳ lúc nào)
python main.py uav

# 4. Chỉ bật môi trường mô phỏng Unreal Engine / AirSim và hiệu chuẩn mục tiêu nhiệt
python main.py sim

# 5. Chỉ mở Trạm điều khiển 2D (dùng khi Unreal Engine / AirSim đã bật sẵn)
python main.py gcs

# 6. Chỉ mở Cửa sổ Bản đồ Đám mây Điểm 3D thời gian thực
python main.py lidar

# 7. Mở hộp thoại chọn file (.ply) để xem lại bản đồ 3D và các mốc nạn nhân đã lưu
python main.py map
```

Nếu muốn mở xem ngay bản đồ 3D mẫu đã dựng sẵn trong `outputs/` mà không cần bật Unreal Engine:

```powershell
python app/view_saved_map.py outputs/reconstructed_map_3d.ply
```

---

## 7. Bảng Phím Điều khiển Chi tiết

### 7.1. Điều khiển UAV & Trạm Mặt đất 2D (`GCS Station`)

| Chức năng | Bàn phím (Keyboard) | Tay cầm Xbox (XInput) |
| :--- | :--- | :--- |
| **Cất cánh (Arm/Takeoff) / Hạ cánh (Land)** | **`L`** *(hoặc giữ `Space` để cất cánh)* | Cò **`RT`** (Cất cánh) |
| **Bay Tiến / Lùi / Trái / Phải (Auto Altitude Hold)** | **`W` / `S` / `A` / `D`** | Cần gạt Trái (`Left Stick`) |
| **Tăng / Giảm độ cao (Thả tay tự giữ độ cao)** | **`Space`** (Lên) / **`Shift`** (Xuống) | Cò **`RT`** (Lên) / Cò **`LT`** (Xuống) |
| **Xoay mũi UAV (Yaw Trái / Phải)** | **`Q` / `E`** | Cần gạt Phải ngang (`Right Stick X`) |
| **Bay giữ vị trí (Hover Hold)** | **`H`** | — |
| **Tăng tốc độ bay (Turbo Boost)** | Giữ **`Tab`** hoặc **`Ctrl`** | Nút **`A`** |
| **Tăng / Giảm mức tốc độ Turbo Boost (`10–30 m/s`)** | Phím **`+`** / **`-`** | — |
| **Chỉnh góc nghiêng Camera (Gimbal Pitch `0°` đến `-90°`)** | **`↑` / `↓`** *(hoặc `PgUp`/`PgDn`, `[`/`]`)* | Cần gạt Phải dọc (`Right Stick Y`) |
| **Góc nhanh Gimbal (`0°`, `-45°`, `-90°` / Xoay vòng)** | Phím **`1` / `2` / `3`** hoặc **`G`** | Nhấn cần Phải (**`R3`**) |
| **Đảo chiều trục kéo Camera (Invert Pitch)** | **`I`** *(hoặc nút trên Sidebar)* | — |
| **Bật / Tắt Chống rung Camera (Gimbal Stabilization)** | **`Z`** *(hoặc nút trên Sidebar)* | — |
| **Bật / Tắt AI `YOLO26m` phát hiện người** | **`M`** *(hoặc nút trên Sidebar)* | — |
| **Đánh dấu & Lưu tọa độ Nạn nhân ($\le 40\text{ m}$, LiDAR xác nhận)** | **`Y`** | Nút **`Y`** |
| **Xóa toàn bộ các mốc đánh dấu nạn nhân** | **`F8`** | — |
| **Khóa / Mở khóa mục tiêu cứu hộ (Target Lock)** | **`T`** | Nút **`Back`** |
| **Chuyển góc nhìn Camera RGB (FPV bụng ↔ Chase Cam)** | **`V`** | Nút **`LB`** |
| **Đổi bảng màu Camera Nhiệt (`White-Hot`, `Ironbow`...)** | **`R`** | Nút **`RB`** |
| **Bật / Tắt Camera Nhiệt FLIR** | **`U`** | **`D-Pad Trái (◀)`** |
| **Bật / Tắt quét LiDAR** | **`J`** | **`D-Pad Phải (▶)`** |
| **Đổi tầm xa Radar LiDAR 2D / Đổi bảng màu Radar** | **`X`** (Tầm xa) / **`C`** (Bảng màu) | Nút **`X`** (Tầm xa) |
| **Đổi hướng xoay Radar 2D (`Heading-Up` ↔ `North-Up`)** | **`N`** | — |
| **Hiện / Ẩn khung nhận diện Bounding Box trên HUD** | **`O`** | — |
| **Hiện / Ẩn đường nối từ điểm xuất phát tới các nạn nhân** | — | **`D-Pad Lên (▲)`** |
| **Bật / Tắt Ghi dữ liệu Phiên bay (`outputs/session_...`)** | **`K`** hoặc **`F9`** | **`D-Pad Xuống (▼)`** / **`L3`** |
| **Mở / Chuyển tới Cửa sổ Bản đồ 3D (`Open3D`)** | **`F2`** *(hoặc nút `MAP 3D` trên Sidebar)* | — |
| **Tự động căn chỉnh chia đôi cửa sổ / Toàn màn hình** | **`F10`** (Căn cửa sổ) / **`F11`** (Full màn hình) | — |
| **Bật / Tắt bảng hướng dẫn phím tắt trên màn hình** | **`F1`** | Nút **`Start`** |

### 7.2. Điều khiển trên Cửa sổ Bản đồ 3D (`Open3D Viewer` — [`app/key_bindings.py`](app/key_bindings.py))

| Thao tác / Phím tắt | Chức năng trên Bản đồ Đám mây Điểm 3D |
| :--- | :--- |
| **`Chuột Trái + Kéo`** | Xoay góc nhìn 3D tự do quanh khu vực (Orbit) |
| **`Chuột Phải / Giữa + Kéo`** | Trượt di chuyển tâm bản đồ (Pan) |
| **`Con lăn Chuột`** | Phóng to / Thu nhỏ chi tiết bản đồ (Zoom In/Out) |
| **`P`** | Lưu bản đồ đám mây điểm 3D (`.ply`) kèm tọa độ các mục tiêu cứu hộ |
| **`F`** | Bật / Tắt chế độ Camera 3D tự động bay bám theo UAV (`Follow Drone`) |
| **`F3`** | Hiện / Ẩn các khung hộp 3D đánh dấu vị trí nạn nhân |
| **`F4`** | Đặt lại góc nhìn camera 3D về ngay vị trí hiện tại của UAV (`Reset View`) |
| **`F5`** | Đổi bảng màu độ cao địa hình (`Turbo` $\to$ `Jet` $\to$ `Viridis`) |
| **`F6` *(nhấn 2 lần trong 1.5s)*** | Xác nhận xóa sạch bản đồ 3D hiện tại để quét lại từ đầu |
| **`F7` / `F12`** | Tăng (`F7`) / Giảm (`F12`) kích thước hạt điểm (`Point Size`) |
| **`F8`** | Xóa toàn bộ các mốc đánh dấu nạn nhân (đồng bộ trực tiếp với GCS) |
| **`Esc`** | Đóng cửa sổ Bản đồ 3D |
