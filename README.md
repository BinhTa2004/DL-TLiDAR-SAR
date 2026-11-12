# UAV Search-and-Rescue: Thermal–LiDAR 3D Localization & Mapping System

Hệ thống mô phỏng tìm kiếm và cứu nạn (Search and Rescue — SAR) bằng thiết bị bay không người lái (UAV) trong môi trường rừng rậm và địa hình phức tạp, kết hợp **Camera Nhiệt Hồng ngoại (Thermal IR)**, **Camera Quang học (RGB)** và **Cảm biến Quét Laser 3D (LiDAR)** trên nền tảng **Unreal Engine 4 & Microsoft AirSim (`MedievalVillageMegascansS`)**.

---

## 1. Bối cảnh Mô phỏng & Bài toán Thực tế

Trong thực tế tìm kiếm người mất tích ở vùng rừng núi hoặc khu vực thiên tai:
- **Camera RGB thông thường** rất khó phát hiện nạn nhân khi bị che khuất bởi bóng râm, tán cây rậm rạp hoặc điều kiện thiếu sáng.
- **Camera Nhiệt 2D** giúp phát hiện nhanh nguồn nhiệt ấm áp của cơ thể sống nhưng chỉ cho biết vị trí trên mặt phẳng ảnh 2D (không đo được khoảng cách/tọa độ 3D chính xác) và dễ bị báo động giả bởi **động vật hoang dã** hoặc **đá/mái nhà hấp thụ nhiệt mặt trời**.
- **Cảm biến LiDAR 3D** có khả năng phóng các tia laser xuyên qua các khe hở của tán lá để đo tọa độ 3D của mặt đất và vật thể bên dưới, nhưng đám mây điểm 3D thuần túy lại không biết đâu là cơ thể người có nhiệt độ sống.

### Môi trường Mô phỏng 3D (`MedievalVillageMegascansS`)
Dự án xây dựng một kịch bản cứu nạn hoàn chỉnh trong môi trường 3D độ chân thực cao gồm làng cổ, rừng thông rậm rạp, vách đá, sườn đồi và sông suối:
- **12 Nạn nhân mất tích (Ground Truth Targets):** Được bố trí ở nhiều tư thế khác nhau (nằm bất tỉnh, ngồi tựa gốc cây, đứng dưới tán rừng, nằm cạnh bờ suối hoặc khuất sau công trình).
- **Tác nhân gây nhiễu tự nhiên:** Trong bản đồ có các đàn **động vật** (trâu/bò có thân nhiệt ấm) cùng địa hình **đá tảng, thân cây gỗ, mái nhà** để kiểm chứng khả năng phân biệt thật/giả của thuật toán.
- **Tổ hợp Cảm biến gắn trên UAV (`UAV1`):**
  1. **Camera Nhiệt FLIR Boson (`640×512`, FOV `90°`):** Mô phỏng bức xạ nhiệt hồng ngoại với cơ chế truyền nhiệt lõi cơ thể (Torso Core Heat), tán xạ quang học và 4 bảng màu nhiệt (`White-Hot`, `Ironbow`, `Rainbow`, `Black-Hot`).
  2. **Camera Quang học RGB (`640×512`):** Đồng trục với camera nhiệt để quan sát trực quan hiện trường.
  3. **Cảm biến LiDAR 3D Ouster (`Lidar1`):** Quét đám mây điểm 3D đa kênh xung quanh UAV, cho phép tái tạo địa hình 3D và đo đạc kích thước vật thể dưới tán cây.

---

## 2. Khi Chạy Chương trình Hệ thống Hoạt động Như thế nào?

Khi bạn khởi chạy hệ thống (bằng `run.bat` hoặc `python main.py`), một quy trình tác chiến cứu hộ khép kín gồm **4 bước** sẽ diễn ra như sau:

### Bước 1: Tự động Khởi tạo Thế giới Mô phỏng & Hiệu chuẩn Nhiệt độ
- Hệ thống tự động bật môi trường Unreal Engine (`MedievalVillageMegascansS.exe`) và kết nối tới máy chủ vật lý AirSim qua cổng `41451`.
- Ngay khi bản đồ tải xong, script [`simulation/scene_setup.py`](simulation/scene_setup.py) tự động quét toàn bộ đối tượng trong cảnh để hiệu chuẩn đặc tính bức xạ nhiệt: nền rừng/sông suối/đất đá mang nhiệt độ môi trường mát, động vật mang nhiệt độ ấm vừa phải, và 12 nạn nhân mang bức xạ nhiệt cơ thể người (`36.5°C – 38.5°C`).

### Bước 2: Mở Đồng thời 2 Cửa sổ Điều hành Thời gian thực
Hệ thống tự động mở và sắp xếp **2 cửa sổ hoạt động song song**:

1. **Cửa sổ 1 — Trạm Điều khiển Mặt đất 2D (`GCS Station` - 60 FPS):**
   - **Khung Camera Nhiệt FLIR (Góc trên bên trái):** Hiển thị luồng ảnh nhiệt kèm các khung nhận diện AI **YOLO26m (`960×960`)** tự động bám theo mục tiêu. Trên mỗi khung hiển thị rõ: Phân loại (`NGƯỜI`, `ĐỘNG VẬT`, hoặc `TÍN HIỆU NHIỆT`), khoảng cách 3D tính bằng mét, và số điểm LiDAR xác nhận.
   - **Khung Camera RGB (Góc trên bên phải):** Hiển thị hình ảnh màu thực tế tại cùng góc nhìn camera nhiệt kèm thước ngắm tâm (Crosshair) và thông số góc Gimbal.
   - **Khung Radar LiDAR 2D (Góc dưới bên trái):** Hiển thị mặt cắt đám mây điểm xung quanh UAV theo hướng mũi bay (`Heading-Up` hoặc `North-Up`), giúp phi công quan sát địa hình và vật cản.
   - **Khung Phóng đại Mục tiêu ROI & Bảng Điều khiển Cứu hộ (Góc dưới bên phải & Cột phải):** Phóng to mục tiêu đang theo dõi, hiển thị trạng thái kết nối, tốc độ bay, độ cao AGL, và **Danh sách các nạn nhân đã đánh dấu** kèm tọa độ 3D `(X, Y, Z)`.

2. **Cửa sổ 2 — Bản đồ Đám mây Điểm 3D Thời gian thực (`Open3D Viewer`):**
   - Nhận trực tiếp luồng điểm LiDAR từ trạm GCS thông qua bộ nhớ dùng chung (`Shared Memory mmap`) với độ trễ bằng 0.
   - **UAV bay đến đâu, bản đồ 3D được vẽ thêm đến đó:** Toàn bộ nhà cửa, rừng cây, sườn đồi được tái tạo thành bản đồ 3D sống động tô màu theo độ cao địa hình, kèm mô hình khung UAV di chuyển theo thời gian thực.

### Bước 3: Bay Tuần tra, Hợp nhất Cảm biến & Đánh dấu Cứu hộ
Trong lúc bạn điều khiển UAV bay tuần tra trên khu vực rừng và ngôi làng:
1. **Phát hiện từ xa ($> 40\text{ m}$):** Khi phát hiện đốm nhiệt ở xa, hệ thống hiển thị khung cảnh báo `TÍN HIỆU NHIỆT` để phi công điều khiển UAV bay lại gần kiểm tra.
2. **Phân loại & Xác minh Hình học 3D ($\le 40\text{ m}$):** Khi UAV tiếp cận trong bán kính `40 m`:
   - Mô hình **YOLO26m** phân loại hình dáng nhiệt trên ảnh 2D.
   - Đồng thời, lõi thuật toán [`pipeline/`](pipeline/) chiếu các điểm LiDAR 3D vào trong khung nhận diện 2D, gom cụm không gian–nhiệt **4D DBSCAN** và tính toán đặc trưng hình học **PCA** (kích thước dài/rộng/cao, thể tích, độ phẳng):
     - Nếu cụm điểm là mặt phẳng ngang (đá nóng, mặt đất) hoặc cột cao hẹp (thân cây) $\to$ **Tự động loại bỏ (`REJECTED`)**.
     - Nếu là động vật $\to$ Hiển thị nhãn **`ĐỘNG VẬT`** (màu vàng), khóa chức năng đánh dấu để tránh báo động giả.
     - Nếu kích thước và nhiệt độ khớp với cơ thể người $\to$ Hiển thị nhãn **`NGƯỜI | LiDAR XÁC NHẬN`** (màu xanh lá) và tính chính xác tọa độ tâm 3D của nạn nhân trên mặt đất.
3. **Đánh dấu Nạn nhân (Phím `M` hoặc `Enter`):**
   - Khi mục tiêu đã được xác nhận là **`NGƯỜI`**, bạn bấm phím **`M`** (hoặc `Enter` / nút `A` trên tay cầm Xbox).
   - **Trên Bản đồ 3D:** Lập tức xuất hiện một **Khung hộp 3D màu đỏ cùng Cột tiêu Cứu hộ (Rescue Beacon)** dựng thẳng đứng ngay tại vị trí nạn nhân trên mặt đất.
   - **Trong thư mục `outputs/`:** Hệ thống tự động chụp lưu 2 bức ảnh bằng chứng (1 ảnh RGB + 1 ảnh Nhiệt có đóng dấu tọa độ), ghi tọa độ 3D vào file `outputs/rescue_targets_log.csv`, đồng thời đối chiếu ngay với `ground_truth.json` để tính sai số định vị 2D/3D (độ chính xác trung bình thực nghiệm đạt **`0.703 m`** trên toàn bộ `12/12` nạn nhân).

### Bước 4: Lưu Bản đồ 3D & Xem lại Chiến dịch Cứu hộ
- Bạn có thể bấm phím `K` (hoặc `F9`) trong lúc bay để ghi lại toàn bộ luồng dữ liệu tìm kiếm (chuỗi ảnh nhiệt `.jpg` + đám mây điểm `.npy` + nhật ký bay `.csv`) vào `outputs/session_YYYYMMDD_HHMMSS/`.
- Sau chuyến bay, bản đồ 3D toàn khu vực và toàn bộ các cột mốc vị trí nạn nhân được xuất ra file `.ply`. Đội cứu hộ có thể mở xem lại bản đồ này bất cứ lúc nào bằng lệnh `python main.py map` (hỗ trợ bật/tắt đường kẻ vàng nối từ điểm xuất phát của UAV tới từng nạn nhân).

---

## 3. Cấu trúc Thư mục Dự án

```text
UAV Search-and-Rescue/
│
├── main.py                           # Điểm chạy chính điều phối toàn hệ thống (all, sim, gcs, lidar, map, check)
├── run.bat                           # File khởi chạy nhanh 1-Click trên Windows
├── requirements.txt                  # Danh sách các thư viện Python cần thiết
├── README.md                         # Tài liệu mô tả dự án, hướng dẫn cài đặt và vận hành
│
├── configs/                          # 1. CẤU HÌNH HỆ THỐNG
│   ├── config.py                     # Tập trung toàn bộ tham số: YOLO26m, Voxel 0.10m, DBSCAN, ngưỡng lọc hình học & nhiệt độ
│   └── setup_environment.py          # Script hỗ trợ cài đặt thư viện vào môi trường Python nội bộ
│
├── pipeline/                         # 2. LÕI THUẬT TOÁN XỬ LÝ & HỢP NHẤT CẢM BIẾN (Tách biệt khỏi giao diện)
│   ├── acquisition.py                # Giải mã khung ảnh RGB/Thermal và gom cửa sổ quét LiDAR 0.9s
│   ├── thermal_detector.py           # Tổng hợp bức xạ nhiệt FLIR và nhận diện người bằng YOLO26m (960x960)
│   ├── lidar_processing.py           # Lọc cự ly, lọc Voxel 0.10m, gom cụm 4D DBSCAN và trích xuất đặc trưng hình học PCA
│   ├── frustum_alignment.py          # Biến đổi tọa độ camera/UAV, chiếu 3D->2D và gán nhiệt độ tạo điểm 4D (X,Y,Z,T)
│   ├── occlusion_fusion.py           # Dò tia (Ray-casting) xuyên tán lá, lọc vật cản giả và chấm điểm tin cậy đa cảm biến
│   └── localization_3d.py            # Chọn cụm mục tiêu tối ưu và tính tọa độ tâm 3D của nạn nhân
│
├── app/                              # 3. ĐIỀU KHIỂN BAY, GIAO DIỆN 2D HUD & BẢN ĐỒ 3D
│   ├── uav_controller.py             # Bộ điều khiển trung tâm Trạm Mặt đất (GCS) và vòng lặp chính
│   ├── flight_control.py             # Điều khiển bay 33Hz, giữ độ cao (Altitude Hold) và điều khiển góc Gimbal
│   ├── input_devices.py              # Đọc tín hiệu điều khiển từ Bàn phím và Tay cầm Xbox (XInput)
│   ├── sensor_workers.py             # Các luồng chạy ngầm đọc dữ liệu Camera/LiDAR từ AirSim và gọi vào pipeline/
│   ├── hud_display.py                # Entry module cho giao diện điều khiển 2D HUD
│   ├── gcs_display.py                # Vẽ chi tiết giao diện HUD 2D, camera nhiệt/RGB, radar và bảng thông tin cứu hộ
│   ├── layout.py                     # Cấu hình kích thước và bố cục khung hình giao diện GCS
│   ├── viewer_3d.py                  # Entry module cho cửa sổ Bản đồ Đám mây Điểm 3D thời gian thực (Open3D)
│   ├── lidar_mapping.py              # Tích lũy đám mây điểm 3D toàn cục theo thời gian thực và lưu file .ply
│   ├── lidar_display.py              # Vòng lặp hiển thị Open3D 3D, vẽ mô hình UAV và khung đánh dấu nạn nhân
│   ├── shared_lidar_stream.py        # Bộ nhớ dùng chung (mmap) truyền trực tiếp đám mây điểm từ GCS sang cửa sổ 3D
│   ├── mission_records.py            # Chụp ảnh bằng chứng (RGB + Nhiệt), ghi log CSV và lưu dữ liệu chuyến bay
│   ├── detection_policy.py           # Quy tắc phân loại trong bán kính 40m và điều kiện xác nhận mục tiêu cứu hộ
│   ├── runtime_support.py            # Quản lý kết nối AirSim, khởi tạo phiên bay và ghi log hệ thống
│   └── view_saved_map.py             # Công cụ mở xem lại file bản đồ 3D (.ply) và mốc cứu hộ sau chuyến bay
│
├── evaluation/                       # 4. ĐÁNH GIÁ ĐỘ CHÍNH XÁC ĐỊNH VỊ
│   └── metrics.py                    # Đối chiếu tọa độ định vị với Ground Truth, tính sai số 2D/3D (Mean, Median, P90)
│
├── simulation/                       # 5. MÔI TRƯỜNG MÔ PHỎNG UNREAL ENGINE & AIRSIM
│   ├── WindowsNoEditor/              # [TẢI TỪ GOOGLE DRIVE] Bộ cài bản đồ 3D MedievalVillageMegascansS
│   ├── airsim/                       # Thư viện giao tiếp AirSim tích hợp sẵn
│   ├── airsim_settings.json          # Cấu hình thông số Camera và LiDAR cho AirSim
│   ├── simulator_lifecycle.py        # Quản lý khởi động/đóng an toàn tiến trình Unreal Engine
│   ├── scene_setup.py                # Thiết lập bức xạ nhiệt cho các đối tượng trong map và xuất tọa độ chuẩn
│   └── ground_truth.json             # Tọa độ thực 3D của 12 mục tiêu nạn nhân trong bản đồ
│
├── outputs/                          # 6. KẾT QUẢ ĐẦU RA & DỮ LIỆU PHIÊN BAY
│   ├── reconstructed_map_3d.ply      # File bản đồ đám mây điểm 3D đã tái tạo
│   ├── localization_results.csv      # Danh sách tọa độ các mục tiêu đã đánh dấu và sai số chi tiết
│   ├── localization_metrics.csv      # Bảng tổng hợp kết quả đánh giá độ chính xác định vị
│   └── rescue_mission_logs/          # Thư mục chứa ảnh chụp hiện trường (RGB + Nhiệt) khi đánh dấu nạn nhân
│
└── Yolo/                             # 7. TRỌNG SỐ MÔ HÌNH AI
    └── Yolo26m_trained(1).pt         # Trọng số mô hình YOLO26m nhận diện người trên ảnh nhiệt UAV
```

---

## 4. Hướng dẫn Cài đặt & Thiết lập Bản đồ (Map Setup)

### Bước 1: Cài đặt Thư viện Python
- Yêu cầu **Windows 10/11 (64-bit)** và **Python 3.10 – 3.12** (khuyến nghị máy có card đồ họa rời NVIDIA hỗ trợ CUDA).
- Mở Terminal tại thư mục gốc của dự án và chạy:

```powershell
pip install -r requirements.txt
```

*(Nếu thư mục dự án đã đi kèm gói môi trường `python_env/`, bạn có thể bỏ qua bước này).*

---

### Bước 2: Tải và Giải nén Bản đồ Mô phỏng từ Google Drive

Do bộ cài môi trường 3D Unreal Engine (`WindowsNoEditor`) có dung lượng lớn nên được lưu trữ riêng trên Google Drive:

> 🔗 **Link tải Bản đồ Mô phỏng (`WindowsNoEditor`):**  
> **[https://drive.google.com/file/d/1LOmgK6vbJ0kVfmGQegFqbfDdbIUB-yZu/view]**

Sau khi tải về, thực hiện **1 trong 2 cách** sau để gắn bản đồ vào dự án:

- **Cách 1 — Giải nén trực tiếp vào `simulation/` (Khuyên dùng):**  
  Giải nén thư mục `WindowsNoEditor` vào trong thư mục `simulation/` sao cho đường dẫn tới file `.exe` đúng như sau:
  ```text
  UAV Search-and-Rescue/simulation/WindowsNoEditor/MedievalVillageMegascansS.exe
  ```

- **Cách 2 — Nối đường dẫn nhanh (Junction Link) nếu Map đang để ở ổ đĩa khác:**  
  Nếu bạn đã giải nén `WindowsNoEditor` ở một thư mục khác trên máy (ví dụ `D:\Map\WindowsNoEditor`) và không muốn copy lại tốn dung lượng ổ cứng, hãy mở **Command Prompt (CMD)** tại thư mục dự án và chạy:
  ```cmd
  mklink /J "simulation\WindowsNoEditor" "D:\Map\WindowsNoEditor"
  ```

Kiểm tra nhanh xem dự án đã nhận đủ Map, Model và thư viện chưa bằng lệnh:

```powershell
python main.py check
```

---

## 5. Cách Chạy Chương trình

### Cách 1: Chạy nhanh bằng `run.bat` (1-Click)
Bấm đúp chuột trực tiếp vào file **`run.bat`** ở thư mục gốc. Hệ thống sẽ tự động mở Simulator, hiệu chuẩn 12 mục tiêu nhiệt, và bật đồng thời **Trạm Điều khiển Mặt đất 2D (GCS HUD)** cùng **Cửa sổ Bản đồ 3D Thời gian thực (Open3D)**.

### Cách 2: Chạy bằng lệnh Python theo từng chế độ

```powershell
# 1. Chạy toàn bộ hệ thống (Mặc định: ẩn cửa sổ render của Unreal để dồn tài nguyên GPU cho GCS & Bản đồ 3D)
python main.py

# 2. Chạy toàn bộ hệ thống và hiển thị cả cửa sổ 3D của Unreal Engine bên cạnh GCS
python main.py all --show-unreal

# 3. Chỉ bật môi trường mô phỏng Unreal Engine / AirSim và hiệu chuẩn mục tiêu nhiệt
python main.py sim

# 4. Chỉ mở Trạm điều khiển 2D (dùng khi Simulator đã chạy sẵn)
python main.py gcs

# 5. Chỉ mở Cửa sổ Bản đồ Đám mây Điểm 3D thời gian thực
python main.py lidar

# 6. Mở hộp thoại chọn file (.ply) để xem lại bản đồ 3D và các mốc nạn nhân đã lưu
python main.py map
```

Nếu muốn mở xem ngay bản đồ 3D mẫu đã dựng sẵn trong `outputs/` mà không cần bật Simulator:

```powershell
python app/view_saved_map.py outputs/reconstructed_map_3d.ply
```

---

## 6. Bảng Phím Điều khiển (Bàn phím & Tay cầm Xbox)

| Chức năng | Bàn phím (Keyboard) | Tay cầm Xbox (XInput) |
| :--- | :--- | :--- |
| **Khởi động động cơ (Arm) / Hạ cánh (Land)** | `T` (Arm) / `L` (Land) | Nút `Start` / Nút `Back` |
| **Bay Tiến / Lùi / Trái / Phải** | `W` / `S` / `A` / `D` | Cần gạt Trái (`Left Stick`) |
| **Tăng / Giảm độ cao** | `Space` (Lên) / `Shift` hoặc `Ctrl` (Xuống) | Cò `RT` (Lên) / Cò `LT` (Xuống) |
| **Xoay mũi UAV (Yaw Trái / Phải)** | `Q` / `E` | Cần gạt Phải ngang (`Right Stick X`) |
| **Chỉnh góc nghiêng Camera (Gimbal Pitch)** | `Mũi tên Lên / Xuống` (`↑` / `↓`) | Cần gạt Phải dọc (`Right Stick Y`) |
| **Góc nhanh Gimbal (`0°`, `-45°`, `-90°`)** | Phím `1` / `2` / `3` | — |
| **Tăng tốc độ bay (Boost)** | Giữ phím `F` | Nhấn cần Trái (`L3`) |
| **Đánh dấu & Lưu tọa độ Nạn nhân** | **`M`** hoặc **`Enter`** | Nút **`A`** |
| **Khóa mục tiêu tự động bám góc nhìn (Lock)** | `Tab` | Nút `X` |
| **Bật / Tắt quét LiDAR** | `O` | `D-Pad Lên (↑)` |
| **Mở / Chuyển sang Cửa sổ Bản đồ 3D** | `V` | Nút `RB` |
| **Bật / Tắt Ghi dữ liệu Phiên bay** | `K` hoặc `F9` | `D-Pad Xuống (↓)` |
| **Đổi bảng màu Nhiệt (White-Hot, Ironbow...)** | `P` | `D-Pad Phải (→)` |
| **Hiện hướng dẫn phím tắt trên màn hình** | `H` hoặc `F1` | — |

**Điều khiển trên Cửa sổ Bản đồ 3D (`Open3D Viewer`):**
- `Chuột trái + Kéo`: Xoay góc nhìn 3D | `Con lăn chuột`: Phóng to / Thu nhỏ | `Chuột phải + Kéo`: Di chuyển tâm bản đồ.
- `F`: Bật/Tắt chế độ Camera đi theo UAV | `R`: Đặt lại góc nhìn toàn cảnh | `C`: Lưu nhanh bản đồ `.ply`.
