"""Open a saved PLY and its matching rescue targets without AirSim or GCS."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from configs.config import DATA, local_path

import argparse
import json
import numpy as np
import open3d as o3d
from app.viewer_3d import generate_target_box_wires, UAVLiDAR3DReconstruction


def load_map(filename):
    path = local_path(filename)
    if path.suffix.lower() != ".ply" or not path.is_file():
        raise ValueError("Hãy chọn file bản đồ .ply có trong thư mục dự án.")
    cloud = o3d.io.read_point_cloud(str(path))
    if len(cloud.points) == 0:
        raise ValueError("Bản đồ trống hoặc không đọc được file PLY.")
    targets_path = local_path(path.with_name(path.stem + "_targets.json"))
    targets = []
    if targets_path.is_file():
        targets = json.loads(targets_path.read_text(encoding="utf-8"))
        if not isinstance(targets, list) or any(not isinstance(t, dict) for t in targets):
            raise ValueError("File vị trí nạn nhân phải chứa danh sách các mục tiêu.")
    else:
        print("CẢNH BÁO: Bản đồ cũ không có file _targets.json; chỉ hiển thị địa hình.", flush=True)
    return path, cloud, targets


def target_position(target):
    if all(k in target for k in ("x_enu", "y_enu", "z_enu")):
        position = [target[k] for k in ("x_enu", "y_enu", "z_enu")]
    elif all(k in target for k in ("x_ned", "y_ned", "z_ned")):
        position = [target["y_ned"], target["x_ned"], -float(target["z_ned"])]
    elif all(k in target for k in ("x", "y", "z")):
        position = [target[k] for k in ("x", "y", "z")]
    else:
        raise ValueError(f"Mục tiêu #{target.get('id', '?')} thiếu tọa độ 3D.")
    position = np.asarray(position, dtype=np.float64)
    if not np.all(np.isfinite(position)):
        raise ValueError("Tọa độ nạn nhân không hợp lệ.")
    return position


def build_markers(cloud, targets):
    points, edges, colors = [], [], []
    scan = np.asarray(cloud.points)
    for target in targets:
        position = target_position(target)
        # Match the ground snapping used in the live viewer. Stored coordinates
        # remain unchanged; only the visual box is grounded on the saved scan.
        center = UAVLiDAR3DReconstruction.snap_target_to_pointcloud(None, position, scan)
        corners, lines = generate_target_box_wires(center, extent_enu=(1.2, 1.2, 1.8), beacon_h=3.8)
        offset = len(points)
        points.extend(corners)
        edges.extend([[a + offset, b + offset] for a, b in lines])
        colors.extend([[1.0, 0.1, 0.2]] * len(lines))
    markers = o3d.geometry.LineSet()
    markers.points = o3d.utility.Vector3dVector(np.asarray(points).reshape(-1, 3))
    markers.lines = o3d.utility.Vector2iVector(np.asarray(edges, dtype=np.int32).reshape(-1, 2))
    markers.colors = o3d.utility.Vector3dVector(np.asarray(colors).reshape(-1, 3))
    return markers


def load_map_metadata(path):
    metadata_path = path.with_name(path.stem + "_metadata.json")
    if not metadata_path.is_file():
        return None, False
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if not isinstance(metadata, dict):
        raise ValueError("File _metadata.json phải chứa thông tin phiên bay.")
    position = metadata.get("start_pos_enu")
    if position is None and metadata.get("start_pos_ned") is not None:
        ned = validate_start_position(metadata["start_pos_ned"])
        position = [ned[1], ned[0], -ned[2]]
    return (validate_start_position(position) if position is not None else None,
            bool(metadata.get("route_lines_enabled", False)))


def validate_start_position(position):
    position = np.asarray(position, dtype=np.float64)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("Vị trí xuất phát phải gồm 3 tọa độ hữu hạn E, N, U (m).")
    return position


def build_departure_geometry(cloud, targets, start_position):
    """Keep the departure marker and direct connections independently visible."""
    start = validate_start_position(start_position)
    top = start + [0, 0, 1.5]
    marker = o3d.geometry.LineSet()
    marker.points = o3d.utility.Vector3dVector([
        start, top, top + [-0.65, 0, 0], top + [0.65, 0, 0],
        top + [0, -0.65, 0], top + [0, 0.65, 0],
    ])
    marker.lines = o3d.utility.Vector2iVector([[0, 1], [2, 3], [4, 5]])
    marker.paint_uniform_color([0.15, 1.0, 0.35])
    scan = np.asarray(cloud.points)
    centers = [UAVLiDAR3DReconstruction.snap_target_to_pointcloud(
        None, target_position(target), scan) for target in targets]
    routes = o3d.geometry.LineSet()
    routes.points = o3d.utility.Vector3dVector([top, *centers])
    routes.lines = o3d.utility.Vector2iVector(
        np.asarray([[0, i + 1] for i in range(len(centers))], dtype=np.int32).reshape(-1, 2))
    routes.paint_uniform_color([1.0, 0.9, 0.05])
    return marker, routes


def choose_map():
    if sys.platform == "win32":
        return choose_map_windows()
    import tkinter as tk
    from tkinter import filedialog
    window = tk.Tk()
    window.withdraw()
    try:
        return filedialog.askopenfilename(parent=window, title="Chọn bản đồ 3D đã lưu",
                                          initialdir=str(DATA), filetypes=[("Bản đồ 3D", "*.ply")])
    finally:
        window.destroy()


def choose_map_windows():
    """Use the system dialog; portable Python may ship without Tcl/Tk data."""
    import ctypes
    from ctypes import wintypes

    class OPENFILENAMEW(ctypes.Structure):
        _fields_ = [
            ("lStructSize", wintypes.DWORD),
            ("hwndOwner", wintypes.HWND),
            ("hInstance", wintypes.HINSTANCE),
            ("lpstrFilter", wintypes.LPCWSTR),
            ("lpstrCustomFilter", wintypes.LPWSTR),
            ("nMaxCustFilter", wintypes.DWORD),
            ("nFilterIndex", wintypes.DWORD),
            ("lpstrFile", wintypes.LPWSTR),
            ("nMaxFile", wintypes.DWORD),
            ("lpstrFileTitle", wintypes.LPWSTR),
            ("nMaxFileTitle", wintypes.DWORD),
            ("lpstrInitialDir", wintypes.LPCWSTR),
            ("lpstrTitle", wintypes.LPCWSTR),
            ("Flags", wintypes.DWORD),
            ("nFileOffset", wintypes.WORD),
            ("nFileExtension", wintypes.WORD),
            ("lpstrDefExt", wintypes.LPCWSTR),
            ("lCustData", wintypes.LPARAM),
            ("lpfnHook", ctypes.c_void_p),
            ("lpTemplateName", wintypes.LPCWSTR),
            ("pvReserved", ctypes.c_void_p),
            ("dwReserved", wintypes.DWORD),
            ("FlagsEx", wintypes.DWORD),
        ]

    dialog = ctypes.WinDLL("comdlg32", use_last_error=True)
    dialog.GetOpenFileNameW.argtypes = [ctypes.POINTER(OPENFILENAMEW)]
    dialog.GetOpenFileNameW.restype = wintypes.BOOL
    dialog.CommDlgExtendedError.argtypes = []
    dialog.CommDlgExtendedError.restype = wintypes.DWORD
    filename = ctypes.create_unicode_buffer(32768)
    options = OPENFILENAMEW()
    options.lStructSize = ctypes.sizeof(options)
    options.lpstrFilter = "Bản đồ 3D (*.ply)\0*.ply\0\0"
    options.nFilterIndex = 1
    options.lpstrFile = ctypes.cast(filename, wintypes.LPWSTR)
    options.nMaxFile = len(filename)
    options.lpstrInitialDir = str(DATA)
    options.lpstrTitle = "Chọn bản đồ 3D đã lưu"
    # Explorer, existing file/path, preserve cwd, omit recent-document history.
    options.Flags = 0x00080000 | 0x00001000 | 0x00000800 | 0x00000008 | 0x02000000
    options.lpstrDefExt = "ply"
    if dialog.GetOpenFileNameW(ctypes.byref(options)):
        return filename.value
    error = dialog.CommDlgExtendedError()
    if error:
        raise OSError(f"Không mở được hộp chọn bản đồ (mã Windows 0x{error:04X}).")
    return ""  # Cancel is a normal exit, not a dialog failure.


def report_error(error):
    print(f"LỖI: {error}", file=sys.stderr, flush=True)
    if "--check" in sys.argv:
        return
    try:
        if sys.platform == "win32":
            import ctypes
            from ctypes import wintypes
            message = ctypes.WinDLL("user32", use_last_error=True).MessageBoxW
            message.argtypes = [wintypes.HWND, wintypes.LPCWSTR, wintypes.LPCWSTR, wintypes.UINT]
            message.restype = ctypes.c_int
            message(None, str(error), "Không mở được bản đồ", 0x10)
        else:
            import tkinter as tk
            from tkinter import messagebox
            window = tk.Tk()
            window.withdraw()
            try:
                messagebox.showerror("Không mở được bản đồ", str(error), parent=window)
            finally:
                window.destroy()
    except Exception as dialog_error:
        print(f"Không hiển thị được hộp báo lỗi: {dialog_error}", file=sys.stderr, flush=True)


def show_map(path, cloud, targets, markers, start_position=None, routes_enabled=False):
    vis = o3d.visualization.VisualizerWithKeyCallback()
    if not vis.create_window(window_name=f"MAP DA LUU | {path.name} | {len(targets)} moc cuu ho",
                             width=1280, height=720):
        raise RuntimeError("Không mở được cửa sổ Open3D.")
    try:
        vis.add_geometry(cloud)
        if targets:
            vis.add_geometry(markers)
        departure, routes = (build_departure_geometry(cloud, targets, start_position)
                             if start_position is not None else (None, None))
        if departure is not None:
            vis.add_geometry(departure)
        routes_visible = [bool(routes_enabled and targets and routes is not None)]
        if routes_visible[0]:
            vis.add_geometry(routes)
        option = vis.get_render_option()
        option.background_color = np.array([0.05, 0.05, 0.07])
        option.point_size = 1.0
        option.line_width = 2.0
        visible = [True]

        def toggle_markers(view):
            if targets:
                if visible[0]:
                    view.remove_geometry(markers, reset_bounding_box=False)
                else:
                    view.add_geometry(markers, reset_bounding_box=False)
                visible[0] = not visible[0]
            return False

        def reset(view):
            view.reset_view_point(True)
            return False

        def toggle_routes(view):
            if routes is None:
                print("Bản đồ chưa lưu vị trí xuất phát; cần bổ sung --start-enu E N U.", flush=True)
                return False
            if not targets:
                print("Bản đồ chưa có vị trí nạn nhân để nối đường.", flush=True)
                return False
            if routes_visible[0]:
                view.remove_geometry(routes, reset_bounding_box=False)
            else:
                view.add_geometry(routes, reset_bounding_box=False)
            routes_visible[0] = not routes_visible[0]
            print(f"Đường nối xuất phát → nạn nhân: {'HIỆN' if routes_visible[0] else 'ẨN'}", flush=True)
            return False

        def resize(delta):
            def callback(view):
                option.point_size = max(1.0, min(10.0, option.point_size + delta))
                return False
            return callback

        vis.register_key_callback(ord("M"), toggle_markers)
        vis.register_key_callback(ord("N"), toggle_routes)
        vis.register_key_callback(ord("R"), reset)
        vis.register_key_callback(ord("="), resize(0.5))
        vis.register_key_callback(ord("-"), resize(-0.5))
        vis.run()
    finally:
        vis.destroy_window()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map", nargs="?", help="Project-relative PLY path; omit to select a file")
    parser.add_argument("--check", action="store_true", help="Validate map and markers without opening a window")
    parser.add_argument("--start-enu", nargs=3, type=float, metavar=("E", "N", "U"),
                        help="Departure coordinates in meters for a map without saved departure data")
    args = parser.parse_args()
    if args.check and not args.map:
        parser.error("--check requires a map path")
    filename = args.map or choose_map()
    if not filename:
        return 0
    path, cloud, targets = load_map(filename)
    markers = build_markers(cloud, targets)
    start_position, routes_enabled = load_map_metadata(path)
    if args.start_enu is not None:
        start_position = validate_start_position(args.start_enu)
    print(f"Bản đồ: {path.relative_to(ROOT)} | {len(cloud.points):,} điểm | {len(targets)} mốc cứu hộ", flush=True)
    print("Đỏ: mọi mốc nạn nhân. Xanh lá: xuất phát. Vàng: đường nối. M: ẩn/hiện mốc; N: ẩn/hiện đường nối; R: toàn cảnh; +/-: cỡ điểm; Esc: đóng.", flush=True)
    if start_position is None:
        print("Bản đồ cũ chưa lưu vị trí xuất phát. Dùng --start-enu E N U để nhập tọa độ xuất phát đã biết (m).", flush=True)
    else:
        build_departure_geometry(cloud, targets, start_position)
        east, north, up = start_position
        print(f"Xuất phát: E={east:.2f}, N={north:.2f}, U={up:.2f} m", flush=True)
    for target in targets:
        east, north, up = target_position(target)
        print(f"  #{target.get('id', '?')}: E={east:.2f}, N={north:.2f}, U={up:.2f} m | {target.get('verdict', 'UNKNOWN')}", flush=True)
    if not args.check:
        show_map(path, cloud, targets, markers, start_position, routes_enabled)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        report_error(error)
        sys.exit(1)

