# -*- coding: utf-8 -*-
"""Main entry point for the SAR Thermal–LiDAR UAV System.

Connects the 7-stage core pipeline (`pipeline/`), configuration (`configs/`),
runtime application (`app/`), simulation setup (`simulation/`), and accuracy
evaluation (`evaluation/`).
"""
import argparse
from contextlib import redirect_stdout, redirect_stderr
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from configs.config import DATA, SIMULATOR, DETECTOR, local_path
from app.runtime_support import connect_airsim, start_live_session
from simulation.simulator_lifecycle import prepare_fresh_simulator


def console_process_options():
    if os.environ.get("UAV_HIDE_CONSOLE") != "1":
        return {}
    return {
        "creationflags": getattr(subprocess, "CREATE_NO_WINDOW", 0),
        "stdin": subprocess.DEVNULL,
        "stdout": sys.stdout,
        "stderr": sys.stderr,
    }


def run_module_script(relative_path):
    return subprocess.Popen([sys.executable, "-s", str(ROOT / relative_path)], cwd=ROOT,
                            **console_process_options())


def prepare_settings(no_viewport=False):
    source = ROOT / "simulation/airsim_settings.json"
    settings = json.loads(source.read_text(encoding="utf-8"))
    if no_viewport:
        settings["ViewMode"] = "NoDisplay"
    recording = local_path("outputs/AirSimRecording")
    recording.mkdir(parents=True, exist_ok=True)
    settings.setdefault("Recording", {})["Folder"] = str(recording)
    runtime = local_path("outputs/runtime/settings.json")
    runtime.parent.mkdir(parents=True, exist_ok=True)
    runtime.write_text(json.dumps(settings, indent=2, ensure_ascii=False), encoding="utf-8")
    return runtime


def simulator_command(settings):
    user_dir = local_path("outputs/Unreal")
    user_dir.mkdir(parents=True, exist_ok=True)
    return [
        str(SIMULATOR / "MedievalVillageMegascansS.exe"),
        "/Game/Maps/MedievalVillage_P?game=/Script/AirSim.AirSimGameMode",
        f"-settings={settings}", f"-UserDir={user_dir}",
        f"-abslog={DATA / 'Unreal/MedievalVillage.log'}",
        "-windowed", "-WinX=20", "-WinY=20", "-ResX=1024", "-ResY=576",
        "-USEALLAVAILABLECORES", "-NoVSync", "-fps=60",
    ]


def wait_for_airsim(simulator=None, timeout=180):
    """Wait for a responsive game thread, rather than just an open TCP port."""
    started = time.monotonic()
    last_report = -5.0
    last_error = "AirSim chua san sang"
    print("[KHOI DONG] Dang tai ban do va cho AirSim (toi da 180 giay)...", flush=True)
    while time.monotonic() - started < timeout:
        if simulator is not None and simulator.poll() is not None:
            raise RuntimeError("Unreal da thoat khi dang tai map; xem outputs/Unreal/MedievalVillage.log")
        client = None
        try:
            remaining = timeout - (time.monotonic() - started)
            client = connect_airsim(timeout=max(0.1, min(2.0, remaining)))
            client.getMultirotorState(vehicle_name="UAV1")
            print(f"[AIRSIM] Da ket noi sau {time.monotonic() - started:.1f}s. Dang mo giao dien...", flush=True)
            return
        except Exception as exc:
            last_error = str(exc)
        finally:
            if client is not None:
                client.client.close()
        elapsed = time.monotonic() - started
        if elapsed - last_report >= 5.0:
            last_report = elapsed
            print(f"[KHOI DONG] Dang cho map tai xong: {elapsed:.0f}s...", flush=True)
        time.sleep(0.3)
    raise TimeoutError(f"AirSim chua phan hoi sau {timeout}s: {last_error}")


def stop_simulator(simulator):
    """Stop only the simulator tree owned by this launcher, including its UE child."""
    if simulator is None or simulator.poll() is not None:
        return
    if sys.platform == "win32":
        result = subprocess.run(["taskkill", "/PID", str(simulator.pid), "/T", "/F"],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False,
                                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        if result.returncode != 0 and simulator.poll() is None:
            raise RuntimeError("Khong dung duoc Unreal cua phien hien tai")
    else:
        simulator.terminate()
    simulator.wait(timeout=10)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", nargs="?", default="all",
                        choices=["all", "uav", "sim", "gcs", "lidar", "map", "check"])
    parser.add_argument("--verbose", action="store_true", help="Show diagnostic messages")
    viewport = parser.add_mutually_exclusive_group()
    viewport.add_argument("--no-viewport", dest="no_viewport", action="store_true",
                          help="Disable and hide the Unreal main view (default for all/uav)")
    viewport.add_argument("--show-unreal", dest="no_viewport", action="store_false",
                          help="Show the Unreal scene alongside GCS")
    parser.set_defaults(no_viewport=None)
    args = parser.parse_args()
    no_viewport = args.no_viewport
    if no_viewport is None:
        no_viewport = args.mode in ("all", "uav", "check")
    os.environ["UAV_NO_VIEWPORT"] = "1" if no_viewport else "0"
    if args.verbose:
        os.environ["UAV_VERBOSE"] = "1"
    os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
    if args.mode == "check":
        required = ["simulation/airsim_settings.json", "simulation/WindowsNoEditor/MedievalVillageMegascansS.exe",
                    "python_env/python.exe", DETECTOR.model_path]
        missing = [p for p in required if not local_path(p).is_file()]
        modules = ["airsim", "numpy", "cv2", "pygame", "open3d", "torch",
                   "ultralytics", "win32gui", "psutil"]
        missing += [f"module {m}" for m in modules if importlib.util.find_spec(m) is None]
        if importlib.util.find_spec("sklearn") is None:
            print("Warning: sklearn is missing; clustering uses the existing voxel fallback.")
        prepare_settings(no_viewport=no_viewport)
        print("Missing: " + ", ".join(missing) if missing else "OK: local files, dependencies and runtime settings.")
        return int(bool(missing))

    simulator = None
    simulator_ready = False
    viewer = None
    try:
        if args.mode in ("all", "uav", "sim"):
            prepare_fresh_simulator()
            command = simulator_command(prepare_settings(no_viewport=no_viewport))
            if not (SIMULATOR / "MedievalVillageMegascansS.exe").is_file():
                parser.error("Missing simulation/WindowsNoEditor/MedievalVillageMegascansS.exe")
            os.environ["UAV_SESSION_ID"] = str(start_live_session())
            simulator = subprocess.Popen(command, cwd=SIMULATOR)
            wait_for_airsim(simulator)
            simulator_ready = True
        if args.mode == "sim":
            return run_module_script("simulation/scene_setup.py").wait()
        if args.mode == "lidar":
            return run_module_script("app/viewer_3d.py").wait()
        if args.mode == "map":
            return run_module_script("app/view_saved_map.py").wait()
        if args.mode == "gcs":
            wait_for_airsim()
            os.environ["UAV_SESSION_ID"] = str(start_live_session())
        viewer = run_module_script("app/viewer_3d.py") if args.mode == "all" else None
        return run_module_script("app/uav_controller.py").wait()
    except (TimeoutError, ConnectionError, RuntimeError, OSError) as exc:
        print(f"[LOI] {exc}", flush=True)
        return 1
    finally:
        if viewer is not None and viewer.poll() is None:
            viewer.terminate()
            viewer.wait()
        if simulator is not None and (not simulator_ready or (no_viewport and args.mode in ("all", "uav"))):
            stop_simulator(simulator)


def run_entrypoint():
    if os.environ.get("UAV_HIDE_CONSOLE") != "1":
        return main()
    log = DATA / "logs/launcher.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("a", encoding="utf-8") as stream, redirect_stdout(stream), redirect_stderr(stream):
        try:
            return main()
        except Exception:
            traceback.print_exc()
            return 1


if __name__ == "__main__":
    try:
        sys.exit(run_entrypoint())
    except KeyboardInterrupt:
        sys.exit(130)
