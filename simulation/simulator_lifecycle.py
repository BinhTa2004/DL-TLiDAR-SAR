"""Own the project simulator without disrupting an active flight session."""
from pathlib import Path
import psutil

from configs.config import SIMULATOR

ENGINE_NAMES = {"medievalvillagemegascanss.exe", "medievalvillagemegascanss-win64-shipping.exe"}


def is_project_simulator(exe):
    try:
        return bool(exe) and Path(exe).resolve().is_relative_to(SIMULATOR.resolve())
    except OSError:
        return False


def running_project_simulators():
    found = []
    for proc in psutil.process_iter(["pid", "name", "exe"]):
        try:
            name = (proc.info.get("name") or "").lower()
            exe = proc.info.get("exe") or ""
            if name in ENGINE_NAMES and is_project_simulator(exe):
                found.append(proc)
        except (psutil.Error, OSError):
            continue
    return found


def active_gcs_processes():
    active = []
    for proc in psutil.process_iter(["pid", "cmdline"]):
        try:
            cmdline = " ".join(proc.info.get("cmdline") or ()).lower()
            if "uav_controller.py" in cmdline or "uav_gcs_station.py" in cmdline:
                active.append(proc)
        except (psutil.Error, OSError):
            continue
    return active


def find_listener_pid(port=41451):
    try:
        for conn in psutil.net_connections(kind="inet"):
            if conn.status == psutil.CONN_LISTEN and conn.laddr and conn.laddr.port == port:
                return conn.pid
    except (psutil.Error, OSError):
        return None
    return None


def prepare_fresh_simulator():
    """Stop stale project Unreal processes, or fail if 41451 is actively used."""
    if active_gcs_processes():
        raise RuntimeError("GCS dang chay. Hay dong phien bay hien tai truoc khi mo lai he thong.")

    pid = find_listener_pid()
    if pid is not None:
        try:
            owner = psutil.Process(pid)
            exe = owner.exe()
        except (psutil.Error, OSError):
            raise RuntimeError(f"Cong AirSim 41451 dang bi tien trinh PID {pid} chiem.")
        if not is_project_simulator(exe):
            raise RuntimeError(f"Cong AirSim 41451 dang duoc dung boi {exe}. Hay dong chuong trinh do truoc.")

    simulators = running_project_simulators()
    if not simulators:
        return
    print("[KHOI DONG] Dang dong Unreal cu con chay nen de tao phien moi...", flush=True)
    for proc in simulators:
        try:
            proc.terminate()
        except (psutil.Error, OSError):
            pass
    _, alive = psutil.wait_procs(simulators, timeout=5)
    for proc in alive:
        try:
            proc.kill()
        except (psutil.Error, OSError):
            pass
    _, alive = psutil.wait_procs(alive, timeout=5)
    if alive or find_listener_pid() is not None:
        raise RuntimeError("Khong dong duoc Unreal cu dang giu cong 41451.")

