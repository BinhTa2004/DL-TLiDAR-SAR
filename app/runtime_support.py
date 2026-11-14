"""Quiet console output, bounded diagnostic logs, AirSim connections and live session initialization."""
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import socket
import sys
import time

from configs.config import DATA
from .shared_lidar_stream import LidarStreamWriter


def get_logger(name):
    logger = logging.getLogger(name)
    if logger.handlers:
        return logger
    logger.setLevel(logging.DEBUG if os.getenv("UAV_VERBOSE") == "1" else logging.INFO)
    logger.propagate = False
    directory = DATA / "logs"
    directory.mkdir(parents=True, exist_ok=True)
    console = logging.StreamHandler(sys.stdout)
    console.setFormatter(logging.Formatter("%(message)s"))
    logfile = RotatingFileHandler(directory / f"{name}.log", maxBytes=2_000_000,
                                  backupCount=2, encoding="utf-8")
    logfile.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(console)
    logger.addHandler(logfile)
    return logger


def connect_airsim(ip="127.0.0.1", timeout=5):
    # Probe TCP before creating msgpack clients while Unreal loads the map.
    with socket.create_connection((ip, 41451), timeout=min(timeout, 1.0)):
        pass
    import airsim
    client = airsim.MultirotorClient(ip=ip, timeout_value=timeout)
    try:
        if not client.ping():
            raise ConnectionError("AirSim ping returned false")
        if (client.getServerVersion() < client.getMinRequiredServerVersion() or
                client.getClientVersion() < client.getMinRequiredClientVersion()):
            raise RuntimeError("Phien ban AirSim client/server khong tuong thich")
        return client
    except Exception:
        client.client.close()
        raise


def start_live_session(data_dir=DATA, session_id=None):
    data_dir = Path(data_dir)
    data_dir.mkdir(parents=True, exist_ok=True)
    session_id = int(session_id if session_id is not None else time.time_ns())
    # Reuse the mapping in place so an already-open viewer can see the reset.
    writer = LidarStreamWriter(data_dir / "lidar_stream.bin", session_id=session_id)
    writer.close()
    initial = {
        "session_id": session_id, "timestamp": time.time(),
        "uav_pos_ned": None, "uav_pos_enu": None,
        "start_pos_ned": None, "start_pos_enu": None,
        "route_lines_enabled": False, "lidar_enabled": True,
        "active_target": None, "marked_persons": [], "live_detections": [],
    }
    states = {
        "live_targets.json": initial,
        "lidar_state.json": {"session_id": session_id, "timestamp": time.time(),
                             "lidar_enabled": True, "source": "startup"},
    }
    for name, payload in states.items():
        destination = data_dir / name
        temporary = data_dir / f"{name}.{os.getpid()}.session.tmp"
        text = json.dumps(payload, ensure_ascii=False)
        temporary.write_text(text, encoding="utf-8")
        replaced = False
        for _ in range(10):
            try:
                temporary.replace(destination)
                replaced = True
                break
            except PermissionError:
                time.sleep(0.02)
        if not replaced:
            try:
                destination.write_text(text, encoding="utf-8")
            except PermissionError:
                pass
            finally:
                temporary.unlink(missing_ok=True)
    return session_id

