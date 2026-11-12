# -*- coding: utf-8 -*-
"""Low-overhead, single-writer LiDAR stream shared by the GCS and Open3D.

The GCS is the only AirSim LiDAR client. It publishes world-space NED points
to a fixed-size memory-mapped file. Readers use a sequence lock, so they
never consume a partially written scan and never block the acquisition loop.
"""
import mmap
import os
import struct
import time

import numpy as np

MAGIC = b"LDRSTRM1"
VERSION = 2
MAX_POINTS = 120_000
HEADER_SIZE = 64
_HEADER = struct.Struct("<8sIQQI3fQ")
FILE_SIZE = HEADER_SIZE + MAX_POINTS * 3 * np.dtype(np.float32).itemsize


class LidarStreamWriter:
    def __init__(self, path, max_points=MAX_POINTS, session_id=None):
        self.path = os.path.abspath(path)
        self.max_points = min(int(max_points), MAX_POINTS)
        self.session_id = int(session_id if session_id is not None else time.time_ns())
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        # Reuse an existing correctly sized mapping. This lets a viewer that
        # is already open survive a GCS restart without invalidating its mmap.
        reuse = os.path.exists(self.path) and os.path.getsize(self.path) == FILE_SIZE
        self._file = open(self.path, "r+b" if reuse else "w+b")
        if not reuse:
            self._file.truncate(FILE_SIZE)
        self._map = mmap.mmap(self._file.fileno(), FILE_SIZE, access=mmap.ACCESS_WRITE)
        self._seq = 0
        started_ns = time.time_ns()
        self._write_header(1, started_ns, 0, (0.0, 0.0, 0.0))
        self._write_header(self._seq, started_ns, 0, (0.0, 0.0, 0.0))

    def _write_header(self, seq, timestamp_ns, count, uav_pos_ned):
        x, y, z = (float(v) for v in uav_pos_ned)
        _HEADER.pack_into(
            self._map, 0, MAGIC, VERSION, int(seq), int(timestamp_ns),
            int(count), x, y, z, self.session_id,
        )

    def publish(self, points_ned, uav_pos_ned, timestamp_ns=None):
        points = np.asarray(points_ned, dtype=np.float32).reshape(-1, 3)
        if len(points) > self.max_points:
            step = max(1, int(np.ceil(len(points) / self.max_points)))
            points = points[::step][:self.max_points]
        points = np.ascontiguousarray(points)
        timestamp_ns = int(timestamp_ns if timestamp_ns is not None else time.time_ns())

        # Odd sequence means a write is in progress; even means stable.
        write_seq = self._seq + 1
        self._write_header(write_seq, timestamp_ns, len(points), uav_pos_ned)
        if len(points):
            view = np.ndarray(
                (len(points), 3), dtype=np.float32,
                buffer=self._map, offset=HEADER_SIZE,
            )
            view[:] = points
        self._seq = write_seq + 1
        self._write_header(self._seq, timestamp_ns, len(points), uav_pos_ned)
        return self._seq

    def close(self):
        try:
            self._map.close()
        finally:
            self._file.close()


class LidarStreamReader:
    def __init__(self, path):
        self.path = os.path.abspath(path)
        self._file = None
        self._map = None

    def _open_if_ready(self):
        if self._map is not None:
            return True
        try:
            if os.path.getsize(self.path) != FILE_SIZE:
                return False
            self._file = open(self.path, "rb")
            self._map = mmap.mmap(self._file.fileno(), FILE_SIZE, access=mmap.ACCESS_READ)
            magic, version, *_ = _HEADER.unpack_from(self._map, 0)
            if magic != MAGIC or version != VERSION:
                self.close()
                return False
            return True
        except (FileNotFoundError, OSError, ValueError):
            self.close()
            return False

    def read_latest(self, max_age_s=2.0):
        if not self._open_if_ready():
            return None
        try:
            first = _HEADER.unpack_from(self._map, 0)
            magic, version, seq1, timestamp_ns, count, x, y, z, session_id = first
            if magic != MAGIC or version != VERSION or (seq1 & 1):
                return None
            if count < 0 or count > MAX_POINTS:
                return None
            age_s = (time.time_ns() - timestamp_ns) / 1e9
            if timestamp_ns <= 0 or not -2.0 <= age_s <= max_age_s:
                return None
            points = np.ndarray(
                (count, 3), dtype=np.float32,
                buffer=self._map, offset=HEADER_SIZE,
            ).copy()
            second = _HEADER.unpack_from(self._map, 0)
            if first != second or (second[2] & 1):
                return None
            return {
                "sequence": int(seq1),
                "session_id": int(session_id),
                "timestamp_ns": int(timestamp_ns),
                "points_ned": points,
                "uav_pos_ned": np.array([x, y, z], dtype=np.float32),
            }
        except (BufferError, OSError, ValueError):
            self.close()
            return None

    def close(self):
        if self._map is not None:
            try:
                self._map.close()
            except Exception:
                pass
            self._map = None
        if self._file is not None:
            try:
                self._file.close()
            except Exception:
                pass
            self._file = None
