"""三路录制。

每个单元一个 UnitRecorder:
  - 彩色帧 → frames/<idx>.jpg (压缩，省空间)
  - 深度帧 → frames/<idx>.npy (按需，头部后处理用)
  - IMU 样本 → imu.bin (追加二进制) + timestamps.csv

为什么 JPG: 512G 三路长录偏紧，未压缩 RGB 几小时爆盘。
深度 .npy 只在关键时刻录(save_depth=True)。

目录布局:
  <out_dir>/<session>/<unit>/
      frames/000001.jpg ...
      frames/000001.npy ... (可选)
      imu.bin
      camera_ts.csv    (idx, frame_number, ts_mono, rx_mono, ts_wall, has_depth)
      imu_ts.csv       (counter, ts_mono, rx_mono, ts_wall)
      gripper_encoder.csv (STM32 encoder state on the shared monotonic timeline)
"""

from __future__ import annotations
import csv
import os
import struct
import threading
import time
from pathlib import Path
from queue import Full, Queue

import numpy as np

from ..gripper import ManualGripperCalibration, ManualGripperTracker

# IMU 二进制记录(40 字节):
#   double ts, uint32 counter, float gx gy gz ax ay az temp
IMU_PACK_FMT = "<dI7f"
IMU_PACK_SIZE = struct.calcsize(IMU_PACK_FMT)   # = 8+4+28 = 40


class UnitRecorder:
    def __init__(
        self,
        unit_name: str,
        out_dir: Path,
        jpg_quality: int = 90,
        save_depth: bool = False,
        imu_bin: bool = True,
        record_gripper: bool = False,
        max_queue: int = 600,
    ):
        self.unit_name = unit_name
        self.dir = out_dir / unit_name
        self.frames_dir = self.dir / "frames"
        self.jpg_quality = jpg_quality
        self.save_depth = save_depth
        self.imu_bin = imu_bin
        self.record_gripper = record_gripper

        self._q: Queue = Queue(maxsize=max_queue)
        self._running = False
        self._thread = None
        self._imu_fp = None
        self._raw_imu_fp = None
        self._cam_csv = None
        self._cam_csv_w = None
        self._imu_csv = None
        self._imu_csv_w = None
        self._gripper_csv = None
        self._gripper_csv_w = None
        self._gripper_calibration = None
        self._gripper_tracker = None
        self.dropped = 0     # 队列满丢掉的帧
        self.gripper_samples_written = 0
        self.gripper_invalid_samples = 0
        self._write_error_lock = threading.Lock()
        self._first_write_error = None

    @property
    def first_write_error(self):
        with self._write_error_lock:
            return self._first_write_error

    def _latch_write_error(self, error: Exception):
        with self._write_error_lock:
            if self._first_write_error is None:
                self._first_write_error = f"{type(error).__name__}: {error}"

    def start(self):
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        if self.imu_bin:
            self._imu_fp = open(self.dir / "imu.bin", "wb")
            self._raw_imu_fp = open(self.dir / "raw_imu_packets.bin", "wb")
        self._cam_csv = open(self.dir / "camera_ts.csv", "w", newline="", encoding="utf-8")
        self._cam_csv_w = csv.writer(self._cam_csv)
        self._cam_csv_w.writerow([
            "idx", "frame_number", "ts_mono", "rx_mono", "ts_wall", "has_depth"
        ])
        self._imu_csv = open(self.dir / "imu_ts.csv", "w", newline="", encoding="utf-8")
        self._imu_csv_w = csv.writer(self._imu_csv)
        self._imu_csv_w.writerow(["counter", "ts_mono", "rx_mono", "ts_wall"])
        if self.record_gripper:
            self._gripper_calibration = ManualGripperCalibration.load()
            self._gripper_tracker = ManualGripperTracker(self._gripper_calibration)
            self._gripper_csv = open(
                self.dir / "gripper_encoder.csv", "w", newline="", encoding="utf-8"
            )
            self._gripper_csv_w = csv.writer(self._gripper_csv)
            self._gripper_csv_w.writerow([
                "schema", "calibration_id", "protocol", "sequence", "imu_counter",
                "imu_ts_mono", "encoder_ts_mono", "sensor_pair_delta_us",
                "imu_first_byte_rx_us", "encoder_read_us", "raw_flags",
                "encoder_valid", "raw_count", "angle_deg", "direction",
                "closure_ratio", "estimated_no_load_gap_mm",
                "no_load_uncertainty_mm", "dual_closing_distance_mm",
                "single_jaw_travel_mm", "loaded_object_size_valid", "status",
            ])

        self._running = True
        self._thread = threading.Thread(target=self._loop, name=f"rec-{self.unit_name}", daemon=True)
        self._thread.start()

    def stop(self, timeout_s: float = 3.0) -> bool:
        self._running = False
        try:
            self._q.put_nowait(None)   # 哨兵
        except Full:
            pass
        if self._thread:
            self._thread.join(timeout=timeout_s)
            if self._thread.is_alive():
                self._latch_write_error(
                    TimeoutError("recorder shutdown timed out")
                )
                return False
        return True

    def put_color(
        self,
        idx: int,
        frame_number: int,
        ts_mono: float,
        rx_mono: float,
        color,
        ts_wall: float,
    ):
        if self._q.qsize() >= self._q.maxsize:
            self.dropped += 1
            return
        self._q.put(("color", idx, frame_number, ts_mono, rx_mono, color, ts_wall))

    def put_depth(self, idx: int, ts_mono: float, depth, ts_wall: float):
        if not self.save_depth:
            return
        if self._q.qsize() >= self._q.maxsize:
            self.dropped += 1
            return
        self._q.put(("depth", idx, ts_mono, depth, ts_wall))

    def put_imu(self, sample):
        if self._q.qsize() >= self._q.maxsize:
            self.dropped += 1
            return
        self._q.put(("imu", sample))

    def put_raw_imu_packet(self, packet: bytes) -> None:
        if self._q.qsize() >= self._q.maxsize:
            self.dropped += 1
            return
        self._q.put(("raw_imu", bytes(packet)))

    def _loop(self):
        while self._running or not self._q.empty():
            item = self._q.get()
            if item is None:
                break
            kind = item[0]
            try:
                if kind == "color":
                    import cv2
                    _, idx, frame_number, ts_mono, rx_mono, color, ts_wall = item
                    path = self.frames_dir / f"{idx:06d}.jpg"
                    ok, buf = cv2.imencode(".jpg", color, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpg_quality])
                    if ok:
                        buf.tofile(path)
                    self._cam_csv_w.writerow([
                        idx,
                        frame_number,
                        f"{ts_mono:.9f}",
                        f"{rx_mono:.9f}",
                        f"{ts_wall:.6f}",
                        0,
                    ])
                elif kind == "depth":
                    _, idx, ts_mono, depth, ts_wall = item
                    np.save(self.frames_dir / f"{idx:06d}.npy", depth)
                    # 同时补一行 depth 标记(合并到 camera_ts)
                    self._cam_csv_w.writerow([
                        idx, "", f"{ts_mono:.9f}", "", f"{ts_wall:.6f}", 1
                    ])
                elif kind == "imu":
                    s = item[1]
                    if self._imu_fp:
                        self._imu_fp.write(struct.pack(
                            IMU_PACK_FMT, s.ts, s.counter,
                            s.gx, s.gy, s.gz, s.ax, s.ay, s.az, s.temp,
                        ))
                        self._imu_fp.flush()
                    self._imu_csv_w.writerow([
                        s.counter,
                        f"{s.ts:.9f}",
                        f"{s.rx_time:.9f}",
                        f"{time.time():.6f}",
                    ])
                    if self._gripper_csv_w is not None and s.protocol == "stm32_combined_v1":
                        valid = (
                            s.encoder_ts is not None
                            and s.encoder_response is not None
                            and bool(s.flags & (1 << 1))
                            and not bool(s.flags & ((1 << 2) | (1 << 3)))
                        )
                        raw_count = (
                            int(s.encoder_response) & 0x3FFF
                            if s.encoder_response is not None else 0
                        )
                        angle_deg = raw_count * 360.0 / 16384.0
                        state = self._gripper_tracker.update(
                            angle_deg, encoder_valid=valid
                        )
                        self._gripper_csv_w.writerow([
                            "umi_gripper_sample_v1",
                            self._gripper_calibration.profile_id,
                            s.protocol,
                            "" if s.sequence is None else s.sequence,
                            s.counter,
                            f"{s.ts:.9f}",
                            "" if s.encoder_ts is None else f"{s.encoder_ts:.9f}",
                            "" if s.encoder_sensor_gap_us is None else s.encoder_sensor_gap_us,
                            "" if s.imu_first_byte_rx_us is None else s.imu_first_byte_rx_us,
                            "" if s.encoder_read_us is None else s.encoder_read_us,
                            s.flags,
                            int(valid),
                            raw_count,
                            f"{angle_deg:.9f}",
                            state.direction,
                            "" if state.closure_ratio is None else f"{state.closure_ratio:.9f}",
                            "" if state.estimated_no_load_gap_mm is None else f"{state.estimated_no_load_gap_mm:.6f}",
                            "" if state.no_load_uncertainty_mm is None else f"{state.no_load_uncertainty_mm:.6f}",
                            "" if state.dual_closing_distance_mm is None else f"{state.dual_closing_distance_mm:.6f}",
                            "" if state.single_jaw_travel_mm is None else f"{state.single_jaw_travel_mm:.6f}",
                            0,
                            state.status,
                        ])
                        self.gripper_samples_written += 1
                        if not valid:
                            self.gripper_invalid_samples += 1
                elif kind == "raw_imu":
                    if self._raw_imu_fp:
                        self._raw_imu_fp.write(item[1])
            except Exception as e:
                self._latch_write_error(e)
                print(f"[rec-{self.unit_name}] 写盘错误: {e}")

        for fp in (
            self._imu_fp,
            self._raw_imu_fp,
            self._cam_csv,
            self._imu_csv,
            self._gripper_csv,
        ):
            if fp is None:
                continue
            try:
                fp.flush()
                fp.close()
            except Exception as e:
                self._latch_write_error(e)


class Recorder:
    """多单元录制管理。"""

    def __init__(self, unit_names, out_root: Path, **kw):
        self.out_root = out_root
        self.units = {n: UnitRecorder(n, out_root, **kw) for n in unit_names}

    def start(self):
        self.out_root.mkdir(parents=True, exist_ok=True)
        for u in self.units.values():
            u.start()

    def stop(self):
        for u in self.units.values():
            u.stop()

    def get(self, unit_name: str) -> UnitRecorder:
        return self.units[unit_name]
