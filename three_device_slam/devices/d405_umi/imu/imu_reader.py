"""KT-EX9-2 军工 IMU UART 读取 + 解析。

从 C 版(rdk_x5_capture/src/imu_reader.c)移植，去掉 GPIO PPS 捕获
(小电脑没有 GPIO)，改为: PPS 接 IMU DIO2 让 counter 规整，系统时钟打时间戳。

线程模型:
  - 读线程: serial.read() → ring buffer(字节流)
  - 解析线程: 从 buffer 找帧头 → 校验 → 解析 → 回调(ImuSample)
"""

from __future__ import annotations
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional

from ..timing import OnlineCounterFitter
from .stream_protocol import StreamDecoder, TimerUnwrapper

FRAME_SIZE = 37
HEADER0 = 0xEB
HEADER1 = 0x90
EXPECTED_LEN = 0x22     # 数据包长度字节
IMU_HZ = 400            # 名义采样率，用于丢帧诊断
TRANSPORT_COUNTER_KEYS = (
    "frames_ok",
    "frames_bad",
    "resyncs",
    "dropped_frames",
    "counter_resets",
    "counter_stalls",
    "sequence_gaps",
    "invalid_imu_flags",
    "queue_overflow_flags",
    "serial_errors",
    "serial_reconnects",
)


@dataclass
class ImuSample:
    """一帧解析后的 IMU 数据。ts 是系统绝对时间(秒)。"""
    ts: float           # 系统时间戳(收到时打)
    counter: int        # 1..400，PPS 周期内序号
    gx: float; gy: float; gz: float    # °/s
    ax: float; ay: float; az: float    # g
    temp: float         # ℃
    rx_time: float      # 原始收到时刻(诊断抖动用)
    protocol: str = "kt_ex9_37"
    sequence: int | None = None
    flags: int = 0
    imu_first_byte_rx_us: int | None = None
    encoder_read_us: int | None = None
    encoder_response: int | None = None
    encoder_ts: float | None = None
    encoder_sensor_gap_us: int | None = None


def verify_checksum(buf: bytes) -> bool:
    """校验和 = 字节[0..35] 累加的低 8 位 == buf[36]。"""
    if len(buf) < FRAME_SIZE:
        return False
    return (sum(buf[0:36]) & 0xFF) == buf[36]


def parse_frame(buf: bytes) -> Optional[ImuSample]:
    """解析一帧(必须已校验通过)。时间戳留空，由 reader 打。"""
    if len(buf) < FRAME_SIZE:
        return None
    if buf[0] != HEADER0 or buf[1] != HEADER1 or buf[2] != EXPECTED_LEN:
        return None
    if not verify_checksum(buf):
        return None
    gx, gy, gz, ax, ay, az, temp = struct.unpack("<7f", buf[4:32])
    counter = struct.unpack("<I", buf[32:36])[0]
    now = time.monotonic()
    return ImuSample(
        ts=now, counter=counter,
        gx=gx, gy=gy, gz=gz,
        ax=ax, ay=ay, az=az,
        temp=temp, rx_time=now,
    )


def find_frames(stream: bytes):
    """从字节流里抽出所有合法帧，返回 (帧列表, 消费后的剩余字节, 统计)。

    用于离线解析 + 单元测试。
    """
    frames = []
    bad = 0
    resync = 0
    i = 0
    n = len(stream)
    while i <= n - FRAME_SIZE:
        if stream[i] == HEADER0 and stream[i + 1] == HEADER1 and stream[i + 2] == EXPECTED_LEN:
            frame = stream[i:i + FRAME_SIZE]
            if verify_checksum(frame):
                frames.append(frame)
                i += FRAME_SIZE
                continue
            else:
                bad += 1
        resync += 1
        i += 1
    return frames, stream[i:], {"bad_checksum": bad, "resyncs": resync}


def fit_counter_timestamps(ts_list, counter_list):
    """用硬件 counter 对 PC 接收时刻做鲁棒线性拟合, 消除串口/USB 接收抖动。

    IMU 采样由设备晶振驱动(等间隔), PC 接收时刻含串口+驱动+调度抖动。
    拟合 ts = a*counter + b 后, 每帧时间戳改取拟合值 —— 常数偏移 b 由
    Kalibr 的时间标定吸收, 我们要的是消除"逐帧不一致"的抖动。

    counter 通常自由递增；设备异常复位或 PPS/DIO2 输入会让它从 1
    重新开始。离线先把这些事件展开成单调序列。

    返回 (fitted_ts, info): fitted_ts 为 np.ndarray; info 含
    rate_hz(实测采样率)、sigma_ms(拟合残差 RMS, 即接收抖动指标)、
    outliers(剔除的离群点数)。
    """
    import numpy as np

    ts_arr = np.asarray(ts_list, dtype=np.float64)
    cnt = np.asarray(counter_list, dtype=np.int64)
    if len(ts_arr) < 10:
        return ts_arr.copy(), {"rate_hz": 0.0, "sigma_ms": 0.0, "outliers": 0}

    # 展开回绕。协议允许外部 DIO/PPS 把 counter 清零；实机也可能在
    # 无 PPS 时异常复位并从 1 重启。只有接近 UINT32_MAX -> 小值才是
    # 真正的 uint32 回绕。
    ec = np.empty(len(cnt), dtype=np.int64)
    wraps = 0
    prev = None
    for i, c in enumerate(cnt):
        if prev is not None and c < prev:
            if prev >= 0xF0000000 and c <= 0x0FFFFFFF:
                wraps += 1 << 32
            else:
                # PPS/reset 后的 c 表示新周期内已经产生的样本数。
                wraps += prev
        ec[i] = c + wraps
        prev = c

    A = np.vstack([ec.astype(np.float64), np.ones(len(ec))]).T
    mask = np.ones(len(ec), dtype=bool)
    a = b = 0.0
    for _ in range(4):
        a, b = np.linalg.lstsq(A[mask], ts_arr[mask], rcond=None)[0]
        res = ts_arr - (a * ec + b)
        sigma = float(res[mask].std())
        new_mask = np.abs(res) < 3.0 * max(sigma, 1e-4)
        if (new_mask == mask).all():
            break
        mask = new_mask
    fitted = a * ec + b
    sigma_ms = float((ts_arr[mask] - fitted[mask]).std() * 1000.0)
    info = {
        "rate_hz": 1.0 / a if a > 0 else 0.0,
        "sigma_ms": sigma_ms,
        "outliers": int((~mask).sum()),
    }
    return fitted, info


class ImuReader:
    """后台线程读串口 + 解析。每解出一帧调 on_sample。

    用法:
        r = ImuReader(port="/dev/ttyUSB0", on_sample=callback)
        r.start()
        ...
        r.stop()
    """

    def __init__(
        self,
        port: str,
        baud: int = 921600,
        on_sample: Optional[Callable[[ImuSample], None]] = None,
        on_raw_packet: Optional[Callable[[bytes], None]] = None,
        name: str = "imu",
        warmup_frames: int = 500,
        protocol: str = "auto",
    ):
        self.port = port
        self.baud = baud
        self.on_sample = on_sample
        self.on_raw_packet = on_raw_packet
        self.name = name
        self.protocol = protocol
        self._warmup_frames = max(0, int(warmup_frames))
        self._warmup_remaining = self._warmup_frames
        self._decoder = StreamDecoder(protocol, on_raw_candidate=on_raw_packet)
        self._mcu_timer = TimerUnwrapper()
        self.detected_protocols = set()
        self._formal_detected_protocols = set()

        self._ser = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._buf = bytearray()

        # 在线时间戳去抖: IMU 400Hz counter, 每 50 点拟合一次。启动阶段
        # 隐藏 500 点，让一秒滑窗排除打开串口时的 USB 积压后再发布。
        # 无 PPS 时不能在预热后冻结时钟相位：设备晶振与主机/相机存在
        # 微小频差，长时间冻结会让 camera-IMU 相位持续漂移。固定名义周期，
        # 但继续用主机接收时钟缓慢驯服相位。
        self._ts_fitter = OnlineCounterFitter(
            counter_wrap=None,
            window_size=400,
            fit_every=50,
            nominal_rate_hz=IMU_HZ,
            freeze_after=None,
        )
        # No-PPS transition clock. The device counter can unexpectedly restart
        # at 1; feeding that raw value into the timestamp fit can create a large
        # time step. Keep a host-side monotonic sample index and use the raw
        # counter only to preserve small, credible packet-loss gaps.
        self._clock_counter = 0

        # 统计
        self.frames_ok = 0
        self.frames_bad = 0
        self.resyncs = 0
        self.last_counter = None
        self.dropped_frames = 0      # counter 不连续 = 丢帧
        self.counter_resets = 0
        self.counter_stalls = 0      # counter 原地不增（如 DIO2 毛刺后连续为 1）
        self.sequence_gaps = 0
        self.last_sequence = None
        self.invalid_imu_flags = 0
        self.queue_overflow_flags = 0
        self.serial_errors = 0
        self.serial_reconnects = 0
        self.recent_dt = deque(maxlen=400)   # 最近帧间隔，算抖动
        self._warmup_stats = self._transport_snapshot() if self._warmup_frames == 0 else None

    def _transport_snapshot(self) -> dict:
        return {key: int(getattr(self, key, 0)) for key in TRANSPORT_COUNTER_KEYS}

    def _open_port(self) -> bool:
        try:
            import serial
        except ImportError as e:
            raise RuntimeError("需要 pyserial: pip install pyserial") from e

        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.001)
            # Discard bytes queued while the port was closed. Mixing a few
            # stale packets with the current stream creates a large counter
            # jump and corrupts the first online clock fit.
            self._ser.reset_input_buffer()
        except Exception as e:
            print(f"[{self.name}] 打开 {self.port} 失败: {e}")
            return False
        return True

    def start(self) -> bool:
        if not self._open_port():
            return False

        self._running = True
        self._warmup_remaining = self._warmup_frames
        self._decoder = StreamDecoder(
            self.protocol, on_raw_candidate=self.on_raw_packet
        )
        self._mcu_timer = TimerUnwrapper()
        self.detected_protocols.clear()
        self._formal_detected_protocols.clear()
        self._warmup_stats = self._transport_snapshot() if self._warmup_frames == 0 else None
        self._clock_counter = 0
        self.last_counter = None
        self.last_sequence = None
        self._ts_fitter.reset()
        self._thread = threading.Thread(target=self._loop, name=f"imu-{self.name}", daemon=True)
        self._thread.start()
        return True

    def _disconnect_serial(self, reason) -> None:
        self.serial_errors += 1
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        print(f"[{self.name}] IMU 串口断开: {reason}; 等待按 by-id 自动重连")

    def _try_reconnect(self) -> bool:
        if not self._open_port():
            return False
        self.serial_reconnects += 1
        self._buf.clear()
        self._decoder = StreamDecoder(
            self.protocol, on_raw_candidate=self.on_raw_packet
        )
        self._mcu_timer = TimerUnwrapper()
        self._warmup_remaining = self._warmup_frames
        # 断线期间的样本无法恢复。重新锚定到当前主机时钟，保留真实
        # 时间空档；主机虚拟 counter 继续单调，设备 raw counter 仍用于
        # 诊断复位事件。
        self._ts_fitter.reset()
        print(
            f"[{self.name}] IMU 串口已重连 #{self.serial_reconnects}: "
            f"{self.port}; 重新预热 {self._warmup_frames} 帧"
        )
        return True

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def _loop(self):
        next_reconnect = 0.0
        while self._running:
            if self._ser is None:
                now = time.monotonic()
                if now >= next_reconnect:
                    if self._try_reconnect():
                        next_reconnect = 0.0
                        continue
                    next_reconnect = now + 1.0
                time.sleep(0.05)
                continue
            try:
                # 有多少读多少,不等凑批(read(512)要等~34ms填满,是抖动主因)
                n = self._ser.in_waiting
                data = self._ser.read(n if n > 0 else 1)
            except Exception as e:
                self._disconnect_serial(e)
                continue
            if not data:
                continue
            self._consume_data(data)

    def _consume(self):
        """兼容旧调用：把暂存字节交给双协议流解码器。"""
        if self._buf:
            data = bytes(self._buf)
            self._buf.clear()
            self._consume_data(data)

    def _consume_data(self, data: bytes) -> None:
        """解析一个串口批次，并在批次边界开启正式统计窗口。"""
        formal_batch = self._warmup_stats is not None
        for packet in self._decoder.feed(data):
            self.detected_protocols.add(packet.protocol)
            if formal_batch:
                self._formal_detected_protocols.add(packet.protocol)
            self.frames_ok += 1
            if not packet.imu_valid:
                self.invalid_imu_flags += 1
                continue
            if packet.sequence is not None:
                if self.last_sequence is not None:
                    delta = (int(packet.sequence) - int(self.last_sequence)) & 0xFFFFFFFF
                    if delta != 1:
                        self.sequence_gaps += 1
                self.last_sequence = packet.sequence
            if packet.flags & ((1 << 5) | (1 << 6)):
                self.queue_overflow_flags += 1
            rx = time.monotonic()
            sample = ImuSample(
                ts=rx,
                counter=packet.counter,
                gx=packet.gx,
                gy=packet.gy,
                gz=packet.gz,
                ax=packet.ax,
                ay=packet.ay,
                az=packet.az,
                temp=packet.temperature_c,
                rx_time=rx,
                protocol=packet.protocol,
                sequence=packet.sequence,
                flags=packet.flags,
                imu_first_byte_rx_us=packet.imu_first_byte_rx_us,
                encoder_read_us=packet.encoder_read_us,
                encoder_response=packet.encoder_response,
            )
            self._accept_sample(
                sample,
                formal_batch=formal_batch,
                imu_first_byte_rx_us=packet.imu_first_byte_rx_us,
                encoder_read_us=packet.encoder_read_us,
                count_frame=False,
            )
        self.frames_bad = self._decoder.crc_or_checksum_errors
        self.resyncs = self._decoder.discarded_bytes
        self._finish_batch()

    def _handle(self, frame: bytes):
        s = parse_frame(frame)
        if s is None:
            return
        formal_batch = self._warmup_stats is not None
        self.detected_protocols.add("kt_ex9_37")
        if formal_batch:
            self._formal_detected_protocols.add("kt_ex9_37")
        self._accept_sample(s, formal_batch=formal_batch, count_frame=True)
        self._finish_batch()

    def _accept_sample(
        self,
        s: ImuSample,
        *,
        formal_batch: bool,
        imu_first_byte_rx_us: Optional[int] = None,
        encoder_read_us: Optional[int] = None,
        count_frame: bool,
    ) -> None:
        if count_frame:
            self.frames_ok += 1

        # 丢帧检测: counter 应连续递增。
        # 当前系统不接 PPS，counter 应自由递增。回到 1 说明 IMU/DIO
        # 发生了复位；时间拟合器会保持时间连续，但这里仍计为异常事件。
        clock_step = 1
        if self.last_counter is not None:
            prev = self.last_counter
            raw_delta = int(s.counter) - int(prev)
            if 1 <= raw_delta <= 8:
                clock_step = raw_delta
            if raw_delta == 0:
                # DIO2 被保持/毛刺触发时，设备可能连续多帧报告相同
                # counter。它不是多次复位，也不能拿来推进发布时间轴。
                self.counter_stalls += 1
            elif s.counter == 1:
                # 只把“进入 1”的边沿算作一次复位；后续连续的 1 由
                # counter_stalls 统计，避免一次毛刺显示成几十次复位。
                self.counter_resets += 1
                print(
                    f"[{self.name}] 警告: IMU counter 异常复位 "
                    f"{prev} -> 1（当前未使用 PPS，发布时间保持连续）"
                )
            elif raw_delta != 1:
                self.dropped_frames += 1
                # The USB-UART can emit a few stale FIFO packets immediately
                # after opening, followed by a large jump to the live stream.
                # Do not let that startup discontinuity contaminate the clock
                # fit. Small real packet drops still retain their counter gap.
                if s.counter > prev and s.counter - prev > IMU_HZ:
                    clock_step = 1
        self.last_counter = s.counter
        self._clock_counter += clock_step

        if imu_first_byte_rx_us is not None:
            # 新STM32包使用MCU捕获到IMU首字节的时间。MCU晶振与
            # D405/主机时钟存在频差，不能只在第一帧计算一次固定偏移，
            # 否则约一分钟后相机线程会持续等待“未来IMU”。沿用400 Hz
            # 设备节拍，并用主机接收时钟缓慢驯服相位；逐帧USB抖动不会
            # 直接进入发布时间戳。
            imu_device_us = self._mcu_timer.extend(imu_first_byte_rx_us)
            s.ts = self._ts_fitter.feed(self._clock_counter, s.rx_time)
            # Encoder and IMU timestamps are captured by the same STM32 timer.
            # Preserve their measured sub-frame MCU gap on the disciplined
            # common host timeline used by camera/VINS/model-training export.
            if encoder_read_us not in (None, 0):
                encoder_device_us = self._mcu_timer.extend(encoder_read_us)
                s.encoder_sensor_gap_us = encoder_device_us - imu_device_us
                s.encoder_ts = s.ts + s.encoder_sensor_gap_us / 1_000_000.0
        else:
            # 旧TTL链继续使用主机侧连续样本序号去抖。
            s.ts = self._ts_fitter.feed(self._clock_counter, s.ts)

        # 时间间隔抖动统计(仍用原始接收时刻, 作为链路质量诊断)
        if not hasattr(self, "_first_rx"):
            self._first_rx = s.rx_time
        if self.recent_dt:
            self.recent_dt.append(s.rx_time - self._last_rx)
        else:
            self.recent_dt.append(0.0)
        self._last_rx = s.rx_time

        if self._warmup_remaining > 0:
            self._warmup_remaining -= 1
            return

        if formal_batch and self.on_sample:
            try:
                self.on_sample(s)
            except Exception:
                pass

    def _finish_batch(self) -> None:
        if self._warmup_remaining == 0 and self._warmup_stats is None:
            # 解码器的坏字节统计按输入批次更新。转换批次整体归入预热，
            # 防止同一USB批次中的启动噪声被藏进正式窗口。
            self._warmup_stats = self._transport_snapshot()

    @property
    def _last_rx(self):
        return getattr(self, "_last_rx_val", 0.0)

    @_last_rx.setter
    def _last_rx(self, v):
        self._last_rx_val = v

    def stats(self) -> dict:
        dts = list(self.recent_dt)[1:] if len(self.recent_dt) > 1 else []
        dt_ms = [d * 1000 for d in dts]
        # 帧率用 累计帧数/累计时长(USB 批量到达会让逐帧 dt 统计虚高)
        first_rx = getattr(self, "_first_rx", None)
        rate = 0.0
        if first_rx is not None and self._last_rx > first_rx:
            rate = self.frames_ok / (self._last_rx - first_rx)
        detected_protocol = (
            next(iter(self.detected_protocols))
            if len(self.detected_protocols) == 1
            else "mixed" if self.detected_protocols else "unknown"
        )
        return {
            "protocol": detected_protocol,
            "frames_ok": self.frames_ok,
            "frames_bad": self.frames_bad,
            "resyncs": self.resyncs,
            "dropped_frames": self.dropped_frames,
            "counter_resets": self.counter_resets,
            "counter_stalls": self.counter_stalls,
            "sequence_gaps": self.sequence_gaps,
            "invalid_imu_flags": self.invalid_imu_flags,
            "queue_overflow_flags": self.queue_overflow_flags,
            "serial_errors": self.serial_errors,
            "serial_reconnects": self.serial_reconnects,
            "serial_connected": self._ser is not None,
            "rate_hz": rate,
            "dt_min_ms": min(dt_ms) if dt_ms else 0.0,
            "dt_max_ms": max(dt_ms) if dt_ms else 0.0,
            "dt_jitter_ms": (max(dt_ms) - min(dt_ms)) if dt_ms else 0.0,
        }

    def warmup_stats(self) -> dict:
        """Return cumulative transport counters at the warm-up boundary."""
        return dict(self._warmup_stats or {})

    def stats_since_warmup(self) -> dict:
        """Return reader statistics with transport counters scoped to formal data."""
        current = self.stats()
        if self._warmup_stats is None:
            return {}
        formal = dict(current)
        for key in TRANSPORT_COUNTER_KEYS:
            formal[key] = int(current.get(key, 0)) - int(self._warmup_stats.get(key, 0))
        formal["protocol"] = (
            next(iter(self._formal_detected_protocols))
            if len(self._formal_detected_protocols) == 1
            else "mixed" if self._formal_detected_protocols else "unknown"
        )
        return formal
