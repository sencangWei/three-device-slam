"""在线 counter/帧序号时钟拟合, 给实时采集提供去抖时间戳。

离线转 bag 用的 fit_counter_timestamps 是对整段数据做拟合;
这里提供滑动窗口版, 每收到若干样本拟合一次, 中间样本用最新参数预测。
"""
from __future__ import annotations

from collections import deque
from typing import Optional

import numpy as np


class OnlineCounterFitter:
    """用硬件 counter(或帧序号)对 PC 接收时间做在线线性拟合去抖。

    模型: ts_fitted = a * unwrapped_counter + b
    - counter 严格等间隔(设备晶振), 所以拟合斜率 a 就是采样周期。
    - 截距 b 吸收固定传输延迟, 不需要在实时里精确估计, 只要稳定即可。
    """

    def __init__(
        self,
        counter_wrap: Optional[int] = None,
        window_size: int = 400,
        fit_every: int = 50,
        nominal_rate_hz: Optional[float] = None,
        freeze_after: Optional[int] = None,
    ):
        self.counter_wrap = counter_wrap
        self.window_size = window_size
        self.fit_every = fit_every
        self.nominal_rate_hz = nominal_rate_hz
        self.freeze_after = freeze_after
        self._samples = deque(maxlen=window_size)
        self._a: Optional[float] = None
        self._b: Optional[float] = None
        self._last_raw_counter: Optional[int] = None
        self._wrap_acc = 0
        self._fit_count = 0
        self._total_count = 0
        self._last_unwrapped_counter: Optional[int] = None
        self._last_output_ts: Optional[float] = None

    def reset(self) -> None:
        """Drop the current fit and anchor again on the next sample."""
        self._samples.clear()
        self._a = None
        self._b = None
        self._last_raw_counter = None
        self._wrap_acc = 0
        self._fit_count = 0
        self._total_count = 0
        self._last_unwrapped_counter = None
        self._last_output_ts = None

    def feed(self, raw_counter: int, raw_ts: float) -> float:
        """喂入一个新样本, 返回去抖后的时间戳。"""
        # 展开回绕
        if self._last_raw_counter is not None and raw_counter < self._last_raw_counter:
            # KT-EX9-2 may restart its free-running counter at 1 after an
            # unexpected device/DIO reset. PPS installations use the same
            # low counter range. Preserve continuity when the host observes
            # no matching receive-time gap; only UINT32_MAX -> small is a
            # true uint32 wrap.
            if (
                self.counter_wrap is not None
                and raw_counter <= self.counter_wrap
                and (
                    self._last_output_ts is None
                    or raw_ts - self._last_output_ts < 0.1
                )
            ):
                # A reset can follow a long free-running count. Preserve the
                # sample sequence across it when reception itself is continuous.
                wrap = self._last_raw_counter
            elif self._last_raw_counter >= 0xF0000000 and raw_counter <= 0x0FFFFFFF:
                wrap = 1 << 32
            else:
                self.reset()
                wrap = 0
            self._wrap_acc += wrap
        self._last_raw_counter = raw_counter
        uc = raw_counter + self._wrap_acc

        self._samples.append((uc, raw_ts))
        self._fit_count += 1
        self._total_count += 1

        if len(self._samples) >= 10 and self._fit_count >= self.fit_every:
            self._fit()
            self._fit_count = 0

        # Follow both fitted frequency and fitted phase without ever stepping
        # backwards. Startup and large phase errors recover quickly; once the
        # fit is locked, corrections stay tiny so an estimator does not see
        # receive-jitter-shaped timestamp steps.
        if self._a is not None and self._a > 0.0 and self._last_output_ts is not None:
            delta = uc - self._last_unwrapped_counter
            period = max(self._a, 1e-9)
            base = self._last_output_ts + period * max(delta, 1)
            predicted = self._a * uc + self._b
            if (
                self.nominal_rate_hz is not None
                and self.freeze_after is not None
                and self._total_count == self.freeze_after
            ):
                # This sample is still hidden by ImuReader warmup. Snap once
                # to the final device-clock map so every published interval
                # afterwards is exactly the nominal hardware period.
                output_ts = predicted
                self._last_unwrapped_counter = uc
                self._last_output_ts = float(output_ts)
                return self._last_output_ts
            startup_limit = min(self.window_size, 500)
            phase_error = predicted - base
            fast_recovery = (
                self._total_count <= startup_limit
                or abs(phase_error) > 2.0 * period
            )
            correction_fraction = 0.9 if fast_recovery else 0.005
            max_correction = correction_fraction * period
            correction = min(max(phase_error, -max_correction), max_correction)
            output_ts = base + correction
        elif (
            self.nominal_rate_hz is not None
            and self.nominal_rate_hz > 0.0
            and self._last_output_ts is not None
            and self._last_unwrapped_counter is not None
        ):
            # Before the first ten-sample fit, preserve the device's nominal
            # spacing instead of exposing multiple packets from one USB read
            # batch with nearly identical host-arrival timestamps.
            delta = max(uc - self._last_unwrapped_counter, 1)
            output_ts = self._last_output_ts + delta / self.nominal_rate_hz
        else:
            output_ts = raw_ts
            if self._last_output_ts is not None:
                output_ts = max(output_ts, self._last_output_ts + 1e-9)

        self._last_unwrapped_counter = uc
        self._last_output_ts = float(output_ts)
        return self._last_output_ts

    def _fit(self):
        """对窗口内样本做鲁棒线性拟合。"""
        if (
            self.freeze_after is not None
            and self._total_count > self.freeze_after
            and self._a is not None
        ):
            return

        arr = np.asarray(self._samples, dtype=np.float64)
        ec = arr[:, 0]
        ts = arr[:, 1]

        if self.nominal_rate_hz is not None:
            a = 1.0 / self.nominal_rate_hz
            # Median transport phase rejects USB delivery bursts. ImuReader
            # hides the fitting interval, then freezes this device-clock map.
            self._a = a
            self._b = float(np.median(ts - a * ec))
            return

        A = np.vstack([ec, np.ones(len(ec))]).T
        # 第一次普通最小二乘
        a, b = np.linalg.lstsq(A, ts, rcond=None)[0]
        res = ts - (a * ec + b)
        sigma = float(res.std())
        # 剔除 3σ 离群点再拟合
        mask = np.abs(res) < 3.0 * max(sigma, 1e-4)
        if mask.sum() >= 10:
            a, b = np.linalg.lstsq(A[mask], ts[mask], rcond=None)[0]
        self._a = float(a)
        self._b = float(b)

    @property
    def rate_hz(self) -> Optional[float]:
        return 1.0 / self._a if self._a and self._a > 0 else None
