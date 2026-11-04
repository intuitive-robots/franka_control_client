"""Thread-safe rate meters and ring buffers feeding the dashboard."""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np


class RateMeter:
    """Event frequency over a sliding time window."""

    def __init__(self, window_s: float = 2.0) -> None:
        self.window_s = window_s
        self._ticks: Deque[float] = deque()
        self._lock = threading.Lock()

    def tick(self, t: Optional[float] = None) -> None:
        t = time.time() if t is None else t
        with self._lock:
            self._ticks.append(t)
            self._trim(t)

    def rate(self, now: Optional[float] = None) -> float:
        now = time.time() if now is None else now
        with self._lock:
            self._trim(now)
            if len(self._ticks) < 2:
                return 0.0
            span = self._ticks[-1] - self._ticks[0]
            # A stream that stopped must read as 0 Hz, not its last value.
            if span <= 0 or now - self._ticks[-1] > max(1.0, 3 * span / (len(self._ticks) - 1)):
                return 0.0
            return (len(self._ticks) - 1) / span

    def reset(self) -> None:
        with self._lock:
            self._ticks.clear()

    def _trim(self, now: float) -> None:
        while self._ticks and now - self._ticks[0] > self.window_s:
            self._ticks.popleft()


class Series:
    """Ring buffer of (t, vector) samples, read incrementally by the dashboard."""

    def __init__(self, maxlen: int) -> None:
        self._data: Deque[Tuple[float, List[float]]] = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def append(self, t: float, values: Sequence[float]) -> None:
        row = [round(float(v), 5) for v in values]
        with self._lock:
            self._data.append((t, row))

    def since(self, t: float, max_points: int = 400) -> List[List[float]]:
        with self._lock:
            rows = [[round(ts, 4)] + v for ts, v in self._data if ts > t]
        if len(rows) > max_points:  # decimate for the browser, keep the newest
            step = int(np.ceil(len(rows) / max_points))
            rows = rows[::-1][::step][::-1]
        return rows

    def clear(self) -> None:
        with self._lock:
            self._data.clear()


class Percentiles:
    def __init__(self, maxlen: int = 200) -> None:
        self._values: Deque[float] = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def add(self, v: float) -> None:
        with self._lock:
            self._values.append(float(v))

    def summary(self) -> Dict[str, Optional[float]]:
        with self._lock:
            arr = np.asarray(self._values, dtype=np.float64)
        if arr.size == 0:
            return {"last": None, "p50": None, "p95": None, "max": None}
        return {
            "last": float(arr[-1]),
            "p50": float(np.percentile(arr, 50)),
            "p95": float(np.percentile(arr, 95)),
            "max": float(arr.max()),
        }

    def clear(self) -> None:
        with self._lock:
            self._values.clear()


class Telemetry:
    """Everything the dashboard plots. Times are wall-clock seconds."""

    def __init__(self, history_s: float = 30.0) -> None:
        self.control_rate = RateMeter()
        self.infer_rate = RateMeter(window_s=5.0)
        self.command_rate = RateMeter()
        self.state_rate = RateMeter()  # new arm state messages actually received
        self.latency_ms = Percentiles()  # robot-side round trip of one infer call
        self.model_ms = Percentiles()  # server-reported pure model time
        # Series sized for ~history_s at their nominal rates.
        self.state_series = Series(maxlen=int(history_s * 50))  # q(7) + gripper
        self.ee_series = Series(maxlen=int(history_s * 50))  # xyz + quat
        self.cmd_series = Series(maxlen=int(history_s * 100))  # arm cmd(7)
        # Executed rows of each chunk as the policy generated them (before joint-limit
        # clipping / speed limiting), stamped at the time each row is scheduled.
        self.gen_series = Series(maxlen=int(history_s * 50))  # 8: arm(7) + gripper
        self.grip_cmd_series = Series(maxlen=int(history_s * 50))  # gripper command (0..1)
        self.latency_series = Series(maxlen=600)  # [rtt_ms, model_ms]
        self.rate_series = Series(maxlen=int(history_s * 4))  # [control_hz, infer_hz]
        self.counters: Dict[str, int] = {"infer": 0, "control_steps": 0, "commands": 0, "rejected_chunks": 0}
        self.last_chunk: Optional[Dict[str, Any]] = None
        self._lock = threading.Lock()

    def count(self, key: str, n: int = 1) -> None:
        with self._lock:
            self.counters[key] = self.counters.get(key, 0) + n

    def reset_episode(self) -> None:
        with self._lock:
            for key in self.counters:
                self.counters[key] = 0
            self.last_chunk = None
        self.latency_ms.clear()
        self.model_ms.clear()
        self.infer_rate.reset()

    def snapshot(self, since: float) -> Dict[str, Any]:
        now = time.time()
        with self._lock:
            counters = dict(self.counters)
            last_chunk = self.last_chunk
        return {
            "rates": {
                "control_hz": self.control_rate.rate(now),
                "infer_hz": self.infer_rate.rate(now),
                "command_hz": self.command_rate.rate(now),
                "state_hz": self.state_rate.rate(now),
            },
            "latency_ms": self.latency_ms.summary(),
            "model_ms": self.model_ms.summary(),
            "counters": counters,
            "last_chunk": last_chunk,
            "series": {
                "state": self.state_series.since(since),
                "ee": self.ee_series.since(since),
                "cmd": self.cmd_series.since(since),
                "gen": self.gen_series.since(since),
                "gripcmd": self.grip_cmd_series.since(since),
                "latency": self.latency_series.since(since),
                "rates": self.rate_series.since(since),
            },
        }
