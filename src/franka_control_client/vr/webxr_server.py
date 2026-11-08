"""WebSocket + static file server for the WebXR teleop page.

Runs aiohttp in a daemon thread. Only the latest pose sample of the newest
connection is kept; older connections are ignored once a new one opens.
"""

import asyncio
import json
import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Optional, Tuple

import numpy as np
from aiohttp import WSMsgType, web

STATIC_DIR = Path(__file__).parent / "webxr" / "static"
STATS_PERIOD_S = 5.0

log = logging.getLogger("webxr")


@dataclass(frozen=True)
class XRSample:
    seq: int
    t_client_ms: float
    pos: Tuple[float, float, float]
    rot: Tuple[float, float, float, float]  # xyzw
    trigger: float
    grip: float
    a: bool
    b: bool
    recv_t: float  # time.monotonic() on arrival
    conn_id: int
    stick: bool = False  # right thumbstick pressed (realign)
    head_rot: Optional[Tuple[float, float, float, float]] = None  # headset orientation, xyzw


@dataclass(frozen=True)
class XRSnapshot:
    sample: Optional[XRSample]
    conn_id: int  # id of the active connection (0: none yet)
    connected: bool
    reset_count: int  # bumped by recenter, session end, visibility loss and new connections
    rtt_ms: Optional[float]
    frame_reset_count: int = 0  # bumped only when the WebXR frame itself changes: recenter, session end


@dataclass(frozen=True)
class XRStats:
    hz: float
    interarrival_p95_ms: float
    rtt_ms: Optional[float]


class WebXRServer:
    def __init__(self, host: str = "127.0.0.1", port: int = 8000) -> None:
        self.host = host
        self.port = port
        self._lock = threading.Lock()
        self._sample: Optional[XRSample] = None
        self._conn_id = 0
        self._connected = False
        self._reset_count = 0
        self._frame_reset_count = 0
        self._rtt_ms: Optional[float] = None
        self._arrivals: Deque[float] = deque()
        self._thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._started = threading.Event()
        self._start_error: Optional[BaseException] = None

    # ---- public API (thread-safe) ----

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="webxr-server", daemon=True)
        self._thread.start()
        self._started.wait(timeout=5.0)
        if self._start_error is not None:
            raise RuntimeError(f"WebXR server failed to start on {self.host}:{self.port}") from self._start_error
        log.info("WebXR page at http://%s:%d", self.host, self.port)

    def stop(self) -> None:
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._loop.stop)

    def snapshot(self) -> XRSnapshot:
        with self._lock:
            return XRSnapshot(
                self._sample, self._conn_id, self._connected, self._reset_count, self._rtt_ms,
                self._frame_reset_count,
            )

    def stats(self) -> XRStats:
        now = time.monotonic()
        with self._lock:
            arrivals = [t for t in self._arrivals if now - t <= STATS_PERIOD_S]
            rtt = self._rtt_ms
        if len(arrivals) < 2:
            return XRStats(0.0, float("nan"), rtt)
        gaps = np.diff(arrivals) * 1e3
        hz = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])
        return XRStats(hz, float(np.percentile(gaps, 95)), rtt)

    # ---- server thread ----

    def _run(self) -> None:
        loop = asyncio.new_event_loop()
        self._loop = loop
        asyncio.set_event_loop(loop)
        runner = web.AppRunner(self._make_app(), access_log=None)
        try:
            loop.run_until_complete(runner.setup())
            loop.run_until_complete(web.TCPSite(runner, self.host, self.port).start())
        except BaseException as exc:  # e.g. port in use
            self._start_error = exc
            self._started.set()
            return
        loop.create_task(self._log_stats())
        self._started.set()
        try:
            loop.run_forever()
        finally:
            loop.run_until_complete(runner.cleanup())
            loop.close()

    def _make_app(self) -> web.Application:
        @web.middleware
        async def no_cache(request: web.Request, handler):
            resp = await handler(request)
            resp.headers["Cache-Control"] = "no-store"
            return resp

        app = web.Application(middlewares=[no_cache])
        app.router.add_get("/", lambda _: web.FileResponse(STATIC_DIR / "index.html"))
        app.router.add_get("/ws", self._ws_handler)
        app.router.add_static("/static/", STATIC_DIR)
        return app

    async def _ws_handler(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=2.0)
        await ws.prepare(request)
        with self._lock:
            self._conn_id += 1
            conn_id = self._conn_id
            self._connected = True
            self._reset_count += 1
            self._sample = None
            self._arrivals.clear()
        log.info("WebXR client %d connected from %s", conn_id, request.remote)
        last_seq = -1
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                recv_t = time.monotonic()
                try:
                    data = json.loads(msg.data)
                    kind = data.get("type")
                except (ValueError, AttributeError):
                    continue
                if kind == "pose":
                    seq = int(data["seq"])
                    if seq <= last_seq:
                        continue
                    last_seq = seq
                    sample = XRSample(
                        seq=seq,
                        t_client_ms=float(data["t"]),
                        pos=tuple(float(v) for v in data["pos"]),
                        rot=tuple(float(v) for v in data["rot"]),
                        trigger=float(data["trigger"]),
                        grip=float(data["grip"]),
                        a=bool(data["a"]),
                        b=bool(data["b"]),
                        recv_t=recv_t,
                        conn_id=conn_id,
                        stick=bool(data.get("stick", False)),
                        head_rot=tuple(float(v) for v in data["head"]) if data.get("head") else None,
                    )
                    with self._lock:
                        if conn_id == self._conn_id:
                            self._sample = sample
                            self._arrivals.append(recv_t)
                            while self._arrivals and recv_t - self._arrivals[0] > STATS_PERIOD_S:
                                self._arrivals.popleft()
                elif kind in ("reset", "recenter", "session_end"):
                    log.info("WebXR client %d: %s", conn_id, kind)
                    with self._lock:
                        if conn_id == self._conn_id:
                            self._reset_count += 1
                            if kind != "reset":
                                self._frame_reset_count += 1
                            self._sample = None
                elif kind == "ping":
                    if data.get("rtt") is not None:
                        with self._lock:
                            if conn_id == self._conn_id:
                                self._rtt_ms = float(data["rtt"])
                    await ws.send_str(json.dumps({"type": "pong", "id": data.get("id"), "t": data.get("t")}))
        except (KeyError, TypeError, ValueError) as exc:
            log.warning("WebXR client %d sent a malformed message, closing: %s", conn_id, exc)
        finally:
            with self._lock:
                if conn_id == self._conn_id:
                    self._connected = False
                    self._sample = None
                    self._rtt_ms = None
                    self._arrivals.clear()
            log.info("WebXR client %d disconnected", conn_id)
        return ws

    async def _log_stats(self) -> None:
        while True:
            await asyncio.sleep(STATS_PERIOD_S)
            snap = self.snapshot()
            if not snap.connected:
                continue
            s = self.stats()
            rtt = f"{s.rtt_ms:.1f} ms" if s.rtt_ms is not None else "n/a"
            log.info("WebXR %.1f Hz, inter-arrival p95 %.1f ms, RTT %s", s.hz, s.interarrival_p95_ms, rtt)
