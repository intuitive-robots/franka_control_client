"""Tiny stdlib HTTP server exposing the session to the browser dashboard.

GET  /                      dashboard page
GET  /api/status?since=&log_after=   state, metrics, incremental series, logs
GET  /api/config            current SessionConfig
POST /api/config            partial update (JSON)
POST /api/action/<name>     connect | disconnect | reconnect_policy | start | stop |
                            estop | home | save | discard | clear_error
GET  /api/episodes          saved rollouts of the current batch
GET  /api/camera/<name>     latest frame as JPEG (?w=<max width>)
"""

from __future__ import annotations

import json
import logging
import math
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict
from urllib.parse import parse_qs, urlparse

import numpy as np

from .session import ActionError, InferenceSession

LOG = logging.getLogger(__name__)
STATIC_DIR = Path(__file__).parent / "static"


def _clean(obj: Any) -> Any:
    """Make JSON browser-safe: NaN/inf -> null, numpy -> python."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {str(k): _clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_clean(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return _clean(obj.tolist())
    if isinstance(obj, (np.floating,)):
        return _clean(float(obj))
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.bool_,)):
        return bool(obj)
    return obj


def make_handler(session: InferenceSession):
    class Handler(BaseHTTPRequestHandler):
        server_version = "FrankaInferenceGUI/1.0"

        def log_message(self, fmt: str, *args: Any) -> None:  # silence per-request logs
            LOG.debug("%s - %s", self.address_string(), fmt % args)

        # ---------------------------------------------------------- helpers
        def _send_json(self, payload: Any, status: int = 200) -> None:
            body = json.dumps(_clean(payload), allow_nan=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_bytes(self, body: bytes, content_type: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _read_json(self) -> Dict[str, Any]:
            length = int(self.headers.get("Content-Length") or 0)
            if length == 0:
                return {}
            data = json.loads(self.rfile.read(length).decode("utf-8"))
            if not isinstance(data, dict):
                raise ActionError("Expected a JSON object.")
            return data

        # ------------------------------------------------------------ GET
        def do_GET(self) -> None:  # noqa: N802
            url = urlparse(self.path)
            query = parse_qs(url.query)
            try:
                if url.path in ("/", "/index.html"):
                    self._send_bytes((STATIC_DIR / "index.html").read_bytes(), "text/html; charset=utf-8")
                elif url.path == "/api/status":
                    since = float(query.get("since", ["0"])[0])
                    log_after = int(query.get("log_after", ["0"])[0])
                    self._send_json(session.status(since, log_after))
                elif url.path == "/api/config":
                    self._send_json(session.cfg.to_dict())
                elif url.path == "/api/episodes":
                    self._send_json({"batch_dir": str(session.batch_dir.resolve()), "episodes": session.episodes()})
                elif url.path.startswith("/api/camera/"):
                    self._camera(url.path[len("/api/camera/"):], query)
                else:
                    self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as exc:
                LOG.exception("GET %s failed", self.path)
                self._send_json({"error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        def _camera(self, name: str, query: Dict[str, Any]) -> None:
            import cv2

            frame = session.camera_frame(name)
            if frame is None:
                self._send_json({"error": f"camera {name!r} not available"}, HTTPStatus.NOT_FOUND)
                return
            max_w = int(query.get("w", ["480"])[0])
            h, w = frame.shape[:2]
            if w > max_w > 0:
                frame = cv2.resize(frame, (max_w, int(h * max_w / w)), interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                raise RuntimeError("JPEG encoding failed")
            self._send_bytes(buf.tobytes(), "image/jpeg")

        # ----------------------------------------------------------- POST
        def do_POST(self) -> None:  # noqa: N802
            url = urlparse(self.path)
            try:
                payload = self._read_json()
                if url.path == "/api/config":
                    cfg = session.update_config(payload)
                    self._send_json({"ok": True, "config": cfg.to_dict()})
                elif url.path.startswith("/api/action/"):
                    result = session.do(url.path[len("/api/action/"):], payload)
                    self._send_json({"ok": True, **result})
                else:
                    self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            except ActionError as exc:
                self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.CONFLICT)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as exc:
                LOG.exception("POST %s failed", self.path)
                self._send_json({"ok": False, "error": f"{type(exc).__name__}: {exc}"}, HTTPStatus.INTERNAL_SERVER_ERROR)

    return Handler


def serve(session: InferenceSession, host: str, port: int) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((host, port), make_handler(session))
    server.daemon_threads = True
    return server
