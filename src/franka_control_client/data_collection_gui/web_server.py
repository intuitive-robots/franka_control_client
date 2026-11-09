"""Stdlib HTTP server for the data collection dashboard.

GET  /                         page
GET  /api/status?log_after=    state, recording stats, robot, export/convert progress, logs
GET  /api/config               settings
GET  /api/episodes             episodes of the current task folder
GET  /api/camera/<name>?w=     live camera frame (JPEG)
GET  /api/fs?path=             folder browser
GET  /api/irl/scan?path=       IRL episodes below a folder
GET  /api/viz/episode?ep=      numeric series + video spans of an episode (open dataset)
GET  /api/viz/video?key=&chunk=&file=   dataset mp4 (HTTP Range, for seeking)
GET  /api/viz/frame?ep=&key=&i=&w=      one dataset frame as JPEG
POST /api/config               partial settings update
POST /api/action/<name>        see CollectionSession.do
"""

from __future__ import annotations

import json
import logging
import math
import re
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict
from urllib.parse import parse_qs, urlparse

import numpy as np

from .session import ActionError, CollectionSession, list_dir

LOG = logging.getLogger(__name__)
STATIC_DIR = Path(__file__).parent / "static"


def _clean(obj: Any) -> Any:
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {str(k): _clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_clean(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return _clean(obj.tolist())
    if isinstance(obj, np.floating):
        return _clean(float(obj))
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, np.bool_):
        return bool(obj)
    if isinstance(obj, Path):
        return str(obj)
    return obj


def make_handler(session: CollectionSession):
    class Handler(BaseHTTPRequestHandler):
        server_version = "FrankaDataCollectionGUI/1.0"
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt: str, *args: Any) -> None:
            LOG.debug("%s - %s", self.address_string(), fmt % args)

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

        def _send_file_range(self, path: Path, content_type: str) -> None:
            size = path.stat().st_size
            start, end = 0, size - 1
            m = re.match(r"bytes=(\d*)-(\d*)", self.headers.get("Range") or "")
            partial = bool(m)
            if m:
                if m.group(1):
                    start = int(m.group(1))
                    if m.group(2):
                        end = min(int(m.group(2)), size - 1)
                elif m.group(2):  # suffix range: last N bytes
                    start = max(size - int(m.group(2)), 0)
                if start > end or start >= size:
                    self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                    self.send_header("Content-Range", f"bytes */{size}")
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
            length = end - start + 1
            self.send_response(HTTPStatus.PARTIAL_CONTENT if partial else HTTPStatus.OK)
            self.send_header("Content-Type", content_type)
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Content-Length", str(length))
            if partial:
                self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            self.end_headers()
            with open(path, "rb") as f:
                f.seek(start)
                remaining = length
                while remaining > 0:
                    chunk = f.read(min(1 << 20, remaining))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    remaining -= len(chunk)

        def _read_json(self) -> Dict[str, Any]:
            length = int(self.headers.get("Content-Length") or 0)
            if length == 0:
                return {}
            data = json.loads(self.rfile.read(length).decode("utf-8"))
            if not isinstance(data, dict):
                raise ActionError("Expected a JSON object.")
            return data

        # ------------------------------------------------------------- GET
        def do_GET(self) -> None:  # noqa: N802
            url = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(url.query).items()}
            try:
                if url.path in ("/", "/index.html"):
                    self._send_bytes((STATIC_DIR / "index.html").read_bytes(), "text/html; charset=utf-8")
                elif url.path == "/api/status":
                    self._send_json(session.status(int(q.get("log_after", "0"))))
                elif url.path == "/api/config":
                    self._send_json(session.cfg.to_dict())
                elif url.path == "/api/episodes":
                    self._send_json(session.episodes())
                elif url.path.startswith("/api/camera/"):
                    self._camera(url.path[len("/api/camera/"):], int(q.get("w", "480")))
                elif url.path == "/api/fs":
                    self._send_json(list_dir(q.get("path", "")))
                elif url.path == "/api/irl/scan":
                    self._send_json(session.scan_irl(q.get("path", "")))
                elif url.path == "/api/viz/episode":
                    self._send_json(session.viewer_or_error().episode(int(q["ep"])))
                elif url.path == "/api/viz/video":
                    view = session.viewer_or_error()
                    self._send_file_range(view.video_path(q["key"], int(q["chunk"]), int(q["file"])), "video/mp4")
                elif url.path == "/api/viz/frame":
                    view = session.viewer_or_error()
                    jpeg = view.frame_jpeg(int(q["ep"]), q["key"], int(q.get("i", "0")), int(q.get("w", "640")))
                    self._send_bytes(jpeg, "image/jpeg")
                else:
                    self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except ActionError as exc:
                self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.CONFLICT)
            except (KeyError, FileNotFoundError) as exc:
                self._send_json({"ok": False, "error": f"not found: {exc}"}, HTTPStatus.NOT_FOUND)
            except Exception as exc:
                LOG.exception("GET %s failed", self.path)
                self._send_json({"ok": False, "error": f"{type(exc).__name__}: {exc}"}, HTTPStatus.INTERNAL_SERVER_ERROR)

        def _camera(self, name: str, max_w: int) -> None:
            import cv2

            frame = session.camera_frame(name)
            if frame is None:
                self._send_json({"error": f"camera {name!r} not available"}, HTTPStatus.NOT_FOUND)
                return
            h, w = frame.shape[:2]
            if w > max_w > 0:
                frame = cv2.resize(frame, (max_w, int(h * max_w / w)), interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                raise RuntimeError("JPEG encoding failed")
            self._send_bytes(buf.tobytes(), "image/jpeg")

        # ------------------------------------------------------------ POST
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


def serve(session: CollectionSession, host: str, port: int) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((host, port), make_handler(session))
    server.daemon_threads = True
    return server
