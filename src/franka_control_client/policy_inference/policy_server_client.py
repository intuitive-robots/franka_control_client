from __future__ import annotations

import pickle
import socket
import struct
import threading
from typing import Any, Optional


HEADER_STRUCT = struct.Struct("!Q")


def send_message(sock: socket.socket, payload: dict[str, Any]) -> None:
    data = pickle.dumps(payload, protocol=pickle.HIGHEST_PROTOCOL)
    sock.sendall(HEADER_STRUCT.pack(len(data)))
    sock.sendall(data)


def receive_message(sock: socket.socket) -> dict[str, Any]:
    header = _receive_exact(sock, HEADER_STRUCT.size)
    size = HEADER_STRUCT.unpack(header)[0]
    data = _receive_exact(sock, size)
    payload = pickle.loads(data)
    if not isinstance(payload, dict):
        raise TypeError(f"Expected dict payload, got {type(payload)!r}")
    return payload


def _receive_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError("Socket closed while receiving message.")
        chunks.extend(chunk)
    return bytes(chunks)


class PolicyServerClient:
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8765,
        timeout_s: float = 30.0,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def connect(self) -> None:
        if self._sock is not None:
            return
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        sock.settimeout(self.timeout_s)
        self._sock = sock

    def close(self) -> None:
        if self._sock is None:
            return
        try:
            self.request({"type": "close"})
        except Exception:
            pass
        try:
            self._sock.close()
        finally:
            self._sock = None

    def health(self) -> dict[str, Any]:
        return self.request({"type": "health"})

    def reset(self) -> dict[str, Any]:
        return self.request({"type": "reset"})

    def infer(self, observation: dict[str, Any]) -> dict[str, Any]:
        return self.request({"type": "infer", "observation": observation})

    def request(self, payload: dict[str, Any]) -> dict[str, Any]:
        self.connect()
        if self._sock is None:
            raise ConnectionError("Policy server socket is not connected.")
        with self._lock:
            send_message(self._sock, payload)
            response = receive_message(self._sock)
        if response.get("ok", False):
            return response
        error = response.get("error", "Unknown policy server error.")
        raise RuntimeError(str(error))
