"""WebSocket policy client — vendored from starVLA/deployment/model_server/tools/websocket_policy_client.py.

Kept here so franka_control_client has no dependency on the starVLA repo at runtime.
Communicates with a starVLA policy server started via:
    python deployment/model_server/server_policy.py --ckpt_path <ckpt> --port 10093
"""

import logging
import os
import time
from typing import Dict, Optional, Tuple

import websockets.sync.client

from . import msgpack_numpy


class WebsocketClientPolicy:
    """Sends inference requests to a starVLA policy server over WebSocket (msgpack protocol)."""

    def __init__(self, host: str = "127.0.0.1", port: Optional[int] = 10093) -> None:
        self._uri = f"ws://{host}"
        if port is not None:
            self._uri += f":{port}"
        self._packer = msgpack_numpy.Packer()
        self._ws, self._server_metadata = self._wait_for_server()

    def get_server_metadata(self) -> Dict:
        return self._server_metadata

    def _wait_for_server(self, timeout: float = 300) -> Tuple[websockets.sync.client.ClientConnection, Dict]:
        logging.info("Waiting for server at %s ...", self._uri)
        start_time = time.time()

        for k in ("HTTP_PROXY", "http_proxy", "HTTPS_PROXY", "https_proxy", "ALL_PROXY", "all_proxy"):
            os.environ.pop(k, None)

        while True:
            if time.time() - start_time > timeout:
                raise TimeoutError(f"Failed to connect to server within {timeout}s")
            try:
                conn = websockets.sync.client.connect(
                    self._uri,
                    compression=None,
                    max_size=None,
                    open_timeout=150,
                    ping_interval=None,
                    ping_timeout=60,
                )
                metadata = msgpack_numpy.unpackb(conn.recv())
                return conn, metadata
            except ConnectionRefusedError:
                logging.info("Still waiting for server %s ...", self._uri)
                time.sleep(2)

    def close(self) -> None:
        try:
            self._ws.close()
        except Exception:
            pass

    def predict_action(self, query_info: Dict) -> Dict:
        data = self._packer.pack(query_info)
        self._ws.send(data)
        response = self._ws.recv()
        if isinstance(response, str):
            raise RuntimeError(f"Error from inference server:\n{response}")
        return msgpack_numpy.unpackb(response)
