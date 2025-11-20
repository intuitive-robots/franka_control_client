"""
Core module for remote device control.

This module contains the abstract base class RemoteDevice that defines
the common interface for all remote devices (robots, cameras, etc.).
"""

import struct
import threading
from abc import ABC
from dataclasses import dataclass
from traceback import print_exc
from typing import Final, List, Optional, Tuple

import zmq

from .exception import CommandError, DeviceConnectionError


@dataclass(frozen=True)
class State:
    """State of the remote device."""

    timestamp_ms: int


class RemoteDevice(ABC):
    """
    Abstract base class for all remote devices.

    This class defines the common interface that all remote devices
    (robots, cameras, sensors, etc.) must implement.

    Attributes:
        device_addr (str): The address of the remote device.
        device_port (int): The port for communicating with the remote device.
    """

    _HEADER_STRUCT: Final = struct.Struct("!BHx")
    _HEADER_SIZE: Final = _HEADER_STRUCT.size

    def __init__(self, device_addr: str, device_port: int):
        """
        Initialize the remote device.

        Args:
            device_addr (str): The address of the remote device.
            device_port (int): The port for communicating with the remote device.
        """
        self._device_addr = device_addr
        self._device_port = device_port
        self._ctx: zmq.Context = zmq.Context.instance()

        self._req_socket: zmq.Socket = self._ctx.socket(zmq.REQ)
        self._listen_flag: threading.Event = threading.Event()

    def connect(self) -> None:
        """
        Connects to the server.
        """
        if self._req_socket is not None and not self._req_socket.closed:
            try:
                self._req_socket.close()
            except zmq.ZMQError:
                pass
        self._req_socket = self._ctx.socket(zmq.REQ)
        assert self._req_socket is not None
        try:
            self._req_socket.connect(
                f"tcp://{self._device_addr}:{self._device_port}"
            )
        except zmq.ZMQError as e:
            raise DeviceConnectionError(
                f"Failed to connect to "
                f"tcp://{self._device_addr}:{self._device_port}: {e}"
            ) from e

    def disconnect(self) -> None:
        """Disconnects from the server."""
        try:
            self._req_socket.disconnect(
                f"tcp://{self._device_addr}:{self._device_port}"
            )
        except zmq.ZMQError:
            pass

    # def get_state_buffer(self) -> List[State]:
    #     return list(self._state_buffer)

    # def latest_state(self) -> Optional[State]:
    #     return self._state_buffer[-1] if self._state_buffer else None

    def request(
        self, service_name: str, payload: bytes, timeout: int = 2
    ) -> Tuple[str, Optional[bytes]]:
        result: List[bytes] = []
        try:
            self._req_socket.send_multipart(
                [service_name.encode("utf-8"), payload]
            )
            if self._req_socket.poll(timeout * 1000) & zmq.POLLIN:
                result = self._req_socket.recv_multipart()
            else:
                raise CommandError(
                    f"Timeout waiting for response (> {timeout}s)"
                )
            if len(result) != 2:
                raise CommandError("Received empty or too short response")
        except Exception as e:
            print(
                f"Error when sending message from send_message function in "
                f"simpub.core.utils: {e}"
            )
            print_exc()
        if len(result) != 2:
            return "", None
        return result[0].decode("utf-8"), result[1]
