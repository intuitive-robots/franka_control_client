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
from typing import Final, Optional, Tuple

import zmq

from .exception import CommandError
from .message import MsgHeader, MsgID, RequestResultID


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
        except zmq.ZMQError:
            raise ConnectionError(
                f"Failed to connect to "
                f"tcp://{self._device_addr}:{self._device_port}"
            )

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
        self, msg_id: MsgID, payload: bytes, timeout: int = 2
    ) -> Tuple[Optional[MsgHeader], Optional[bytes]]:
        result = None
        header = MsgHeader(message_id=msg_id, payload_length=len(payload))
        message = header.to_bytes() + payload
        try:
            self._req_socket.send(message, copy=False)
            if self._req_socket.poll(timeout * 1000) & zmq.POLLIN:
                result = self._req_socket.recv()
            else:
                raise CommandError(
                    f"Timeout waiting for response (> {timeout}s)"
                )
            if result is None or len(result) < MsgHeader.SIZE:
                raise CommandError("Received empty or too short response")

        except Exception as e:
            print(
                f"Error when sending message from send_message function in "
                f"simpub.core.utils: {e}"
            )
            print_exc()
        finally:
            if result is None:
                return None, None
            response_header = MsgHeader.from_bytes(result)
            if response_header.message_id != RequestResultID.SUCCESS:
                print(
                    f"Received error response: "
                    f"MsgID={response_header.message_id}, "
                    f"Flags=0x{response_header.flags:02X}, "
                    f"Len={response_header.payload_length}, "
                    f"Ts={response_header.timestamp}, "
                    f"{result[MsgHeader.SIZE :].decode('utf-8')}"
                )
            return response_header, result[MsgHeader.SIZE :]
