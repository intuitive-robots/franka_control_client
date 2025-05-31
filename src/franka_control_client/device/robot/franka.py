from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import IntEnum
from typing import Deque, Final, List, Optional, Tuple
import socket
import struct
import threading
import time

import zmq

from ...core.remote_device import RemoteDevice
from ...core.exception import CommandError


_HEADER_STRUCT: Final = struct.Struct("!BHx")  # uint8 id | uint16 len | pad
_HEADER_SIZE: Final = _HEADER_STRUCT.size

_STATE_STRUCT: Final = struct.Struct(
    "!I"  # timestamp [ms]                             –   4 B
    "16d16d"  # O_T_EE, O_T_EE_d                           – 256 B
    "7d7d7d7d7d"  # q, q_d, dq, dq_d, tau_ext_hat_filt         – 280 B
    "6d6d"  # O_F_ext_hat_K, K_F_ext_hat_K               –  96 B
)
_STATE_SIZE: Final = _STATE_STRUCT.size


class MsgID(IntEnum):
    """Message identifiers"""

    GET_STATE_REQ = 0x01
    QUERY_STATE_REQ = 0x02
    START_CONTROL_REQ = 0x03
    GET_SUB_PORT_REQ = 0x04

    GET_STATE_RESP = 0x51
    QUERY_STATE_RESP = 0x52
    START_CONTROL_RESP = 0x53
    GET_SUB_PORT_RESP = 0x54

    ERROR = 0xFF


class ControlMode(IntEnum):
    """Control modes supported by the robot."""

    CARTESIAN_POSITION = 0
    CARTESIAN_VELOCITY = 1
    JOINT_POSITION = 2
    JOINT_VELOCITY = 3
    HUMAN_MODE = 4


@dataclass(frozen=True)
class FrankaArmState:
    """The robot state."""

    timestamp_ms: int

    O_T_EE: Tuple[float, ...]  # actual end-effector pose (4 × 4 matrix, row major)
    O_T_EE_d: Tuple[float, ...]  # set-point pose

    q: Tuple[float, ...]  # measured joint positions [rad]
    q_d: Tuple[float, ...]  # set-point joint positions [rad]
    dq: Tuple[float, ...]  # measured joint velocities [rad s⁻¹]
    dq_d: Tuple[float, ...]  # set-point joint velocities [rad s⁻¹]

    tau_ext_hat_filtered: Tuple[float, ...]  # external torque estimate [N m]
    O_F_ext_hat_K: Tuple[float, ...]  # Cartesian force estimate [N, N m]
    K_F_ext_hat_K: Tuple[float, ...]  # desired wrench [N, N m]


class RemoteFranka(RemoteDevice):
    """
    RemoteFranka class for controlling a Franka robot.

    This class extends the RemoteDevice class and provides
    specific functionality for interacting with a Franka robot.
    Attributes:
        device_addr (str): Address of the Franka robot.
        device_port (int): Port for communication with the Franka robot.
    """

    def __init__(self, device_addr: str, device_port: int) -> None:
        """
        Initialize the RemoteFranka instance.

        Args:
            device_addr (str): Address of the Franka robot.
            device_port (int): Port for communication with the Franka robot.
        """
        super().__init__(device_addr, device_port)
        self._device_addr: str = device_addr
        self._ctx: zmq.Context = zmq.Context.instance()

        self._sock_req: zmq.Socket = self._ctx.socket(zmq.REQ)
        self._sock_req.connect(f"tcp://{device_addr}:{device_port}")

        self._state_buffer: Deque[FrankaArmState] = deque()
        self._sub_sock: Optional[zmq.Socket] = None
        self._sub_thread: Optional[threading.Thread] = None
        self._listen_flag: threading.Event = threading.Event()

    def close(self) -> None:
        self.stop_state_listener()
        if not self._sock_req.closed:
            self._sock_req.close()

    def get_state(self) -> FrankaArmState:
        """Return a single state sample"""
        self._send(MsgID.GET_STATE_REQ, b"")
        payload = self._recv_expect(MsgID.GET_STATE_RESP)
        return self._decode_state(payload)

    def query_state(self) -> ControlMode:
        """Return the currently active control mode."""
        self._send(MsgID.QUERY_STATE_REQ, b"")
        payload = self._recv_expect(MsgID.QUERY_STATE_RESP)
        return ControlMode(payload[0])

    def get_sub_port(self) -> int:
        """Ask the Server on which TCP port it publishes state updates."""
        self._send(MsgID.GET_SUB_PORT_REQ, b"")
        payload = self._recv_expect(MsgID.GET_SUB_PORT_RESP)

        if len(payload) != 2:
            raise CommandError("SUB-port payload must be exactly 2 bytes")

        (port,) = struct.unpack("!H", payload)
        return port

    def start_control(
        self,
        mode: ControlMode,
        *,
        controller_ip: Optional[str] = None,
        controller_port: Optional[int] = None,
        subscribe_server: bool = False,
    ) -> None:
        """
        Switch the robot into *mode*.

        For any mode **other than** :pyattr:`ControlMode.HUMAN_MODE` you *must*
        provide ``controller_ip`` and ``controller_port`` – these parameters
        tell the robot where your *SUB* socket lives.  If ``subscribe_server``
        is *True* we start a background thread that subscribes to
        state updates.
        """
        payload = bytearray([mode.value])

        if mode != ControlMode.HUMAN_MODE:
            if controller_ip is None or controller_port is None:
                raise ValueError("Controller_ip and Controller_port required")

            try:
                payload += socket.inet_aton(controller_ip)
            except OSError as exc:
                raise ValueError(f"Invalid IPv4 address: {controller_ip}") from exc

            payload += struct.pack("!H", controller_port)

        self._send(MsgID.START_CONTROL_REQ, payload)
        status = self._recv_expect(MsgID.START_CONTROL_RESP)[0]
        if status != 0:
            raise CommandError(f"START_CONTROL failed (status={status})")
        if subscribe_server:
            self.start_state_listener()

    def _send(self, msg_id: MsgID, payload: bytes) -> None:
        """Serialize *msg_id* + *payload* and send it to the Server."""
        self._sock_req.send(_HEADER_STRUCT.pack(msg_id, len(payload)) + payload)

    def _recv_expect(self, expected_id: MsgID) -> bytes:
        """
        Receive one frame and validate it against *expected* MsgID.

        Raises:
            CommandError: On checksum/length mismatch or if the controller reports an error.
        """
        raw = self._sock_req.recv()
        if len(raw) < _HEADER_SIZE:
            raise CommandError("Frame too short")

        msg_id, length = _HEADER_STRUCT.unpack_from(raw, 0)
        payload = raw[_HEADER_SIZE:]

        if len(payload) != length:
            raise CommandError(f"Payload length mismatch ({len(payload)} ≠ {length})")

        if msg_id == MsgID.ERROR:
            code = payload[0] if payload else 0xFF
            raise CommandError(f"Server returned error code 0x{code:02X}")

        if msg_id != expected_id:
            raise CommandError(
                f"Unexpected Msg-ID 0x{msg_id:02X} (expected 0x{expected_id:02X})"
            )

        return payload

    def start_state_listener(
        self,
        *,
        port: Optional[int] = None,
        buffer_size: int = 2048,
    ) -> None:
        if self._sub_thread and self._sub_thread.is_alive():
            return

        if port is None:
            port = self.get_sub_port()

        self._state_buffer = deque(maxlen=buffer_size)
        sock: zmq.Socket = self._ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        sock.connect(f"tcp://{self._device_addr}:{port}")
        self._sub_sock = sock

        self._listen_flag.set()

        def _worker() -> None:
            while self._listen_flag.is_set():
                raw = sock.recv()
                try:
                    state = self._decode_state(raw)
                    self._state_buffer.append(state)
                except CommandError:
                    continue

        self._sub_thread = threading.Thread(target=_worker, daemon=True)
        self._sub_thread.start()

    def stop_state_listener(self) -> None:
        if not self._sub_thread:
            return
        self._listen_flag.clear()
        self._sub_thread.join(timeout=1.0)
        self._sub_thread = None
        if self._sub_sock and not self._sub_sock.closed:
            self._sub_sock.close()
            self._sub_sock = None

    def get_state_buffer(self) -> List[FrankaArmState]:
        """Return a *copy* of the internal state buffer."""
        return list(self._state_buffer)

    def latest_state(self) -> Optional[FrankaArmState]:
        """Return the most recent *RobotState* or *None* if buffer is empty."""
        return self._state_buffer[-1] if self._state_buffer else None

    @staticmethod
    def _decode_state(buf: bytes) -> FrankaArmState:
        """Convert the bytes into a :class:`RobotState` instance."""
        if len(buf) != _STATE_SIZE:
            raise CommandError("RobotState payload size mismatch")
        values = _STATE_STRUCT.unpack(buf)
        i = 0

        ts = values[i]
        i += 1
        O_T_EE = values[i : i + 16]
        i += 16
        O_T_EE_d = values[i : i + 16]
        i += 16
        q = values[i : i + 7]
        i += 7
        q_d = values[i : i + 7]
        i += 7
        dq = values[i : i + 7]
        i += 7
        dq_d = values[i : i + 7]
        i += 7
        tau_ext_hat = values[i : i + 7]
        i += 7
        O_F_ext_hat_K = values[i : i + 6]
        i += 6
        K_F_ext_hat_K = values[i : i + 6]

        return FrankaArmState(
            timestamp_ms=ts,
            O_T_EE=O_T_EE,
            O_T_EE_d=O_T_EE_d,
            q=q,
            q_d=q_d,
            dq=dq,
            dq_d=dq_d,
            tau_ext_hat_filtered=tau_ext_hat,
            O_F_ext_hat_K=O_F_ext_hat_K,
            K_F_ext_hat_K=K_F_ext_hat_K,
        )
