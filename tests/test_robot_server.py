import socket
import struct
import unittest
from collections import deque
from typing import Deque, List, Optional

from franka_control_client.device.robot import franka as rf_mod
from franka_control_client.core.exception import CommandError

_HEADER_STRUCT = rf_mod._HEADER_STRUCT
_STATE_STRUCT = rf_mod._STATE_STRUCT
_STATE_SIZE = rf_mod._STATE_SIZE
MsgID = rf_mod.MsgID
ControlMode = rf_mod.ControlMode
RemoteFranka = rf_mod.RemoteFranka


class _FakeSocket:
    """Fake socket used for unit‑testing."""

    def __init__(self, frames: Optional[List[bytes]] = None) -> None:
        self._frames: Deque[bytes] = deque(frames or [])
        self.sent: list[bytes] = []
        self.closed = False

    def send(self, data: bytes) -> None:
        self.sent.append(data)

    def recv(self, flags: int = 0) -> bytes:
        try:
            return self._frames.popleft()
        except IndexError as exc:
            raise RuntimeError("No more frames queued for FakeSocket.recv()") from exc

    def connect(self, *_a, **_kw) -> None:
        pass

    def setsockopt(self, *_a, **_kw) -> None:
        pass

    def close(self) -> None:
        self.closed = True


def _make_frame(msg_id: int, payload: bytes = b"") -> bytes:
    """Return header+payload frame complying with the custom protocol."""
    return _HEADER_STRUCT.pack(msg_id, len(payload)) + payload


def _instantiate_rf(fake_socket: _FakeSocket) -> RemoteFranka:
    """Instantiate *RemoteFranka* without running its real __init__."""
    rf: RemoteFranka = RemoteFranka.__new__(RemoteFranka)
    rf._sock_req = fake_socket  # type: ignore[attr-defined]
    rf._device_addr = "127.0.0.1"  # type: ignore[attr-defined]
    rf._ctx = None  # type: ignore[attr-defined]
    rf._state_buffer = deque()  # type: ignore[attr-defined]
    rf._sub_sock = None  # type: ignore[attr-defined]
    rf._sub_thread = None  # type: ignore[attr-defined]
    rf._listen_flag = None  # type: ignore[attr-defined]
    return rf


class TestRemoteFranka(unittest.TestCase):

    def test_recv_expect_success(self) -> None:
        frame = _make_frame(MsgID.GET_STATE_RESP, b"")
        fake_sock = _FakeSocket([frame])
        rf = _instantiate_rf(fake_sock)

        payload = rf._recv_expect(MsgID.GET_STATE_RESP)
        self.assertEqual(payload, b"")

    def test_recv_expect_length_mismatch_raises(self) -> None:
        payload = b"x"
        frame = _HEADER_STRUCT.pack(MsgID.GET_STATE_RESP, 2) + payload
        fake_sock = _FakeSocket([frame])
        rf = _instantiate_rf(fake_sock)

        with self.assertRaises(CommandError):
            rf._recv_expect(MsgID.GET_STATE_RESP)

    def test_recv_expect_error_code_raises(self) -> None:
        frame = _make_frame(MsgID.ERROR, b"\x13")
        fake_sock = _FakeSocket([frame])
        rf = _instantiate_rf(fake_sock)

        with self.assertRaises(CommandError):
            rf._recv_expect(MsgID.GET_STATE_RESP)


class TestRemoteFrankaAPI(unittest.TestCase):

    @staticmethod
    def _dummy_state_bytes() -> bytes:
        values = [
            123456,  # timestamp
            *([0.1] * 16),  # O_T_EE
            *([0.2] * 16),  # O_T_EE_d
            *([0.3] * 7),  # q
            *([0.4] * 7),  # q_d
            *([0.5] * 7),  # dq
            *([0.6] * 7),  # dq_d
            *([0.7] * 7),  # tau_ext_hat_filtered
            *([0.8] * 6),  # O_F_ext_hat_K
        ] + [
            0.9
        ] * 6  # K_F_ext_hat_K
        return _STATE_STRUCT.pack(*values)

    def test_get_state_decodes_payload_and_sends_request(
        self,
    ) -> None:
        state_bytes = self._dummy_state_bytes()
        resp_frame = _make_frame(MsgID.GET_STATE_RESP, state_bytes)
        fake_sock = _FakeSocket([resp_frame])
        rf = _instantiate_rf(fake_sock)

        state = rf.get_state()

        self.assertEqual(state.timestamp_ms, 123456)
        self.assertAlmostEqual(state.q[0], 0.3)
        self.assertEqual(len(fake_sock.sent), 1)

        expected_header = _HEADER_STRUCT.pack(MsgID.GET_STATE_REQ, 0)
        self.assertTrue(fake_sock.sent[0].startswith(expected_header))

    def test_query_state_returns_enum(self) -> None:
        resp_frame = _make_frame(
            MsgID.QUERY_STATE_RESP, bytes([ControlMode.JOINT_POSITION])
        )
        fake_sock = _FakeSocket([resp_frame])
        rf = _instantiate_rf(fake_sock)

        mode = rf.query_state()
        self.assertEqual(mode, ControlMode.JOINT_POSITION)

    def test_start_control_requires_ip_port(self) -> None:
        fake_sock = _FakeSocket([_make_frame(MsgID.START_CONTROL_RESP, b"\x00")])
        rf = _instantiate_rf(fake_sock)

        with self.assertRaises(ValueError):
            rf.start_control(ControlMode.CARTESIAN_POSITION)

    def test_start_control_payload_and_success(self) -> None:
        ip = "192.168.0.2"
        port = 4000
        payload = bytearray([ControlMode.JOINT_POSITION])
        payload += socket.inet_aton(ip)
        payload += struct.pack("!H", port)
        fake_resp = _make_frame(MsgID.START_CONTROL_RESP, b"\x00")
        fake_sock = _FakeSocket([fake_resp])
        rf = _instantiate_rf(fake_sock)

        rf.start_control(
            ControlMode.JOINT_POSITION, controller_ip=ip, controller_port=port
        )

        self.assertEqual(len(fake_sock.sent), 1)
        expected_header = _HEADER_STRUCT.pack(MsgID.START_CONTROL_REQ, len(payload))
        self.assertEqual(fake_sock.sent[0], expected_header + payload)


if __name__ == "__main__":
    unittest.main()
