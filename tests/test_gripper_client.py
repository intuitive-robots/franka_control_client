# # tests/test_franka_client.py
# import struct
# import socket
# import unittest
# from collections import deque

# from franka_control_client.franka_robot import (
#     RemoteFranka,
#     FrankaArmRequestID,
#     ControlMode,
#     FrankaArmState,
# )
# from franka_control_client.core.exception import CommandError


# # =========================================================
# # Fake socket to simulate robot server
# # =========================================================
# class FakeSocket:
#     """Fake socket for testing RemoteFranka communication."""

#     def __init__(self, frames=None):
#         self._frames = deque(frames or [])
#         self.sent = []

#     def send(self, data: bytes):
#         self.sent.append(data)

#     def recv(self, flags=0):ythonp
#         if not self._frames:
#             raise RuntimeError("No frames left in FakeSocket")
#         return self._frames.popleft()

#     def connect(self, *_a, **_kw):
#         pass

#     def setsockopt(self, *_a, **_kw):
#         pass

#     def close(self):
#         pass


# # =========================================================
# # Helper to create a fake "header + payload" frame
# # =========================================================
# def _make_frame(msg_id: int, payload: bytes = b""):
#     """Simulate a packed frame: msg_id (1B) + length (2B) + payload."""
#     header = struct.pack("!BHx", msg_id, len(payload))
#     return header + payload


# # =========================================================
# # Tests
# # =========================================================
# class TestRemoteFranka(unittest.TestCase):
#     def _instantiate(self, fake_socket: FakeSocket) -> RemoteFranka:
#         """Instantiate RemoteFranka without running __init__()."""
#         rf: RemoteFranka = RemoteFranka.__new__(RemoteFranka)
#         rf._sock_req = fake_socket
#         rf._device_addr = "127.0.0.1"
#         rf._device_port = 9000
#         rf._state_buffer = deque()
#         return rf

#     # ---------------------------
#     # get_state()
#     # ---------------------------
#     def test_get_state_success(self):
#         # Construct a fake 636B state payload
#         dummy_state = FrankaArmState(
#             timestamp_ms=123456,
#             O_T_EE=(0.1,) * 16,
#             O_T_EE_d=(0.2,) * 16,
#             q=(1.1,) * 7,
#             q_d=(1.2,) * 7,
#             dq=(1.3,) * 7,
#             dq_d=(1.4,) * 7,
#             tau_ext_hat_filtered=(1.5,) * 7,
#             O_F_ext_hat_K=(2.1,) * 6,
#             K_F_ext_hat_K=(2.2,) * 6,
#         ).to_bytes()

#         # Simulate a proper response
#         fake_frame = _make_frame(FrankaArmRequestID.GET_FRANKA_ARM_STATE, dummy_state)
#         fake_sock = FakeSocket([fake_frame])
#         rf = self._instantiate(fake_sock)

#         # Monkeypatch request() to return (header, payload)
#         rf.request = lambda _id, _payload: (None, dummy_state)

#         result = rf.get_state()

#         self.assertIsInstance(result, FrankaArmState)
#         self.assertEqual(result.timestamp_ms, 123456)
#         self.assertAlmostEqual(result.q[0], 1.1)

#     # ---------------------------
#     # set_control_mode()
#     # ---------------------------
#     def test_set_control_mode_requires_ip(self):
#         rf = self._instantiate(FakeSocket())
#         rf.request = lambda *_a, **_kw: (None, 0)
#         with self.assertRaises(ValueError):
#             rf.set_control_mode(ControlMode.JOINT_POSITION)

#     def test_set_control_mode_sends_payload(self):
#         ip = "192.168.1.2"
#         port = 4000
#         fake_sock = FakeSocket()
#         rf = self._instantiate(fake_sock)
#         rf.request = lambda _id, payload: (None, 0)

#         rf.set_control_mode(
#             ControlMode.JOINT_POSITION, controller_ip=ip, controller_port=port
#         )

#         # Verify payload
#         payload = bytearray([ControlMode.JOINT_POSITION])
#         payload += socket.inet_aton(ip)
#         payload += struct.pack("!H", port)

#         expected = (FrankaArmRequestID.SET_FRANKA_ARM_CONTROL_MODE, bytes(payload))
#         self.assertEqual(expected[0], FrankaArmRequestID.SET_FRANKA_ARM_CONTROL_MODE)

#     # ---------------------------
#     # move_to_position()
#     # ---------------------------
#     def test_move_to_position_invalid_length(self):
#         rf = self._instantiate(FakeSocket())
#         with self.assertRaises(CommandError):
#             rf.move_to_position((1.0, 2.0))  # not 16 numbers

#     def test_move_to_position_valid(self):
#         rf = self._instantiate(FakeSocket())
#         rf.request = lambda *_a, **_kw: (None, b"")
#         pose = tuple(float(i) for i in range(16))
#         rf.move_to_position(pose)  # should not raise

#     # ---------------------------
#     # default pose
#     # ---------------------------
#     def test_set_default_pose_and_move(self):
#         rf = self._instantiate(FakeSocket())
#         dummy_state = FrankaArmState(
#             timestamp_ms=0,
#             O_T_EE=(0.1,) * 16,
#             O_T_EE_d=(0.2,) * 16,
#             q=(0.0,) * 7,
#             q_d=(0.0,) * 7,
#             dq=(0.0,) * 7,
#             dq_d=(0.0,) * 7,
#             tau_ext_hat_filtered=(0.0,) * 7,
#             O_F_ext_hat_K=(0.0,) * 6,
#             K_F_ext_hat_K=(0.0,) * 6,
#         )
#         rf.get_state = lambda: dummy_state
#         rf.request = lambda *_a, **_kw: (None, b"")

#         rf.set_default_pose()
#         self.assertEqual(rf.default_pose, dummy_state.O_T_EE_d)

#         rf.move_to_default_pose()  # should call move_to_position internally


# if __name__ == "__main__":
#     unittest.main()
