# from __future__ import annotations

# import struct
# from dataclasses import dataclass
# from typing import Final

# from ..core.exception import CommandError
# from ..core.message import MsgID
# from ..core.remote_device import RemoteDevice, State

# _STATE_STRUCT: Final = struct.Struct(
#     "!Idd?H"
# )  # timestamp, width, max_width, grasped, temperature
# _STATE_SIZE: Final = _STATE_STRUCT.size


# @dataclass(frozen=True)
# class GripperState(State):
#     """State of the gripper."""

#     width: float
#     max_width: float
#     grasped: bool
#     temperature: int


# class RemoteGripper(RemoteDevice):
#     """Remote client for a gripper device."""

#     # def __init__(self, device_addr: str, device_port: int) -> None:
#     #     super().__init__(device_addr, device_port)

#     def get_state(self) -> GripperState:
#         """Return a single state sample"""
#         self._send(MsgID.GET_STATE_REQ, b"")
#         payload = self._recv_expect(MsgID.GET_STATE_RESP)
#         return self._decode_state(payload)

#     def start_state_listener(
#         self,
#         *,
#         port: int | None = None,
#         buffer_size: int = 2048,
#         topic: bytes = b"franka_gripper",
#     ) -> None:
#         return super().start_state_listener(
#             port=port,
#             buffer_size=buffer_size,
#             topic=topic,
#         )

#     @staticmethod
#     def _decode_state(buf: bytes) -> GripperState:
#         if len(buf) != _STATE_SIZE:
#             raise CommandError("State payload size mismatch")
#         timestamp, width, max_width, grasped, temperature = (
#             _STATE_STRUCT.unpack(buf)
#         )
#         return GripperState(
#             timestamp_ms=timestamp,
#             width=width,
#             max_width=max_width,
#             grasped=grasped,
#             temperature=temperature,
#         )
