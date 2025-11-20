# import random
# import struct
# import threading
# import time
# from typing import Tuple

# import zmq

# from franka_control_client.core.message import MsgHeader, RequestResultID
# from franka_control_client.franka_robot.franka_arm import (
#     ControlMode,
#     FrankaArmRequestID,
#     FrankaArmState,
# )


# def create_fake_franka_arm_state() -> FrankaArmState:
#     """Construct a fake FrankaArmState instance with random data."""

#     def rand_tuple(n):
#         return tuple(random.uniform(-1.0, 1.0) for _ in range(n))

#     return FrankaArmState(
#         timestamp_ms=int(time.time_ns() // 1_000_000) % (2**32),
#         O_T_EE=rand_tuple(16),
#         O_T_EE_d=rand_tuple(16),
#         q=rand_tuple(7),
#         q_d=rand_tuple(7),
#         dq=rand_tuple(7),
#         dq_d=rand_tuple(7),
#         tau_ext_hat_filtered=rand_tuple(7),
#         O_F_ext_hat_K=rand_tuple(6),
#         K_F_ext_hat_K=rand_tuple(6),
#     )


# def create_success_response(payload: bytes) -> Tuple[MsgHeader, bytes]:
#     """Create a success response header and payload."""
#     reply_header = MsgHeader(
#         message_id=RequestResultID.SUCCESS,
#         payload_length=len(payload),
#     )
#     return reply_header, payload


# def create_error_response(error_message: str) -> Tuple[MsgHeader, bytes]:
#     """Create an error response header and payload with a UTF-8 message."""
#     payload = error_message.encode("utf-8")
#     reply_header = MsgHeader(
#         message_id=RequestResultID.FAIL,
#         payload_length=len(payload),
#     )
#     return reply_header, payload


# class FakeFrankaServer:
#     """
#     Minimal ZMQ-based fake Franka robot server that responds to known MsgIDs.
#     """

#     def __init__(self, host="127.0.0.1", res_port=5555, pub_port=5556):
#         self.host = host
#         self.res_port = res_port
#         self.pub_port = pub_port
#         self._running = False
#         self._res_thread = None
#         self._pub_thread = None
#         self.ctx = zmq.Context.instance()
#         self._rep_socket: zmq.Socket = self.ctx.socket(zmq.REP)
#         self._rep_socket.bind(f"tcp://{self.host}:{self.res_port}")
#         print(f"[FakeFrankaServer] Running on {self.host}:{self.res_port}")
#         self._pub_socket: zmq.Socket = self.ctx.socket(zmq.PUB)
#         self._pub_socket.bind(f"tcp://{self.host}:{self.pub_port}")
#         print(f"[FakeFrankaServer] Publishing on {self.host}:{self.pub_port}")
#         self.control_mode: ControlMode = ControlMode.HUMAN_MODE

#     def start(self):
#         """Start the fake server in a background thread."""
#         self._running = True
#         self._res_thread = threading.Thread(target=self._serve, daemon=True)
#         self._res_thread.start()
#         self._pub_thread = threading.Thread(
#             target=self._publish_state_loop, daemon=True
#         )
#         self._pub_thread.start()
#         # Wait a bit to ensure socket bind is complete
#         time.sleep(0.1)

#     def stop(self):
#         """Stop the fake server."""
#         self._running = False
#         if self._res_thread:
#             self._res_thread.join(timeout=1)
#         if self._pub_thread:
#             self._pub_thread.join(timeout=1)
#         if self._rep_socket:
#             try:
#                 self._rep_socket.close(0)
#             except zmq.ZMQError:
#                 pass
#             self._rep_socket = None
#         if self._pub_socket:
#             try:
#                 self._pub_socket.close(0)
#             except zmq.ZMQError:
#                 pass
#             self._pub_socket = None

#     def _serve(self):
#         """Server main loop (ZMQ REP pattern)."""
#         while self._running and self._rep_socket is not None:
#             try:
#                 if self._rep_socket.poll(200) & zmq.POLLIN:
#                     message = self._rep_socket.recv()
#                     if len(message) < MsgHeader.SIZE:
#                         raise ValueError(
#                             "Received message too short to contain header"
#                         )
#                     header = MsgHeader.from_bytes(message)
#                     payload = message[MsgHeader.SIZE :]
#                     # Handle the message based on its ID
#                     header, response_payload = self._handle_message(
#                         header, payload
#                     )
#                     self._rep_socket.send(header.to_bytes() + response_payload)
#             except zmq.ZMQError:
#                 break
#         if self._rep_socket is not None:
#             self._rep_socket.close(0)
#         self._rep_socket = None

#     def _handle_message(
#         self, header: MsgHeader, payload: bytes
#     ) -> Tuple[MsgHeader, bytes]:
#         """Handle incoming messages based on their message_id."""
#         if header.message_id == FrankaArmRequestID.GET_FRANKA_ARM_STATE:
#             return self.handle_get_fake_franka_arm_state()
#         elif (
#             header.message_id == FrankaArmRequestID.SET_FRANKA_ARM_CONTROL_MODE
#         ):
#             return self.handle_set_franka_arm_control_mode(payload)
#         elif (
#             header.message_id
#             == FrankaArmRequestID.GET_FRANKA_ARM_STATE_PUB_PORT
#         ):
#             return self.handle_get_franka_arm_state_pub_port()
#         elif (
#             header.message_id == FrankaArmRequestID.GET_FRANKA_ARM_CONTROL_MODE
#         ):
#             return self.handle_get_franka_arm_control_mode()
#         else:
#             return create_error_response("Invalid message ID")

#     def handle_get_fake_franka_arm_state(self) -> Tuple[MsgHeader, bytes]:
#         """Send a fake FrankaArmState response."""
#         fake_state = create_fake_franka_arm_state().to_bytes()
#         return create_success_response(fake_state)

#     def handle_set_franka_arm_control_mode(
#         self, payload: bytes
#     ) -> Tuple[MsgHeader, bytes]:
#         """Handle setting the Franka arm control mode."""
#         # For simplicity, always return success
#         return create_success_response(self.control_mode.to_bytes(1))

#     def handle_get_franka_arm_state_pub_port(self):
#         """Send the publication port."""
#         payload = struct.pack("!H", self.pub_port)
#         return create_success_response(payload)

#     def handle_get_franka_arm_control_mode(self) -> Tuple[MsgHeader, bytes]:
#         """Send the current control mode."""
#         payload = struct.pack("!B", self.control_mode.value)
#         return create_success_response(payload)

#     def _publish_state_loop(self):
#         try:
#             while self._running and self._pub_socket is not None:
#                 payload = create_fake_franka_arm_state().to_bytes()
#                 header = MsgHeader(
#                     message_id=FrankaArmRequestID.GET_FRANKA_ARM_STATE,
#                     payload_length=len(payload),
#                 )
#                 try:
#                     self._pub_socket.send(header.to_bytes() + payload)
#                 except zmq.ZMQError:
#                     break
#                 time.sleep(0.001)
#         finally:
#             if self._pub_socket is not None:
#                 self._pub_socket.close(0)
#             self._pub_socket = None
