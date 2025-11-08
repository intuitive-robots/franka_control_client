"""
Defines message identifiers used in the communication protocol.
"""

from __future__ import annotations

import struct
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import IntEnum
from typing import ClassVar


class MsgID(IntEnum):
    """Abstract Identifiers for different message types."""


class RequestResultID(MsgID):
    """Identifiers for request result messages."""

    SUCCESS = 200
    FAIL = 201
    INVALID_ARG = 202
    BUSY = 203
    UNSUPPORTED = 204
    TIMEOUT = 205
    COMM_ERROR = 206
    INTERNAL_ERROR = 207


class BinaryMsg(ABC):
    """Abstract base for any binary message that supports pack/unpack."""

    _STRUCT: ClassVar[struct.Struct]
    SIZE: ClassVar[int]

    @abstractmethod
    def to_bytes(self) -> bytes:
        """Pack this structure into bytes."""

    @classmethod
    @abstractmethod
    def from_bytes(cls, data: bytes):
        """Unpack structure from bytes."""


# =========================================================
# MsgHeader (12 bytes)
# =========================================================
@dataclass
class MsgHeader(BinaryMsg):
    """
    Represents the fixed 12-byte message header.

    Format: <BBHQ (little-endian)
      message_id (uint8)   : 1 byte  — Message ID (0-255)
      flags (uint8)        : 1 byte  — Bit flags (default: 0)
      payload_length (uint16): 2 bytes — Payload size in bytes
      timestamp (uint64)   : 8 bytes — Nanosecond timestamp (auto)
    """

    message_id: int
    payload_length: int
    flags: int = 0
    timestamp: int = field(default_factory=time.time_ns)

    _STRUCT = struct.Struct("<BBHQ")
    SIZE = _STRUCT.size  # 12 bytes

    def to_bytes(self) -> bytes:
        return self._STRUCT.pack(
            self.message_id,
            self.flags,
            self.payload_length,
            self.timestamp,
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "MsgHeader":
        if len(data) < cls.SIZE:
            raise ValueError(
                f"Data too short: expected {cls.SIZE} bytes, got {len(data)}"
            )
        message_id, flags, payload_length, timestamp = cls._STRUCT.unpack_from(
            data
        )
        return cls(message_id, payload_length, flags, timestamp)

    def __repr__(self):
        return (
            f"MsgHeader("
            f"id={self.message_id}, flags=0x{self.flags:02X}, "
            f"len={self.payload_length}, ts={self.timestamp})"
        )
