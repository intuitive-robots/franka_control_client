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
        name (str): The name of the remote device.
    """

    def __init__(self, name: str) -> None:
        """
        Initialize the remote device.

        Args:
            name (str): The name of the remote device.
        """
        self._name = name
