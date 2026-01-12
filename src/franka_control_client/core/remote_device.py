"""
Core module for remote device control.

This module contains the abstract base class RemoteDevice that defines
the common interface for all remote devices (robots, cameras, etc.).
"""

from abc import ABC
from dataclasses import dataclass
from traceback import print_exc
from typing import Final, List, Optional, Tuple
import time
import pyzlc

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

    def connect(self) -> None:
        """
        Connect to the remote device.

        Raises:
            DeviceConnectionError: If the connection fails.
        """
        # Check if the device is reachable for a few attempts
        for _ in range(5):
            if pyzlc.check_node_info(self._name):
                pyzlc.info("Connected to remote device '%s'.", self._name)
                print(pyzlc.check_node_info(self._name))
                return
            time.sleep(0.5)
        if not pyzlc.check_node_info(self._name):
            raise DeviceConnectionError(
                f"Failed to connect to device '{self._name}': Node not found."
            )
