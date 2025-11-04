from __future__ import annotations

from dataclasses import dataclass

from ..core.remote_device import RemoteDevice


@dataclass
class CameraState:
    pass


class CameraDevice(RemoteDevice):
    def __init__(self, device_addr: str, device_port: int):
        super().__init__(device_addr, device_port)
