class RemoteDeviceError(Exception):
    """Base exception for remote device operations."""


class DeviceConnectionError(RemoteDeviceError):
    """Exception raised when connection fails."""


class DeviceNotReadyError(RemoteDeviceError):
    """Exception raised when device is not ready for operation."""


class CommandError(RemoteDeviceError):
    """Exception raised when command execution fails."""


class MessageError(RemoteDeviceError):
    """Exception raised when message processing fails."""
