class RemoteDeviceError(Exception):
    """Base exception for remote device operations."""


class ConnectionError(RemoteDeviceError):
    """Exception raised when connection fails."""


class DeviceNotReadyError(RemoteDeviceError):
    """Exception raised when device is not ready for operation."""


class CommandError(RemoteDeviceError):
    """Exception raised when command execution fails."""
