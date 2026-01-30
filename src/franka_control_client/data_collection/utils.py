import threading
from typing import Protocol
from ..camera.camera import CameraDevice
from ..franka_robot.franka_arm import RemoteFranka
from ..franka_robot.franka_gripper import RemoteFrankaGripper

import sys
import select
import tty
import termios


class UIConsole:
    """
    Manages terminal output by separating persistent logs from
    transient interactive hints using ANSI escape sequences.
    """

    def __init__(self):
        self._current_hint = ""
        self._lock = threading.Lock()

    def update_hint(self, message: str):
        """
        Updates the bottom interactive instruction.
        This will be overwritten by the next hint or pushed down by a log.
        """
        with self._lock:
            self._current_hint = message
            # \r: Carriage return (to start of line)
            # \033[K: Clear line from cursor to end
            sys.stdout.write(f"\r\033[K{self._current_hint}")
            sys.stdout.flush()

    def log(self, message: str):
        """
        Prints a persistent log message that scrolls upward.
        Automatically restores the current hint at the bottom.
        """
        with self._lock:
            # 1. Clear the current hint line
            sys.stdout.write("\r\033[K")
            # 2. Print the log message with a newline
            sys.stdout.write(f"{message}\n")
            # 3. Restore the hint at the bottom
            sys.stdout.write(self._current_hint)
            sys.stdout.flush()


class NonBlockingKeyPress(object):
    """
    This class was copied and adapted from: https://stackoverflow.com/a/10079805
    Note that this solution is sometimes confused when spamming a character and that there are problems with special characters such as arrow keys.
    """

    def __enter__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_data(self):
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            # Read one character
            data = sys.stdin.read(1)
            # Flush received but not read and written but not transmitted data
            # This does no fully fix that the input is confused if a key is spammed
            termios.tcflush(sys.stdin, termios.TCIOFLUSH)
            return data
        return False


class DataCollector(Protocol):

    def connect(self) -> None: ...

    def close(self) -> None: ...

    def save_step(self) -> None: ...

    def save_episode(self) -> None: ...

    def reset(self) -> None: ...

    def discard(self) -> None: ...


class ImageDataWrapper:
    def __init__(self, camera_device: CameraDevice) -> None:
        self.camera_device = camera_device

    def save_step(self) -> None:
        # Implement the logic to save image data from the camera device
        pass

    def save_episode(self) -> None:
        # Implement the logic to save the entire episode data from the camera device
        pass

    def __getattr__(self, name):
        return getattr(self.camera_device, name)


class RobotDataWrapper:
    def __init__(
        self, arm: RemoteFranka, gripper: RemoteFrankaGripper
    ) -> None:
        self.arm = arm
        self.gripper = gripper

    def save_step(self) -> None:
        # Implement the logic to save robot state data
        pass

    def save_episode(self) -> None:
        # Implement the logic to save the entire episode data from the robot device
        pass

    def __getattr__(self, name):
        return getattr(self.robot_device, name)
