from __future__ import annotations

import threading
import sys
import select
import tty
import termios
from typing import Callable, List


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


class VoidEvent:
    """An event that takes no arguments.

    Usage:
        on_connected: VoidEvent = VoidEvent()

        def handler():
            print("Connected!")

        on_connected += handler
        on_connected.emit()
    """

    def __init__(self) -> None:
        self._handlers: List[Callable[[], None]] = []

    def subscribe(self, handler: Callable[[], None]) -> None:
        """Subscribe a handler to this event."""
        if handler not in self._handlers:
            self._handlers.append(handler)

    def unsubscribe(self, handler: Callable[[], None]) -> None:
        """Unsubscribe a handler from this event."""
        if handler in self._handlers:
            self._handlers.remove(handler)

    def emit(self) -> None:
        """Emit the event, notifying all subscribed handlers."""
        for handler in self._handlers[:]:
            handler()

    def clear(self) -> None:
        """Remove all subscribed handlers."""
        self._handlers.clear()

    def __iadd__(self, handler: Callable[[], None]) -> VoidEvent:
        """Subscribe using += operator."""
        self.subscribe(handler)
        return self

    def __isub__(self, handler: Callable[[], None]) -> VoidEvent:
        """Unsubscribe using -= operator."""
        self.unsubscribe(handler)
        return self

    def __call__(self) -> None:
        """Allow calling the event directly to emit."""
        self.emit()

    def __len__(self) -> int:
        """Return the number of subscribed handlers."""
        return len(self._handlers)

    def __bool__(self) -> bool:
        """Return True if there are any subscribed handlers."""
        return len(self._handlers) > 0
