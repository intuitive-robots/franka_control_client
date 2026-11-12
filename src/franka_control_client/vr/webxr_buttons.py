"""Quest buttons as data-collection keys.

A starts an episode ('n'), B saves it ('s'), X (left) deletes the running or the just-saved
episode ('x'), Y (left) homes the robot ('r').

Reads the server's cumulative press counters, so a tap is not missed while the collection
loop is busy, and every press is handled once. Presses made before the first poll are ignored.
"""

from typing import Dict, List, Optional

from .webxr_server import WebXRServer

DEFAULT_KEYS: Dict[str, str] = {"a": "n", "b": "s", "x": "x", "y": "r"}
BUTTONS = ("a", "b", "x", "y")


class WebXRButtons:
    def __init__(self, server: WebXRServer, keys: Optional[Dict[str, str]] = None) -> None:
        self.server = server
        self.keys = dict(DEFAULT_KEYS if keys is None else keys)
        self._seen = self._counts()

    def _counts(self) -> Dict[str, int]:
        snap = self.server.snapshot()
        return {button: getattr(snap, f"{button}_presses") for button in BUTTONS}

    def poll(self) -> List[str]:
        out: List[str] = []
        for button, count in self._counts().items():
            if count > self._seen[button] and button in self.keys:
                out.append(self.keys[button])  # several presses since the last poll act once
            self._seen[button] = count
        return out
