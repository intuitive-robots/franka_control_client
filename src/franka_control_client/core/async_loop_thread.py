from __future__ import annotations

import asyncio
import concurrent.futures
import threading
from abc import abstractmethod
from collections.abc import Coroutine
from typing import ClassVar, Optional

import zmq
from zmq.asyncio import Context as AsyncZMQContext
from zmq.asyncio import Socket as AsyncZMQSocket
import pyzlc

from .utils import get_socket_bind_url


class AsyncLoop:
    """A singleton class that runs an asyncio event loop in a separate thread."""

    _instance: Optional[AsyncLoop] = None
    _lock: ClassVar[threading.Lock] = threading.Lock()

    def __init__(self) -> None:
        self._loop_started = threading.Event()
        self.loop: asyncio.AbstractEventLoop = asyncio.new_event_loop()
        self.thread: threading.Thread = threading.Thread(
            target=self._run_loop, daemon=True
        )
        self._ctx: AsyncZMQContext = AsyncZMQContext.instance()
        self.thread.start()
        self._loop_started.wait(timeout=3)

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self.loop)
        self._loop_started.set()
        self.loop.run_forever()

    @classmethod
    def get_instance(cls) -> AsyncLoop:
        """Get the singleton instance of AsyncLoop."""
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def get_zmq_context(self) -> AsyncZMQContext:
        """Get an asyncio-compatible ZMQ context."""
        return self._ctx

    def submit_task(self, coro: Coroutine) -> concurrent.futures.Future:
        """Submit a coroutine to be executed on the loop thread."""
        return asyncio.run_coroutine_threadsafe(coro, self.loop)


class LatestMsgSubscriber:
    """A ZMQ SUB socket that keeps only the latest received message."""

    def __init__(self, topic_name: str) -> None:
        self.topic_name: str = topic_name
        pyzlc.register_sub

    def connect(self, url: str, topic: str = "") -> None:
        """Connect the subscriber and start receiving messages."""
        self.url = url
        self.topic = topic
        self.running = True
        self.future = self.loop.submit_task(self._run())

    async def _run(self) -> None:
        socket: AsyncZMQSocket = self.ctx.socket(zmq.SUB)
        socket.connect(self.url)
        socket.setsockopt_string(zmq.SUBSCRIBE, self.topic)
        print(
            f"[Subscriber] Connected to {self.url}, subscribed to '{self.topic}'"
        )
        while self.running:
            try:
                msg: bytes = await socket.recv()
                self.latest_bytes_message = msg
            except Exception as e:
                print(f"[Subscriber] Error on {self.url}: {e}")
                await asyncio.sleep(1)

    @abstractmethod
    def get_latest(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        """Stop the subscriber and cancel its running task."""
        self.running = False
        if self.future:
            self.future.cancel()
        print(f"[Subscriber] {self.url} stopped")


class CommandPublisher:
    """A ZMQ PUB socket for sending command messages."""

    def __init__(self, ip_addr) -> None:
        self.socket: zmq.Socket = zmq.Context.instance().socket(zmq.PUB)
        self.socket.bind(f"tcp://{ip_addr}:0")
        self.url = get_socket_bind_url(self.socket)

    def send_command(self, data: bytes) -> None:
        """Send a command message."""
        self.socket.send(data)
