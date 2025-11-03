from __future__ import annotations
from abc import abstractmethod
import asyncio
import threading
import concurrent.futures
import zmq
import zmq.asyncio
from typing import Optional, ClassVar
from collections.abc import Coroutine


class AsyncLoopThread:
    _instance: Optional[AsyncLoopThread] = None
    _lock: ClassVar[threading.Lock] = threading.Lock()

    def __init__(self) -> None:
        self.loop: asyncio.AbstractEventLoop = asyncio.new_event_loop()
        self.thread: threading.Thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    @classmethod
    def get_instance(cls) -> AsyncLoopThread:
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def submit_coroutine(self, coro: Coroutine) -> concurrent.futures.Future:
        """Submit a coroutine to be executed on the loop thread."""
        return asyncio.run_coroutine_threadsafe(coro, self.loop)


class LatestMessageSubscriber:
    """A ZMQ SUB socket that keeps only the latest received message."""

    def __init__(self, url: str, topic: str = "") -> None:
        self.url: str = url
        self.topic: str = topic
        self.ctx: zmq.asyncio.Context = zmq.asyncio.Context.instance()
        self.latest_bytes_message: Optional[bytes] = None
        self.running: bool = True

        self.loop_thread: AsyncLoopThread = AsyncLoopThread.get_instance()
        self.future: asyncio.Future = self.loop_thread.submit_coroutine(self._run())

    async def _run(self) -> None:
        socket = self.ctx.socket(zmq.SUB)
        socket.connect(self.url)
        socket.setsockopt_string(zmq.SUBSCRIBE, self.topic)
        print(f"[Subscriber] Connected to {self.url}, subscribed to '{self.topic}'")

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
        self.future.cancel()
        print(f"[Subscriber] {self.url} stopped")
