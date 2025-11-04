import pytest
from fake_device.fake_franka_arm import FakeFrankaServer


@pytest.fixture(scope="session", autouse=True)
def fake_franka_server():
    """Start fake ZMQ Franka server for all integration tests."""
    server = FakeFrankaServer()
    server.start()
    yield server
    server.stop()
