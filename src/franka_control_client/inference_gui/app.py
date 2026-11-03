"""Robot-PC entry point: inference session + browser dashboard.

    python -m franka_control_client.inference_gui.app --config configs/inference_gui.json
    # then open http://localhost:8080

The dashboard can move the robot, so it binds to localhost by default. To use
it from another machine prefer an SSH tunnel (ssh -L 8080:localhost:8080 robot-pc)
or pass --host 0.0.0.0 on a trusted lab network only.
"""

from __future__ import annotations

import argparse
import logging
import signal
import threading
from pathlib import Path

from .config import SessionConfig
from .session import InferenceSession
from .web_server import serve


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=None,
                        help="JSON session config; GUI changes are written back to it")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--sim", action="store_true", help="use simulated robot/gripper/cameras")
    parser.add_argument("--connect", action="store_true", help="connect right away")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    if args.config is not None and args.config.exists():
        cfg = SessionConfig.load(args.config)
    else:
        cfg = SessionConfig()
        if args.config is not None:
            cfg.save(args.config)
    if args.sim:
        cfg.sim = True

    session = InferenceSession(cfg, config_path=args.config)
    server = serve(session, args.host, args.port)
    logging.getLogger("inference_gui").info(
        "Dashboard on http://%s:%d %s", "localhost" if args.host in ("127.0.0.1", "0.0.0.0") else args.host,
        args.port, "(SIMULATION)" if cfg.sim else "",
    )
    if args.connect:
        session.connect()

    stop = threading.Event()

    def _signal(*_):
        stop.set()

    signal.signal(signal.SIGINT, _signal)
    signal.signal(signal.SIGTERM, _signal)
    thread = threading.Thread(target=server.serve_forever, name="http", daemon=True)
    thread.start()
    try:
        while not stop.wait(0.5):
            pass
    finally:
        logging.getLogger("inference_gui").info("Shutting down...")
        server.shutdown()
        session.shutdown()
        if session._pyzlc_initialised:
            import pyzlc

            pyzlc.shutdown()


if __name__ == "__main__":
    main()
