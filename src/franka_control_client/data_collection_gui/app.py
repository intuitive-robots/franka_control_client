"""Data collection GUI entry point (robot PC).

    python -m franka_control_client.data_collection_gui.app --config configs/data_collection_gui.json
    # then open http://localhost:8090

The page can move the robot, so it binds to localhost by default. From another machine use
an SSH tunnel (ssh -L 8090:localhost:8090 robot-pc), or --host 0.0.0.0 on a trusted network only.
"""

from __future__ import annotations

import argparse
import logging
import signal
import threading
from pathlib import Path

from .config import CollectionConfig
from .session import CollectionSession
from .web_server import serve


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=None, help="JSON settings; GUI changes are written back")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--sim", action="store_true", help="simulated robot, leaders and cameras")
    parser.add_argument("--connect", action="store_true", help="connect right away")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    if args.config is not None and args.config.exists():
        cfg = CollectionConfig.load(args.config)
    else:
        cfg = CollectionConfig()
        if args.config is not None:
            cfg.save(args.config)
    if args.sim:
        cfg.sim = True

    session = CollectionSession(cfg, config_path=args.config)
    server = serve(session, args.host, args.port)
    log = logging.getLogger("data_collection_gui")
    log.info("Dashboard on http://%s:%d %s", "localhost" if args.host in ("127.0.0.1", "0.0.0.0") else args.host,
             args.port, "(SIMULATION)" if cfg.sim else "")
    if args.connect:
        session.connect()

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    thread = threading.Thread(target=server.serve_forever, name="http", daemon=True)
    thread.start()
    try:
        while not stop.wait(0.5):
            pass
    finally:
        log.info("Shutting down...")
        server.shutdown()
        session.shutdown()
        if session._pyzlc_initialised:
            import pyzlc

            pyzlc.shutdown()


if __name__ == "__main__":
    main()
