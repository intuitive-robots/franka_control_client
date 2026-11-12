"""Keep the pyzlc multicast heartbeat alive across transient network errors.

pyzlc 2.2.4 (MulticastWorker._multicast_loop) stops sending heartbeats for good on the first
sendto() error ("Error in multicast loop: ..."). The other nodes then drop this process after 3 s
("heartbeat timeout" on the proxy, "Node FrankaPanda is considered offline" here) and it stays
invisible until restarted. This replacement logs the error, recreates the socket and keeps sending,
so a network blip costs a few seconds instead of the session.

Call patch_pyzlc_heartbeat() before pyzlc.init().
"""

import logging
import socket

log = logging.getLogger("zlc_heartbeat")


def _make_socket(worker) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(worker.local_ip))
    return sock


def _resilient_multicast_loop(self, interval: float = 0.5) -> None:
    from pyzlc.utils.msg import HeartbeatMessage

    sock = None
    failing = False
    while not self._stop_event.is_set():
        try:
            if sock is None:
                sock = _make_socket(self)
            msg = HeartbeatMessage(
                zlc_version=self.protocol_version,
                node_id=self.local_info["nodeID"],
                info_id=self.local_info["infoID"],
                service_port=self.service_port,
                group_name=self.group_name,
            )
            sock.sendto(msg.to_bytes(), (self.group, self.group_port))
            if failing:
                log.warning("pyzlc multicast heartbeat recovered")
                failing = False
        except Exception as exc:  # noqa: BLE001 - any error must not end the heartbeat
            if not failing:
                log.error("pyzlc multicast heartbeat send failed, retrying: %s", exc)
                failing = True
            if sock is not None:
                sock.close()
                sock = None
        self._stop_event.wait(timeout=interval)
    if sock is not None:
        sock.close()


def patch_pyzlc_heartbeat() -> bool:
    """Replace pyzlc's heartbeat sender. Returns False (and warns) if pyzlc's internals changed."""
    try:
        from pyzlc.nodes.multicast import MulticastWorker
    except ImportError:
        log.warning("pyzlc.nodes.multicast not found, heartbeat not patched")
        return False
    if not hasattr(MulticastWorker, "_multicast_loop"):
        log.warning("pyzlc MulticastWorker has no _multicast_loop, heartbeat not patched")
        return False
    MulticastWorker._multicast_loop = _resilient_multicast_loop
    return True
