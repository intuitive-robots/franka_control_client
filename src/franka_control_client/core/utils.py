"""Utility functions for network operations."""

from __future__ import annotations

import ipaddress
import socket

import psutil
import zmq


def get_local_ip_in_same_subnet(target_ip: str) -> str | None:
    """
    Return the local IP address of the interface that shares a subnet with the target IP.

    Args:
        target_ip (str): The remote device's IP address, e.g., '192.168.1.10'.

    Returns:
        str | None: Local interface IP on the same subnet, or None if not found.
    """
    target = ipaddress.ip_address(target_ip)

    for _, iface_addrs in psutil.net_if_addrs().items():
        for addr in iface_addrs:
            if addr.family == socket.AF_INET:  # IPv4 only
                local_ip = ipaddress.ip_interface(
                    f"{addr.address}/{addr.netmask}"
                )
                if target in local_ip.network:
                    return addr.address
    return None


def get_socket_bind_url(sock: zmq.Socket) -> str:
    """
    Return the actual bind URL of a ZeroMQ socket.
    Works for tcp, ipc, inproc, etc.
    """
    return sock.getsockopt_string(zmq.LAST_ENDPOINT)
