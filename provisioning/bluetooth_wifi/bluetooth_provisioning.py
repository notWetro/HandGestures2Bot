#!/usr/bin/env python3
"""Bluetooth-based Wi-Fi provisioning for TurtleBot3 (system-level service).

Runs an RFCOMM server to accept Wi-Fi credentials over Bluetooth Classic, uses
NetworkManager (nmcli) to join the requested network, and replies with the
assigned IPv4 address. Keep this outside the ROS2 workspace to separate device
provisioning from application logic.
"""
import json
import logging
import socket
import subprocess
import sys
from typing import Any, Dict, Optional, Tuple

import netifaces

RFCOMM_CHANNEL = 1
BUFFER_SIZE = 1024
MAX_PAYLOAD_SIZE = 8192
READ_TIMEOUT_SECONDS = 60
WLAN_INTERFACE = "wlan0"
ENCODING = "utf-8"


def send_json(conn: socket.socket, payload: Dict[str, Any]) -> None:
    """Send a JSON message terminated with a newline for simple framing."""
    message = json.dumps(payload) + "\n"
    conn.sendall(message.encode(ENCODING))


def receive_json(conn: socket.socket) -> Dict[str, Any]:
    """Read one JSON object from the RFCOMM connection with newline framing."""
    conn.settimeout(READ_TIMEOUT_SECONDS)
    buffer = ""

    while len(buffer) < MAX_PAYLOAD_SIZE:
        try:
            chunk = conn.recv(BUFFER_SIZE)
        except socket.timeout as exc:
            raise TimeoutError("Timed out waiting for provisioning payload") from exc

        if not chunk:
            raise ConnectionError("Client disconnected before sending payload")

        buffer += chunk.decode(ENCODING, errors="replace")
        candidates = buffer.splitlines() if "\n" in buffer else [buffer]

        for candidate in candidates:
            candidate = candidate.strip()
            if not candidate:
                continue
            try:
                return json.loads(candidate)
            except json.JSONDecodeError:
                continue

    raise ValueError("Provisioning payload exceeded limit or was not valid JSON")


def connect_wifi(ssid: str, password: str) -> Tuple[bool, Optional[str]]:
    """Connect to Wi-Fi via NetworkManager without disturbing existing profiles."""
    cmd = [
        "nmcli",
        "dev",
        "wifi",
        "connect",
        ssid,
        "password",
        password,
        "ifname",
        WLAN_INTERFACE,
    ]

    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        reason = (result.stderr or result.stdout).strip() or (
            f"nmcli failed with code {result.returncode}"
        )
        return False, reason

    return True, None


def get_wlan_ip() -> Optional[str]:
    """Return the current IPv4 address on wlan0 using netifaces."""
    try:
        iface_data = netifaces.ifaddresses(WLAN_INTERFACE)
        ipv4_list = iface_data.get(netifaces.AF_INET, [])
        if not ipv4_list:
            return None
        return ipv4_list[0].get("addr")
    except Exception as exc:  # Defensive against missing interface
        logging.exception("Failed to read IP address from netifaces: %s", exc)
        return None


def handle_client(conn: socket.socket) -> None:
    """Handle one provisioning session then close the connection."""
    try:
        peer = conn.getpeername()
    except OSError:
        peer = "unknown"
    logging.info("Phone connected over RFCOMM: %s", peer)

    send_json(conn, {"status": "connected"})

    try:
        payload = receive_json(conn)
    except Exception as exc:
        logging.warning("Provisioning payload not received: %s", exc)
        send_json(conn, {"status": "failed", "reason": str(exc)})
        return

    ssid = payload.get("ssid")
    password = payload.get("password")

    if not ssid or not isinstance(ssid, str):
        send_json(conn, {"status": "failed", "reason": "Missing or invalid ssid"})
        return

    if password is None or not isinstance(password, str):
        send_json(conn, {"status": "failed", "reason": "Missing or invalid password"})
        return

    success, reason = connect_wifi(ssid, password)
    if not success:
        logging.warning("Wi-Fi connect failed for %s: %s", ssid, reason)
        send_json(conn, {"status": "failed", "reason": reason or "Wi-Fi connect failed"})
        return

    ip = get_wlan_ip()
    if not ip:
        send_json(
            conn,
            {"status": "failed", "reason": "Connected to Wi-Fi but no IPv4 on wlan0"},
        )
        return

    send_json(conn, {"status": "connected", "ssid": ssid, "ip": ip})
    logging.info("Provisioning complete for ssid=%s ip=%s", ssid, ip)


def run_server() -> None:
    """Start RFCOMM server on channel 1 and serve one client at a time."""
    try:
        server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    except OSError as exc:
        logging.error("Bluetooth socket creation failed: %s", exc)
        sys.exit(1)

    try:
        server.bind((socket.BDADDR_ANY, RFCOMM_CHANNEL))
        server.listen(1)
        logging.info("Bluetooth provisioning server listening on RFCOMM channel %s", RFCOMM_CHANNEL)
    except OSError as exc:
        logging.error("Bluetooth socket bind/listen failed: %s", exc)
        server.close()
        sys.exit(1)

    try:
        while True:
            logging.info("Waiting for phone to connect over Bluetooth…")
            try:
                conn, _ = server.accept()
            except OSError as exc:
                logging.error("Accept failed: %s", exc)
                continue

            with conn:
                handle_client(conn)
                logging.info("Provisioning session closed")
    except KeyboardInterrupt:
        logging.info("Provisioning server interrupted; shutting down")
    finally:
        server.close()


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )
    run_server()


if __name__ == "__main__":
    main()
