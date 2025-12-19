#!/usr/bin/env python3
"""Bluetooth Low Energy (BLE) Wi-Fi provisioning for TurtleBot3 (system-level service).

Runs a BLE GATT server to accept Wi-Fi credentials over Bluetooth Low Energy.
Works on both Android and iOS. Uses NetworkManager (nmcli) to join the requested
network, and replies with the assigned IPv4 address. Keep this outside the ROS2
workspace to separate device provisioning from application logic.
"""
import asyncio
import json
import logging
import subprocess
import sys
from typing import Any, Dict, Optional, Tuple
import uuid

import netifaces
from bleak import BleakServer
from bleak.backends.characteristic import BleakGATTCharacteristic

# BLE Configuration
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
SSID_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"
PASSWORD_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef2"
STATUS_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef3"
IP_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef4"

WLAN_INTERFACE = "wlan0"
ENCODING = "utf-8"
READ_TIMEOUT_SECONDS = 60


class ProvisioningContext:
    """Holds the state for an ongoing provisioning session."""
    def __init__(self):
        self.ssid: Optional[str] = None
        self.password: Optional[str] = None
        self.status: str = "idle"
        self.ip: Optional[str] = None
        self.lock = asyncio.Lock()

    async def set_ssid(self, value: str) -> None:
        async with self.lock:
            self.ssid = value
            logging.info("SSID received: %s", value)
            await self._try_provision()

    async def set_password(self, value: str) -> None:
        async with self.lock:
            self.password = value
            logging.info("Password received")
            await self._try_provision()

    async def _try_provision(self) -> None:
        """Attempt provisioning if both SSID and password are set."""
        if self.ssid and self.password:
            self.status = "connecting"
            success, reason = connect_wifi(self.ssid, self.password)
            
            if success:
                ip = get_wlan_ip()
                if ip:
                    self.status = "connected"
                    self.ip = ip
                    logging.info("Provisioning complete: %s -> %s", self.ssid, ip)
                else:
                    self.status = "failed"
                    logging.warning("Connected to Wi-Fi but no IPv4 address")
            else:
                self.status = "failed"
                logging.warning("Wi-Fi connect failed: %s", reason)

            # Reset for next provisioning attempt
            await asyncio.sleep(2)
            self.ssid = None
            self.password = None
            self.status = "idle"

    def get_status_value(self) -> bytes:
        """Return current status as JSON bytes."""
        payload = {"status": self.status}
        if self.ip:
            payload["ip"] = self.ip
        return json.dumps(payload).encode(ENCODING)



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


def handle_ssid_write(data: bytes) -> None:
    """Handle SSID write from characteristic."""
    pass

def handle_password_write(data: bytes) -> None:
    """Handle password write from characteristic."""
    pass

class ProvisioningCharacteristic:
    """Wrapper for BLE characteristics."""
    def __init__(self, uuid: str, properties: list, permissions: list):
        self.uuid = uuid
        self.properties = properties
        self.permissions = permissions
        self.value = b""

async def run_server(context: ProvisioningContext) -> None:
    """Start BLE GATT server and advertise provisioning service."""
    
    async def read_status(offset: int = 0) -> bytes:
        """Callback for reading status characteristic."""
        return context.get_status_value()[offset:]

    async def read_ip(offset: int = 0) -> bytes:
        """Callback for reading IP characteristic."""
        if context.ip:
            return context.ip.encode(ENCODING)[offset:]
        return b""

    async def write_ssid(data: bytes) -> None:
        """Callback for writing SSID characteristic."""
        try:
            ssid = data.decode(ENCODING).strip()
            await context.set_ssid(ssid)
        except Exception as exc:
            logging.error("Error writing SSID: %s", exc)

    async def write_password(data: bytes) -> None:
        """Callback for writing password characteristic."""
        try:
            password = data.decode(ENCODING).strip()
            await context.set_password(password)
        except Exception as exc:
            logging.error("Error writing password: %s", exc)

    # Create characteristic definitions
    characteristics = [
        {
            "uuid": SSID_CHAR_UUID,
            "properties": ["write"],
            "write_callback": write_ssid,
        },
        {
            "uuid": PASSWORD_CHAR_UUID,
            "properties": ["write"],
            "write_callback": write_password,
        },
        {
            "uuid": STATUS_CHAR_UUID,
            "properties": ["read", "notify"],
            "read_callback": read_status,
        },
        {
            "uuid": IP_CHAR_UUID,
            "properties": ["read"],
            "read_callback": read_ip,
        },
    ]

    # Create the BLE server
    async with BleakServer(
        [SERVICE_UUID],
        characteristics,
        local_name="TurtleBot3-Provisioning",
    ) as server:
        logging.info("BLE GATT server started: TurtleBot3-Provisioning")
        logging.info("Service UUID: %s", SERVICE_UUID)
        
        try:
            await server.start()
            logging.info("BLE server is advertising...")
            # Keep server running
            while True:
                await asyncio.sleep(1)
        except Exception as exc:
            logging.error("BLE server error: %s", exc)
        finally:
            logging.info("BLE server stopped")


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )
    
    context = ProvisioningContext()
    
    try:
        asyncio.run(run_server(context))
    except KeyboardInterrupt:
        logging.info("Provisioning server interrupted; shutting down")
    except Exception as exc:
        logging.error("Fatal error: %s", exc)
        sys.exit(1)


if __name__ == "__main__":
    main()
