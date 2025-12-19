#!/usr/bin/env python3
"""Bluetooth Low Energy (BLE) Wi-Fi provisioning for TurtleBot3.

Uses the 'bless' library for cross-platform BLE GATT server.
Accepts Wi-Fi credentials from Android/iOS clients and connects via NetworkManager.
"""
import asyncio
import json
import logging
import subprocess
import sys
from typing import Optional, Tuple

import netifaces
from bless import BlessServer, BlessGATTCharacteristic, GATTCharacteristicProperties, GATTAttributePermissions

# BLE Configuration
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
CHAR_SSID_UUID = "12345678-1234-5678-1234-56789abcdef1"
CHAR_PASSWORD_UUID = "12345678-1234-5678-1234-56789abcdef2"
CHAR_STATUS_UUID = "12345678-1234-5678-1234-56789abcdef3"
CHAR_ACK_UUID = "12345678-1234-5678-1234-56789abcdef5"

WLAN_INTERFACE = "wlan0"
ENCODING = "utf-8"
DEVICE_NAME = "TurtleBot3-Provisioning"

# Global state
ssid_value: Optional[str] = None
password_value: Optional[str] = None
status_value: str = "idle"
ip_value: Optional[str] = None
server: Optional[BlessServer] = None


def connect_wifi(ssid: str, password: str) -> Tuple[bool, Optional[str]]:
    """Connect to Wi-Fi via NetworkManager."""
    cmd = [
        "nmcli", "dev", "wifi", "connect", ssid,
        "password", password, "ifname", WLAN_INTERFACE,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        reason = (result.stderr or result.stdout).strip() or f"nmcli failed with code {result.returncode}"
        return False, reason
    return True, None


def get_wlan_ip() -> Optional[str]:
    """Return the current IPv4 address on wlan0."""
    try:
        iface_data = netifaces.ifaddresses(WLAN_INTERFACE)
        ipv4_list = iface_data.get(netifaces.AF_INET, [])
        if not ipv4_list:
            return None
        return ipv4_list[0].get("addr")
    except Exception as exc:
        logging.exception("Failed to read IP: %s", exc)
        return None


def get_status_json() -> str:
    """Return current status as JSON."""
    payload = {"status": status_value}
    if ip_value:
        payload["ip"] = ip_value
    return json.dumps(payload)


def update_status_char():
    """Update the status characteristic value."""
    global server
    if server:
        try:
            value = get_status_json().encode(ENCODING)
            server.get_characteristic(CHAR_STATUS_UUID).value = bytearray(value)
        except Exception as e:
            logging.error("Failed to update status char: %s", e)


async def try_provision():
    """Attempt Wi-Fi provisioning if both SSID and password are set."""
    global ssid_value, password_value, status_value, ip_value
    
    if ssid_value and password_value:
        status_value = "connecting"
        update_status_char()
        
        logging.info("Connecting to Wi-Fi: %s", ssid_value)
        success, reason = connect_wifi(ssid_value, password_value)
        
        if success:
            await asyncio.sleep(2)  # Wait for IP assignment
            ip = get_wlan_ip()
            if ip:
                status_value = "connected"
                ip_value = ip
                logging.info("Provisioning complete: %s -> %s", ssid_value, ip)
            else:
                status_value = "failed"
                logging.warning("Connected but no IPv4 address")
        else:
            status_value = "failed"
            logging.warning("Wi-Fi connect failed: %s", reason)
        
        update_status_char()
        
        # Reset after delay
        await asyncio.sleep(5)
        ssid_value = None
        password_value = None
        status_value = "idle"
        update_status_char()


def read_request(characteristic: BlessGATTCharacteristic, **kwargs) -> bytearray:
    """Handle read requests."""
    uuid = str(characteristic.uuid).lower()
    logging.info("Read request: %s", uuid)
    
    if uuid == CHAR_STATUS_UUID.lower():
        value = get_status_json().encode(ENCODING)
        logging.info("Returning status: %s", value)
        return bytearray(value)
    
    return characteristic.value


def write_request(characteristic: BlessGATTCharacteristic, value: bytearray, **kwargs):
    """Handle write requests."""
    global ssid_value, password_value
    
    uuid = str(characteristic.uuid).lower()
    text = bytes(value).decode(ENCODING).strip()
    logging.info("Write request: %s = %s", uuid, text[:50] if len(text) > 50 else text)
    
    if uuid == CHAR_SSID_UUID.lower():
        ssid_value = text
        logging.info("SSID received: %s", ssid_value)
        asyncio.create_task(try_provision())
        
    elif uuid == CHAR_PASSWORD_UUID.lower():
        password_value = text
        logging.info("Password received (length=%d)", len(password_value))
        asyncio.create_task(try_provision())
        
    elif uuid == CHAR_ACK_UUID.lower():
        try:
            payload = json.loads(text)
            if payload.get("status") == "connected" or payload.get("connected"):
                logging.info("Client ACK: connected")
        except:
            logging.info("Client ACK: %s", text)


async def main():
    global server
    
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )
    
    logging.info("Starting BLE Provisioning Server...")
    logging.info("Device name: %s", DEVICE_NAME)
    logging.info("Service UUID: %s", SERVICE_UUID)
    
    # Create BLE server
    server = BlessServer(name=DEVICE_NAME)
    server.read_request_func = read_request
    server.write_request_func = write_request
    
    # Add service
    await server.add_new_service(SERVICE_UUID)
    
    # Add characteristics
    # SSID - write only
    await server.add_new_characteristic(
        SERVICE_UUID, CHAR_SSID_UUID,
        GATTCharacteristicProperties.write,
        bytearray(),
        GATTAttributePermissions.writeable,
    )
    logging.info("  SSID characteristic: %s", CHAR_SSID_UUID)
    
    # Password - write only
    await server.add_new_characteristic(
        SERVICE_UUID, CHAR_PASSWORD_UUID,
        GATTCharacteristicProperties.write,
        bytearray(),
        GATTAttributePermissions.writeable,
    )
    logging.info("  Password characteristic: %s", CHAR_PASSWORD_UUID)
    
    # Status - read + notify
    await server.add_new_characteristic(
        SERVICE_UUID, CHAR_STATUS_UUID,
        GATTCharacteristicProperties.read | GATTCharacteristicProperties.notify,
        bytearray(get_status_json().encode(ENCODING)),
        GATTAttributePermissions.readable,
    )
    logging.info("  Status characteristic: %s", CHAR_STATUS_UUID)
    
    # ACK - write only
    await server.add_new_characteristic(
        SERVICE_UUID, CHAR_ACK_UUID,
        GATTCharacteristicProperties.write,
        bytearray(),
        GATTAttributePermissions.writeable,
    )
    logging.info("  ACK characteristic: %s", CHAR_ACK_UUID)
    
    # Start advertising
    await server.start()
    logging.info("BLE Server started and advertising!")
    logging.info("Waiting for connections...")
    
    # Keep running
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logging.info("Shutting down...")
    finally:
        await server.stop()


if __name__ == "__main__":
    asyncio.run(main())
