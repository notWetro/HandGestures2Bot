#!/usr/bin/env python3
"""Bluetooth Low Energy (BLE) Wi-Fi provisioning for TurtleBot3.

Uses bluezero library for BLE GATT server on Linux.
Accepts Wi-Fi credentials from Android/iOS clients and connects via NetworkManager.
"""
import json
import logging
import subprocess
import sys
from typing import Optional, Tuple

import netifaces
from bluezero import peripheral
from bluezero import adapter

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


def connect_wifi(ssid: str, password: str) -> Tuple[bool, Optional[str]]:
    """Connect to Wi-Fi via NetworkManager with timeout."""
    # Try to find nmcli
    nmcli_paths = ["/usr/bin/nmcli", "/bin/nmcli", "nmcli"]
    nmcli_cmd = None
    for path in nmcli_paths:
        try:
            subprocess.run([path, "--version"], capture_output=True, timeout=5)
            nmcli_cmd = path
            break
        except:
            continue
    
    if not nmcli_cmd:
        return False, "nmcli not found"
    
    cmd = [
        nmcli_cmd, "dev", "wifi", "connect", ssid,
        "password", password, "ifname", WLAN_INTERFACE,
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        if result.returncode != 0:
            reason = (result.stderr or result.stdout).strip() or f"nmcli failed with code {result.returncode}"
            return False, reason
        return True, None
    except subprocess.TimeoutExpired:
        return False, "Connection timeout (15s)"
    except Exception as e:
        return False, str(e)


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


def try_provision():
    """Attempt Wi-Fi provisioning if both SSID and password are set."""
    global ssid_value, password_value, status_value, ip_value
    
    if ssid_value and password_value:
        status_value = "connecting"
        
        # Capture values and reset immediately
        ssid = ssid_value
        password = password_value
        ssid_value = None
        password_value = None
        
        logging.info("Connecting to Wi-Fi: %s", ssid)
        success, reason = connect_wifi(ssid, password)
        
        if success:
            import time
            time.sleep(2)  # Wait for IP assignment
            ip = get_wlan_ip()
            if ip:
                status_value = "connected"
                ip_value = ip
                logging.info("Provisioning complete: %s -> %s", ssid, ip)
            else:
                status_value = "failed"
                logging.warning("Connected but no IPv4 address")
        else:
            status_value = "failed"
            logging.warning("Wi-Fi connect failed: %s", reason)


# Callbacks for bluezero
def read_status():
    """Read callback for status characteristic."""
    value = get_status_json().encode(ENCODING)
    logging.info("Read status: %s", value)
    return list(value)


def write_ssid(value, options):
    """Write callback for SSID characteristic."""
    global ssid_value
    text = bytes(value).decode(ENCODING).strip()
    ssid_value = text
    logging.info("SSID received: %s", ssid_value)
    try_provision()


def write_password(value, options):
    """Write callback for password characteristic."""
    global password_value
    text = bytes(value).decode(ENCODING).strip()
    password_value = text
    logging.info("Password received (length=%d)", len(password_value))
    try_provision()


def write_ack(value, options):
    """Write callback for ACK characteristic."""
    text = bytes(value).decode(ENCODING).strip()
    try:
        payload = json.loads(text)
        if payload.get("status") == "connected" or payload.get("connected"):
            logging.info("Client ACK: connected")
    except:
        logging.info("Client ACK: %s", text)


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )
    
    logging.info("Starting BLE Provisioning Server...")
    logging.info("Device name: %s", DEVICE_NAME)
    logging.info("Service UUID: %s", SERVICE_UUID)
    
    # Get the first available adapter
    adapters = list(adapter.Adapter.available())
    if not adapters:
        logging.error("No Bluetooth adapters found!")
        sys.exit(1)
    
    adapter_address = adapters[0].address
    logging.info("Using Bluetooth adapter: %s", adapter_address)
    
    # Create peripheral
    ble_peripheral = peripheral.Peripheral(adapter_address=adapter_address, local_name=DEVICE_NAME)
    
    # Add service
    ble_peripheral.add_service(srv_id=1, uuid=SERVICE_UUID, primary=True)
    
    # SSID characteristic - write only
    ble_peripheral.add_characteristic(
        srv_id=1,
        chr_id=1,
        uuid=CHAR_SSID_UUID,
        value=[],
        notifying=False,
        flags=['write'],
        write_callback=write_ssid,
    )
    logging.info("  SSID characteristic: %s", CHAR_SSID_UUID)
    
    # Password characteristic - write only
    ble_peripheral.add_characteristic(
        srv_id=1,
        chr_id=2,
        uuid=CHAR_PASSWORD_UUID,
        value=[],
        notifying=False,
        flags=['write'],
        write_callback=write_password,
    )
    logging.info("  Password characteristic: %s", CHAR_PASSWORD_UUID)
    
    # Status characteristic - read + notify
    ble_peripheral.add_characteristic(
        srv_id=1,
        chr_id=3,
        uuid=CHAR_STATUS_UUID,
        value=list(get_status_json().encode(ENCODING)),
        notifying=False,
        flags=['read', 'notify'],
        read_callback=read_status,
    )
    logging.info("  Status characteristic: %s", CHAR_STATUS_UUID)
    
    # ACK characteristic - write only
    ble_peripheral.add_characteristic(
        srv_id=1,
        chr_id=4,
        uuid=CHAR_ACK_UUID,
        value=[],
        notifying=False,
        flags=['write'],
        write_callback=write_ack,
    )
    logging.info("  ACK characteristic: %s", CHAR_ACK_UUID)
    
    # Publish and start
    ble_peripheral.publish()
    logging.info("BLE Server started and advertising!")
    logging.info("Waiting for connections...")
    
    try:
        from gi.repository import GLib
        mainloop = GLib.MainLoop()
        mainloop.run()
    except KeyboardInterrupt:
        logging.info("Shutting down...")


if __name__ == "__main__":
    main()
