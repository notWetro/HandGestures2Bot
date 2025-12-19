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

import dbus
import dbus.service
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib
import netifaces

# BLE Configuration
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
SSID_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"
PASSWORD_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef2"
STATUS_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef3"
IP_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef4"

WLAN_INTERFACE = "wlan0"
ENCODING = "utf-8"


class BLECharacteristic(dbus.service.Object):
    """BLE GATT Characteristic."""
    def __init__(self, bus, path, uuid, properties, value=b""):
        dbus.service.Object.__init__(self, bus, path)
        self.uuid = uuid
        self.properties = properties
        self.value = value
        self.bus = bus
        self.path = path

    def update_value(self, value):
        """Update characteristic value."""
        self.value = value
        self.PropertiesChanged(
            "org.bluez.GattCharacteristic1",
            {"Value": dbus.Array(value, signature="y")},
            []
        )

    @dbus.service.method("org.bluez.GattCharacteristic1", in_signature="a{sv}ays", out_signature="")
    def WriteValue(self, options, value):
        """Handle write requests."""
        pass

    @dbus.service.method("org.bluez.GattCharacteristic1", in_signature="a{sv}", out_signature="ay")
    def ReadValue(self, options):
        """Handle read requests."""
        return dbus.Array(self.value, signature="y")

    @dbus.service.signal("org.freedesktop.DBus.Properties", signature="sa{sv}as")
    def PropertiesChanged(self, iface, changed, invalidated):
        pass

    @dbus.service.method("org.bluez.GattCharacteristic1", in_signature="", out_signature="s")
    def GetUUID(self):
        return self.uuid

    @dbus.service.method("org.bluez.GattCharacteristic1", in_signature="", out_signature="as")
    def GetFlags(self):
        return self.properties

    @dbus.service.method("org.bluez.GattCharacteristic1", in_signature="", out_signature="o")
    def GetService(self):
        return dbus.ObjectPath("/org/bluez/hci0/gatt0/svc0")


class BLEService(dbus.service.Object):
    """BLE GATT Service."""
    def __init__(self, bus, path, uuid):
        dbus.service.Object.__init__(self, bus, path)
        self.uuid = uuid
        self.bus = bus
        self.path = path

    @dbus.service.method("org.bluez.GattService1", in_signature="", out_signature="s")
    def GetUUID(self):
        return self.uuid

    @dbus.service.method("org.bluez.GattService1", in_signature="", out_signature="b")
    def IsPrimary(self):
        return True

    @dbus.service.method("org.bluez.GattService1", in_signature="", out_signature="ao")
    def GetCharacteristics(self):
        return [dbus.ObjectPath("/org/bluez/hci0/gatt0/svc0/chr0")]


class ProvisioningContext:
    """Holds the state for an ongoing provisioning session."""
    def __init__(self):
        self.ssid: Optional[str] = None
        self.password: Optional[str] = None
        self.status: str = "idle"
        self.ip: Optional[str] = None

    def set_ssid(self, value: str) -> None:
        self.ssid = value
        logging.info("SSID received: %s", value)
        self._try_provision()

    def set_password(self, value: str) -> None:
        self.password = value
        logging.info("Password received")
        self._try_provision()

    def _try_provision(self) -> None:
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

            # Reset for next provisioning attempt after a delay
            def reset():
                self.ssid = None
                self.password = None
                self.status = "idle"
            GLib.timeout_add(2000, reset)

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

def run_server(context: ProvisioningContext) -> None:
    """Start BLE GATT server and advertise provisioning service."""
    DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    
    # Register the BLE service and characteristics
    service = BLEService(bus, "/org/bluez/hci0/gatt0/svc0", SERVICE_UUID)
    
    # SSID characteristic
    ssid_char = BLECharacteristic(bus, "/org/bluez/hci0/gatt0/svc0/chr0", SSID_CHAR_UUID, ["write"])
    original_write = ssid_char.WriteValue
    
    def ssid_write_handler(options, value):
        try:
            ssid = "".join(chr(b) for b in value).strip()
            context.set_ssid(ssid)
        except Exception as exc:
            logging.error("Error writing SSID: %s", exc)
    
    ssid_char.WriteValue = ssid_write_handler
    
    # Password characteristic
    password_char = BLECharacteristic(bus, "/org/bluez/hci0/gatt0/svc0/chr1", PASSWORD_CHAR_UUID, ["write"])
    
    def password_write_handler(options, value):
        try:
            password = "".join(chr(b) for b in value).strip()
            context.set_password(password)
        except Exception as exc:
            logging.error("Error writing password: %s", exc)
    
    password_char.WriteValue = password_write_handler
    
    # Status characteristic
    status_char = BLECharacteristic(bus, "/org/bluez/hci0/gatt0/svc0/chr2", STATUS_CHAR_UUID, ["read", "notify"])
    status_char.value = context.get_status_value()
    
    # IP characteristic
    ip_char = BLECharacteristic(bus, "/org/bluez/hci0/gatt0/svc0/chr3", IP_CHAR_UUID, ["read"])
    if context.ip:
        ip_char.value = context.ip.encode(ENCODING)
    
    logging.info("BLE GATT server registered")
    logging.info("Service UUID: %s", SERVICE_UUID)
    
    try:
        mainloop = GLib.MainLoop()
        logging.info("BLE server is advertising...")
        mainloop.run()
    except KeyboardInterrupt:
        logging.info("Provisioning server interrupted; shutting down")
        mainloop.quit()


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )
    
    context = ProvisioningContext()
    
    try:
        run_server(context)
    except KeyboardInterrupt:
        logging.info("Provisioning server interrupted; shutting down")
    except Exception as exc:
        logging.error("Fatal error: %s", exc)
        sys.exit(1)


if __name__ == "__main__":
    main()
