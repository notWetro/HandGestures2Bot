#!/usr/bin/env python3
"""Bluetooth Low Energy (BLE) Wi-Fi provisioning for TurtleBot3.

Advertises a BLE service that accepts Wi-Fi credentials from both Android and iOS.
Uses NetworkManager (nmcli) to join the requested network, and provides the
assigned IPv4 address back to the client.
"""
import json
import logging
import subprocess
import sys
from typing import Optional, Tuple
import netifaces
import dbus
from dbus.service import Object, method
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib

# BLE Configuration
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
CHAR_SSID_UUID = "12345678-1234-5678-1234-56789abcdef1"
CHAR_PASSWORD_UUID = "12345678-1234-5678-1234-56789abcdef2"
CHAR_STATUS_UUID = "12345678-1234-5678-1234-56789abcdef3"
CHAR_ACK_UUID = "12345678-1234-5678-1234-56789abcdef5"

WLAN_INTERFACE = "wlan0"
ENCODING = "utf-8"

# Global context
context = None


class BLECharacteristic(Object):
    """BLE GATT Characteristic implementation."""
    
    def __init__(self, bus, path, uuid, flags):
        super().__init__(bus, path)
        self.path = path
        self.uuid = uuid
        self.flags = flags
        self.value = bytearray()

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @method("org.freedesktop.DBus.Properties", in_signature="ss", out_signature="v")
    def Get(self, interface, prop):
        if interface == "org.bluez.GattCharacteristic1":
            if prop == "UUID":
                return dbus.String(self.uuid)
            elif prop == "Flags":
                return dbus.Array(self.flags, "s")
            elif prop == "Value":
                return dbus.Array(self.value, "y")
        return None

    @method("org.freedesktop.DBus.Properties", in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface == "org.bluez.GattCharacteristic1":
            return {
                "UUID": dbus.String(self.uuid),
                "Flags": dbus.Array(self.flags, "s"),
                "Value": dbus.Array(self.value, "y"),
            }
        return {}

    @method("org.bluez.GattCharacteristic1", in_signature="a{sv}", out_signature="ay")
    def ReadValue(self, options):
        logging.info("Read %s: %s", self.uuid, bytes(self.value))
        return dbus.Array(self.value, "y")

    @method("org.bluez.GattCharacteristic1", in_signature="aya{sv}", out_signature="")
    def WriteValue(self, value, options):
        self.value = bytearray(value)
        logging.info("Write %s: %s", self.uuid, bytes(self.value))
        
        global context
        if self.uuid == CHAR_SSID_UUID:
            try:
                ssid = bytes(self.value).decode(ENCODING).strip()
                context.set_ssid(ssid)
            except Exception as e:
                logging.error("Error parsing SSID: %s", e)
        elif self.uuid == CHAR_PASSWORD_UUID:
            try:
                password = bytes(self.value).decode(ENCODING).strip()
                context.set_password(password)
            except Exception as e:
                logging.error("Error parsing password: %s", e)
        elif self.uuid == CHAR_ACK_UUID:
            # Client-side acknowledgement hook
            try:
                text = bytes(self.value).decode(ENCODING).strip()
                payload = {}
                try:
                    payload = json.loads(text)
                except Exception:
                    # Accept plain text 'connected' too
                    payload = {"status": text}

                status = str(payload.get("status", "")).lower()
                connected_flag = bool(payload.get("connected", False))
                if status == "connected" or connected_flag:
                    logging.info("Client ACK: connected")
                    # Reflect ACK in status characteristic without IP
                    context.status = "connected"
                    context.ip = context.ip  # unchanged
                    context._update_status_char()
                else:
                    logging.info("Client ACK payload: %s", payload)
            except Exception as e:
                logging.error("Error handling ACK write: %s", e)


class BLEService(Object):
    """BLE GATT Service implementation."""
    
    def __init__(self, bus, path, uuid):
        super().__init__(bus, path)
        self.path = path
        self.uuid = uuid

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @method("org.freedesktop.DBus.Properties", in_signature="ss", out_signature="v")
    def Get(self, interface, prop):
        if interface == "org.bluez.GattService1":
            if prop == "UUID":
                return dbus.String(self.uuid)
            elif prop == "Primary":
                return dbus.Boolean(True)
        return None

    @method("org.freedesktop.DBus.Properties", in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface == "org.bluez.GattService1":
            return {
                "UUID": dbus.String(self.uuid),
                "Primary": dbus.Boolean(True),
            }
        return {}


class ProvisioningContext:
    """Holds provisioning state."""
    def __init__(self):
        self.ssid: Optional[str] = None
        self.password: Optional[str] = None
        self.status: str = "idle"
        self.ip: Optional[str] = None
        self.status_char = None

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
            self._update_status_char()
            
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

            self._update_status_char()
            
            # Reset after 2 seconds
            def reset():
                self.ssid = None
                self.password = None
                self.status = "idle"
                self._update_status_char()
            
            GLib.timeout_add_seconds(2, reset)

    def _update_status_char(self):
        """Update status characteristic value."""
        if self.status_char:
            payload = {"status": self.status}
            if self.ip:
                payload["ip"] = self.ip
            json_str = json.dumps(payload)
            self.status_char.value = bytearray(json_str.encode(ENCODING))

    def get_status_json(self) -> str:
        """Return status as JSON."""
        payload = {"status": self.status}
        if self.ip:
            payload["ip"] = self.ip
        return json.dumps(payload)



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
    except Exception as exc:
        logging.exception("Failed to read IP address from netifaces: %s", exc)
        return None


class GattApplication(Object):
    """Root GATT Application object for BlueZ."""
    
    def __init__(self, bus, path, services):
        Object.__init__(self, bus, path)
        self.services = services
        self.path = path

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @method("org.freedesktop.DBus.Properties", in_signature="ss", out_signature="v")
    def Get(self, interface, prop):
        return None

    @method("org.freedesktop.DBus.Properties", in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        return {}

    @method("org.bluez.GattApplication1", in_signature="", out_signature="a{oa{sa{sv}}}")
    def GetManagedObjects(self):
        """Return all GATT objects (required by BlueZ)."""
        logging.debug("GetManagedObjects() called")
        managed = {}
        for svc in self.services:
            managed[dbus.ObjectPath(svc.path)] = {
                "org.bluez.GattService1": {
                    "UUID": dbus.String(svc.uuid),
                    "Primary": dbus.Boolean(True),
                }
            }
            # Add characteristics
            if hasattr(svc, "characteristics"):
                for char in svc.characteristics:
                    managed[dbus.ObjectPath(char.path)] = {
                        "org.bluez.GattCharacteristic1": {
                            "UUID": dbus.String(char.uuid),
                            "Service": dbus.ObjectPath(svc.path),
                            "Flags": dbus.Array(char.flags, "s"),
                            "Value": dbus.Array(char.value, "y"),
                        }
                    }
        logging.debug("GetManagedObjects returning %d objects", len(managed))
        return managed


class LEAdvertisement(Object):
    """Bluetooth LE Advertisement for service discovery."""
    
    def __init__(self, bus, path, service_uuid):
        Object.__init__(self, bus, path)
        self.path = path
        self.service_uuid = service_uuid
        self.ad_type = "peripheral"

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @method("org.freedesktop.DBus.Properties", in_signature="ss", out_signature="v")
    def Get(self, interface, prop):
        if interface == "org.bluez.LEAdvertisement1":
            if prop == "Type":
                return dbus.String(self.ad_type)
            elif prop == "ServiceUUIDs":
                return dbus.Array([dbus.String(self.service_uuid)], "s")
            elif prop == "LocalName":
                return dbus.String("TurtleBot3-Provisioning")
            elif prop == "Appearance":
                return dbus.UInt16(0)
        return None

    @method("org.freedesktop.DBus.Properties", in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface == "org.bluez.LEAdvertisement1":
            return {
                "Type": dbus.String(self.ad_type),
                "ServiceUUIDs": dbus.Array([dbus.String(self.service_uuid)], "s"),
                "LocalName": dbus.String("TurtleBot3-Provisioning"),
                "Appearance": dbus.UInt16(0),
            }
        return {}

    @method("org.bluez.LEAdvertisement1", in_signature="", out_signature="")
    def Release(self):
        logging.info("LE Advertisement released")
        pass


def setup_ble():
    """Set up BLE service, characteristics, and LE advertisement."""
    DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    
    # Create service
    service_path = "/org/bluez/hci0/gatt/svc0"
    service = BLEService(bus, service_path, SERVICE_UUID)
    
    # Create characteristics
    ssid_char_path = service_path + "/chr0"
    ssid_char = BLECharacteristic(bus, ssid_char_path, CHAR_SSID_UUID, ["write"])
    
    password_char_path = service_path + "/chr1"
    password_char = BLECharacteristic(bus, password_char_path, CHAR_PASSWORD_UUID, ["write"])
    
    status_char_path = service_path + "/chr2"
    status_char = BLECharacteristic(bus, status_char_path, CHAR_STATUS_UUID, ["read", "notify"])
    
    # Client ACK characteristic (write-only)
    ack_char_path = service_path + "/chr3"
    ack_char = BLECharacteristic(bus, ack_char_path, CHAR_ACK_UUID, ["write"])
    
    # Attach characteristics to service
    service.characteristics = [ssid_char, password_char, status_char, ack_char]
    
    # Store reference to status characteristic for updates
    global context
    context.status_char = status_char
    context._update_status_char()
    
    logging.info("BLE Service registered: %s", SERVICE_UUID)
    logging.info("  SSID characteristic: %s", CHAR_SSID_UUID)
    logging.info("  Password characteristic: %s", CHAR_PASSWORD_UUID)
    logging.info("  Status characteristic: %s", CHAR_STATUS_UUID)
    logging.info("  Ack characteristic: %s", CHAR_ACK_UUID)
    
    # Create GATT application
    app_path = "/org/bluez/hci0/gatt"
    app = GattApplication(bus, app_path, [service])
    
    # Create LE Advertisement
    ad_path = "/org/bluez/hci0/advertisement0"
    advertisement = LEAdvertisement(bus, ad_path, SERVICE_UUID)
    
    try:
        adapter_path = "/org/bluez/hci0"
        adapter_obj = bus.get_object("org.bluez", adapter_path)
        
        # Configure adapter properties via DBus
        props = dbus.Interface(adapter_obj, "org.freedesktop.DBus.Properties")
        props.Set("org.bluez.Adapter1", "Powered", dbus.Boolean(True))
        props.Set("org.bluez.Adapter1", "Discoverable", dbus.Boolean(True))
        props.Set("org.bluez.Adapter1", "DiscoverableTimeout", dbus.UInt32(0))
        props.Set("org.bluez.Adapter1", "Pairable", dbus.Boolean(True))
        props.Set("org.bluez.Adapter1", "Name", dbus.String("TurtleBot3-Provisioning"))
        logging.info("Adapter configured: powered, discoverable, pairable, name=TurtleBot3-Provisioning")
        
        # Register GATT application
        gatt_manager = dbus.Interface(adapter_obj, "org.bluez.GattManager1")
        gatt_manager.RegisterApplication(dbus.ObjectPath(app_path), {}, timeout=5000)
        logging.info("GATT Application registered successfully")
        
    except dbus.exceptions.DBusException as e:
        logging.warning("GATT registration failed: %s", e)
    except Exception as e:
        logging.error("Error during GATT registration: %s", e)
    
    try:
        # Register LE Advertisement
        ad_manager = dbus.Interface(adapter_obj, "org.bluez.LEAdvertisingManager1")
        ad_manager.RegisterAdvertisement(dbus.ObjectPath(ad_path), {}, timeout=5000)
        logging.info("LE Advertisement registered: advertising service UUID %s", SERVICE_UUID)
        
    except dbus.exceptions.DBusException as e:
        logging.warning("LE Advertisement registration failed: %s", e)
    except Exception as e:
        logging.error("Error during LE Advertisement registration: %s", e)
    
    logging.info("BLE setup complete: device is advertising")
    
    return bus, app, service, ssid_char, password_char, status_char, ack_char


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )
    
    global context
    context = ProvisioningContext()
    
    try:
        bus, app, service, ssid_char, password_char, status_char, ack_char = setup_ble()
        
        logging.info("BLE Provisioning server started. Waiting for connections...")
        logging.info("Device name: TurtleBot3-Provisioning")
        
        mainloop = GLib.MainLoop()
        mainloop.run()
    except KeyboardInterrupt:
        logging.info("Provisioning server interrupted; shutting down")
    except Exception as exc:
        logging.error("Fatal error: %s", exc, exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
