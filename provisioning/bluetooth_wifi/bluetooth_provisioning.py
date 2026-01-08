#!/usr/bin/env python3
"""Bluetooth Low Energy (BLE) Wi-Fi provisioning for TurtleBot3.

Uses bluezero library for BLE GATT server on Linux.
Accepts Wi-Fi credentials from Android/iOS clients and connects via NetworkManager.
"""
import json
import logging
import subprocess
import sys
import threading
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
last_error: Optional[str] = None

_provision_lock = threading.Lock()
_provision_thread: Optional[threading.Thread] = None


def _run(cmd, timeout: int = 10) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def _nmcli_cmd() -> Optional[str]:
    nmcli_paths = ["/usr/bin/nmcli", "/bin/nmcli", "nmcli"]
    for path in nmcli_paths:
        try:
            _run([path, "--version"], timeout=5)
            return path
        except FileNotFoundError:
            continue
        except Exception:
            continue
    return None


def _nmcli_get_active_connection(nmcli: str) -> Optional[Tuple[str, str]]:
    """Return (name, uuid) of active connection on WLAN_INTERFACE, if any."""
    try:
        res = _run([nmcli, "-t", "-f", "NAME,UUID,DEVICE", "con", "show", "--active"], timeout=5)
        if res.returncode != 0:
            return None
        for line in (res.stdout or "").splitlines():
            parts = line.split(":")
            if len(parts) >= 3 and parts[2] == WLAN_INTERFACE:
                return parts[0], parts[1]
    except Exception:
        return None
    return None


def _nmcli_restore_connection(nmcli: str, prev: Tuple[str, str]) -> None:
    name, uuid = prev
    logging.info("Restoring previous Wi-Fi connection: %s (%s)", name, uuid)
    # Try UUID first, then name
    _run([nmcli, "con", "up", "uuid", uuid, "ifname", WLAN_INTERFACE], timeout=20)
    _run([nmcli, "con", "up", "id", name, "ifname", WLAN_INTERFACE], timeout=20)


def _wpa_get_current_network_id() -> Optional[str]:
    try:
        res = _run(["wpa_cli", "-i", WLAN_INTERFACE, "status"], timeout=5)
        if res.returncode != 0:
            return None
        for line in (res.stdout or "").splitlines():
            if line.startswith("id="):
                return line.split("=", 1)[1].strip()
    except FileNotFoundError:
        return None
    except Exception:
        return None
    return None


def _wpa_restore_network(prev_id: str) -> None:
    logging.info("Restoring previous wpa_supplicant network id=%s", prev_id)
    _run(["wpa_cli", "-i", WLAN_INTERFACE, "enable_network", prev_id], timeout=5)
    _run(["wpa_cli", "-i", WLAN_INTERFACE, "select_network", prev_id], timeout=5)


def _wifi_is_already_connected_to(ssid: str) -> bool:
    """Best-effort check to avoid switching when already on target SSID."""
    nmcli = _nmcli_cmd()
    if not nmcli:
        return False
    try:
        res = _run([nmcli, "-t", "-f", "ACTIVE,SSID,DEVICE", "dev", "wifi"], timeout=5)
        if res.returncode != 0:
            return False
        for line in (res.stdout or "").splitlines():
            parts = line.split(":")
            if len(parts) >= 3 and parts[0] == "yes" and parts[2] == WLAN_INTERFACE:
                return parts[1] == ssid
    except Exception:
        return False
    return False


def connect_wifi(ssid: str, password: str) -> Tuple[bool, Optional[str]]:
    """Connect to Wi-Fi. Tries wpa_supplicant first, then nmcli."""

    # If already connected to that SSID, treat as success.
    if _wifi_is_already_connected_to(ssid):
        return True, None

    prev_wpa_id = _wpa_get_current_network_id()
    prev_nmcli = _nmcli_cmd()
    prev_nm_conn = _nmcli_get_active_connection(prev_nmcli) if prev_nmcli else None
    
    # Method 1: Try wpa_cli (for wpa_supplicant managed interfaces)
    try:
        # Add network
        result = subprocess.run(
            ["wpa_cli", "-i", WLAN_INTERFACE, "add_network"],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            network_id = result.stdout.strip().split('\n')[-1]
            
            # Set SSID
            subprocess.run(
                ["wpa_cli", "-i", WLAN_INTERFACE, "set_network", network_id, "ssid", f'"{ssid}"'],
                capture_output=True, timeout=5
            )
            
            # Set password
            subprocess.run(
                ["wpa_cli", "-i", WLAN_INTERFACE, "set_network", network_id, "psk", f'"{password}"'],
                capture_output=True, timeout=5
            )
            
            # Enable network
            subprocess.run(
                ["wpa_cli", "-i", WLAN_INTERFACE, "enable_network", network_id],
                capture_output=True, timeout=5
            )
            
            # Select network (disconnect from current and connect to new)
            result = subprocess.run(
                ["wpa_cli", "-i", WLAN_INTERFACE, "select_network", network_id],
                capture_output=True, text=True, timeout=5
            )
            
            if "OK" in result.stdout:
                # Wait for connection
                import time
                for _ in range(10):
                    time.sleep(1)
                    status = subprocess.run(
                        ["wpa_cli", "-i", WLAN_INTERFACE, "status"],
                        capture_output=True, text=True, timeout=5
                    )
                    if "wpa_state=COMPLETED" in status.stdout:
                        # Request IP via DHCP
                        subprocess.run(["dhclient", WLAN_INTERFACE], capture_output=True, timeout=15)
                        return True, None
                
                # Cleanup + restore previous Wi-Fi
                try:
                    _run(["wpa_cli", "-i", WLAN_INTERFACE, "remove_network", network_id], timeout=5)
                except Exception:
                    pass
                if prev_wpa_id is not None:
                    _wpa_restore_network(prev_wpa_id)
                return False, "Connection timeout (wpa_supplicant)"
            else:
                try:
                    _run(["wpa_cli", "-i", WLAN_INTERFACE, "remove_network", network_id], timeout=5)
                except Exception:
                    pass
                if prev_wpa_id is not None:
                    _wpa_restore_network(prev_wpa_id)
                return False, f"wpa_cli select failed: {result.stdout}"
    except FileNotFoundError:
        pass  # wpa_cli not available, try nmcli
    except Exception as e:
        logging.warning("wpa_cli method failed: %s, trying nmcli", e)
        if prev_wpa_id is not None:
            _wpa_restore_network(prev_wpa_id)
    
    # Method 2: Try nmcli (for NetworkManager managed interfaces)
    nmcli_cmd = _nmcli_cmd()
    if not nmcli_cmd:
        return False, "Neither wpa_cli nor nmcli available"
    
    cmd = [
        nmcli_cmd, "dev", "wifi", "connect", ssid,
        "password", password, "ifname", WLAN_INTERFACE,
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        if result.returncode != 0:
            reason = (result.stderr or result.stdout).strip() or f"nmcli failed with code {result.returncode}"
            # Restore previous connection if we had one
            if prev_nm_conn is not None:
                _nmcli_restore_connection(nmcli_cmd, prev_nm_conn)
            return False, reason
        return True, None
    except subprocess.TimeoutExpired:
        if prev_nm_conn is not None:
            _nmcli_restore_connection(nmcli_cmd, prev_nm_conn)
        return False, "Connection timeout (15s)"
    except Exception as e:
        if prev_nm_conn is not None:
            _nmcli_restore_connection(nmcli_cmd, prev_nm_conn)
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
    if last_error:
        payload["error"] = last_error
    return json.dumps(payload)


def try_provision():
    """Attempt Wi-Fi provisioning if both SSID and password are set."""
    global ssid_value, password_value, status_value, ip_value, last_error

    if not (ssid_value and password_value):
        return

    # Capture values and reset immediately so repeated writes don't trigger more work.
    ssid = ssid_value
    password = password_value
    ssid_value = None
    password_value = None

    def _worker(target_ssid: str, target_password: str) -> None:
        global status_value, ip_value, last_error
        try:
            logging.info("Connecting to Wi-Fi: %s", target_ssid)
            success, reason = connect_wifi(target_ssid, target_password)
            if success:
                import time
                time.sleep(2)  # Wait for IP assignment
                ip = get_wlan_ip()
                if ip:
                    status_value = "connected"
                    ip_value = ip
                    last_error = None
                    logging.info("Provisioning complete: %s -> %s", target_ssid, ip)
                else:
                    status_value = "failed"
                    last_error = "Connected but no IPv4 address"
                    logging.warning("Connected but no IPv4 address")
            else:
                status_value = "failed"
                last_error = reason
                ip_value = get_wlan_ip() or ip_value
                logging.warning("Wi-Fi connect failed: %s", reason)
        except Exception as exc:
            status_value = "failed"
            last_error = str(exc)
            logging.exception("Provisioning worker failed: %s", exc)
        finally:
            # Allow a future provisioning attempt
            with _provision_lock:
                global _provision_thread
                _provision_thread = None

    with _provision_lock:
        global _provision_thread
        if _provision_thread is not None and _provision_thread.is_alive():
            logging.info("Provisioning already in progress; ignoring new request")
            return
        status_value = "connecting"
        last_error = None
        _provision_thread = threading.Thread(target=_worker, args=(ssid, password), daemon=True)
        _provision_thread.start()


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

    # bluezero peripheral needs a GLib mainloop; fail fast with a clear message
    try:
        from gi.repository import GLib  # type: ignore
    except Exception as exc:
        logging.error("Missing GLib bindings for Python (python3-gi).")
        logging.error("Install on Ubuntu/Debian: sudo apt install -y python3-gi gir1.2-glib-2.0")
        logging.error("Details: %s", exc)
        sys.exit(3)
    
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
        mainloop = GLib.MainLoop()
        mainloop.run()
    except KeyboardInterrupt:
        logging.info("Shutting down...")


if __name__ == "__main__":
    main()
