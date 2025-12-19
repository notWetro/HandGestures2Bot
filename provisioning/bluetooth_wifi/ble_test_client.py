#!/usr/bin/env python3
import asyncio
import json
import os
import sys
from typing import Optional

from bleak import BleakScanner, BleakClient

SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
CHAR_SSID_UUID = "12345678-1234-5678-1234-56789abcdef1"
CHAR_PASSWORD_UUID = "12345678-1234-5678-1234-56789abcdef2"
CHAR_STATUS_UUID = "12345678-1234-5678-1234-56789abcdef3"

TARGET_NAME = os.environ.get("TARGET_NAME", "TurtleBot3-Provisioning")
SSID = os.environ.get("SSID")
WIFI_PASSWORD = os.environ.get("WIFI_PASSWORD")
TIMEOUT_SEC = int(os.environ.get("TIMEOUT", "45"))


def pick_device(devices) -> Optional[str]:
    # Prefer exact name match
    for d in devices:
        if (d.name or "").strip() == TARGET_NAME:
            return d.address
    # Otherwise, prefer any that advertises our service UUID
    for d in devices:
        uuids = set((d.metadata or {}).get("uuids", []) or [])
        if SERVICE_UUID.lower() in {u.lower() for u in uuids}:
            return d.address
    # Fallback: if only one device found, return it
    if devices:
        return devices[0].address
    return None


async def main():
    if not SSID or not WIFI_PASSWORD:
        print("ERROR: Set SSID and WIFI_PASSWORD environment variables.")
        print("Example: SSID=MyWiFi WIFI_PASSWORD=secret1234 python3 ble_test_client.py")
        sys.exit(2)

    print("Scanning for BLE devices (5s)...")
    devices = await BleakScanner.discover(timeout=5.0)
    if not devices:
        print("No BLE devices found. Ensure robot is advertising and nearby.")
        sys.exit(1)

    target_addr = pick_device(devices)
    if not target_addr:
        print("Could not find target device. Set TARGET_NAME or retry scanning.")
        for d in devices:
            print(f"- {d.name} @ {d.address} uuids={(d.metadata or {}).get('uuids')}")
        sys.exit(1)

    print(f"Connecting to {target_addr} ...")
    async with BleakClient(target_addr) as client:
        if not client.is_connected:
            print("Failed to connect.")
            sys.exit(1)
        print("Connected.")

        # Write SSID and password
        print(f"Writing SSID: {SSID}")
        await client.write_gatt_char(CHAR_SSID_UUID, SSID.encode("utf-8"), response=True)
        print("Writing password: ******")
        await client.write_gatt_char(CHAR_PASSWORD_UUID, WIFI_PASSWORD.encode("utf-8"), response=True)

        # Poll status until connected/failed or timeout
        print("Waiting for status ...")
        deadline = asyncio.get_event_loop().time() + TIMEOUT_SEC
        last = None
        while asyncio.get_event_loop().time() < deadline:
            try:
                raw = await client.read_gatt_char(CHAR_STATUS_UUID)
                if raw:
                    try:
                        data = json.loads(raw.decode("utf-8"))
                        if data != last:
                            print("Status:", data)
                            last = data
                        if data.get("status") in {"connected", "failed"}:
                            break
                    except Exception:
                        print("Status (raw):", raw)
                await asyncio.sleep(1.5)
            except Exception as e:
                print("Read status error:", e)
                await asyncio.sleep(2)

        print("Done.")


if __name__ == "__main__":
    asyncio.run(main())
