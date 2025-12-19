#!/usr/bin/env python3
import asyncio
import os
import sys

from bleak import BleakScanner, BleakClient

# Your robot's BLE identifiers
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
CHAR_STATUS_UUID = "12345678-1234-5678-1234-56789abcdef3"

# Optional overrides via env vars
TARGET_NAME = os.environ.get("TARGET_NAME", "TurtleBot3-Provisioning")
ADDRESS = os.environ.get("ADDRESS")  # e.g., AA:BB:CC:DD:EE:FF
SCAN_TIMEOUT = float(os.environ.get("SCAN_TIMEOUT", "6"))
CONNECT_TIMEOUT = float(os.environ.get("CONNECT_TIMEOUT", "15"))


def pick_device(devices):
    # Prefer exact name match; metadata may not exist on some Bleak versions
    for d in devices:
        name = (d.name or "").strip()
        if name == TARGET_NAME:
            return d
    # Fallback: first device with a non-empty name
    for d in devices:
        if (d.name or "").strip():
            return d
    # Last resort: first device seen
    return devices[0] if devices else None


async def main():
    target_address = ADDRESS

    if not target_address:
        print(f"Scanning for {TARGET_NAME} or service {SERVICE_UUID} ...")
        devices = await BleakScanner.discover(timeout=SCAN_TIMEOUT)
        if not devices:
            print("ERROR: No BLE devices seen. Ensure robot is advertising and nearby.")
            sys.exit(2)
        dev = pick_device(devices)
        if not dev:
            print("ERROR: Target not found. Devices seen:")
            for d in devices:
                print(f"- {d.name} @ {d.address}")
            sys.exit(3)
        target_address = dev.address

    print(f"Connecting to {target_address} ...")
    try:
        async with BleakClient(target_address, timeout=CONNECT_TIMEOUT) as client:
            if not client.is_connected:
                print("ERROR: Could not connect.")
                sys.exit(4)
            print("OK: Connected")

            # Confirm the service exists
            try:
                services = await client.get_services()
                has_service = any(s.uuid.lower() == SERVICE_UUID.lower() for s in services)
                print(f"Service present: {has_service}")
                # Optional single read of status characteristic if present
                if any(c.uuid.lower() == CHAR_STATUS_UUID.lower() for s in services for c in s.characteristics):
                    try:
                        raw = await client.read_gatt_char(CHAR_STATUS_UUID)
                        print(f"Status bytes: {raw}")
                    except Exception as e:
                        print(f"Status read skipped: {e}")
            except Exception as e:
                print(f"Service discovery skipped: {e}")

            print("OK: Disconnecting")
        print("Done.")
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(5)


if __name__ == "__main__":
    asyncio.run(main())
