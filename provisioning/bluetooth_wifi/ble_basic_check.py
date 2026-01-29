#!/usr/bin/env python3
import asyncio
import os
import sys
import json

from bleak import BleakScanner, BleakClient


SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
CHAR_STATUS_UUID = "12345678-1234-5678-1234-56789abcdef3"
CHAR_ACK_UUID = "12345678-1234-5678-1234-56789abcdef5"


TARGET_NAME = os.environ.get("TARGET_NAME", "TurtleBot3-Provisioning")
ADDRESS = os.environ.get("ADDRESS")  
SCAN_TIMEOUT = float(os.environ.get("SCAN_TIMEOUT", "6"))
CONNECT_TIMEOUT = float(os.environ.get("CONNECT_TIMEOUT", "15"))
ACK_JSON = os.environ.get("ACK_JSON", json.dumps({"status": "connected"}))


def pick_device(devices):
    
    for d in devices:
        name = (d.name or "").strip()
        if name == TARGET_NAME:
            return d
    
    for d in devices:
        if (d.name or "").strip():
            return d
    
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

            # Send client-side acknowledgement JSON to ACK characteristic
            try:
                payload = ACK_JSON.encode("utf-8")
                await client.write_gatt_char(CHAR_ACK_UUID, payload, response=True)
                print(f"ACK written to {CHAR_ACK_UUID}: {ACK_JSON}")
            except Exception as e:
                print(f"ACK write skipped: {e}")

            
            try:
                raw = await client.read_gatt_char(CHAR_STATUS_UUID)
                
                print(f"Status bytes: {raw}")
                try:
                    print("Status text:", raw.decode("utf-8"))
                except Exception:
                    pass
            except Exception as e:
                print(f"Direct status read skipped: {e}")

            # Confirm the service exists 
            try:
                services = None
                if hasattr(client, "get_services"):
                    try:
                        services = await client.get_services()  
                    except TypeError:
                        # Some versions have sync get_services
                        services = client.get_services()
                elif hasattr(client, "services"):
                    services = client.services  

                if services is not None:
                    has_service = any(getattr(s, "uuid", "").lower() == SERVICE_UUID.lower() for s in services)
                    print(f"Service present: {has_service}")
                else:
                    print("Service discovery unavailable on this Bleak version.")
            except Exception as e:
                print(f"Service discovery skipped: {e}")

            print("OK: Disconnecting")
        print("Done.")
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(5)


if __name__ == "__main__":
    asyncio.run(main())
