#!/usr/bin/env python3
"""Quick BLE scan to see all visible devices."""
import asyncio
from bleak import BleakScanner

async def main():
    print("Scanning for BLE devices (15 seconds)...")
    devices = await BleakScanner.discover(timeout=15)
    print(f"Found {len(devices)} BLE devices:")
    for d in devices:
        name = d.name if d.name else "(no name)"
        print(f"  {name} @ {d.address}")

if __name__ == "__main__":
    asyncio.run(main())
