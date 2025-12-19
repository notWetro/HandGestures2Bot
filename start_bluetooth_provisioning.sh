#!/bin/bash
# Simple script to start Bluetooth provisioning manually without systemd

echo "Starting Bluetooth Wi-Fi Provisioning..."
echo "This script must be run with root privileges (sudo)"
echo "Press Ctrl+C to stop the service"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: This script must be run as root"
    echo "Please run: sudo ./start_bluetooth_provisioning.sh"
    exit 1
fi

# Change to the bluetooth_wifi directory
cd "$(dirname "$0")/provisioning/bluetooth_wifi" || exit 1

# Run the provisioning script
python3 bluetooth_provisioning.py
