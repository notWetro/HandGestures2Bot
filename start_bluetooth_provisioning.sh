#!/bin/bash
# =============================================================================
# Bluetooth WiFi Provisioning - Starter
# =============================================================================
# Runs the BLE provisioning server with the root privileges required for
# BlueZ/D-Bus access.
#
# Usage:
#   ./start_bluetooth_provisioning.sh
#
# Logs:
#   /tmp/turtlebot_logs/bluetooth_provisioning.log
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="/tmp/turtlebot_logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/bluetooth_provisioning.log"

CMD=(python3 "$SCRIPT_DIR/provisioning/bluetooth_wifi/bluetooth_provisioning.py")

if [ "$EUID" -ne 0 ]; then
  echo "Bluetooth provisioning needs sudo. You may be prompted for your password..."
  sudo -v
  echo "Starting Bluetooth provisioning (root) ..."
  sudo -E "${CMD[@]}" > "$LOG_FILE" 2>&1 &
else
  echo "Starting Bluetooth provisioning (already root) ..."
  "${CMD[@]}" > "$LOG_FILE" 2>&1 &
fi

PID=$!
echo "Bluetooth provisioning started (PID: $PID)"
echo "Log: $LOG_FILE"
