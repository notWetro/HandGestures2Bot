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

# Make logs readable for the normal 'turtlebot' user even when started via sudo.
chmod 755 "$LOG_DIR" 2>/dev/null || true

# Always create the log file and capture this script's output to it.
: > "$LOG_FILE"
chmod 644 "$LOG_FILE" 2>/dev/null || true
exec > >(tee -a "$LOG_FILE") 2>&1

CMD=(python3 "$SCRIPT_DIR/provisioning/bluetooth_wifi/bluetooth_provisioning.py")

echo "Checking Bluetooth provisioning dependencies..."
if ! command -v python3 >/dev/null 2>&1; then
  echo "ERROR: python3 not found"
  exit 2
fi

if ! command -v bluetoothctl >/dev/null 2>&1; then
  echo "WARN: bluetoothctl not found (bluez may be missing)"
fi

if ! command -v nmcli >/dev/null 2>&1; then
  echo "WARN: nmcli not found (NetworkManager may be missing)"
fi

python3 - <<'PY'
try:
    import netifaces  # noqa: F401
    from bluezero import peripheral  # noqa: F401
    from bluezero import adapter  # noqa: F401
except Exception as e:
    print("ERROR: Python dependency missing:")
    print(e)
    print("\nFix (recommended):")
    print("  sudo apt update")
    print("  sudo apt install -y python3-pip bluez network-manager")
    print("  sudo pip3 install -U bluezero netifaces")
    raise SystemExit(3)
print("OK: Python deps present (bluezero, netifaces)")
PY

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
