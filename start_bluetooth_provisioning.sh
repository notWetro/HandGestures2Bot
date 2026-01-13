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
PID_FILE="$LOG_DIR/bluetooth_provisioning.pid"

# Make logs readable for the normal 'turtlebot' user even when started via sudo.
chmod 755 "$LOG_DIR" 2>/dev/null || true

# Always create the log file and capture this script's output to it.
: > "$LOG_FILE"
chmod 644 "$LOG_FILE" 2>/dev/null || true
exec > >(tee -a "$LOG_FILE") 2>&1

CMD=(python3 "$SCRIPT_DIR/provisioning/bluetooth_wifi/bluetooth_provisioning.py")

ensure_bluetooth_ready() {
  echo "Ensuring bluetooth service/controller is ready..."

  if command -v systemctl >/dev/null 2>&1; then
    systemctl start bluetooth >/dev/null 2>&1 || true
  fi

  if command -v bluetoothctl >/dev/null 2>&1; then
    # These can fail harmlessly if controller isn't ready yet.
    bluetoothctl power on >/dev/null 2>&1 || true
    bluetoothctl discoverable-timeout 0 >/dev/null 2>&1 || true
    bluetoothctl pairable-timeout 0 >/dev/null 2>&1 || true
    bluetoothctl discoverable on >/dev/null 2>&1 || true
    bluetoothctl pairable on >/dev/null 2>&1 || true
    bluetoothctl agent NoInputNoOutput >/dev/null 2>&1 || true
    bluetoothctl default-agent >/dev/null 2>&1 || true

    # Wait a bit for the controller to show up
    for _ in $(seq 1 10); do
      if bluetoothctl list 2>/dev/null | grep -qi "controller"; then
        echo "OK: Bluetooth controller detected"
        return 0
      fi
      sleep 1
    done

    echo "WARN: No Bluetooth controller detected yet (continuing anyway)"
    return 0
  fi

  echo "WARN: bluetoothctl not available; cannot power on controller"
  return 0
}

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

ensure_bluetooth_ready

if [ "$EUID" -ne 0 ]; then
  echo "Bluetooth provisioning needs root privileges."
  if sudo -n true 2>/dev/null; then
    echo "Re-running as root via sudo -n (no password prompt) ..."
    exec sudo -n -E bash "$0"
  fi
  echo "ERROR: sudo would prompt for a password (not allowed in this script)."
  echo "Fix options (choose one):"
  echo "  1) Configure NOPASSWD for this script in /etc/sudoers.d (recommended)"
  echo "     Example (edit with visudo):"
  echo "       <YOUR_USER> ALL=(root) NOPASSWD: /bin/bash $SCRIPT_DIR/start_bluetooth_provisioning.sh"
  echo "  2) Run this script as root"
  echo "  3) Run provisioning as a systemd service started at boot"
  exit 20
else
  echo "Starting Bluetooth provisioning (already root) ..."
  # Detach from this shell so the server keeps running after start_robot.sh returns.
  nohup "${CMD[@]}" </dev/null >> "$LOG_FILE" 2>&1 &
fi

PID=$!
echo "$PID" > "$PID_FILE" 2>/dev/null || true
chmod 644 "$PID_FILE" 2>/dev/null || true
echo "Bluetooth provisioning started (PID: $PID)"
echo "Log: $LOG_FILE"

# Quick health check
sleep 2
if ! kill -0 "$PID" 2>/dev/null; then
  echo "ERROR: Provisioning process exited immediately. Last log lines:"
  tail -n 50 "$LOG_FILE" || true
  exit 10
fi
