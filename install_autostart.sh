#!/bin/bash
# =============================================================================
# Install TurtleBot3 Hand Gestures Auto-Start Service
# =============================================================================
# Run this script ONCE to enable auto-start on boot.
# Usage: sudo ./install_autostart.sh
# =============================================================================

set -e

SERVICE_NAME="turtlebot-handgestures"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_FILE="$SCRIPT_DIR/provisioning/systemd/turtlebot-handgestures.service"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Please run with sudo: sudo $0${NC}"
    exit 1
fi

echo -e "${YELLOW}Installing TurtleBot3 Hand Gestures auto-start service...${NC}"
echo ""

# Create log directory
mkdir -p /tmp/turtlebot_logs
chown turtlebot:turtlebot /tmp/turtlebot_logs

# Copy service file
cp "$SERVICE_FILE" /etc/systemd/system/
echo -e "${GREEN}✓${NC} Copied service file to /etc/systemd/system/"

# Reload systemd
systemctl daemon-reload
echo -e "${GREEN}✓${NC} Reloaded systemd daemon"

# Enable service (start on boot)
systemctl enable "$SERVICE_NAME"
echo -e "${GREEN}✓${NC} Enabled auto-start on boot"

echo ""
echo -e "${GREEN}Installation complete!${NC}"
echo ""
echo "Commands:"
echo "  Start now:     sudo systemctl start $SERVICE_NAME"
echo "  Stop:          sudo systemctl stop $SERVICE_NAME"
echo "  Status:        sudo systemctl status $SERVICE_NAME"
echo "  Disable:       sudo systemctl disable $SERVICE_NAME"
echo "  View logs:     journalctl -u $SERVICE_NAME -f"
echo ""
echo -e "${YELLOW}The robot will auto-start on next boot.${NC}"
echo -e "${YELLOW}To start now: sudo systemctl start $SERVICE_NAME${NC}"
