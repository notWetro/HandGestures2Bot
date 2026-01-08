#!/bin/bash
# =============================================================================
# TurtleBot3 Real Robot - Complete Startup Script
# =============================================================================
# This script does the following in sequence:
# 1. Sources ROS 2 Humble
# 2. Builds the ROS 2 workspace
# 3. Sources the workspace setup
# 4. Exports TurtleBot3 environment variables
# 5. Launches TurtleBot3 bringup (background)
# 6. Starts Bluetooth provisioning (background)
# 7. Starts the Safety Scanner (background)
# 8. Starts the WebSocket server (background)
#
# All background processes are logged to files in /tmp/turtlebot_logs/
# =============================================================================

set -e  # Exit on first error during build phase

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Log directory
LOG_DIR="/tmp/turtlebot_logs"
mkdir -p "$LOG_DIR"

# PID file to track background processes
PID_FILE="$LOG_DIR/robot_pids.txt"
> "$PID_FILE"  # Clear previous PIDs

# Script directory (where this script is located)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR/ros2_ws"

echo -e "${CYAN}=======================================${NC}"
echo -e "${CYAN}  TurtleBot3 Real Robot - Full Startup${NC}"
echo -e "${CYAN}=======================================${NC}"
echo ""

# =============================================================================
# Step 1: Source ROS 2 Humble
# =============================================================================
echo -e "${BLUE}[1/8]${NC} Sourcing ROS 2 Humble..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✓${NC} ROS 2 Humble sourced successfully"
else
    echo -e "${RED}✗ ERROR: ROS 2 Humble not found at /opt/ros/humble/setup.bash${NC}"
    exit 1
fi

# =============================================================================
# Step 2: Build the ROS 2 workspace
# =============================================================================
echo -e "${BLUE}[2/8]${NC} Building ROS 2 workspace..."
cd "$WS"
colcon build --symlink-install 2>&1 | tee "$LOG_DIR/build.log"
if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo -e "${GREEN}✓${NC} Workspace built successfully"
else
    echo -e "${RED}✗ ERROR: Build failed. Check $LOG_DIR/build.log${NC}"
    exit 1
fi

# =============================================================================
# Step 3: Source workspace setup
# =============================================================================
echo -e "${BLUE}[3/8]${NC} Sourcing workspace setup..."
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
    echo -e "${GREEN}✓${NC} Workspace setup sourced"
else
    echo -e "${RED}✗ ERROR: Workspace install/setup.bash not found${NC}"
    exit 1
fi

# =============================================================================
# Step 4: Export TurtleBot3 environment variables
# =============================================================================
echo -e "${BLUE}[4/8]${NC} Setting environment variables..."
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
echo -e "${GREEN}✓${NC} TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo -e "${GREEN}✓${NC} ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# From here on, don't exit on error - we want to try starting everything
set +e

# =============================================================================
# Step 5: Launch TurtleBot3 bringup (background)
# =============================================================================
echo -e "${BLUE}[5/8]${NC} Launching TurtleBot3 bringup..."
ros2 launch turtlebot3_bringup robot.launch.py > "$LOG_DIR/turtlebot_bringup.log" 2>&1 &
BRINGUP_PID=$!
echo "$BRINGUP_PID turtlebot_bringup" >> "$PID_FILE"

# Wait a bit to check if it started successfully
sleep 3
if kill -0 $BRINGUP_PID 2>/dev/null; then
    echo -e "${GREEN}✓${NC} TurtleBot3 bringup started (PID: $BRINGUP_PID)"
    echo -e "   Log: $LOG_DIR/turtlebot_bringup.log"
else
    echo -e "${RED}✗ ERROR: TurtleBot3 bringup failed to start${NC}"
    echo -e "   Check log: $LOG_DIR/turtlebot_bringup.log"
    # Continue anyway to try other services
fi

# =============================================================================
# Step 6: Start Bluetooth provisioning (background)
# =============================================================================
echo -e "${BLUE}[6/8]${NC} Starting Bluetooth provisioning..."

# Bluetooth needs root privileges. If we are not root, prompt once and start via sudo.
BT_LOG="$LOG_DIR/bluetooth_provisioning.log"

if [ "$EUID" -ne 0 ]; then
    echo -e "${YELLOW}⚠${NC} Bluetooth provisioning requires root privileges"
    echo -e "   Requesting sudo (you may be prompted)..."
    sudo -v
    sudo -E bash "$SCRIPT_DIR/start_bluetooth_provisioning.sh"
else
    bash "$SCRIPT_DIR/start_bluetooth_provisioning.sh"
fi

# Try to discover the process PID for reporting
sleep 2
BT_PID=$(pgrep -f "bluetooth_provisioning.py" | head -n 1)
if [ -n "$BT_PID" ] && kill -0 "$BT_PID" 2>/dev/null; then
    echo "$BT_PID bluetooth_provisioning" >> "$PID_FILE"
    echo -e "${GREEN}✓${NC} Bluetooth provisioning started (PID: $BT_PID)"
    echo -e "   Log: $BT_LOG"
else
    echo -e "${RED}✗ ERROR: Bluetooth provisioning failed to start${NC}"
    echo -e "   Check log: $BT_LOG"
fi

# =============================================================================
# Step 7: Start Safety Scanner (background)
# =============================================================================
echo -e "${BLUE}[7/8]${NC} Starting Safety Scanner..."
cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0
ros2 run hand_gestures_bot safety_scanner > "$LOG_DIR/safety_scanner.log" 2>&1 &
SCANNER_PID=$!
echo "$SCANNER_PID safety_scanner" >> "$PID_FILE"

sleep 2
if kill -0 $SCANNER_PID 2>/dev/null; then
    echo -e "${GREEN}✓${NC} Safety Scanner started (PID: $SCANNER_PID)"
    echo -e "   Log: $LOG_DIR/safety_scanner.log"
else
    echo -e "${RED}✗ ERROR: Safety Scanner failed to start${NC}"
    echo -e "   Check log: $LOG_DIR/safety_scanner.log"
fi

# =============================================================================
# Step 8: Start WebSocket server (background)
# =============================================================================
echo -e "${BLUE}[8/8]${NC} Starting WebSocket server..."
cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0
ros2 run hand_gestures_bot start_server > "$LOG_DIR/websocket_server.log" 2>&1 &
SERVER_PID=$!
echo "$SERVER_PID websocket_server" >> "$PID_FILE"

sleep 2
if kill -0 $SERVER_PID 2>/dev/null; then
    echo -e "${GREEN}✓${NC} WebSocket server started (PID: $SERVER_PID)"
    echo -e "   Log: $LOG_DIR/websocket_server.log"
else
    echo -e "${RED}✗ ERROR: WebSocket server failed to start${NC}"
    echo -e "   Check log: $LOG_DIR/websocket_server.log"
fi

# =============================================================================
# Summary
# =============================================================================
echo ""
echo -e "${CYAN}=======================================${NC}"
echo -e "${CYAN}           Startup Complete           ${NC}"
echo -e "${CYAN}=======================================${NC}"
echo ""
echo -e "${GREEN}All services are running in background.${NC}"
echo ""
echo -e "Log files:"
echo -e "  • Build:       $LOG_DIR/build.log"
echo -e "  • Bringup:     $LOG_DIR/turtlebot_bringup.log"
echo -e "  • Bluetooth:   $LOG_DIR/bluetooth_provisioning.log"
echo -e "  • Scanner:     $LOG_DIR/safety_scanner.log"
echo -e "  • Server:      $LOG_DIR/websocket_server.log"
echo ""
echo -e "PIDs saved to: $PID_FILE"
echo ""
echo -e "To stop all services, run:"
echo -e "  ${YELLOW}./stop_robot.sh${NC}"
echo ""
echo -e "To test if services are running:"
echo -e "  ${YELLOW}./test_robot.sh${NC}"
echo ""
