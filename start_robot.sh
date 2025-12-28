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
# 7. Starts the WebSocket server (background)
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
echo -e "${BLUE}[1/7]${NC} Sourcing ROS 2 Humble..."
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
echo -e "${BLUE}[2/7]${NC} Building ROS 2 workspace..."
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
echo -e "${BLUE}[3/7]${NC} Sourcing workspace setup..."
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
echo -e "${BLUE}[4/7]${NC} Setting environment variables..."
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
echo -e "${BLUE}[5/7]${NC} Launching TurtleBot3 bringup..."
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
echo -e "${BLUE}[6/7]${NC} Starting Bluetooth provisioning..."

# Check if we're running as root for Bluetooth
if [ "$EUID" -eq 0 ]; then
    cd "$SCRIPT_DIR/provisioning/bluetooth_wifi"
    python3 bluetooth_provisioning.py > "$LOG_DIR/bluetooth_provisioning.log" 2>&1 &
    BT_PID=$!
    echo "$BT_PID bluetooth_provisioning" >> "$PID_FILE"
    
    sleep 2
    if kill -0 $BT_PID 2>/dev/null; then
        echo -e "${GREEN}✓${NC} Bluetooth provisioning started (PID: $BT_PID)"
        echo -e "   Log: $LOG_DIR/bluetooth_provisioning.log"
    else
        echo -e "${RED}✗ ERROR: Bluetooth provisioning failed to start${NC}"
        echo -e "   Check log: $LOG_DIR/bluetooth_provisioning.log"
    fi
else
    echo -e "${YELLOW}⚠${NC} Bluetooth provisioning requires root privileges"
    echo -e "   Skipping. Run separately with: sudo $SCRIPT_DIR/start_bluetooth_provisioning.sh"
fi

# =============================================================================
# Step 7: Start WebSocket server (background)
# =============================================================================
echo -e "${BLUE}[7/7]${NC} Starting WebSocket server..."
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
