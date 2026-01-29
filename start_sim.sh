#!/bin/bash
# =============================================================================
# TurtleBot3 Simulation - Complete Startup Script
# =============================================================================
# This script does the following in sequence:
# 1. Sources ROS 2 Humble
# 2. Builds the ROS 2 workspace
# 3. Sources the workspace setup
# 4. Exports TurtleBot3 environment variables
# 5. Launches Gazebo simulation (background)
# 6. Starts the Safety Scanner (background)
# 7. Starts the WebSocket server (background)
#
# NOTE: Bluetooth provisioning is NOT started in simulation mode
#       (it's only needed for real robot Wi-Fi setup)
#
# All background processes are logged to files in /tmp/turtlebot_sim_logs/
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
LOG_DIR="/tmp/turtlebot_sim_logs"
mkdir -p "$LOG_DIR"

# PID file to track background processes
PID_FILE="$LOG_DIR/sim_pids.txt"
> "$PID_FILE"  # Clear previous PIDs

# Script directory (where this script is located)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR/ros2_ws"

echo -e "${CYAN}=======================================${NC}"
echo -e "${CYAN}  TurtleBot3 Simulation - Full Startup${NC}"
echo -e "${CYAN}=======================================${NC}"
echo ""

# =============================================================================
# Step 1: Source ROS 2 Humble
# =============================================================================
echo -e "${BLUE}[1/5]${NC} Sourcing ROS 2 Humble..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}${NC} ROS 2 Humble sourced successfully"
else
    echo -e "${RED} ERROR: ROS 2 Humble not found at /opt/ros/humble/setup.bash${NC}"
    exit 1
fi

# =============================================================================
# Step 2: Build the ROS 2 workspace
# =============================================================================
echo -e "${BLUE}[2/7]${NC} Building ROS 2 workspace..."
cd "$WS"
colcon build --symlink-install 2>&1 | tee "$LOG_DIR/build.log"
if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo -e "${GREEN}${NC} Workspace built successfully"
else
    echo -e "${RED} ERROR: Build failed. Check $LOG_DIR/build.log${NC}"
    exit 1
fi

# =============================================================================
# Step 3: Source workspace setup
# =============================================================================
echo -e "${BLUE}[3/7]${NC} Sourcing workspace setup..."
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
    echo -e "${GREEN}${NC} Workspace setup sourced"
else
    echo -e "${RED} ERROR: Workspace install/setup.bash not found${NC}"
    exit 1
fi

# =============================================================================
# Step 4: Export TurtleBot3 environment variables
# =============================================================================
echo -e "${BLUE}[4/7]${NC} Setting environment variables..."
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
echo -e "${GREEN}${NC} TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo -e "${GREEN}${NC} ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# From here on, don't exit on error - we want to try starting everything
set +e

# =============================================================================
# Step 5: Launch Gazebo Simulation (background)
# =============================================================================
echo -e "${BLUE}[5/7]${NC} Launching Gazebo simulation..."
echo -e "${YELLOW}${NC} Gazebo may take some time to load..."

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py > "$LOG_DIR/gazebo_sim.log" 2>&1 &
GAZEBO_PID=$!
echo "$GAZEBO_PID gazebo_simulation" >> "$PID_FILE"

# Wait a bit longer for Gazebo to initialize
echo -e "   Waiting for Gazebo to initialize (10 seconds)..."
sleep 10

if kill -0 $GAZEBO_PID 2>/dev/null; then
    echo -e "${GREEN}${NC} Gazebo simulation started (PID: $GAZEBO_PID)"
    echo -e "   Log: $LOG_DIR/gazebo_sim.log"
else
    echo -e "${RED} ERROR: Gazebo simulation failed to start${NC}"
    echo -e "   Check log: $LOG_DIR/gazebo_sim.log"
    echo -e "   Common issues:"
    echo -e "     - Gazebo not installed: sudo apt install ros-humble-turtlebot3-gazebo"
    echo -e "     - Display not available (check DISPLAY env variable)"
    exit 1
fi

# =============================================================================
# Step 6: Start Safety Scanner (background)
# =============================================================================
echo -e "${BLUE}[6/7]${NC} Starting Safety Scanner..."
cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0

# Allow overriding the scan topic (useful if sim/robot namespaces it).
SCAN_TOPIC_VALUE="${SCAN_TOPIC:-}"
if [ -z "$SCAN_TOPIC_VALUE" ]; then
    SCAN_TOPIC_VALUE=$(ros2 topic list 2>/dev/null | grep -E '/scan$' | head -n 1)
fi
if [ -z "$SCAN_TOPIC_VALUE" ]; then
    SCAN_TOPIC_VALUE="/scan"
fi

echo -e "   Using scan topic: ${YELLOW}${SCAN_TOPIC_VALUE}${NC}"
ros2 run hand_gestures_bot safety_scanner --ros-args -p scan_topic:="$SCAN_TOPIC_VALUE" > "$LOG_DIR/safety_scanner.log" 2>&1 &
SCANNER_PID=$!
echo "$SCANNER_PID safety_scanner" >> "$PID_FILE"

sleep 2
if kill -0 $SCANNER_PID 2>/dev/null; then
    echo -e "${GREEN}${NC} Safety Scanner started (PID: $SCANNER_PID)"
    echo -e "   Log: $LOG_DIR/safety_scanner.log"
else
    echo -e "${RED} ERROR: Safety Scanner failed to start${NC}"
    echo -e "   Check log: $LOG_DIR/safety_scanner.log"
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
    echo -e "${GREEN}${NC} WebSocket server started (PID: $SERVER_PID)"
    echo -e "   Log: $LOG_DIR/websocket_server.log"
else
    echo -e "${RED} ERROR: WebSocket server failed to start${NC}"
    echo -e "   Check log: $LOG_DIR/websocket_server.log"
fi

# =============================================================================
# Summary
# =============================================================================
echo ""
echo -e "${CYAN}=======================================${NC}"
echo -e "${CYAN}       Simulation Startup Complete    ${NC}"
echo -e "${CYAN}=======================================${NC}"
echo ""
echo -e "${GREEN}All services are running in background.${NC}"
echo ""
echo -e "Log files:"
echo -e "  • Build:   $LOG_DIR/build.log"
echo -e "  • Gazebo:  $LOG_DIR/gazebo_sim.log"
echo -e "  • Scanner: $LOG_DIR/safety_scanner.log"
echo -e "  • Server:  $LOG_DIR/websocket_server.log"
echo ""
echo -e "PIDs saved to: $PID_FILE"
echo ""
echo -e "To stop all services, run:"
echo -e "  ${YELLOW}./stop_sim.sh${NC}"
echo ""
echo -e "To test if services are running:"
echo -e "  ${YELLOW}./test_sim.sh${NC}"
echo ""
echo -e "${CYAN}Tip:${NC} You can send commands from your Flutter app now!"
echo ""
