#!/bin/bash
# =============================================================================
# TurtleBot3 Boot Script - Starts everything EXCEPT WebSocket
# =============================================================================
# WebSocket server is started by bluetooth_provisioning ONLY after user
# enters WiFi credentials via Bluetooth (not automatically).
# This script is meant for systemd auto-start on boot.
# =============================================================================

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

# Log file for this boot script
BOOT_LOG="$LOG_DIR/boot_startup.log"

# PID file to track background processes
PID_FILE="$LOG_DIR/robot_pids.txt"
> "$PID_FILE"  # Clear previous PIDs

# Script directory (where this script is located)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR/ros2_ws"

echo ""
echo "======================================="
echo "  TurtleBot3 Boot Startup"
echo "  $(date)"
echo "  (WebSocket starts after BT provisioning)"
echo "======================================="
echo ""

# =============================================================================
# Function: Wait for a condition with timeout
# =============================================================================
wait_for() {
    local description="$1"
    local check_cmd="$2"
    local timeout_sec="${3:-30}"
    local interval="${4:-2}"
    
    echo -n "   Waiting for $description..."
    local elapsed=0
    while [ $elapsed -lt $timeout_sec ]; do
        if eval "$check_cmd" > /dev/null 2>&1; then
            echo -e " ${GREEN}OK${NC}"
            return 0
        fi
        sleep $interval
        elapsed=$((elapsed + interval))
        echo -n "."
    done
    echo -e " ${RED}TIMEOUT${NC}"
    return 1
}

# =============================================================================
# Step 0: Wait for system to be ready (systemd boot delay)
# =============================================================================
echo -e "${BLUE}[0/7]${NC} Waiting for system to be ready..."
sleep 5
echo -e "${GREEN}✓${NC} System ready"

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
if colcon build --symlink-install 2>&1 | tee "$LOG_DIR/build.log"; then
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
export LDS_MODEL=LDS-02
echo -e "${GREEN}✓${NC} TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo -e "${GREEN}✓${NC} ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo -e "${GREEN}✓${NC} LDS_MODEL=$LDS_MODEL"

# =============================================================================
# Step 5: Launch TurtleBot3 bringup (background) - WAIT FOR IT TO BE READY
# =============================================================================
echo -e "${BLUE}[5/7]${NC} Launching TurtleBot3 bringup..."
ros2 launch turtlebot3_bringup robot.launch.py > "$LOG_DIR/turtlebot_bringup.log" 2>&1 &
BRINGUP_PID=$!
echo "$BRINGUP_PID turtlebot_bringup" >> "$PID_FILE"

sleep 3
if kill -0 $BRINGUP_PID 2>/dev/null; then
    echo -e "${GREEN}✓${NC} TurtleBot3 bringup started (PID: $BRINGUP_PID)"
    echo -e "   Log: $LOG_DIR/turtlebot_bringup.log"
else
    echo -e "${RED}✗ ERROR: TurtleBot3 bringup failed to start${NC}"
    echo -e "   Check log: $LOG_DIR/turtlebot_bringup.log"
    # Continue anyway - try to start other services
fi

# Wait for /scan topic to be available (LiDAR ready)
echo -e "   Waiting for LiDAR to be ready..."
wait_for "/scan topic" "ros2 topic list 2>/dev/null | grep -q '/scan'" 60 3
if [ $? -eq 0 ]; then
    echo -e "   ${GREEN}✓${NC} LiDAR topic detected"
else
    echo -e "   ${YELLOW}⚠ LiDAR not detected, continuing anyway${NC}"
fi

# Wait for /cmd_vel topic (robot motors ready)
wait_for "/cmd_vel topic" "ros2 topic list 2>/dev/null | grep -q '/cmd_vel'" 30 2
if [ $? -eq 0 ]; then
    echo -e "   ${GREEN}✓${NC} Motor control ready"
fi

# =============================================================================
# Step 6: Start Safety Scanner (background) - WAIT FOR IT
# =============================================================================
echo -e "${BLUE}[6/7]${NC} Starting Safety Scanner..."
cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0
export LDS_MODEL=LDS-02

SCAN_TOPIC_VALUE="${SCAN_TOPIC:-}"
if [ -z "$SCAN_TOPIC_VALUE" ]; then
    SCAN_TOPIC_VALUE=$(ros2 topic list 2>/dev/null | grep -E '/scan$' | head -n 1)
fi
if [ -z "$SCAN_TOPIC_VALUE" ]; then
    SCAN_TOPIC_VALUE="/scan"
fi

FAIL_SAFE_VALUE="${DISABLE_LIDAR_FAILSAFE:-0}"
if [ "$FAIL_SAFE_VALUE" = "1" ]; then
    echo -e "   ${RED}⚠️ LiDAR fail-safe DISABLED${NC}"
    FAIL_SAFE_ARG="-p fail_safe_on_scan_timeout:=false"
else
    FAIL_SAFE_ARG="-p fail_safe_on_scan_timeout:=true"
fi

echo -e "   Using scan topic: ${YELLOW}${SCAN_TOPIC_VALUE}${NC}"
ros2 run hand_gestures_bot safety_scanner --ros-args -p scan_topic:="$SCAN_TOPIC_VALUE" $FAIL_SAFE_ARG > "$LOG_DIR/safety_scanner.log" 2>&1 &
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

# Wait for safety scanner to be publishing
wait_for "/obstacle_status topic" "ros2 topic list 2>/dev/null | grep -q '/obstacle_status'" 15 2

# =============================================================================
# Step 7: Start Bluetooth Provisioning - WAIT FOR BLUETOOTH TO BE READY
# =============================================================================
echo -e "${BLUE}[7/7]${NC} Starting Bluetooth Provisioning..."
echo -e "   ${YELLOW}WebSocket will start ONLY after user enters WiFi via Bluetooth${NC}"

# Wait for Bluetooth to be ready
wait_for "Bluetooth adapter" "hciconfig hci0 2>/dev/null | grep -q 'UP RUNNING'" 30 2
if [ $? -ne 0 ]; then
    echo -e "   ${YELLOW}Trying to bring up Bluetooth...${NC}"
    sudo hciconfig hci0 up 2>/dev/null
    sleep 2
fi

cd "$SCRIPT_DIR"
# Use the start_bluetooth_provisioning.sh which handles sudo/permissions
bash "$SCRIPT_DIR/start_bluetooth_provisioning.sh" >> "$LOG_DIR/bluetooth_provisioning.log" 2>&1 &
BT_PID=$!
echo "$BT_PID bluetooth_provisioning" >> "$PID_FILE"

sleep 3
if kill -0 $BT_PID 2>/dev/null; then
    echo -e "${GREEN}✓${NC} Bluetooth Provisioning started (PID: $BT_PID)"
    echo -e "   Log: $LOG_DIR/bluetooth_provisioning.log"
else
    echo -e "${RED}✗ ERROR: Bluetooth Provisioning failed to start${NC}"
    echo -e "   Check log: $LOG_DIR/bluetooth_provisioning.log"
fi

# Wait for BLE GATT to be registered
sleep 3
if grep -q "GATT application registered" "$LOG_DIR/bluetooth_provisioning.log" 2>/dev/null; then
    echo -e "   ${GREEN}✓${NC} BLE GATT server ready"
else
    echo -e "   ${YELLOW}⚠ BLE GATT may still be initializing${NC}"
fi

# =============================================================================
# Done - Summary
# =============================================================================
echo ""
echo "======================================="
echo "  Boot Startup Complete!"
echo "  $(date)"
echo "======================================="
echo ""
echo "Running services:"
cat "$PID_FILE" | while read line; do
    pid=$(echo "$line" | awk '{print $1}')
    name=$(echo "$line" | awk '{print $2}')
    if kill -0 $pid 2>/dev/null; then
        echo "  ✓ $name (PID: $pid)"
    else
        echo "  ✗ $name (PID: $pid) - NOT RUNNING"
    fi
done
echo ""
echo "Log directory: $LOG_DIR"
echo ""
echo "STATUS:"
echo "  - TurtleBot3 bringup: Running"
echo "  - Safety Scanner: Running"
echo "  - Bluetooth Provisioning: Running (waiting for user)"
echo "  - WebSocket Server: NOT STARTED (starts after BT provisioning)"
echo ""
echo "To start WebSocket: Send WiFi credentials via Bluetooth app"
echo ""

# =============================================================================
# Keep the script alive for systemd (Type=simple requires main process to stay)
# =============================================================================
echo "Boot script staying alive to keep services running..."
echo "Press Ctrl+C to stop (or use: sudo systemctl stop turtlebot-handgestures)"

# Trap SIGTERM/SIGINT to cleanly stop when systemd stops the service
trap 'echo "Received stop signal, exiting..."; exit 0' SIGTERM SIGINT

# Wait forever - systemd will send SIGTERM when stopping
while true; do
    sleep 60
done
