#!/bin/bash


# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

LOG_DIR="/tmp/turtlebot_logs"
PID_FILE="$LOG_DIR/robot_pids.txt"

echo -e "${CYAN}=======================================${NC}"
echo -e "${CYAN}  TurtleBot3 Real Robot - Stopping    ${NC}"
echo -e "${CYAN}=======================================${NC}"
echo ""

# Function to stop a process
stop_process() {
    local pid="$1"
    local name="$2"
    
    if kill -0 "$pid" 2>/dev/null; then
        echo -e "  Stopping $name (PID: $pid)..."
        kill "$pid" 2>/dev/null
        sleep 1
        # Force kill if still running
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null
        fi
        echo -e "  ${GREEN}✓${NC} $name stopped"
    else
        echo -e "  ${YELLOW}→${NC} $name (PID: $pid) not running"
    fi
}

# Stop processes from PID file
if [ -f "$PID_FILE" ]; then
    echo -e "${YELLOW}Stopping services from PID file...${NC}"
    while read line; do
        pid=$(echo "$line" | awk '{print $1}')
        name=$(echo "$line" | awk '{print $2}')
        stop_process "$pid" "$name"
    done < "$PID_FILE"
    rm -f "$PID_FILE"
else
    echo -e "${YELLOW}No PID file found. Searching for processes...${NC}"
fi

# Also kill any remaining related processes
echo ""
echo -e "${YELLOW}Checking for remaining processes...${NC}"

# Kill TurtleBot bringup
pkill -f "turtlebot3_bringup" 2>/dev/null && echo -e "  ${GREEN}✓${NC} Killed turtlebot3_bringup"
pkill -f "robot.launch.py" 2>/dev/null && echo -e "  ${GREEN}✓${NC} Killed robot.launch.py"

# Kill Bluetooth provisioning
pkill -f "bluetooth_provisioning" 2>/dev/null && echo -e "  ${GREEN}✓${NC} Killed bluetooth_provisioning"

# Kill WebSocket server and Safety Scanner (force kill with -9)
pkill -9 -f "start_server" 2>/dev/null && echo -e "  ${GREEN}✓${NC} Killed start_server"
pkill -9 -f "safety_scanner" 2>/dev/null && echo -e "  ${GREEN}✓${NC} Killed safety_scanner"
pkill -9 -f "hand_gestures_bot" 2>/dev/null && echo -e "  ${GREEN}✓${NC} Killed hand_gestures_bot"

# Kill any process using port 8765 (WebSocket port)
if command -v lsof > /dev/null 2>&1; then
    PORT_PID=$(lsof -ti:8765 2>/dev/null)
    if [ -n "$PORT_PID" ]; then
        kill -9 $PORT_PID 2>/dev/null && echo -e "  ${GREEN}✓${NC} Killed process on port 8765"
    fi
elif command -v fuser > /dev/null 2>&1; then
    fuser -k 8765/tcp 2>/dev/null && echo -e "  ${GREEN}✓${NC} Killed process on port 8765"
fi

# Wait for cleanup
sleep 2

# Reset ROS 2 daemon to clear any stale state
echo ""
echo -e "${YELLOW}Resetting ROS 2 daemon...${NC}"
ros2 daemon stop 2>/dev/null
ros2 daemon start 2>/dev/null
echo -e "${GREEN}✓${NC} ROS 2 daemon restarted"

echo ""
echo -e "${GREEN}All services stopped.${NC}"
echo ""
