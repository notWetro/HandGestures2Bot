#!/bin/bash
# =============================================================================
# TurtleBot3 Simulation - Stop Script
# =============================================================================
# Stops all background services started by start_sim.sh
# =============================================================================

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

LOG_DIR="/tmp/turtlebot_sim_logs"
PID_FILE="$LOG_DIR/sim_pids.txt"

echo -e "${CYAN}=======================================${NC}"
echo -e "${CYAN}  TurtleBot3 Simulation - Stopping    ${NC}"
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

# Kill Gazebo
pkill -f "gzserver" 2>/dev/null && echo -e "  ${GREEN}✓${NC} Killed gzserver"
pkill -f "gzclient" 2>/dev/null && echo -e "  ${GREEN}✓${NC} Killed gzclient"
pkill -f "gazebo" 2>/dev/null && echo -e "  ${GREEN}✓${NC} Killed gazebo"
pkill -f "turtlebot3_gazebo" 2>/dev/null && echo -e "  ${GREEN}✓${NC} Killed turtlebot3_gazebo"

# Kill WebSocket server
pkill -f "start_server" 2>/dev/null && echo -e "  ${GREEN}✓${NC} Killed start_server"
pkill -f "hand_gestures_bot" 2>/dev/null && echo -e "  ${GREEN}✓${NC} Killed hand_gestures_bot"

echo ""
echo -e "${GREEN}All simulation services stopped.${NC}"
echo ""
