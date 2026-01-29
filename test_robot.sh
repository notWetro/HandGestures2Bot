#!/bin/bash
# =============================================================================
# TurtleBot3 Real Robot - Test Script
# =============================================================================
# This script tests if all robot services are running correctly:
# 1. Checks if TurtleBot3 bringup is running (via ROS topics)
# 2. Checks if Bluetooth provisioning is running
# 3. Checks if WebSocket server is running
# 4. Tests ROS topic availability (/scan, /cmd_vel)
# 5. Tests connectivity
# =============================================================================

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Source ROS 2
source /opt/ros/humble/setup.bash 2>/dev/null

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR/ros2_ws"

# Source workspace if available
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash" 2>/dev/null
fi

export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0

echo -e "${CYAN}=======================================${NC}"
echo -e "${CYAN}  TurtleBot3 Real Robot - Test Suite  ${NC}"
echo -e "${CYAN}=======================================${NC}"
echo ""

# Track test results
TESTS_PASSED=0
TESTS_FAILED=0

# =============================================================================
# Test function
# =============================================================================
run_test() {
    local test_name="$1"
    local test_result="$2"  # 0 = pass, 1 = fail
    
    if [ "$test_result" -eq 0 ]; then
        echo -e "  ${GREEN} PASS${NC}: $test_name"
        ((TESTS_PASSED++))
    else
        echo -e "  ${RED} FAIL${NC}: $test_name"
        ((TESTS_FAILED++))
    fi
}

# =============================================================================
# Test 1: Check if ROS 2 daemon is running
# =============================================================================
echo -e "${BLUE}[Test 1]${NC} ROS 2 Daemon Status"
ros2 daemon status > /dev/null 2>&1
run_test "ROS 2 daemon is running" $?

# =============================================================================
# Test 2: Check TurtleBot3 bringup via process
# =============================================================================
echo -e "${BLUE}[Test 2]${NC} TurtleBot3 Bringup Process"
if pgrep -f "turtlebot3_bringup" > /dev/null 2>&1 || pgrep -f "robot.launch.py" > /dev/null 2>&1; then
    run_test "TurtleBot3 bringup process found" 0
else
    run_test "TurtleBot3 bringup process found" 1
fi

# =============================================================================
# Test 3: Check /scan topic exists
# =============================================================================
echo -e "${BLUE}[Test 3]${NC} LaserScan Topic (/scan)"
SCAN_TOPIC=$(ros2 topic list 2>/dev/null | grep -w "/scan")
if [ -n "$SCAN_TOPIC" ]; then
    run_test "/scan topic exists" 0
    
    # Check if data is being published
    echo -e "  ${YELLOW}→${NC} Checking for data (5 second timeout)..."
    SCAN_DATA=$(timeout 5 ros2 topic echo /scan --once 2>/dev/null | head -n 5)
    if [ -n "$SCAN_DATA" ]; then
        run_test "/scan is publishing data" 0
    else
        run_test "/scan is publishing data" 1
    fi
else
    run_test "/scan topic exists" 1
fi

# =============================================================================
# Test 4: Check /cmd_vel topic exists
# =============================================================================
echo -e "${BLUE}[Test 4]${NC} Velocity Command Topic (/cmd_vel)"
CMD_VEL_TOPIC=$(ros2 topic list 2>/dev/null | grep -w "/cmd_vel")
if [ -n "$CMD_VEL_TOPIC" ]; then
    run_test "/cmd_vel topic exists" 0
else
    run_test "/cmd_vel topic exists" 1
fi

# =============================================================================
# Test 5: Check Bluetooth provisioning process
# =============================================================================
echo -e "${BLUE}[Test 5]${NC} Bluetooth Provisioning"
if pgrep -f "bluetooth_provisioning" > /dev/null 2>&1; then
    run_test "Bluetooth provisioning process running" 0
else
    run_test "Bluetooth provisioning process running" 1
    echo -e "  ${YELLOW}→${NC} Note: Bluetooth requires sudo to run"
fi

# Check if Bluetooth adapter is available
if command -v hciconfig > /dev/null 2>&1; then
    BT_STATUS=$(hciconfig 2>/dev/null | grep "UP RUNNING")
    if [ -n "$BT_STATUS" ]; then
        run_test "Bluetooth adapter is UP" 0
    else
        run_test "Bluetooth adapter is UP" 1
    fi
else
    echo -e "  ${YELLOW}→${NC} hciconfig not available, skipping adapter check"
fi

# =============================================================================
# Test 6: Check Safety Scanner process
# =============================================================================
echo -e "${BLUE}[Test 6]${NC} Safety Scanner"
if pgrep -f "safety_scanner" > /dev/null 2>&1; then
    run_test "Safety Scanner process running" 0
else
    run_test "Safety Scanner process running" 1
fi

# Check /obstacle_status topic
OBSTACLE_TOPIC=$(ros2 topic list 2>/dev/null | grep -w "/obstacle_status")
if [ -n "$OBSTACLE_TOPIC" ]; then
    run_test "/obstacle_status topic exists" 0
else
    run_test "/obstacle_status topic exists" 1
fi

# Check /cmd_vel_in topic (safety scanner input)
CMD_VEL_IN_TOPIC=$(ros2 topic list 2>/dev/null | grep -w "/cmd_vel_in")
if [ -n "$CMD_VEL_IN_TOPIC" ]; then
    run_test "/cmd_vel_in topic exists" 0
else
    run_test "/cmd_vel_in topic exists" 1
fi

# =============================================================================
# Test 7: Check WebSocket server process
# =============================================================================
echo -e "${BLUE}[Test 7]${NC} WebSocket Server"
if pgrep -f "start_server" > /dev/null 2>&1 || pgrep -f "hand_gestures_bot" > /dev/null 2>&1; then
    run_test "WebSocket server process running" 0
else
    run_test "WebSocket server process running" 1
fi

# Check if port 8765 is listening (common WebSocket port)
if command -v ss > /dev/null 2>&1; then
    WS_PORT=$(ss -tlnp 2>/dev/null | grep ":8765")
    if [ -n "$WS_PORT" ]; then
        run_test "WebSocket port 8765 is listening" 0
    else
        run_test "WebSocket port 8765 is listening" 1
        echo -e "  ${YELLOW}→${NC} Server might be using a different port"
    fi
elif command -v netstat > /dev/null 2>&1; then
    WS_PORT=$(netstat -tlnp 2>/dev/null | grep ":8765")
    if [ -n "$WS_PORT" ]; then
        run_test "WebSocket port 8765 is listening" 0
    else
        run_test "WebSocket port 8765 is listening" 1
    fi
fi

# =============================================================================
# Test 8: Check ROS nodes
# =============================================================================
echo -e "${BLUE}[Test 8]${NC} Active ROS 2 Nodes"
NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
if [ "$NODE_COUNT" -gt 0 ]; then
    run_test "ROS 2 nodes are running ($NODE_COUNT nodes)" 0
    echo -e "  ${YELLOW}→${NC} Active nodes:"
    ros2 node list 2>/dev/null | head -n 10 | while read node; do
        echo -e "       • $node"
    done
else
    run_test "ROS 2 nodes are running" 1
fi

# =============================================================================
# Test 9: Network connectivity
# =============================================================================
echo -e "${BLUE}[Test 9]${NC} Network Connectivity"
# Get local IP
LOCAL_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
if [ -n "$LOCAL_IP" ]; then
    run_test "Local IP address available: $LOCAL_IP" 0
else
    run_test "Local IP address available" 1
fi

# =============================================================================
# Summary
# =============================================================================
echo ""
echo -e "${CYAN}=======================================${NC}"
echo -e "${CYAN}            Test Summary              ${NC}"
echo -e "${CYAN}=======================================${NC}"
echo ""
echo -e "  Tests Passed: ${GREEN}$TESTS_PASSED${NC}"
echo -e "  Tests Failed: ${RED}$TESTS_FAILED${NC}"
echo ""

TOTAL_TESTS=$((TESTS_PASSED + TESTS_FAILED))
if [ "$TESTS_FAILED" -eq 0 ]; then
    echo -e "${GREEN} All tests passed! Robot is ready.${NC}"
    exit 0
elif [ "$TESTS_PASSED" -gt "$TESTS_FAILED" ]; then
    echo -e "${YELLOW}Some tests failed. Check the issues above.${NC}"
    exit 1
else
    echo -e "${RED}Multiple tests failed. Robot may not be functioning correctly.${NC}"
    exit 1
fi
