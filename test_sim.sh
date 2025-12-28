#!/bin/bash
# =============================================================================
# TurtleBot3 Simulation - Test Script
# =============================================================================
# This script tests if all simulation services are running correctly:
# 1. Checks if Gazebo simulation is running
# 2. Checks if WebSocket server is running
# 3. Tests ROS topic availability (/scan, /cmd_vel)
# 4. Tests simulated sensor data
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
echo -e "${CYAN}  TurtleBot3 Simulation - Test Suite  ${NC}"
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
        echo -e "  ${GREEN}✓ PASS${NC}: $test_name"
        ((TESTS_PASSED++))
    else
        echo -e "  ${RED}✗ FAIL${NC}: $test_name"
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
# Test 2: Check Gazebo process
# =============================================================================
echo -e "${BLUE}[Test 2]${NC} Gazebo Simulation Process"
if pgrep -f "gzserver" > /dev/null 2>&1 || pgrep -f "gazebo" > /dev/null 2>&1; then
    run_test "Gazebo server process running" 0
else
    run_test "Gazebo server process running" 1
fi

if pgrep -f "gzclient" > /dev/null 2>&1; then
    run_test "Gazebo client (GUI) running" 0
else
    run_test "Gazebo client (GUI) running" 1
    echo -e "  ${YELLOW}→${NC} GUI might be disabled (headless mode)"
fi

# =============================================================================
# Test 3: Check simulation-specific topics
# =============================================================================
echo -e "${BLUE}[Test 3]${NC} Simulation Topics"

# Check /clock (simulation time)
CLOCK_TOPIC=$(ros2 topic list 2>/dev/null | grep -w "/clock")
if [ -n "$CLOCK_TOPIC" ]; then
    run_test "/clock topic exists (simulation time)" 0
else
    run_test "/clock topic exists (simulation time)" 1
fi

# Check /gazebo topics
GAZEBO_TOPICS=$(ros2 topic list 2>/dev/null | grep -c "gazebo" || echo "0")
if [ "$GAZEBO_TOPICS" -gt 0 ]; then
    run_test "Gazebo topics found ($GAZEBO_TOPICS topics)" 0
else
    run_test "Gazebo topics found" 1
fi

# =============================================================================
# Test 4: Check /scan topic exists
# =============================================================================
echo -e "${BLUE}[Test 4]${NC} LaserScan Topic (/scan)"
SCAN_TOPIC=$(ros2 topic list 2>/dev/null | grep -w "/scan")
if [ -n "$SCAN_TOPIC" ]; then
    run_test "/scan topic exists" 0
    
    # Check if data is being published
    echo -e "  ${YELLOW}→${NC} Checking for simulated scan data (5 second timeout)..."
    SCAN_DATA=$(timeout 5 ros2 topic echo /scan --once 2>/dev/null | head -n 5)
    if [ -n "$SCAN_DATA" ]; then
        run_test "/scan is publishing simulated data" 0
    else
        run_test "/scan is publishing simulated data" 1
    fi
else
    run_test "/scan topic exists" 1
fi

# =============================================================================
# Test 5: Check /cmd_vel topic exists
# =============================================================================
echo -e "${BLUE}[Test 5]${NC} Velocity Command Topic (/cmd_vel)"
CMD_VEL_TOPIC=$(ros2 topic list 2>/dev/null | grep -w "/cmd_vel")
if [ -n "$CMD_VEL_TOPIC" ]; then
    run_test "/cmd_vel topic exists" 0
else
    run_test "/cmd_vel topic exists" 1
fi

# =============================================================================
# Test 6: Check WebSocket server process
# =============================================================================
echo -e "${BLUE}[Test 6]${NC} WebSocket Server"
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
# Test 7: Check ROS nodes
# =============================================================================
echo -e "${BLUE}[Test 7]${NC} Active ROS 2 Nodes"
NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
if [ "$NODE_COUNT" -gt 0 ]; then
    run_test "ROS 2 nodes are running ($NODE_COUNT nodes)" 0
    echo -e "  ${YELLOW}→${NC} Active nodes:"
    ros2 node list 2>/dev/null | head -n 15 | while read node; do
        echo -e "       • $node"
    done
else
    run_test "ROS 2 nodes are running" 1
fi

# =============================================================================
# Test 8: Check robot model in simulation
# =============================================================================
echo -e "${BLUE}[Test 8]${NC} Robot Model in Simulation"
# Check for TurtleBot-specific nodes/topics
TB_TOPICS=$(ros2 topic list 2>/dev/null | grep -c "turtlebot" || echo "0")
DIFF_DRIVE=$(ros2 topic list 2>/dev/null | grep -c "diff_drive" || echo "0")
ODOM=$(ros2 topic list 2>/dev/null | grep -w "/odom")

if [ "$TB_TOPICS" -gt 0 ] || [ "$DIFF_DRIVE" -gt 0 ]; then
    run_test "TurtleBot3 model loaded in simulation" 0
elif [ -n "$ODOM" ]; then
    run_test "Robot model detected (via /odom)" 0
else
    run_test "TurtleBot3 model loaded in simulation" 1
fi

# =============================================================================
# Test 9: Test sending a velocity command
# =============================================================================
echo -e "${BLUE}[Test 9]${NC} Velocity Command Test"
echo -e "  ${YELLOW}→${NC} Sending zero velocity command..."

# Try to publish a zero velocity message
timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" > /dev/null 2>&1
if [ $? -eq 0 ]; then
    run_test "Can publish to /cmd_vel" 0
else
    run_test "Can publish to /cmd_vel" 1
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
    echo -e "${GREEN}✓ All tests passed! Simulation is ready.${NC}"
    echo ""
    echo -e "${CYAN}Tip:${NC} You can now:"
    echo -e "  • Control the robot from your Flutter app"
    echo -e "  • Run: ros2 run hand_gestures_bot environment_scanner"
    echo -e "  • Use teleop: ros2 run turtlebot3_teleop teleop_keyboard"
    exit 0
elif [ "$TESTS_PASSED" -gt "$TESTS_FAILED" ]; then
    echo -e "${YELLOW}⚠ Some tests failed. Check the issues above.${NC}"
    exit 1
else
    echo -e "${RED}✗ Multiple tests failed. Simulation may not be functioning correctly.${NC}"
    exit 1
fi
