#!/bin/bash

echo "==============================="
echo " Starting TurtleBot Burger"
echo "==============================="

# Exit immediately if something fails
set -e

# Source ROS2 (change distro if needed: humble / iron / jazzy)
source /opt/ros/humble/setup.bash

# Source workspace (if it exists)
WS=~/HandGestures2Bot/ros2_ws
if [ -f "$WS/install/setup.bash" ]; then
    source $WS/install/setup.bash
else
    echo "⚠️  Workspace not built yet. Skipping workspace source."
fi

# Export TurtleBot model
export TURTLEBOT3_MODEL=burger

# Optional: network sanity
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo "Launching TurtleBot bringup..."

# Launch TurtleBot
ros2 launch turtlebot3_bringup robot.launch.py
