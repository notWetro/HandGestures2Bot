#!/bin/bash

echo "==============================="
echo " Starting TurtleBot3 Simulation"
echo "==============================="

set -e

# Source ROS2 (change distro if needed)
source /opt/ros/humble/setup.bash

# Source workspace (if it exists)
WS=~/HandGestures2Bot/ros2_ws
if [ -f "$WS/install/setup.bash" ]; then
    source $WS/install/setup.bash
else
    echo "⚠️  Workspace not built yet. Skipping workspace source."
fi

export TURTLEBOT3_MODEL=burger

# Optional: network sanity
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo "Launching TurtleBot3 simulation in Gazebo..."

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py