#!/bin/bash

echo "==============================="
echo " Building & Starting Robot Server"
echo "==============================="

set -e

# Source ROS2 (distroyu gerekirse değiştir)
source /opt/ros/humble/setup.bash

# Go to workspace
WS=~/HandGestures2Bot/ros2_ws
cd $WS

# Build workspace
echo "Running colcon build..."
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Environment variables
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "Starting server node (hand_gestures_bot)..."

# ✅ Correct entry point
ros2 run hand_gestures_bot start_server
