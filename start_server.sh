#!/bin/bash

echo "==============================="
echo " Building & Starting Robot Server"
echo "==============================="

set -e

# Source ROS2 (distroyu gerekirse değiştir)
source /opt/ros/humble/setup.bash


SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR/ros2_ws"
cd "$WS"


echo "Running colcon build..."
colcon build --symlink-install


source install/setup.bash


export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "Starting server node (hand_gestures_bot)..."


ros2 run hand_gestures_bot start_server
