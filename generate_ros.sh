#!/bin/bash

echo "Initializing ROS2 Project Structure inside HandGestures2Bot..."

# 1. Create Directory Structure
mkdir -p ros2_ws/src/hand_gestures_bot/hand_gestures_bot
mkdir -p ros2_ws/src/hand_gestures_bot/resource
mkdir -p .devcontainer

# 2. Create Codespaces Configuration (Enables Simulation)
echo "Creating .devcontainer config..."
cat << 'EOF' > .devcontainer/devcontainer.json
{
    "name": "ROS 2 HandGestures Bot",
    "image": "osrf/ros:humble-desktop",
    "customizations": {
        "vscode": {
            "extensions": [
                "ms-iot.vscode-ros",
                "ms-python.python"
            ]
        }
    },
    "postCreateCommand": "apt-get update && apt-get install -y python3-pip ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-simulations && pip3 install websockets",
    "runArgs": [
        "--network=host"
    ]
}
EOF

# 3. Create ROS2 Package Files
cd ros2_ws/src/hand_gestures_bot

echo "Creating package.xml..."
cat << 'EOF' > package.xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>hand_gestures_bot</name>
  <version>0.0.1</version>
  <description>ROS2 Node for Hand Gesture Control via WebSockets</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

echo "Creating setup.py..."
cat << 'EOF' > setup.py
from setuptools import setup

package_name = 'hand_gestures_bot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 Node for Hand Gesture Control via WebSockets',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start_server = hand_gestures_bot.server:main',
        ],
    },
)
EOF

echo "Creating setup.cfg..."
cat << 'EOF' > setup.cfg
[develop]
script_dir=$base/lib/hand_gestures_bot
[install]
install_scripts=$base/lib/hand_gestures_bot
EOF

# Create empty resource file
touch resource/hand_gestures_bot

# 4. Create Python Source Codes
cd hand_gestures_bot
touch __init__.py

echo "Creating turtlebot_controller.py (Hardware Interface)..."
cat << 'EOF' > turtlebot_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_command_time = time.time()
        self.timeout_duration = 0.5 
        self.timer = self.create_timer(0.1, self.watchdog_check)
        self.get_logger().info("TurtleBot Controller Initialized (Watchdog Active)")

    def move_forward(self):
        """Scenario 1: Forward Movement"""
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self._publish_command(msg)

    def stop(self):
        """Scenario 2: Stop Movement"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self._publish_command(msg)

    def _publish_command(self, msg):
        self.last_command_time = time.time()
        self.publisher_.publish(msg)

    def watchdog_check(self):
        """Scenario 4: Fail-safe Stop"""
        if (time.time() - self.last_command_time) > self.timeout_duration:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.publisher_.publish(stop_msg)
EOF

echo "Creating gesture_handler.py (Logic)..."
cat << 'EOF' > gesture_handler.py
class GestureHandler:
    def __init__(self, bot_controller):
        self.bot = bot_controller

    def handle_gesture(self, gesture_name):
        gesture = gesture_name.strip()
        if gesture == "Fist":
            self.bot.get_logger().info(f"Handler: Received '{gesture}' -> Moving Forward")
            self.bot.move_forward()
        elif gesture == "Open_Palm":
            self.bot.get_logger().info(f"Handler: Received '{gesture}' -> Stopping")
            self.bot.stop()
        else:
            self.bot.get_logger().warn(f"Handler: Unknown gesture '{gesture}'")
            self.bot.stop()
EOF

echo "Creating server.py (WebSocket Entry Point)..."
cat << 'EOF' > server.py
import rclpy
import asyncio
import websockets
import json
from threading import Thread
from .turtlebot_controller import TurtleBotController
from .gesture_handler import GestureHandler

gesture_handler = None

async def websocket_listener(websocket, path):
    global gesture_handler
    async for message in websocket:
        try:
            data = json.loads(message)
            current_gesture = data.get("gesture", None)
            if current_gesture and gesture_handler:
                gesture_handler.handle_gesture(current_gesture)
            else:
                print("Invalid JSON or Handler not ready")
        except Exception as e:
            print(f"WebSocket Error: {e}")

def main(args=None):
    global gesture_handler
    rclpy.init(args=args)
    bot_node = TurtleBotController()
    gesture_handler = GestureHandler(bot_node)

    ros_thread = Thread(target=rclpy.spin, args=(bot_node,), daemon=True)
    ros_thread.start()

    print("WebSocket Server listening on port 8765...")
    start_server = websockets.serve(websocket_listener, "0.0.0.0", 8765)

    loop = asyncio.get_event_loop()
    loop.run_until_complete(start_server)
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        bot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

echo "SUCCESS: ROS2 Package and Codespace config created!"
