# HandGestures2Bot

A multimodal robot control system for TurtleBot3 that enables intuitive interaction through hand gestures, voice commands, and AI-powered natural language processing. Control your robot with gestures captured by your smartphone camera, speak commands, or simply chat with the onboard Gemma AI assistant.

![Project Demo](screenshots/Cam%20Page.jpeg)

## Features

### Hand Gesture Control
- Real-time hand gesture recognition using smartphone camera
- Customizable gesture-to-command mappings
- Visual feedback for detected gestures
- MediaPipe-based hand tracking

### Voice Control
- Voice-to-text command recognition
- Text-to-speech feedback
- Natural language processing via Gemma AI
- Voice-activated robot movements

### AI Assistant Integration
- On-device Gemma AI model for natural language understanding
- Conversational interface for robot control
- Context-aware command interpretation

### Cross-Platform Flutter App
- Android, iOS, Windows, macOS, Linux support
- Real-time WebSocket communication with robot
- Bluetooth and WiFi configuration for robot
- Dance choreography creator
- Custom command sequences

### Safety Features
- LiDAR-based collision avoidance
- Automatic safety scanner with obstacle detection
- Emergency stop capability
- Configurable safety distance thresholds

## Architecture

```
┌─────────────────────────────────────────┐
│         Flutter Mobile App              │
│  (Hand Gestures, Voice, AI Interface)   │
└──────────────┬──────────────────────────┘
               │ WebSocket
               ▼
┌─────────────────────────────────────────┐
│       ROS2 Humble Backend               │
│  ┌─────────────────────────────────┐   │
│  │  WebSocket Server Node          │   │
│  │  (hand_gestures_bot)           │   │
│  └──────────┬──────────────────────┘   │
│             │                            │
│  ┌──────────▼──────────────────────┐   │
│  │  TurtleBot Controller           │   │
│  └──────────┬──────────────────────┘   │
│             │                            │
│  ┌──────────▼──────────────────────┐   │
│  │  Safety Scanner Node            │   │
│  │  (LiDAR Collision Avoidance)   │   │
│  └──────────┬──────────────────────┘   │
└─────────────┼──────────────────────────┘
              │ ROS2 /cmd_vel
              ▼
┌─────────────────────────────────────────┐
│       TurtleBot3 Hardware / Gazebo      │
└─────────────────────────────────────────┘
```

## Prerequisites

### Hardware
- **TurtleBot3 Burger** (or compatible robot)
- **LiDAR sensor** (LDS-02/LD08 recommended)
- **Raspberry Pi 4** or similar SBC for robot
- **Smartphone** (Android/iOS) with camera

### Software
- **ROS 2 Humble** Hawksbill
- **Ubuntu 22.04** (or compatible Linux distribution)
- **Flutter 3.9.2+**
- **Python 3.10+**
- **Gazebo** (for simulation)

## Installation

### 1. Clone Repository
```bash
git clone https://github.com/yourusername/HandGestures2Bot.git
cd HandGestures2Bot
```

### 2. Install ROS 2 Dependencies
```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Install TurtleBot3 packages
sudo apt update
sudo apt install ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs

# Build workspace
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Install Flutter App
```bash
cd flutterapp
flutter pub get
flutter run
```

### 4. Configure Environment
```bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export LDS_MODEL=LDS-02  # For newer TurtleBot3 models
```

## Usage

### Simulation Mode

Start the complete simulation stack:
```bash
./start_sim.sh
```

This will:
1. Build the ROS 2 workspace
2. Launch Gazebo simulation
3. Start the safety scanner
4. Start the WebSocket server

Test the simulation:
```bash
./test_sim.sh
```

Stop all services:
```bash
./stop_sim.sh
```

### Real Robot Mode

Start all robot services:
```bash
./start_robot.sh
```

This will:
1. Build the ROS 2 workspace
2. Launch TurtleBot3 bringup
3. Start Bluetooth provisioning
4. Start the safety scanner
5. Start the WebSocket server

Test the robot:
```bash
./test_robot.sh
```

Stop all services:
```bash
./stop_robot.sh
```

### Manual Mode

Start only the WebSocket server:
```bash
./start_server.sh
```

### Bluetooth Provisioning (Robot WiFi Setup)

Configure robot WiFi via Bluetooth from the Flutter app:
```bash
./start_bluetooth_provisioning.sh
```

## Mobile App Features

### Gesture Configuration
- Open the app and navigate to "Gesture Configuration"
- Train custom gestures by recording movements
- Map gestures to robot commands (forward, backward, left, right, stop)
- Test gesture recognition in real-time

### Voice Commands
- Navigate to "Voice Page"
- Tap microphone icon to speak
- Say commands like "move forward", "turn left", "stop"
- AI assistant provides natural language responses

### Command Sequences
- Create custom command sequences
- Chain multiple movements
- Save and replay dance choreographies
- Share sequences with other users

### Bluetooth & WiFi Setup
- Connect to robot via Bluetooth
- Configure robot WiFi credentials
- Test connection status
- Monitor network information


## Project Statistics

```
Language                Files    Code    Comments    Blank
--------------------------------------------------------
Dart                      23     3098        125      315
Bourne Shell              11     1265        349      231
Swift                     16     1170        148      298
Python                     8     1087        204      252
Kotlin                     6      821        100      144
C++                        8      448         70      124
TeX                        1      255         28       76
--------------------------------------------------------
Total                    140    11602       1304     1877
```

## Project Structure

```
HandGestures2Bot/
├── flutterapp/              # Flutter mobile application
│   ├── lib/                 # Dart source code
│   ├── assets/              # App assets (images, sounds)
│   └── pubspec.yaml         # Flutter dependencies
├── ros2_ws/                 # ROS 2 workspace
│   └── src/
│       └── hand_gestures_bot/  # Main ROS 2 package
│           ├── websocket_server.py
│           ├── turtlebot_controller.py
│           └── safety_scanner.py
├── provisioning/            # Robot provisioning scripts
│   ├── bluetooth_wifi/      # Bluetooth WiFi setup
│   └── systemd/             # Systemd service files
├── doc/                     # Documentation
│   ├── Latex/              # LaTeX documentation
│   └── decisions/          # Architecture decision records
├── screenshots/             # App screenshots
├── start_sim.sh            # Start simulation
├── start_robot.sh          # Start real robot
├── start_server.sh         # Start WebSocket server only
├── stop_sim.sh             # Stop simulation
├── stop_robot.sh           # Stop real robot
├── test_sim.sh             # Test simulation
└── test_robot.sh           # Test real robot
```


## Acknowledgments

- **ROBOTIS** - TurtleBot3 platform
- **Flutter Team** - Cross-platform framework
- **ROS 2 Community** - Robot Operating System
- **MediaPipe** - Hand tracking solution
- **Google** - Gemma AI model

## Contact

For questions or support, please open an issue on GitHub.

---

**Made for Mobile and Embedded Development**
