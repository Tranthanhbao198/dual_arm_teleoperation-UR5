# 🤖 Dual UR5 Hand Teleoperation

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-yellow.svg)](LICENSE)

Real-time hand gesture teleoperation system for dual UR5 robotic arms using MediaPipe and ROS2.

---

## ✨ Features

- 🤚 **Hand Tracking Control** - Use webcam to control robot arms with hand gestures
- ⌨️ **Keyboard Control** - Alternative manual control mode
- 🎬 **Automatic Demo** - Wave animation showcase
- 📹 **RViz Visualization** - Real-time 3D visualization with meshes
- 🎮 **Dual Arm Support** - Independent control of left and right arms

---

## 🎯 Demo

**Hand Tracking Mode:**
- Move your hands → Robot arms follow
- Pinch fingers → Close gripper
- Release pinch → Open gripper

---

## 📋 Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS2 Humble
- **Hardware**: Webcam or iPad with Iriun

---

## 🚀 Quick Start

### 1. Install Dependencies
```bash
# Install ROS2 Humble (if not installed)
sudo apt update
sudo apt install -y ros-humble-desktop

# Install Python packages
pip3 install mediapipe opencv-python --user
```

### 2. Clone and Build
```bash
# Create workspace
mkdir -p ~/ros2_dual_arm_ws/src
cd ~/ros2_dual_arm_ws/src

# Clone repository
git clone https://github.com/YOUR_USERNAME/dual_arm_teleoperation.git

# Build
cd ~/ros2_dual_arm_ws
colcon build
source install/setup.bash
```

### 3. Run

**Terminal 1 - Launch Visualization:**
```bash
ros2 launch dual_arm_teleoperation display_for_keyboard.launch.py
```

**Terminal 2 - Start Hand Tracking:**
```bash
ros2 run dual_arm_teleoperation hand_teleop_node
```

**Controls:**
- Right hand controls LEFT arm (mirrored)
- Left hand controls RIGHT arm (mirrored)
- Press `Q` to quit

---

## 🎮 Control Modes

### 1. Hand Tracking (Primary)
```bash
ros2 run dual_arm_teleoperation hand_teleop_node
```

### 2. Keyboard Control
```bash
ros2 run dual_arm_teleoperation keyboard_teleop
```

**Keys:**
- `TAB` - Switch arm
- `1-6` - Select joint
- `+/-` or `w/s` - Move joint
- `o/c` - Open/close gripper
- `q` - Quit

### 3. Wave Demo
```bash
ros2 run dual_arm_teleoperation dual_arm_controller
```

---

## 📁 Project Structure
```
dual_arm_teleoperation/
├── dual_arm_teleoperation/     # Python nodes
│   ├── hand_teleop_node.py     # Hand tracking control
│   ├── keyboard_teleop.py      # Keyboard control
│   └── dual_arm_controller.py  # Wave demo
├── launch/                      # Launch files
│   └── display_for_keyboard.launch.py
├── urdf/                        # Robot description
│   └── dual_ur5.urdf.xacro
├── meshes/                      # 3D models
│   ├── visual/                  # .dae files
│   └── collision/               # .stl files
└── rviz/                        # Visualization config
    └── dual_arm.rviz
```

---

## 🔧 Technical Details

### Hand Tracking
- **Library**: MediaPipe Hands
- **Landmarks**: 21 points per hand
- **Control Rate**: 50Hz
- **Smoothing**: Exponential moving average (α=0.3)

### Robot
- **Model**: UR5 (6-DOF) x2
- **Gripper**: Simple 2-finger parallel
- **Workspace**: Scaled by factor 2.0

### ROS2
- **Topics**: `/joint_states` (sensor_msgs/JointState)
- **Node**: `hand_teleop_node`, `keyboard_teleop`, `dual_arm_controller`

---

## 🐛 Troubleshooting

### Camera not detected
```bash
# Check camera devices
ls /dev/video*

# Test camera
python3 -c "import cv2; cap = cv2.VideoCapture(0); ret, _ = cap.read(); print('OK' if ret else 'FAIL')"
```

### Robot not visible in RViz
```bash
# Verify joint states
ros2 topic hz /joint_states  # Should be ~50Hz
```

### Build errors
```bash
# Clean rebuild
cd ~/ros2_dual_arm_ws
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

---

## 📚 Documentation

- [Detailed Setup Guide](DETAILED_GUIDE.md) - Step-by-step tutorial
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [MediaPipe Hands](https://google.github.io/mediapipe/solutions/hands.html)

---

## 🤝 Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push and open a Pull Request

---

## 📝 License

Apache License 2.0 - See [LICENSE](LICENSE)

---

## 🙏 Acknowledgments

- **MediaPipe** by Google - Hand tracking
- **ROS2** by Open Robotics - Robotics middleware
- **UR5 Meshes** by Universal Robots

---

## 📧 Contact

**Author**: Thanh Bao  
**GitHub**: [@YOUR_USERNAME](https://github.com/YOUR_USERNAME)

---

⭐ **Star this repo if you find it helpful!**
