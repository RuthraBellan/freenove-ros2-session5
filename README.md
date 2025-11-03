# ROS 2 Session 5: Interactive Installation & Lane Following

Interactive learning materials for installing ROS 2 Humble and building an autonomous lane-following robot with the Freenove 4WD Smart Car Kit.

**Course:** Engineering Teamwork III - AI and Autonomous Systems Lab  
**Institution:** Berlin University of Applied Sciences

---

## Prerequisites

- Raspberry Pi 4 (2GB+ RAM)
- **Ubuntu Server 22.04 LTS (64-bit)** flashed on SD card
- Freenove 4WD Smart Car Kit (assembled)
- SSH access to your Pi
- Internet connection

---

## Quick Start for Students

### 1. SSH into Your Raspberry Pi
```bash
ssh pi@robot1.local
# Password: raspberry
```

### 2. Download and Run Interactive Installation
```bash
wget https://raw.githubusercontent.com/RuthraBellan/freenove-ros2-session5/main/setup/interactive_install_resumable.sh
chmod +x interactive_install.sh
./interactive_install.sh
```

### 3. Follow Along with Activity Sheet
- Download: [activity_sheet.md](docs/activity_sheet.md)
- Write explanations as the script runs
- Complete all sections during the lab

⏱️ **Total time:** 2.5-3 hours

---

## Repository Structure
```
freenove-ros2-session5/
├── setup/                    # Installation scripts
│   ├── interactive_install.sh    # Main interactive script 
│   ├── commands.md              # All commands reference
│   └── QUICK_REFERENCE.md       # Student cheat sheet
├── nodes/                    # ROS 2 Python nodes
│   ├── camera_node.py
│   ├── motor_control_node.py
│   └── lane_follower_node.py
├── mock/                     # Testing without hardware
│   ├── mock_motor.py
│   └── mock_picamera2.py
└── docs/                     # Documentation
    └── activity_sheet.md        # Student worksheet
```

---

## For Students

### What You'll Learn
- Install and configure ROS 2 Humble
- Understand what each installation command does
- Build a ROS 2 workspace
- Create a three-node autonomous system
- Use computer vision for lane following
- Debug with ROS 2 tools

### Session Flow
1. **Interactive Installation** (45 min) - Learn while installing
2. **Workspace Setup** (30 min) - Build your ROS package
3. **Run Three Nodes** (30 min) - Camera, lane follower, motor control
4. **Test & Debug** (45 min) - Make the robot follow lanes

---

## What Gets Installed

- **ROS 2 Humble Base** - Core framework
- **Development Tools** - colcon, rosdep
- **Computer Vision** - cv_bridge, OpenCV
- **Python Libraries** - NumPy for image processing

**Disk space:** ~5GB

---

## Detailed Instructions

### Option 1: Interactive Learning (Recommended)
Run `interactive_install.sh` - explains each command before executing.
Perfect for learning and understanding each step!

### Option 2: Manual Installation
Follow [commands.md](setup/commands.md) step-by-step if you prefer
to type commands yourself or need to troubleshoot.

---

## After Installation

### Download Node Code
```bash
cd ~
git clone https://github.com/RuthraBellan/freenove-ros2-session5.git
```

### Create Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create freenove_car --build-type ament_python \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs cv_bridge
```

### Copy Node Files
```bash
cd ~/ros2_ws/src/freenove_car/freenove_car
cp ~/freenove-ros2-session5/nodes/*.py .
chmod +x *.py
```

### Build
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Run
```bash
# Terminal 1
ros2 run freenove_car camera_node

# Terminal 2
ros2 run freenove_car lane_follower_node

# Terminal 3
sudo -E ros2 run freenove_car motor_control_node
```

---

## Troubleshooting

### "ros2: command not found"
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### "Package 'freenove_car' not found"
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Camera not working
```bash
ls /dev/video0  # Check if camera detected
# If not found, check connections and reboot
```

See [QUICK_REFERENCE.md](setup/QUICK_REFERENCE.md) for more debugging tips.

---

## Related Resources

- **Official Freenove Code:** [Freenove 4WD Kit](https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi)
- **ROS 2 Documentation:** [docs.ros.org](https://docs.ros.org/en/humble/)
- **OpenCV Tutorials:** [opencv.org](https://docs.opencv.org/)

---

## Contributing

Found an issue? Have a suggestion?
- Open an issue
- Submit a pull request
- Contact the instructor

---

## License

Educational use - SRH University of Applied Sciences

---

## Acknowledgments

- Freenove for the excellent robotics platform
- ROS 2 community
- OpenCV project
- SRH University of Applied Sciences

