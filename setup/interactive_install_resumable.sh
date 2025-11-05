#!/bin/bash
# Complete Interactive Session 5 Script
# ROS 2 Humble Installation + Workspace Setup + Lane Following
# Engineering Teamwork III - AI and Autonomous Systems Lab
# Berlin University of Applied Sciences

# Colors for better visibility
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

# Configuration
GITHUB_REPO="https://github.com/RuthraBellan/freenove-ros2-session5.git"

# Checkpoint system
PROGRESS_FILE="$HOME/.ros2_session5_progress"
CHECKPOINT_DIR="$HOME/.ros2_session5_checkpoints"

# Create checkpoint directory
mkdir -p "$CHECKPOINT_DIR"

# Checkpoint functions
save_checkpoint() {
    local step_name="$1"
    echo "$step_name" > "$PROGRESS_FILE"
    touch "$CHECKPOINT_DIR/$step_name"
}

is_checkpoint_complete() {
    local step_name="$1"
    [ -f "$CHECKPOINT_DIR/$step_name" ]
}

get_last_checkpoint() {
    if [ -f "$PROGRESS_FILE" ]; then
        cat "$PROGRESS_FILE"
    else
        echo ""
    fi
}

command_exists() {
    command -v "$1" >/dev/null 2>&1
}
FREENOVE_REPO="https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi"

# Functions
wait_for_user() {
    echo ""
    echo -e "${BLUE}✍️  Write this in your activity sheet, then press Enter to continue...${NC}"
    read -r
}

wait_for_confirmation() {
    echo ""
    echo -e "${CYAN}Press Enter when ready to continue...${NC}"
    read -r
}

execute_command() {
    echo -e "${GREEN}➤ Executing...${NC}"
    echo ""
    eval "$1"
    local status=$?
    echo ""
    if [ $status -eq 0 ]; then
        echo -e "${GREEN}✓ Command completed successfully!${NC}"
    else
        echo -e "${RED}✗ Command failed with error code: $status${NC}"
        echo "Don't worry - we'll troubleshoot this."
    fi
    echo ""
    read -p "Press Enter to continue..."
    return $status
}

show_step() {
    clear
    echo "=========================================="
    echo "  $1"
    echo "=========================================="
    echo ""
}

show_explanation() {
    echo -e "${YELLOW}📚 WHAT THIS DOES:${NC}"
    echo "$1"
    echo ""
}

show_command() {
    echo -e "${BLUE}💻 COMMAND:${NC}"
    echo "  $1"
    echo ""
}

show_manual_task() {
    echo -e "${MAGENTA}👤 MANUAL TASK:${NC}"
    echo "$1"
    echo ""
}

ask_yes_no() {
    while true; do
        read -p "$1 (y/n): " yn
        case $yn in
            [Yy]* ) return 0;;
            [Nn]* ) return 1;;
            * ) echo "Please answer y or n.";;
        esac
    done
}

# Welcome Screen
clear
cat << "EOF"
==========================================
   🤖 ROS 2 Complete Interactive Setup
   Session 5: ROS 2 Integration
==========================================
EOF
echo ""
echo "Welcome to the COMPLETE Session 5 interactive guide!"
echo ""
echo "This script will take you through:"
echo "  ✓ Part 1: Install ROS 2 Humble"
echo "  ✓ Part 2: Hardware Verification"
echo "  ✓ Part 3: Download Code"
echo "  ✓ Part 4: Create Workspace"
echo "  ✓ Part 5: Configure Package"
echo "  ✓ Part 6: Build Workspace"
echo "  ✓ Part 7: Run System"
echo "  ✓ Part 8: Debugging Tools"
echo ""
echo "HOW IT WORKS:"
echo "  1. Read the explanation"
echo "  2. Write it in your activity sheet"
echo "  3. Press Enter to execute"
echo "  4. Observe the result"
echo ""
echo "⏱️  Total time: 2.5-3 hours"
echo ""
echo "Ready? Let's begin!"
echo ""
read -p "Press Enter to start..."

# Check for previous progress (ONE TIME CHECK)
LAST_CHECKPOINT=$(get_last_checkpoint)
if [ -n "$LAST_CHECKPOINT" ]; then
    echo -e "${YELLOW}══════════════════════════════════════${NC}"
    echo -e "${YELLOW}   📌 PREVIOUS SESSION DETECTED!${NC}"
    echo -e "${YELLOW}══════════════════════════════════════${NC}"
    echo ""
    echo "Last completed: $LAST_CHECKPOINT"
    echo ""
    if ask_yes_no "Do you want to RESUME from where you left off?"; then
        RESUME_MODE=true
        echo ""
        echo -e "${GREEN}✓ Resuming session...${NC}"
        echo "Skipping completed steps."
        echo ""
        sleep 2
    else
        RESUME_MODE=false
        echo ""
        if ask_yes_no "Start fresh? This will reset all progress."; then
            rm -rf "$CHECKPOINT_DIR"
            rm -f "$PROGRESS_FILE"
            mkdir -p "$CHECKPOINT_DIR"
            echo -e "${YELLOW}Progress reset. Starting from beginning.${NC}"
            echo ""
            sleep 2
        else
            echo "Exiting. Run script again to resume."
            exit 0
        fi
    fi
else
    RESUME_MODE=false
fi

# Check if running as root (allow in Docker)
if [ "$EUID" -eq 0 ] && [ ! -f /.dockerenv ]; then 
   echo "Please do not run this script as root (with sudo)"
   echo "Run it as normal user: ./interactive_install_resumable.sh"
   exit 1
fi
#==============================================================================
# BOOTSTRAP: Install Essential Tools (if missing)
#==============================================================================
STEP_NAME="bootstrap"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Bootstrap: Installing Essential Tools"

echo "Checking for essential tools..."
echo ""

MISSING_TOOLS=()

# Check for required tools
if ! command_exists wget; then MISSING_TOOLS+=("wget"); fi
if ! command_exists curl; then MISSING_TOOLS+=("curl"); fi
if ! command_exists git; then MISSING_TOOLS+=("git"); fi
if ! command_exists sudo; then MISSING_TOOLS+=("sudo"); fi

if [ ${#MISSING_TOOLS[@]} -gt 0 ]; then
    echo -e "${YELLOW}⚠  Missing tools detected: ${MISSING_TOOLS[*]}${NC}"
    echo ""
    echo "Installing essential tools first..."
    echo ""
    
    apt update
    apt install -y "${MISSING_TOOLS[@]}"
    
    echo ""
    echo -e "${GREEN}✓ Essential tools installed!${NC}"
    echo ""
else
    echo -e "${GREEN}✓ All essential tools present!${NC}"
    echo ""
fi

save_checkpoint "$STEP_NAME"
wait_for_confirmation

else
    echo -e "${CYAN}⏭  Skipping: Bootstrap (already complete)${NC}"
fi
STEP_NAME="part1_intro"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "PART 1: Install ROS 2 Humble (Steps 1-15)"
echo "In this part, you'll install the ROS 2 framework on your system."
echo ""
echo "What you'll install:"
echo "  • ROS 2 Humble base system"
echo "  • Development tools (colcon)"
echo "  • Computer vision packages"
echo "  • Python libraries"
echo ""
echo "⏱️  Time: ~30-40 minutes"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 1: Update Package Lists
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Install ROS 2 Humble (Steps 1-15) (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step1_update_package_lists"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 1/50: Update Package Lists"

show_command "sudo apt update"

show_explanation "This downloads the latest list of available software packages
from Ubuntu's repositories. Think of it like refreshing the
'app store' to see what software is available.

WHY: Before installing anything, we need to know what
versions are currently available."

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "sudo apt update"

#==============================================================================
# Step 2: Install Basic Tools
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 1 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step2_install_basic_tools"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 2/50: Install Basic Tools"

show_command "sudo apt install -y locales curl software-properties-common"

show_explanation "Installing essential tools we'll need:
  • locales: Sets up language/region settings
  • curl: Downloads files from the internet
  • software-properties-common: Manages software repositories

These are prerequisites for the ROS installation."

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "sudo apt install -y locales curl software-properties-common"

#==============================================================================
# Step 3: Configure Locale
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 2 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step3_set_system_language"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 3/50: Set System Language"

show_command "sudo locale-gen en_US en_US.UTF-8"

show_explanation "Generates English (US) language support for the system.

WHY: ROS messages and commands expect English. This prevents
errors with special characters or different alphabets."

wait_for_user
execute_command "sudo locale-gen en_US en_US.UTF-8"

show_command "export LANG=en_US.UTF-8"

show_explanation "Sets the current terminal session to use English.

The 'export' command sets an environment variable that
programs can read to determine which language to use."

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "export LANG=en_US.UTF-8"

#==============================================================================
# Step 4: Enable Universe Repository
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 3 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step4_enable_ubuntu_universe_reposit"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 4/50: Enable Ubuntu Universe Repository"

show_command "sudo add-apt-repository universe -y"

show_explanation "Ubuntu has different 'repositories' (collections of software):
  • Main: Officially supported software
  • Universe: Community-maintained software (includes ROS!)

We're enabling Universe so we can install ROS packages."

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "sudo add-apt-repository universe -y"

#==============================================================================
# Step 5: Add ROS 2 GPG Key
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 4 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step5_add_ros_2_security_key"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 5/50: Add ROS 2 Security Key"

show_command "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"

show_explanation "Downloads a security key (GPG key) from the ROS project.

WHY: When downloading software, you need to verify it's
authentic and hasn't been tampered with. This key is like
a digital signature proving the software came from the
official ROS project.

SECURITY: Always verify software sources!"

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"

#==============================================================================
# Step 6: Add ROS 2 Repository
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 5 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step6_add_ros_2_package_repository"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 6/50: Add ROS 2 Package Repository"

show_command "echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main\" | sudo tee /etc/apt/sources.list.d/ros2.list"

show_explanation "Tells Ubuntu WHERE to find ROS 2 packages.

BREAKDOWN:
  • 'deb': Debian package format
  • 'arch=...': For your CPU architecture (ARM64 on Pi)
  • 'signed-by=...': Use the security key we just downloaded
  • 'jammy': Ubuntu 22.04 codename
  • URL: Where ROS packages are hosted

After this, Ubuntu knows to look at packages.ros.org for ROS!"

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main\" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null"

#==============================================================================
# Step 7: Update Package Lists Again
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 6 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step7_refresh_package_lists"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 7/50: Refresh Package Lists"

show_command "sudo apt update"

show_explanation "Now that we added the ROS repository, we refresh the
package list again to include ROS packages.

BEFORE Step 6: Ubuntu knew about regular packages
AFTER Step 6: Ubuntu also knows about ROS packages
AFTER Step 7: Package list updated with ROS!"

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "sudo apt update"

#==============================================================================
# Step 8: Upgrade System
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 7 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step8_upgrade_existing_packages"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 8/50: Upgrade Existing Packages"

show_command "sudo apt upgrade -y"

show_explanation "Updates all currently installed software to latest versions.

DIFFERENCE FROM 'apt update':
  • apt update: Refreshes the LIST of what's available
  • apt upgrade: Actually INSTALLS the updates

This ensures you have security patches and bug fixes.
⏱️  May take 2-5 minutes..."

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "sudo apt upgrade -y"

#==============================================================================
# Step 9: Install ROS 2 Humble Base
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 8 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step9_install_ros_2_humble_main_inst"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 9/50: Install ROS 2 Humble (Main Installation)"

show_command "sudo apt install -y ros-humble-ros-base"

show_explanation "THIS IS THE BIG ONE! Installing ROS 2 Humble base system.

WHAT YOU GET:
  • Core ROS 2 libraries (rclcpp, rclpy)
  • Communication system (topics, nodes, messages)
  • Command-line tools (ros2 topic list, ros2 node list)
  • Build tools basics

SIZE: ~500 MB download
⏱️  TIME: 10-20 minutes (longest step!)

DURING THE WAIT:
  • Read ahead in your activity sheet
  • Review Session 2 TurtleSim concepts
  • Discuss: What will each node do?"

save_checkpoint "$STEP_NAME"
wait_for_user
echo -e "${YELLOW}Starting download... This will take a while. ☕${NC}"
execute_command "sudo apt install -y ros-humble-ros-base"

#==============================================================================
# Step 10: Install Development Tools
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 9 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step10_install_development_tools"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 10/50: Install Development Tools"

show_command "sudo apt install -y ros-dev-tools python3-colcon-common-extensions"

show_explanation "Installing build tools for ROS development:

  • ros-dev-tools: General ROS development utilities
  • colcon: Build system for ROS 2 packages

WHY: You'll use 'colcon build' to compile your robot code
in Part 6. Without these tools, you can't build ROS packages!"

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "sudo apt install -y ros-dev-tools python3-colcon-common-extensions"

#==============================================================================
# Step 11: Setup ROS Environment
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 10 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step11_setup_ros_2_environment"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 11/50: Setup ROS 2 Environment"

show_command "source /opt/ros/humble/setup.bash"

show_explanation "'Sources' the ROS 2 environment - loads ROS commands into
your current terminal session.

WHAT IT DOES:
  • Adds ROS commands to your PATH
  • Sets environment variables ROS needs
  • Makes ROS libraries findable

WITHOUT THIS: You'd get 'ros2: command not found' errors!"

wait_for_user
execute_command "source /opt/ros/humble/setup.bash"

show_command "echo \"source /opt/ros/humble/setup.bash\" >> ~/.bashrc"

show_explanation "Adding the source command to ~/.bashrc makes it automatic.

~/.bashrc runs every time you open a new terminal.
Now ROS will be available in ALL future terminals automatically!

BEFORE: Had to type 'source ...' every time
AFTER: ROS loads automatically in new terminals"

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "echo \"source /opt/ros/humble/setup.bash\" >> ~/.bashrc"

#==============================================================================
# Step 12: Verify ROS Installation
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 11 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step12_verify_ros_2_installation"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 12/50: Verify ROS 2 Installation"

show_command "echo \$ROS_DISTRO"

show_explanation "Checking which ROS version is active.

ROS_DISTRO is an environment variable set by ROS.
Should output: 'humble'

If it shows 'humble', ROS is properly loaded!"

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "echo \$ROS_DISTRO"

#==============================================================================
# Step 13: Install Computer Vision Packages
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 12 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step13_install_camera_vision_packages"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 13/50: Install Camera & Vision Packages"

show_command "sudo apt install -y ros-humble-cv-bridge ros-humble-vision-opencv ros-humble-image-transport ros-humble-camera-info-manager"

show_explanation "Installing ROS packages for cameras and images:

  • cv-bridge: Converts between ROS images and OpenCV
    (Remember Session 3? OpenCV for computer vision!)
  
  • vision-opencv: OpenCV integration with ROS 2
  
  • image-transport: Efficiently sends images between nodes
  
  • camera-info-manager: Handles camera calibration data

YOU'LL USE THESE: In camera_node and lane_follower_node!"

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "sudo apt install -y ros-humble-cv-bridge ros-humble-vision-opencv ros-humble-image-transport ros-humble-camera-info-manager"

#==============================================================================
# Step 14: Install Python Dependencies
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 13 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step14_install_python_libraries"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 14/50: Install Python Libraries"

show_command "sudo apt install -y python3-opencv python3-pip"

show_explanation "Installing Python libraries for image processing:

  • python3-opencv: OpenCV library for Python
    (Canny edge detection, Hough transform - Session 3!)
  
  • python3-pip: Python package installer

These provide the computer vision algorithms for
lane_follower_node."

wait_for_user
execute_command "sudo apt install -y python3-opencv python3-pip"

show_command "pip3 install numpy"

show_explanation "Installing NumPy - fundamental package for numerical computing.

NumPy provides:
  • Fast array operations (images are arrays of pixels!)
  • Mathematical functions
  • Linear algebra operations

LANE DETECTION uses NumPy for:
  • Image array manipulation
  • Calculating line slopes
  • Operations on pixel values"

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "pip3 install numpy"

#==============================================================================
# Step 14a: Install Camera System (V4L2 for Ubuntu 22.04)
#==============================================================================

STEP_NAME="step14a_install_camera_system"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 14a/50: Install V4L2 Camera System"

show_explanation "Installing V4L2 camera system for Ubuntu 22.04.

WHAT IS V4L2?
  • Video4Linux2 - standard Linux camera interface
  • Works with Raspberry Pi camera via kernel drivers
  • Direct ROS 2 integration available
  • No picamera2/libcamera compilation needed!

WHAT WE'LL INSTALL:
  • libraspberrypi-bin: Raspberry Pi utilities
  • v4l-utils: V4L2 tools for testing
  • ROS 2 v4l2_camera: Camera node for ROS

This will take 5 minutes!"

wait_for_user

# Install V4L2 support
show_command "sudo apt install -y libraspberrypi-bin v4l-utils"

execute_command "sudo apt install -y libraspberrypi-bin v4l-utils"

# Install ROS 2 camera packages
show_command "sudo apt install -y ros-humble-v4l2-camera ros-humble-image-transport-plugins"

show_explanation "Installing ROS 2 camera packages.

PACKAGES:
  • ros-humble-v4l2-camera: V4L2 camera node
  • ros-humble-image-transport-plugins: Image compression

These integrate camera directly with ROS 2!"

wait_for_user
execute_command "sudo apt install -y ros-humble-v4l2-camera ros-humble-image-transport-plugins"

# Add user to video group
show_command "sudo usermod -aG video,gpio,i2c \$USER"

show_explanation "Adding user to hardware groups.

GROUPS:
  • video: Camera access
  • gpio: GPIO pins
  • i2c: Motor controller

Changes take effect after reboot."

wait_for_user
execute_command "sudo usermod -aG video,gpio,i2c \$USER"

# Configure camera in config.txt
show_manual_task "CAMERA CONFIGURATION

We need to enable camera in /boot/firmware/config.txt

Checking configuration..."

echo ""
if grep -q "^start_x=1" /boot/firmware/config.txt 2>/dev/null; then
    echo -e "\${GREEN}✓ Camera already enabled (start_x=1)\${NC}"
elif grep -q "^start_x=0" /boot/firmware/config.txt 2>/dev/null; then
    echo -e "\${YELLOW}⚠ Camera disabled! Enabling...\${NC}"
    sudo sed -i 's/^start_x=0/start_x=1/' /boot/firmware/config.txt
    echo -e "\${GREEN}✓ Camera enabled\${NC}"
elif grep -q "^#start_x" /boot/firmware/config.txt 2>/dev/null; then
    echo -e "\${YELLOW}⚠ Camera commented out! Enabling...\${NC}"
    sudo sed -i 's/^#start_x=1/start_x=1/' /boot/firmware/config.txt
    echo -e "\${GREEN}✓ Camera enabled\${NC}"
else
    echo -e "\${YELLOW}⚠ Adding camera configuration...\${NC}"
    echo "start_x=1" | sudo tee -a /boot/firmware/config.txt
    echo -e "\${GREEN}✓ Camera enabled\${NC}"
fi

echo ""
echo -e "\${YELLOW}⚠ REBOOT REQUIRED for camera to work!\${NC}"
echo ""

save_checkpoint "$STEP_NAME"
wait_for_confirmation

else
    echo -e "${CYAN}⏭  Skipping: Step 14a (already complete)${NC}"
    sleep 1
fi

#==============================================================================
# Step 14b: Create Camera Test Script
#==============================================================================

STEP_NAME="step14b_create_camera_test"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 14b/50: Create Camera Test Script"

show_explanation "Creating test script to verify camera works.

This script will:
  • Open /dev/video0
  • Capture a test image
  • Save as test_image.jpg

You'll run this after rebooting!"

wait_for_user

cat > ~/test_camera_v4l2.py << 'CAMERA_TEST_EOF'
#!/usr/bin/env python3
# test_camera_v4l2.py

import cv2
import time

print("Testing V4L2 Camera...")
print("-" * 40)

# Open camera
print("Opening /dev/video0...")
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)

if not cap.isOpened():
    print("❌ ERROR: Could not open camera!")
    print("\nTroubleshooting:")
    print("  1. Check if /dev/video0 exists: ls -l /dev/video*")
    print("  2. Check camera detection: vcgencmd get_camera")
    print("  3. Check config.txt has: start_x=1")
    print("  4. Did you reboot after Step 14a?")
    exit(1)

print("✓ Camera opened successfully!")

# Set resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print(f"Resolution: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")

# Warm up camera
print("\nWarming up camera (2 seconds)...")
time.sleep(2)

# Capture frame
print("Capturing image...")
ret, frame = cap.read()

if ret:
    filename = f"test_image_{int(time.time())}.jpg"
    cv2.imwrite(filename, frame)
    print(f"✓ Image saved as: {filename}")
    print(f"  Size: {frame.shape[1]}x{frame.shape[0]}")
else:
    print("❌ ERROR: Could not capture frame!")

# Release camera
cap.release()
print("\n✓ Camera test complete!")
CAMERA_TEST_EOF

chmod +x ~/test_camera_v4l2.py

echo ""
echo -e "\${GREEN}✓ Camera test script created: ~/test_camera_v4l2.py\${NC}"
echo ""

save_checkpoint "$STEP_NAME"
wait_for_confirmation

else
    echo -e "${CYAN}⏭  Skipping: Step 14b (already complete)${NC}"
    sleep 1
fi

#==============================================================================
# Reboot Notice
#==============================================================================

show_manual_task "⚠️  REBOOT REQUIRED! ⚠️

Camera configuration has been updated in /boot/firmware/config.txt

You MUST reboot for camera to work!

WHAT TO DO:
  1. This script will pause here
  2. Note your progress (Step 14b complete)
  3. Reboot: sudo reboot
  4. After reboot, SSH back in
  5. Test camera: python3 ~/test_camera_v4l2.py
  6. If camera works, resume: ./interactive_install_resumable.sh

WHY REBOOT?
  • Camera driver needs to load
  • start_x=1 takes effect on boot
  • GPU memory allocated for camera"

echo ""
if ask_yes_no "Ready to reboot now?"; then
    echo ""
    echo -e "${GREEN}Rebooting in 5 seconds...${NC}"
    echo "After reboot:"
    echo "  1. SSH back in"
    echo "  2. Test camera: python3 ~/test_camera_v4l2.py"
    echo "  3. Resume script: ./interactive_install_resumable.sh"
    sleep 5
    sudo reboot
else
    echo ""
    echo -e "${YELLOW}⚠️  Remember to reboot before continuing!${NC}"
    echo "When ready: sudo reboot"
    echo "After reboot: python3 ~/test_camera_v4l2.py"
    echo "Then: ./interactive_install_resumable.sh"
    echo ""
    exit 0
fi
#==============================================================================
# Step 15: Test ROS Commands
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 14a (already complete)${NC}"
    sleep 1
fi


STEP_NAME="step15_test_ros_2_commands"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 15/50: Test ROS 2 Commands"

show_command "ros2 topic list"

show_explanation "Testing if ROS 2 commands work.

Lists all active ROS topics (communication channels).

EXPECTED OUTPUT:
  /parameter_events
  /rosout

These are ROS 2 system topics - always present even with no user nodes running.

CONNECTION TO SESSION 2: Same command you used in TurtleSim!"

wait_for_user
execute_command "ros2 topic list"

show_command "ros2 node list"

show_explanation "Listing all active ROS nodes (programs).

EXPECTED: Usually empty (no user nodes running yet).
May occasionally show ROS daemon nodes.

CONNECTION TO SESSION 2: Like 'ros2 node list' in TurtleSim
that showed /turtlesim and /teleop_turtle!"

wait_for_user
execute_command "ros2 node list"

#==============================================================================
# Part 1 Complete
#==============================================================================
clear
echo "=========================================="
echo "   ✓ Part 1 Complete!"
echo "=========================================="
echo ""
echo -e "${GREEN}ROS 2 Humble is successfully installed!${NC}"
echo ""
echo "WHAT YOU INSTALLED:"
echo "  ✓ ROS 2 Humble base system"
echo "  ✓ Development tools (colcon)"
echo "  ✓ Computer vision packages"
echo "  ✓ Python libraries (OpenCV, NumPy)"
echo ""
echo "VERIFICATION:"
source /opt/ros/humble/setup.bash
echo "  ROS_DISTRO: $ROS_DISTRO"
echo ""
echo "✍️  In your activity sheet:"
echo "  • Complete Part 1 checkpoint"
echo "  • Verify all commands succeeded"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# PART 2: HARDWARE VERIFICATION
#==============================================================================


else
    echo -e "${CYAN}⏭  Skipping: Step 15 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="part2_intro"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "PART 2: Hardware Verification"

echo "Before we integrate ROS, let's verify your hardware works."
echo ""
echo "In this part, you'll:"
echo "  • Test that motors respond"
echo "  • Test that camera works"
echo ""
echo "This uses the Freenove test scripts (not ROS yet)."
echo ""
echo "⏱️  Time: ~10 minutes"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 16: Download Freenove Code
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Hardware Verification (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step16_download_freenove_hardware_lib"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 16/50: Download Freenove Hardware Library"

echo "First, check if you already have the Freenove code:"
echo ""
if [ -d ~/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi ]; then
    echo -e "${GREEN}✓ Freenove code already exists!${NC}"
    echo ""
    echo "Skipping download..."
    wait_for_confirmation
else
    show_command "cd ~ && git clone --depth 1 $FREENOVE_REPO"

    show_explanation "Downloads the Freenove motor control library from GitHub.

WHAT IS GIT?
  • Version control system (tracks code changes)
  • GitHub hosts code repositories
  • 'clone' means download a copy
  • '--depth 1' means only get latest version (faster)

WHY: ROS 2 will use this library to control motors and camera."

save_checkpoint "$STEP_NAME"
    wait_for_user
    execute_command "cd ~ && git clone --depth 1 $FREENOVE_REPO"
fi

#==============================================================================
# Step 16a: Install Freenove Hardware Library
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 16 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step16a_install_freenove_hardware_lib"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 16a/50: Install Freenove Hardware Library"

show_command "cd ~/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code && sudo python3 setup.py"

show_explanation "Running Freenove's installation script.

WHAT THIS SCRIPT DOES:
  • Installs python3-pyqt5 (GUI library)
  • Installs gpiozero (GPIO control)
  • Installs numpy (if not already installed)
  • Installs rpi-ws281x-python (LED control)
  • Configures camera in /boot/firmware/config.txt
  • Enables SPI interface

INTERACTIVE PROMPTS:
  You will be asked:
  1. Camera model: ov5647 or imx219
     (The kit includes OV5647)
  
  2. (Pi 5 only) Camera port: cam0 or cam1
     (Check which port you connected the cable to)

â±ï¸  TIME: 2-5 minutes

IMPORTANT: A reboot will be required after this!"

echo ""
echo "⚠️  PAY ATTENTION to the prompts!"
echo ""

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "cd ~/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code && sudo python3 setup.py"

show_manual_task "REBOOT REQUIRED!

The Freenove setup.py modified system configuration files.
You MUST reboot for changes to take effect.

WHAT TO DO:
  1. This script will pause here
  2. Note your progress (Step 16a complete)
  3. Reboot your Raspberry Pi: sudo reboot
  4. After reboot, SSH back in
  5. Run this script again: ./interactive_install_resumable.sh
  6. The script will resume from Step 17

WHY REBOOT?
  • Camera configuration changes need reboot
  • SPI interface enabling needs reboot
  • GPIO permissions need session restart"

echo ""
if ask_yes_no "Ready to reboot now?"; then
    echo ""
    echo -e "${GREEN}Rebooting in 5 seconds...${NC}"
    echo "After reboot, run: ./interactive_install_resumable.sh"
    sleep 5
    save_checkpoint "$STEP_NAME"
    sudo reboot
else
    echo ""
    echo -e "${YELLOW}⚠️  Remember to reboot before continuing!${NC}"
    echo "When ready: sudo reboot"
    echo "After reboot: ./interactive_install_resumable.sh"
    echo ""
    save_checkpoint "$STEP_NAME"
    exit 0
fi

#==============================================================================
# Step 17: Test Motors
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 16a (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step17_test_motors_manual_task"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 17/50: Test Motors (Manual Task)"

show_manual_task "Now you'll test that motors work.

SAFETY FIRST:
  ☐ Place robot on blocks OR hold off table
  ☐ Make sure S1 and S2 power switches are ON
  ☐ Battery is charged
  ☐ Clear space around robot
  ☐ Be ready to press Ctrl+C to stop

WHAT TO DO:
  1. Open a NEW terminal (keep this one open!)
  2. Run: cd ~/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server
  3. Run: sudo python3 test.py Motor
  4. Observe: Wheels should move forward, back, left, right
  5. Press Ctrl+C when done
  6. Return to THIS terminal

WHAT THIS TESTS:
  • Motor driver board communication
  • Power supply
  • Motor connections
  • GPIO control"

echo ""
echo "✍️  In your activity sheet:"
echo "  • What does the motor test verify?"
echo "  • Did all 4 wheels respond correctly?"
echo ""

if ask_yes_no "Did the motor test work correctly?"; then
    echo -e "${GREEN}✓ Great! Motors are working.${NC}"
else
    echo -e "${YELLOW}⚠ Motor issues detected.${NC}"
    echo ""
    echo "TROUBLESHOOTING:"
    echo "  • Check S1 and S2 switches are ON"
    echo "  • Check battery charge"
    echo "  • Verify wheel connections"
    echo "  • Ask instructor for help"
    echo ""
    if ask_yes_no "Do you want to continue anyway?"; then
        echo "OK, continuing..."
    else
        echo "Exiting. Fix motors and restart the script."
        exit 1
    fi
fi

save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 18: Test Camera
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 17 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step18_test_camera_manual_task"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 18/50: Test Camera (Manual Task)"

show_manual_task "Now you'll verify the camera works.

WHAT TO DO:
  1. In a NEW terminal, run:
     cd ~/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server
     python3 camera.py
  
  2. Open a web browser on your laptop
  
  3. Go to: http://robot[X].local:8000/camera
     (Replace [X] with your robot number)
  
  4. You should see live camera feed
  
  5. Press Ctrl+C in terminal when done

WHAT THIS TESTS:
  • Camera module connection
  • Camera driver
  • Image capture capability
  • Network streaming"

echo ""
echo "✍️  In your activity sheet:"
echo "  • What does the camera test verify?"
echo "  • What resolution is the camera feed?"
echo ""

if ask_yes_no "Did the camera test work correctly?"; then
    echo -e "${GREEN}✓ Great! Camera is working.${NC}"
else
    echo -e "${YELLOW}⚠ Camera issues detected.${NC}"
    echo ""
    echo "TROUBLESHOOTING:"
    echo "  • Check camera cable connection"
    echo "  • Verify camera is enabled: sudo raspi-config"
    echo "  • Try rebooting"
    echo "  • Ask instructor for help"
    echo ""
    if ask_yes_no "Do you want to continue anyway?"; then
        echo "OK, continuing..."
    else
        echo "Exiting. Fix camera and restart the script."
        exit 1
    fi
fi

wait_for_confirmation

#==============================================================================
# Part 2 Complete
#==============================================================================
clear
echo "=========================================="
echo "   ✓ Part 2 Complete!"
echo "=========================================="
echo ""
echo -e "${GREEN}Hardware verification successful!${NC}"
echo ""
echo "VERIFIED:"
echo "  ✓ Motors respond to commands"
echo "  ✓ Camera captures images"
echo "  ✓ All connections working"
echo ""
echo "✍️  In your activity sheet:"
echo "  • Complete Part 2 checkpoint"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# PART 3: DOWNLOAD ROS NODE CODE
#==============================================================================


else
    echo -e "${CYAN}⏭  Skipping: Step 18 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="part3_intro"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "PART 3: Download ROS Node Code"

echo "Now we'll download the ROS 2 node code for your robot."
echo ""
echo "What you'll get:"
echo "  • camera_node.py - Publishes camera images"
echo "  • motor_control_node.py - Controls motors"
echo "  • lane_follower_node.py - Computer vision & steering"
echo ""
echo "⏱️  Time: ~5 minutes"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 19: Clone ROS Nodes Repository
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Download ROS Node Code (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step19_clone_ros_nodes_from_github"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 19/50: Clone ROS Nodes from GitHub"

show_command "cd ~ && git clone $GITHUB_REPO"

show_explanation "Downloads your ROS 2 node code from GitHub.

WHAT YOU'RE GETTING:
  • Three Python files (the nodes)
  • Mock files for testing without hardware
  • Documentation

WHY FROM GITHUB:
  • Version controlled
  • Easy to update
  • Can share with team
  • Industry standard practice"

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "cd ~ && git clone $GITHUB_REPO"

#==============================================================================
# Step 20: View Downloaded Files
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 19 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step20_explore_downloaded_code"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 20/50: Explore Downloaded Code"

show_command "ls ~/freenove-ros2-session5/nodes/"

show_explanation "Lists the node files you just downloaded.

YOU SHOULD SEE:
  • camera_node.py - Camera capture and publishing
  • motor_control_node.py - Motor control subscriber
  • lane_follower_node.py - Vision processing

These are complete, working ROS 2 nodes ready to use!"

wait_for_user
execute_command "ls ~/freenove-ros2-session5/nodes/"

#==============================================================================
# Part 3 Complete
#==============================================================================
clear
echo "=========================================="
echo "   ✓ Part 3 Complete!"
echo "=========================================="
echo ""
echo -e "${GREEN}ROS node code downloaded!${NC}"
echo ""
echo "CODE LOCATION:"
echo "  ~/freenove-ros2-session5/nodes/"
echo ""
echo "✍️  In your activity sheet:"
echo "  • What are the three node files?"
echo "  • What does each node do?"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# PART 4: CREATE ROS WORKSPACE
#==============================================================================


else
    echo -e "${CYAN}⏭  Skipping: Step 20 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="part4_intro"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "PART 4: Create ROS 2 Workspace"

echo "Now we'll create a ROS 2 workspace for your project."
echo ""
echo "What is a workspace?"
echo "  • A folder that holds your ROS packages"
echo "  • Like a project folder"
echo "  • Has specific structure ROS expects"
echo ""
echo "You'll create:"
echo "  • Workspace directories"
echo "  • ROS package for your robot"
echo "  • Copy node files into package"
echo ""
echo "⏱️  Time: ~15 minutes"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 21: Create Workspace Directories
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Create ROS 2 Workspace (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step21_create_workspace_structure"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 21/50: Create Workspace Structure"

show_command "mkdir -p ~/ros2_ws/src"

show_explanation "Creates the workspace directory structure.

BREAKDOWN:
  • mkdir: Make directory
  • -p: Create parent directories if needed
  • ~/ros2_ws: Your workspace (ROS 2 WorkSpace)
  • /src: Source folder (where packages go)

WORKSPACE STRUCTURE:
  ros2_ws/
  ├── src/      ← Your packages go here
  ├── build/    ← Build files (created by colcon)
  ├── install/  ← Installed files (created by colcon)
  └── log/      ← Build logs (created by colcon)"

wait_for_user
execute_command "mkdir -p ~/ros2_ws/src"

show_command "cd ~/ros2_ws/src"

show_explanation "Changes to the src directory.

WHY: ROS packages must be created inside src/
This is where 'ros2 pkg create' will work."

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "cd ~/ros2_ws/src"

#==============================================================================
# Step 22: Create ROS Package
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 21 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step22_create_ros_2_package"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 22/50: Create ROS 2 Package"

show_command "ros2 pkg create freenove_car --build-type ament_python --dependencies rclpy std_msgs sensor_msgs geometry_msgs cv_bridge"

show_explanation "Creates a new ROS 2 package for your robot.

BREAKDOWN:
  • ros2 pkg create: ROS command to create package
  • freenove_car: Your package name
  • --build-type ament_python: Python package
  • --dependencies: Packages your code needs

DEPENDENCIES EXPLAINED:
  • rclpy: ROS Client Library for Python (core ROS)
  • std_msgs: Standard messages (String, Int32)
  • sensor_msgs: Sensor messages (Image, CameraInfo)
  • geometry_msgs: Geometry messages (Twist for velocity)
  • cv_bridge: Converts between ROS and OpenCV images

WHAT THIS CREATES:
  • package.xml: Package metadata
  • setup.py: Python installation config
  • freenove_car/: Python package directory
  • resource/: Resource files
  • test/: Test files"

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "ros2 pkg create freenove_car --build-type ament_python --dependencies rclpy std_msgs sensor_msgs geometry_msgs cv_bridge"

#==============================================================================
# Step 23: Verify Package Structure
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 22 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step23_explore_package_structure"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 23/50: Explore Package Structure"

show_command "ls ~/ros2_ws/src/freenove_car/"

show_explanation "Shows the files that ros2 pkg create generated.

YOU SHOULD SEE:
  • package.xml - What this package is, dependencies
  • setup.py - How to install this Python package
  • freenove_car/ - Where your node code will go
  • resource/ - Package resources
  • test/ - Test files

This is the standard ROS 2 Python package structure!"

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "ls ~/ros2_ws/src/freenove_car/"

#==============================================================================
# Step 24: Copy Node Files
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 23 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step24_copy_node_files_into_package"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 24/50: Copy Node Files into Package"

show_command "cp ~/freenove-ros2-session5/nodes/*.py ~/ros2_ws/src/freenove_car/freenove_car/"

show_explanation "Copies your three node files into the package.

FROM: ~/freenove-ros2-session5/nodes/
TO: ~/ros2_ws/src/freenove_car/freenove_car/

WHY: ROS needs nodes inside the package directory
to be able to find and run them.

*.py means 'all files ending in .py' (all three nodes)"

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "cp ~/freenove-ros2-session5/nodes/*.py ~/ros2_ws/src/freenove_car/freenove_car/"

#==============================================================================
# Step 25: Verify Files Copied
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 24 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step25_verify_node_files"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 25/50: Verify Node Files"

show_command "ls ~/ros2_ws/src/freenove_car/freenove_car/*.py"

show_explanation "Lists all Python files in your package.

YOU SHOULD SEE:
  • __init__.py (created by ros2 pkg create)
  • camera_node.py (your code)
  • motor_control_node.py (your code)
  • lane_follower_node.py (your code)

These are now part of your ROS package!"

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "ls ~/ros2_ws/src/freenove_car/freenove_car/*.py"

#==============================================================================
# Step 26: Make Files Executable
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 25 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step26_make_node_files_executable"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 26/50: Make Node Files Executable"

show_command "chmod +x ~/ros2_ws/src/freenove_car/freenove_car/*.py"

show_explanation "Makes the Python files executable.

WHAT chmod +x DOES:
  • chmod: Change file mode
  • +x: Add execute permission
  • Allows files to be run directly

WHY: So you can run: ./camera_node.py
Instead of: python3 camera_node.py"

wait_for_user
execute_command "chmod +x ~/ros2_ws/src/freenove_car/freenove_car/*.py"

#==============================================================================
# Part 4 Complete
#==============================================================================
clear
echo "=========================================="
echo "   ✓ Part 4 Complete!"
echo "=========================================="
echo ""
echo -e "${GREEN}ROS 2 workspace created!${NC}"
echo ""
echo "WORKSPACE STRUCTURE:"
echo "  ~/ros2_ws/"
echo "  └── src/"
echo "      └── freenove_car/"
echo "          ├── freenove_car/"
echo "          │   ├── camera_node.py"
echo "          │   ├── motor_control_node.py"
echo "          │   └── lane_follower_node.py"
echo "          ├── package.xml"
echo "          └── setup.py"
echo ""
echo "✍️  In your activity sheet:"
echo "  • Draw the workspace structure"
echo "  • Label what each directory does"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# PART 5: CONFIGURE PACKAGE
#==============================================================================


else
    echo -e "${CYAN}⏭  Skipping: Step 26 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="part5_intro"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "PART 5: Configure Package"

echo "Now we need to configure the package so ROS knows"
echo "how to find and run your nodes."
echo ""
echo "You'll edit:"
echo "  • setup.py - Tell ROS about your executable nodes"
echo "  • package.xml - Verify dependencies (should be OK)"
echo ""
echo "⏱️  Time: ~10 minutes"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 27: Edit setup.py - Add Entry Points
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Part 4 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step27_configure_entry_points_improved"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 27/50: Configure Entry Points in setup.py"

show_explanation "We need to add 'entry points' to setup.py.

WHAT ARE ENTRY POINTS?
  • Tell ROS which Python functions to run
  • Like a 'table of contents' for your nodes
  • Format: 'command_name = package.module:function'

WE NEED TO ADD:
  'camera_node = freenove_car.camera_node:main',
  'motor_control_node = freenove_car.motor_control_node:main',
  'lane_follower_node = freenove_car.lane_follower_node:main',

This goes in the entry_points section of setup.py."

echo ""
echo "Checking setup.py and fixing if needed..."
echo ""

save_checkpoint "$STEP_NAME"
wait_for_user

# Setup file path
SETUP_FILE=~/ros2_ws/src/freenove_car/setup.py

# Check if entry points already configured correctly
if grep -q "camera_node = freenove_car.camera_node:main" "$SETUP_FILE" && \
   grep -q "motor_control_node = freenove_car.motor_control_node:main" "$SETUP_FILE" && \
   grep -q "lane_follower_node = freenove_car.lane_follower_node:main" "$SETUP_FILE"; then
    echo -e "${GREEN}✓ Entry points already configured correctly!${NC}"
else
    # Count how many entry_points sections exist
    ENTRY_POINTS_COUNT=$(grep -c "entry_points\s*=" "$SETUP_FILE")
    
    if [ "$ENTRY_POINTS_COUNT" -gt 1 ]; then
        echo -e "${YELLOW}⚠️  WARNING: Multiple entry_points sections detected!${NC}"
        echo "This will cause build errors. Fixing..."
        
        # Backup the file
        cp "$SETUP_FILE" "${SETUP_FILE}.backup"
        echo "Backup created: ${SETUP_FILE}.backup"
        
        # Remove all entry_points sections and add one correct one
        # This uses a Python script for reliable editing
        python3 << 'PYTHON_SCRIPT'
import re

setup_file = "~/ros2_ws/src/freenove_car/setup.py"
setup_file = setup_file.replace("~", "/home/" + "$(whoami)")

with open(setup_file, 'r') as f:
    content = f.read()

# Remove all entry_points sections (both styles)
content = re.sub(r'entry_points\s*=\s*{[^}]*},?\s*', '', content, flags=re.DOTALL)

# Find the setup() function and add entry_points before the closing )
entry_points_block = """    entry_points={
        'console_scripts': [
            'camera_node = freenove_car.camera_node:main',
            'motor_control_node = freenove_car.motor_control_node:main',
            'lane_follower_node = freenove_car.lane_follower_node:main',
        ],
    },
"""

# Insert before the last ) of setup()
content = re.sub(r'(\s*)\)', r'\n' + entry_points_block + r'\1)', content, count=1, flags=re.MULTILINE)

with open(setup_file, 'w') as f:
    f.write(content)
PYTHON_SCRIPT
        
        echo -e "${GREEN}✓ Entry points fixed!${NC}"
        
    else
        # Only one or no entry_points section - add our entry points
        if grep -q "entry_points\s*=" "$SETUP_FILE"; then
            echo "Updating existing entry_points section..."
            # Entry points exists, but doesn't have our nodes - update it
            sed -i "/entry_points\s*=\s*{/,/},/ c\\
    entry_points={\\
        'console_scripts': [\\
            'camera_node = freenove_car.camera_node:main',\\
            'motor_control_node = freenove_car.motor_control_node:main',\\
            'lane_follower_node = freenove_car.lane_follower_node:main',\\
        ],\\
    }," "$SETUP_FILE"
        else
            echo "Adding entry_points section..."
            # No entry_points at all - add it before the closing ) of setup()
            sed -i '/^)/i\    entry_points={\n        '\''console_scripts'\'': [\n            '\''camera_node = freenove_car.camera_node:main'\'',\n            '\''motor_control_node = freenove_car.motor_control_node:main'\'',\n            '\''lane_follower_node = freenove_car.lane_follower_node:main'\'',\n        ],\n    },' "$SETUP_FILE"
        fi
        
        echo -e "${GREEN}✓ Entry points added to setup.py${NC}"
    fi
fi

echo ""
echo "CONFIGURED ENTRY POINTS:"
echo "  'console_scripts': ["
echo "    'camera_node = freenove_car.camera_node:main',"
echo "    'motor_control_node = freenove_car.motor_control_node:main',"
echo "    'lane_follower_node = freenove_car.lane_follower_node:main',"
echo "  ],"
echo ""
echo "NOW YOU CAN RUN:"
echo "  ros2 run freenove_car camera_node"
echo "  ros2 run freenove_car motor_control_node"
echo "  ros2 run freenove_car lane_follower_node"
echo ""
echo "✏️  In your activity sheet:"
echo "  • What do entry points do?"
echo "  • Why are they needed?"
echo ""

save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 28: Verify package.xml
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 27 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step28_verify_dependencies_in_package"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 28/50: Verify Dependencies in package.xml"

show_command "cat ~/ros2_ws/src/freenove_car/package.xml | grep '<depend>'"

show_explanation "Checking that all dependencies are listed in package.xml.

DEPENDENCIES SHOULD INCLUDE:
  • rclpy
  • std_msgs
  • sensor_msgs
  • geometry_msgs
  • cv_bridge

These were added automatically by 'ros2 pkg create'
when we specified --dependencies."

wait_for_user
execute_command "cat ~/ros2_ws/src/freenove_car/package.xml | grep '<depend>'"

echo ""
echo "✓ All dependencies are present!"
echo ""
wait_for_confirmation

#==============================================================================
# Part 5 Complete
#==============================================================================
clear
echo "=========================================="
echo "   ✓ Part 5 Complete!"
echo "=========================================="
echo ""
echo -e "${GREEN}Package configured!${NC}"
echo ""
echo "CONFIGURED:"
echo "  ✓ Entry points added to setup.py"
echo "  ✓ Dependencies verified in package.xml"
echo "  ✓ Package ready to build"
echo ""
echo "✍️  In your activity sheet:"
echo "  • Complete Part 5 questions"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# PART 6: BUILD WORKSPACE
#==============================================================================


else
    echo -e "${CYAN}⏭  Skipping: Step 28 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="part6_intro"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "PART 6: Build the Workspace"

echo "Now we'll 'build' your workspace."
echo ""
echo "What does building do?"
echo "  • Compiles your code (if needed)"
echo "  • Installs your package"
echo "  • Generates files ROS needs to find your nodes"
echo ""
echo "Think of it like 'compiling' your project."
echo ""
echo "⏱️  Time: ~5 minutes"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 29: Build with Colcon
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Build the Workspace (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step29_build_workspace_with_colcon"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 29/50: Build Workspace with Colcon"

show_command "cd ~/ros2_ws && colcon build --symlink-install"

show_explanation "Builds your ROS 2 workspace using colcon.

WHAT IS COLCON?
  • Build tool for ROS 2
  • Like 'make' or 'cmake'
  • Processes all packages in src/

WHAT --symlink-install DOES:
  • Creates symbolic links instead of copying files
  • If you edit Python code, no rebuild needed!
  • Saves time during development

WHAT GETS CREATED:
  • build/ - Build files
  • install/ - Installed executables
  • log/ - Build logs

⏱️  This takes 30-60 seconds..."

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "cd ~/ros2_ws && colcon build --symlink-install"

#==============================================================================
# Step 30: Source the Workspace
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 29 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step30_source_your_workspace"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 30/50: Source Your Workspace"

show_command "source ~/ros2_ws/install/setup.bash"

show_explanation "Sources your workspace - tells ROS about your custom package.

REMEMBER:
  • source /opt/ros/humble/setup.bash (loads ROS)
  • source ~/ros2_ws/install/setup.bash (loads YOUR package)

NOW ROS KNOWS ABOUT:
  • Core ROS 2 (from /opt/ros/humble)
  • Your freenove_car package (from ~/ros2_ws)

You can now run: ros2 run freenove_car camera_node"

wait_for_user
execute_command "source ~/ros2_ws/install/setup.bash"

show_command "echo \"source ~/ros2_ws/install/setup.bash\" >> ~/.bashrc"

show_explanation "Adding workspace source to ~/.bashrc for convenience.

Now EVERY new terminal will automatically:
  1. Source ROS 2 (from earlier step)
  2. Source your workspace

No more manual sourcing needed!"

save_checkpoint "$STEP_NAME"
wait_for_user
execute_command "echo \"source ~/ros2_ws/install/setup.bash\" >> ~/.bashrc"

#==============================================================================
# Step 31: Verify Package Installation
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 30 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step31_verify_package_is_installed"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 31/50: Verify Package is Installed"

show_command "ros2 pkg list | grep freenove"

show_explanation "Searches for your package in the list of installed packages.

SHOULD OUTPUT: freenove_car

This confirms ROS can find your package!"

wait_for_user
execute_command "ros2 pkg list | grep freenove"

show_command "ros2 pkg executables freenove_car"

show_explanation "Lists all executable nodes in your package.

SHOULD SHOW:
  freenove_car camera_node
  freenove_car motor_control_node
  freenove_car lane_follower_node

These are your three nodes, ready to run!"

wait_for_user
execute_command "ros2 pkg executables freenove_car"

#==============================================================================
# Part 6 Complete
#==============================================================================
clear
echo "=========================================="
echo "   ✓ Part 6 Complete!"
echo "=========================================="
echo ""
echo -e "${GREEN}Workspace built successfully!${NC}"
echo ""
echo "BUILD RESULTS:"
echo "  ✓ Package compiled"
echo "  ✓ Executables installed"
echo "  ✓ ROS can find your nodes"
echo ""
echo "YOU CAN NOW RUN:"
echo "  ros2 run freenove_car camera_node"
echo "  ros2 run freenove_car motor_control_node"
echo "  ros2 run freenove_car lane_follower_node"
echo ""
echo "✍️  In your activity sheet:"
echo "  • What does colcon build do?"
echo "  • Why use --symlink-install?"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# PART 7: RUN & TEST SYSTEM
#==============================================================================


else
    echo -e "${CYAN}⏭  Skipping: Step 31 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="part7_intro"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "PART 7: Run & Test the System"

echo "Time to run your autonomous robot system!"
echo ""
echo "The system has THREE nodes:"
echo "  1. Camera Node - Captures images"
echo "  2. Lane Follower Node - Processes images, decides steering"
echo "  3. Motor Control Node - Controls wheels"
echo ""
echo "They communicate via ROS topics:"
echo "  camera → /freenove/camera/image_raw → lane_follower"
echo "  lane_follower → /freenove/cmd_vel → motor_control"
echo ""
echo "You'll need THREE terminal windows!"
echo ""
echo "⏱️  Time: ~30 minutes"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 32: Prepare for Three Terminals
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Run & Test the System (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step32_setup_three_terminals"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 32/50: Setup Three Terminals"

show_manual_task "You need THREE terminal windows to run the system.

OPTION 1: Use tmux (recommended)
  1. In this terminal, run: tmux
  2. Split panes:
     • Ctrl+b then \" (split horizontal)
     • Ctrl+b then % (split vertical)
  3. Navigate: Ctrl+b then arrow keys

OPTION 2: Use screen
  1. Start screen: screen
  2. Create windows: Ctrl+a then c
  3. Switch windows: Ctrl+a then n

OPTION 3: Three separate SSH connections
  1. Open three terminals on your laptop
  2. SSH into Pi in each: ssh pi@robot[X].local
  3. Source workspace in each:
     source /opt/ros/humble/setup.bash
     source ~/ros2_ws/install/setup.bash

Choose your method and set up three terminals now."

echo ""
if ask_yes_no "Do you have three terminals ready?"; then
    echo -e "${GREEN}✓ Great! Let's start the nodes.${NC}"
else
    echo "Take your time to set up three terminals."
    echo "Press Enter when ready..."
    read -r
fi

echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 33: Start Camera Node
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 32 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step33_start_camera_node_terminal_1"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 33/50: Start Camera Node (Terminal 1)"

show_manual_task "IN TERMINAL 1:

COMMAND TO RUN:
  ros2 run freenove_car camera_node

WHAT IT DOES:
  • Initializes camera
  • Captures images at 30 FPS
  • Publishes to: /freenove/camera/image_raw

EXPECTED OUTPUT:
  [INFO] [camera]: Initializing Picamera2...
  [INFO] [camera]: ✓ Picamera2 initialized successfully
  [INFO] [camera]:   Resolution: 640x480
  [INFO] [camera]:   Format: RGB888
  [INFO] [camera]: Camera node started - publishing to /freenove/camera/image_raw at 30 Hz

TROUBLESHOOTING:
  • If camera error: Check connections
  • If import error: Missing packages (shouldn't happen)

Let it run! Don't close this terminal."

echo ""
echo "✍️  In your activity sheet:"
echo "  • What topic does camera_node publish to?"
echo "  • What message type does it use?"
echo ""
echo ""
if ask_yes_no "Is camera_node running without errors?"; then
    echo -e "${GREEN}✓ Camera node is running!${NC}"
else
    echo -e "${YELLOW}⚠ Camera node has issues.${NC}"
    echo "Check the error message and troubleshoot."
    echo "Ask instructor if needed."
fi

save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 34: Monitor Camera Topic
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 33 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step34_verify_camera_is_publishing"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 34/50: Verify Camera is Publishing"

show_manual_task "IN TERMINAL 2 (temporarily):

Let's verify camera is publishing before starting lane follower.

COMMAND TO RUN:
  ros2 topic list

SHOULD SEE: /freenove/camera/image_raw

THEN CHECK FREQUENCY:
  ros2 topic hz /freenove/camera/image_raw

SHOULD SEE: ~30 Hz

Press Ctrl+C to stop monitoring."

echo ""
if ask_yes_no "Is camera publishing at ~30 Hz?"; then
    echo -e "${GREEN}✓ Camera is publishing correctly!${NC}"
else
    echo -e "${YELLOW}⚠ Camera topic issues.${NC}"
    echo "Troubleshoot before continuing."
fi

save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 35: Start Lane Follower Node
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 34 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step35_start_lane_follower_node_termi"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 35/50: Start Lane Follower Node (Terminal 2)"

show_manual_task "IN TERMINAL 2:

COMMAND TO RUN:
  ros2 run freenove_car lane_follower_node

WHAT IT DOES:
  • Subscribes to camera images
  • Processes images (Canny, Hough transform)
  • Detects lane lines
  • Calculates steering
  • Publishes velocity commands to: /freenove/cmd_vel

EXPECTED OUTPUT:
  [INFO] [lane_follower]: Lane follower node started
  [INFO] [lane_follower]: Subscribing to: /freenove/camera/image_raw
  [INFO] [lane_follower]: Publishing to: /freenove/cmd_vel
  [INFO] [lane_follower]: Lines detected: 2 | Steering: 0.15 rad | Speed: 0.25 m/s

WHAT YOU'LL SEE:
  • Continuous stream of processing messages
  • Steering commands being calculated

Let it run!"

echo ""
echo "✍️  In your activity sheet:"
echo "  • What does lane_follower subscribe to?"
echo "  • What does lane_follower publish to?"
echo "  • What computer vision techniques does it use?"
echo ""
echo ""
if ask_yes_no "Is lane_follower_node running without errors?"; then
    echo -e "${GREEN}✓ Lane follower node is running!${NC}"
else
    echo -e "${YELLOW}⚠ Lane follower has issues.${NC}"
    echo "Check error messages."
fi

save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 36: Monitor Velocity Commands
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 35 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step36_verify_lane_follower_is_publis"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 36/50: Verify Lane Follower is Publishing"

show_manual_task "IN TERMINAL 3 (temporarily):

Let's verify lane follower is publishing velocity commands.

COMMAND:
  ros2 topic echo /freenove/cmd_vel

YOU SHOULD SEE Twist messages:
  linear:
    x: 0.2
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.15

Press Ctrl+C to stop monitoring."

echo ""
if ask_yes_no "Are velocity commands being published?"; then
    echo -e "${GREEN}✓ Lane follower is publishing!${NC}"
else
    echo -e "${YELLOW}⚠ No velocity commands.${NC}"
    echo "Check if camera is publishing images."
fi

save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 37: Start Motor Control Node
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 36 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step37_start_motor_control_node_termi"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 37/50: Start Motor Control Node (Terminal 3)"

echo -e "${RED}⚠️  SAFETY FIRST! ⚠️${NC}"
echo ""
echo "BEFORE starting motor node:"
echo "  ☐ Robot on blocks OR held off table"
echo "  ☐ Clear space around robot"
echo "  ☐ Ready to press Ctrl+C if needed"
echo "  ☐ S2 motor power switch ON"
echo ""
wait_for_confirmation

show_manual_task "IN TERMINAL 3:

COMMAND TO RUN:
  sudo -E ros2 run freenove_car motor_control_node

WHY sudo -E?
  • sudo: Needed for GPIO access (motor control)
  • -E: Preserves environment (ROS variables)

WHAT IT DOES:
  • Subscribes to: /freenove/cmd_vel
  • Receives Twist messages
  • Controls motors based on commands
  • Moves robot!

EXPECTED OUTPUT:
  [INFO] [motor_control]: Freenove motor controller initialized
  [INFO] [motor_control]: Motor control node started
  [INFO] [motor_control]: Subscribing to: /freenove/cmd_vel
  [INFO] [motor_control]: Base speed: 50 (PWM range: 0-100)
  [INFO] [motor_control]: Send Twist messages to control the robot!
  [INFO] [motor_control]: Cmd received - Lin: 0.25 m/s | Ang: 0.15 rad/s | L: 65 R: 35 (PWM)

WHAT SHOULD HAPPEN:
  • Wheels start moving!
  • Robot responds to lane detection
  • You see velocity commands in logs"

echo ""
echo "✍️  In your activity sheet:"
echo "  • Why does motor_control need sudo?"
echo "  • What topic does it subscribe to?"
echo ""
echo ""
if ask_yes_no "Is motor_control_node running and wheels responding?"; then
    echo -e "${GREEN}✓ Motor control is working!${NC}"
    echo -e "${GREEN}✓ ALL THREE NODES ARE RUNNING!${NC}"
else
    echo -e "${YELLOW}⚠ Motor control issues.${NC}"
    echo "Check power, connections, permissions."
fi

save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 38: Test Lane Following
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 37 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step38_test_autonomous_lane_following"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 38/50: Test Autonomous Lane Following"

show_manual_task "Time to test your autonomous robot!

OPTION 1: Test Track
If you have a lane-following test track:
  1. Place robot at start of track
  2. Make sure camera can see lane markings
  3. Observe robot following the lane!

OPTION 2: Paper Test
If no track, use white paper with dark lines:
  1. Draw two parallel lines (lanes)
  2. Hold paper in front of camera at angle
  3. Watch wheels respond as you move paper

WHAT TO OBSERVE:
  • Robot tries to center itself
  • Steering adjusts based on line position
  • Smooth or jerky motion?
  • How well does it follow?

Take notes on behavior for your activity sheet!"

echo ""
echo "✍️  In your activity sheet:"
echo "  • Describe robot behavior"
echo "  • Did it successfully follow lanes?"
echo "  • What could be improved?"
echo ""
echo ""
if ask_yes_no "Did the robot demonstrate lane-following behavior?"; then
    echo -e "${GREEN}🎉 SUCCESS! Your autonomous robot works!${NC}"
else
    echo -e "${YELLOW}Partial success - needs tuning.${NC}"
    echo "This is normal! Real robotics requires iteration."
fi

save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 39: Stop All Nodes
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 38 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step39_stop_the_system"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 39/50: Stop the System"

show_manual_task "Time to stop all nodes safely.

STOPPING ORDER (IMPORTANT):
  1. Terminal 3: Ctrl+C (stop motor control FIRST)
     ↓ Motors stop
  2. Terminal 2: Ctrl+C (stop lane follower)
  3. Terminal 1: Ctrl+C (stop camera)

WHY THIS ORDER?
  • Stop motors first for safety
  • Then stop the nodes that send commands

Do this now in your three terminals."

echo ""
if ask_yes_no "Have you stopped all three nodes?"; then
    echo -e "${GREEN}✓ System stopped safely.${NC}"
else
    echo "Stop all nodes before continuing."
    wait_for_confirmation
fi

echo ""
wait_for_confirmation

#==============================================================================
# Part 7 Complete
#==============================================================================
clear
echo "=========================================="
echo "   ✓ Part 7 Complete!"
echo "=========================================="
echo ""
echo -e "${GREEN}You ran a complete autonomous system!${NC}"
echo ""
echo "WHAT YOU ACCOMPLISHED:"
echo "  ✓ Ran three ROS nodes simultaneously"
echo "  ✓ Nodes communicated via topics"
echo "  ✓ Robot responded to vision processing"
echo "  ✓ Demonstrated autonomous behavior"
echo ""
echo "SYSTEM ARCHITECTURE:"
echo "  Camera → Image Topic → Lane Follower"
echo "  Lane Follower → Velocity Topic → Motor Control"
echo "  Motor Control → Physical Motors → Movement!"
echo ""
echo "✍️  In your activity sheet:"
echo "  • Complete Part 7 observations"
echo "  • Describe robot performance"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# PART 8: DEBUGGING & VERIFICATION
#==============================================================================


else
    echo -e "${CYAN}⏭  Skipping: Step 39 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="part8_intro"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "PART 8: Debugging Tools & Verification"

echo "Now you'll learn the ROS 2 debugging tools."
echo ""
echo "These commands help you:"
echo "  • See what's running"
echo "  • Monitor communication"
echo "  • Troubleshoot issues"
echo "  • Understand the system"
echo ""
echo "Essential skills for robotics development!"
echo ""
echo "⏱️  Time: ~20 minutes"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 40: List Running Nodes
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Debugging Tools & Verification (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step40_debug_command_list_nodes"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 40/50: Debug Command - List Nodes"

show_command "ros2 node list"

show_explanation "Lists all active ROS nodes.

WHEN TO USE:
  • Check if nodes are running
  • Verify node names
  • Troubleshoot communication issues

WITH YOUR SYSTEM STOPPED, should show nothing.

TRY IT YOURSELF NOW:"

wait_for_user
execute_command "ros2 node list"

echo ""
echo "✍️  In your activity sheet:"
echo "  • Write this command"
echo "  • What would you see when all three nodes run?"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 41: List Active Topics
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 40 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step41_debug_command_list_topics"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 41/50: Debug Command - List Topics"

show_command "ros2 topic list"

show_explanation "Lists all active ROS topics (communication channels).

WHEN TO USE:
  • See what topics exist
  • Verify topic names
  • Check node communication

WITH NODES STOPPED, you'll see system topics only.

WHEN YOUR SYSTEM RUNS, you'd see:
  • /freenove/camera/image_raw
  • /freenove/cmd_vel
  • Plus ROS system topics

TRY IT NOW:"

wait_for_user
execute_command "ros2 topic list"

echo ""
echo "✍️  In your activity sheet:"
echo "  • Write this command"
echo "  • Which topics would your system use?"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 42: Get Node Information
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 41 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step42_debug_command_node_info"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 42/50: Debug Command - Node Info"

show_command "ros2 node info /lane_follower"

show_explanation "Shows detailed information about a specific node.

DISPLAYS:
  • Subscriptions (topics it listens to)
  • Publications (topics it publishes to)
  • Services it provides
  • Actions it supports

WHEN TO USE:
  • Understand node connections
  • Verify topics match
  • Troubleshoot communication

NOTE: This only works when the node is running!
Since your nodes are stopped, this command would fail now.

BUT YOU'D SEE:
  Subscriptions:
    /freenove/camera/image_raw: sensor_msgs/msg/Image
  Publications:
    /freenove/cmd_vel: geometry_msgs/msg/Twist"

echo ""
echo "✍️  In your activity sheet:"
echo "  • Write this command"
echo "  • What info does it show?"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 43: Check Topic Information
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 42 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step43_debug_command_topic_info"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 43/50: Debug Command - Topic Info"

show_command "ros2 topic info /freenove/cmd_vel"

show_explanation "Shows information about a specific topic.

DISPLAYS:
  • Message type
  • Number of publishers
  • Number of subscribers

EXAMPLE OUTPUT:
  Type: geometry_msgs/msg/Twist
  Publisher count: 1 (lane_follower)
  Subscription count: 1 (motor_control)

WHEN TO USE:
  • Verify topic exists
  • Check message type
  • Confirm publishers/subscribers match

NOTE: Only works when nodes are running!"

echo ""
echo "✍️  In your activity sheet:"
echo "  • Write this command"
echo "  • What does it tell you?"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 44: Monitor Topic Frequency
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 43 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step44_debug_command_topic_frequency"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 44/50: Debug Command - Topic Frequency"

show_command "ros2 topic hz /freenove/camera/image_raw"

show_explanation "Measures how fast messages are published (frequency).

OUTPUT EXAMPLE:
  average rate: 30.123
    min: 0.032s max: 0.034s

WHEN TO USE:
  • Verify camera frame rate
  • Check if nodes are responsive
  • Diagnose performance issues

YOUR CAMERA should publish at ~30 Hz
YOUR CMD_VEL varies (depends on processing)

NOTE: Only works when nodes are running!
Press Ctrl+C to stop monitoring."

echo ""
echo "✍️  In your activity sheet:"
echo "  • Write this command"
echo "  • What frequency did camera publish at?"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 45: Echo Topic Messages
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 44 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step45_debug_command_echo_messages"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 45/50: Debug Command - Echo Messages"

show_command "ros2 topic echo /freenove/cmd_vel"

show_explanation "Displays messages being published on a topic.

OUTPUT EXAMPLE:
  linear:
    x: 0.25
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.15
  ---

WHEN TO USE:
  • See actual message content
  • Verify correct values
  • Understand data flow
  • Debug steering behavior

CONNECTION TO SESSION 2:
Same command you used in TurtleSim to see /turtle1/cmd_vel!

NOTE: Use --no-arr flag for images (too much data)
Press Ctrl+C to stop echoing."

echo ""
echo "✍️  In your activity sheet:"
echo "  • Write this command"
echo "  • What message type is cmd_vel?"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 46: Visualize ROS Graph
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 45 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step46_debug_tool_ros_graph_visualiza"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 46/50: Debug Tool - ROS Graph Visualization"

show_command "ros2 run rqt_graph rqt_graph"

show_explanation "Opens a GUI showing the ROS computation graph.

DISPLAYS:
  • Nodes as boxes
  • Topics as ovals
  • Arrows showing data flow

EXAMPLE:
  [camera] → /image_raw → [lane_follower]
  [lane_follower] → /cmd_vel → [motor_control]

WHEN TO USE:
  • Visualize architecture
  • Verify connections
  • Debug communication issues
  • Understand system structure

CONNECTION TO SESSION 2:
Same tool you used to visualize TurtleSim!

NOTE: This opens a GUI window.
On headless Pi, you might not see it.
But on your laptop with X11 forwarding: ssh -X pi@robot.local"

echo ""
echo "✍️  In your activity sheet:"
echo "  • Draw the ROS graph for your system"
echo "  • Label nodes and topics"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 47: Manually Publish Commands (Testing)
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 46 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step47_debug_tool_publish_test_comman"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 47/50: Debug Tool - Publish Test Commands"

show_command "ros2 topic pub /freenove/cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.2}, angular: {z: 0.0}}\""

show_explanation "Manually publishes a message to a topic.

USE CASE:
  • Test motor control without lane follower
  • Verify motors respond to commands
  • Debug steering behavior
  • Understand Twist messages

WHAT THIS COMMAND DOES:
  • Publishes to /freenove/cmd_vel
  • Message type: Twist
  • Content: Move forward (x=0.2), no turning (z=0.0)

TO TEST:
  1. Start motor_control_node only
  2. Run this command
  3. Robot should move forward
  4. Ctrl+C to stop

TRY DIFFERENT VALUES:
  • {linear: {x: 0.0}, angular: {z: 0.5}} → Turn left
  • {linear: {x: -0.2}, angular: {z: 0.0}} → Backward

NOTE: Need motor_control running and robot on blocks!"

echo ""
echo "✍️  In your activity sheet:"
echo "  • Write this command"
echo "  • What does it test?"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# Step 48: Save Debugging Reference
#==============================================================================

else
    echo -e "${CYAN}⏭  Skipping: Step 47 (already complete)${NC}"
    sleep 1
fi

STEP_NAME="step48_essential_debugging_commands_s"
if ! is_checkpoint_complete "$STEP_NAME" || [ "$RESUME_MODE" = false ]; then

show_step "Step 48/50: Essential Debugging Commands Summary"

cat << "EOF"

📋 DEBUGGING QUICK REFERENCE:

Check what's running:
  ros2 node list              - List all nodes
  ros2 topic list             - List all topics

Get detailed info:
  ros2 node info /node_name   - Node details
  ros2 topic info /topic_name - Topic details

Monitor communication:
  ros2 topic hz /topic        - Check frequency
  ros2 topic echo /topic      - See messages

Visualize system:
  ros2 run rqt_graph rqt_graph - Show graph

Test manually:
  ros2 topic pub /topic Type "data" - Publish test message

Stop everything:
  pkill -9 -f ros2            - Kill all ROS nodes

EOF

echo ""
echo "✍️  Copy this to your activity sheet or take a photo!"
echo ""
wait_for_confirmation

#==============================================================================
# Part 8 Complete
#==============================================================================
clear
echo "=========================================="
echo "   ✓ Part 8 Complete!"
echo "=========================================="
echo ""
echo -e "${GREEN}You learned essential ROS debugging tools!${NC}"
echo ""
echo "DEBUGGING SKILLS:"
echo "  ✓ List nodes and topics"
echo "  ✓ Monitor message frequency"
echo "  ✓ View message content"
echo "  ✓ Visualize system architecture"
echo "  ✓ Manually test components"
echo ""
echo "These skills are essential for:"
echo "  • Troubleshooting issues"
echo "  • Understanding system behavior"
echo "  • Professional robotics development"
echo ""
save_checkpoint "$STEP_NAME"
wait_for_confirmation

#==============================================================================
# SESSION COMPLETE!
#==============================================================================

clear
cat << "EOF"
==========================================
   🎉 SESSION 5 COMPLETE! 🎉
==========================================
EOF
echo ""
echo -e "${GREEN}Congratulations! You've completed the entire session!${NC}"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "WHAT YOU ACCOMPLISHED TODAY:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "✅ Part 1: Installed ROS 2 Humble"
echo "   • Core framework"
echo "   • Development tools"
echo "   • Computer vision packages"
echo ""
echo "✅ Part 2: Verified Hardware"
echo "   • Tested motors"
echo "   • Tested camera"
echo ""
echo "✅ Part 3: Downloaded Node Code"
echo "   • Three ROS nodes from GitHub"
echo ""
echo "✅ Part 4: Created ROS Workspace"
echo "   • Workspace structure"
echo "   • ROS package"
echo "   • Copied node files"
echo ""
echo "✅ Part 5: Configured Package"
echo "   • Entry points"
echo "   • Dependencies"
echo ""
echo "✅ Part 6: Built Workspace"
echo "   • Compiled with colcon"
echo "   • Sourced workspace"
echo ""
echo "✅ Part 7: Ran Autonomous System"
echo "   • Three nodes running"
echo "   • Lane following demonstrated"
echo ""
echo "✅ Part 8: Learned Debugging Tools"
echo "   • ROS command-line tools"
echo "   • System visualization"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "SYSTEM ARCHITECTURE YOU BUILT:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "  ┌─────────────────┐"
echo "  │  Camera Node    │ Captures images"
echo "  └────────┬────────┘"
echo "           │"
echo "           │ /freenove/camera/image_raw"
echo "           │ (sensor_msgs/Image)"
echo "           ▼"
echo "  ┌─────────────────┐"
echo "  │ Lane Follower   │ Computer vision"
echo "  └────────┬────────┘"
echo "           │"
echo "           │ /freenove/cmd_vel"
echo "           │ (geometry_msgs/Twist)"
echo "           ▼"
echo "  ┌─────────────────┐"
echo "  │ Motor Control   │ Drives wheels"
echo "  └─────────────────┘"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "CONNECTION TO SESSION 2 (TurtleSim):"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Session 2              →  Session 5"
echo "────────────────────────────────────────"
echo "Simulated turtle       →  Real robot"
echo "/turtle1/cmd_vel       →  /freenove/cmd_vel"
echo "Keyboard control       →  Vision control"
echo "ros2 topic list        →  ros2 topic list (same!)"
echo "Twist messages         →  Twist messages (same!)"
echo "rqt_graph              →  rqt_graph (same tool!)"
echo ""
echo "SAME CONCEPTS, REAL HARDWARE! 🤖"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "NEXT STEPS:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📝 Complete Your Activity Sheet:"
echo "   • Fill in any remaining sections"
echo "   • Answer reflection questions"
echo "   • Complete the final checklist"
echo "   • Get instructor sign-off"
echo ""
echo "🔧 Optional Challenges (if time):"
echo "   • Tune lane following parameters"
echo "   • Add safety features"
echo "   • Test on different tracks"
echo "   • Optimize performance"
echo ""
echo "📚 Continue Learning:"
echo "   • Session 6: Advanced features"
echo "   • Explore ROS 2 documentation"
echo "   • Experiment with the code"
echo "   • Build your own nodes!"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "QUICK COMMAND REFERENCE:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Run your system:"
echo "  Terminal 1: ros2 run freenove_car camera_node"
echo "  Terminal 2: ros2 run freenove_car lane_follower_node"
echo "  Terminal 3: sudo -E ros2 run freenove_car motor_control_node"
echo ""
echo "Debug your system:"
echo "  ros2 node list"
echo "  ros2 topic list"
echo "  ros2 topic echo /freenove/cmd_vel"
echo "  ros2 topic hz /freenove/camera/image_raw"
echo ""
echo "Rebuild if you edit code:"
echo "  cd ~/ros2_ws"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo -e "${CYAN}Thank you for following this interactive guide!${NC}"
echo ""
echo "You've learned:"
echo "  • ROS 2 installation and configuration"
echo "  • Workspace and package management"
echo "  • Multi-node system architecture"
echo "  • Computer vision integration"
echo "  • Hardware control with ROS"
echo "  • Professional debugging techniques"
echo ""
echo "These are the foundations of modern robotics!"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Questions? Issues? Ideas?"
echo "  • Ask your instructor"
echo "  • Check the GitHub repo"
echo "  • Review the documentation"
echo "  • Collaborate with your team"
echo ""
echo "Happy robot building! 🤖🚀"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
read -p "Press Enter to exit..."

exit 0

else
    echo -e "${CYAN}⏭  Skipping: Step 48 (already complete)${NC}"
    sleep 1
fi

