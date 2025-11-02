#!/bin/bash
# Interactive ROS 2 Humble Installation Script
# Session 5: ROS 2 Integration
# Engineering Teamwork III - AI and Autonomous Systems Lab
# Berlin University of Applied Sciences

# Colors for better visibility
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to wait for user
wait_for_user() {
    echo ""
    echo -e "${BLUE}✍️  Write this in your activity sheet, then press Enter to continue...${NC}"
    read -r
}

# Function to execute command
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

# Function to display step header
show_step() {
    clear
    echo "=========================================="
    echo "  Step $1: $2"
    echo "=========================================="
    echo ""
}

# Function to show explanation
show_explanation() {
    echo -e "${YELLOW}📚 WHAT THIS DOES:${NC}"
    echo "$1"
    echo ""
}

# Function to show command
show_command() {
    echo -e "${BLUE}💻 COMMAND:${NC}"
    echo "  $1"
    echo ""
}

# Main script starts here
clear
echo "=========================================="
echo "   ROS 2 Humble Interactive Installation"
echo "   Session 5: ROS 2 Integration"
echo "=========================================="
echo ""
echo "Welcome! This script will guide you through"
echo "installing ROS 2 Humble on your Raspberry Pi."
echo ""
echo "HOW IT WORKS:"
echo "  1. You'll see what each command does"
echo "  2. Write the explanation in your activity sheet"
echo "  3. Press Enter to execute the command"
echo "  4. See the result and continue"
echo ""
echo "⏱️  Total time: ~30-40 minutes"
echo ""
echo "Ready? Let's begin!"
echo ""
read -p "Press Enter to start..."

# Check if running as root
if [ "$EUID" -eq 0 ]; then 
   echo "Please do not run this script as root (with sudo)"
   echo "Run it as normal user: ./interactive_install.sh"
   exit 1
fi

#==============================================================================
# STEP 1: Update Package Lists
#==============================================================================
show_step "1" "Update Package Lists"

show_command "sudo apt update"

show_explanation "This downloads the latest list of available software packages
from Ubuntu's repositories. Think of it like refreshing the
'app store' to see what software is available.

WHY WE NEED IT: Before installing anything, we need to know
what versions are available."

wait_for_user

execute_command "sudo apt update"

#==============================================================================
# STEP 2: Install Basic Tools
#==============================================================================
show_step "2" "Install Basic Tools (locales, curl)"

show_command "sudo apt install -y locales curl software-properties-common"

show_explanation "Installing essential tools we'll need:
  • locales: Sets up language/region settings (prevents encoding errors)
  • curl: Downloads files from the internet
  • software-properties-common: Manages software repositories

These are like 'helper tools' we need before installing ROS."

wait_for_user

execute_command "sudo apt install -y locales curl software-properties-common"

#==============================================================================
# STEP 3: Configure Locale
#==============================================================================
show_step "3" "Set System Language to English"

show_command "sudo locale-gen en_US en_US.UTF-8"

show_explanation "This generates English (US) language support for the system.

WHY: ROS messages and commands expect English. This prevents
errors with special characters or different alphabets."

wait_for_user

execute_command "sudo locale-gen en_US en_US.UTF-8"

show_command "export LANG=en_US.UTF-8"

show_explanation "This tells the current terminal session to use English.

The 'export' command sets an environment variable that
programs can read to know which language to use."

wait_for_user

execute_command "export LANG=en_US.UTF-8"

#==============================================================================
# STEP 4: Enable Universe Repository
#==============================================================================
show_step "4" "Enable Ubuntu Universe Repository"

show_command "sudo add-apt-repository universe -y"

show_explanation "Ubuntu has different 'repositories' (collections of software):
  • Main: Officially supported software
  • Universe: Community-maintained software (includes ROS!)

We're adding Universe so we can install ROS packages."

wait_for_user

execute_command "sudo add-apt-repository universe -y"

#==============================================================================
# STEP 5: Add ROS 2 GPG Key
#==============================================================================
show_step "5" "Add ROS 2 Security Key"

show_command "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"

show_explanation "This downloads a security key (GPG key) from the ROS project.

WHY: When you download software, you need to verify it's
authentic and hasn't been tampered with. This key is like
a digital signature that proves the software came from
the official ROS project.

SECURITY: Always verify software sources!"

wait_for_user

execute_command "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"

#==============================================================================
# STEP 6: Add ROS 2 Repository
#==============================================================================
show_step "6" "Add ROS 2 Package Repository"

show_command "echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main\" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null"

show_explanation "This tells Ubuntu WHERE to find ROS 2 packages.

BREAKDOWN:
  • 'deb': Debian package format
  • 'arch=...': For your CPU architecture (ARM64 on Raspberry Pi)
  • 'signed-by=...': Use the security key we just downloaded
  • 'jammy': Ubuntu 22.04 codename
  • The URL: Where ROS packages are hosted

After this, Ubuntu knows to look at packages.ros.org for ROS!"

wait_for_user

execute_command "echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main\" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null"

#==============================================================================
# STEP 7: Update Package Lists Again
#==============================================================================
show_step "7" "Refresh Package Lists"

show_command "sudo apt update"

show_explanation "Now that we added the ROS repository, we need to refresh
the package list again to include ROS packages.

Before Step 6: Ubuntu knew about regular packages
After Step 6: Ubuntu also knows about ROS packages
After Step 7: Package list is updated with ROS!"

wait_for_user

execute_command "sudo apt update"

#==============================================================================
# STEP 8: Upgrade System (Optional but Recommended)
#==============================================================================
show_step "8" "Upgrade Existing Packages"

show_command "sudo apt upgrade -y"

show_explanation "This updates all your currently installed software to the
latest versions.

DIFFERENCE FROM 'apt update':
  • apt update: Refreshes the LIST of what's available
  • apt upgrade: Actually INSTALLS the updates

This ensures you have security patches and bug fixes.
⏱️  This may take 2-5 minutes..."

wait_for_user

execute_command "sudo apt upgrade -y"

#==============================================================================
# STEP 9: Install ROS 2 Humble Base
#==============================================================================
show_step "9" "Install ROS 2 Humble (Main Installation)"

show_command "sudo apt install -y ros-humble-ros-base"

show_explanation "THIS IS THE BIG ONE! Installing ROS 2 Humble base system.

WHAT YOU GET:
  • Core ROS 2 libraries (rclcpp, rclpy)
  • Communication system (topics, nodes, messages)
  • Command-line tools (ros2 topic list, ros2 node list)
  • Build tools basics

SIZE: ~500 MB download
⏱️  TIME: 10-20 minutes (longest step - be patient!)

DURING THE WAIT:
  • Read ahead in your activity sheet
  • Review Session 2 TurtleSim concepts
  • Discuss with your team: What will each node do?"

wait_for_user

echo -e "${YELLOW}Starting download... This will take a while. ☕${NC}"
execute_command "sudo apt install -y ros-humble-ros-base"

#==============================================================================
# STEP 10: Install Development Tools
#==============================================================================
show_step "10" "Install Development Tools"

show_command "sudo apt install -y ros-dev-tools python3-colcon-common-extensions"

show_explanation "Installing build tools for ROS development:

  • ros-dev-tools: General ROS development utilities
  • colcon: Build system for ROS 2 packages (like 'make' for ROS)

WHY: In Part 6, you'll use 'colcon build' to compile your
robot code. Without these tools, you can't build ROS packages!"

wait_for_user

execute_command "sudo apt install -y ros-dev-tools python3-colcon-common-extensions"

#==============================================================================
# STEP 11: Setup ROS Environment
#==============================================================================
show_step "11" "Setup ROS 2 Environment Variables"

show_command "source /opt/ros/humble/setup.bash"

show_explanation "This 'sources' the ROS 2 environment - loading ROS commands
into your current terminal session.

WHAT IT DOES:
  • Adds ROS commands to your PATH (so you can type 'ros2')
  • Sets environment variables ROS needs
  • Makes ROS libraries findable

WITHOUT THIS: You'd get 'ros2: command not found' errors!"

wait_for_user

execute_command "source /opt/ros/humble/setup.bash"

show_command "echo \"source /opt/ros/humble/setup.bash\" >> ~/.bashrc"

show_explanation "Adding the source command to ~/.bashrc makes it automatic.

~/.bashrc runs every time you open a new terminal.
Now ROS will be available in ALL future terminals automatically!

BEFORE: Had to type 'source /opt/ros/humble/setup.bash' every time
AFTER: ROS loads automatically in new terminals"

wait_for_user

execute_command "echo \"source /opt/ros/humble/setup.bash\" >> ~/.bashrc"

#==============================================================================
# STEP 12: Verify ROS Installation
#==============================================================================
show_step "12" "Verify ROS 2 Installation"

show_command "echo \$ROS_DISTRO"

show_explanation "Checking which ROS version is active.

ROS_DISTRO is an environment variable set by ROS.
Should output: 'humble'

If it shows 'humble', ROS is properly loaded!"

wait_for_user

execute_command "echo \$ROS_DISTRO"

#==============================================================================
# STEP 13: Install Computer Vision Packages
#==============================================================================
show_step "13" "Install Camera & Vision Packages"

show_command "sudo apt install -y ros-humble-cv-bridge ros-humble-vision-opencv ros-humble-image-transport ros-humble-camera-info-manager"

show_explanation "Installing ROS packages for working with cameras and images:

  • cv-bridge: Converts between ROS images and OpenCV
    (Remember Session 3? OpenCV for computer vision!)
  
  • vision-opencv: OpenCV integration with ROS 2
  
  • image-transport: Efficiently sends images between nodes
    (Compresses large image data)
  
  • camera-info-manager: Handles camera calibration data

YOU'LL USE THESE: In your camera_node and lane_follower_node!"

wait_for_user

execute_command "sudo apt install -y ros-humble-cv-bridge ros-humble-vision-opencv ros-humble-image-transport ros-humble-camera-info-manager"

#==============================================================================
# STEP 14: Install Python Dependencies
#==============================================================================
show_step "14" "Install Python Libraries"

show_command "sudo apt install -y python3-opencv python3-pip"

show_explanation "Installing Python libraries for image processing:

  • python3-opencv: OpenCV library for Python
    (Canny edge detection, Hough transform - Session 3!)
  
  • python3-pip: Python package installer
    (Lets you install additional Python packages)

These provide the computer vision algorithms you'll use
in lane_follower_node."

wait_for_user

execute_command "sudo apt install -y python3-opencv python3-pip"

show_command "pip3 install numpy"

show_explanation "Installing NumPy - the fundamental package for numerical
computing in Python.

NumPy provides:
  • Fast array operations (images are arrays of pixels!)
  • Mathematical functions
  • Linear algebra operations

LANE DETECTION uses NumPy for:
  • Image array manipulation
  • Calculating line slopes
  • Mathematical operations on pixel values"

wait_for_user

execute_command "pip3 install numpy"

#==============================================================================
# STEP 15: Test ROS Commands
#==============================================================================
show_step "15" "Test ROS 2 Commands"

show_command "ros2 topic list"

show_explanation "Testing if ROS 2 commands work.

This lists all active ROS topics (communication channels).

EXPECTED: Should run without errors (but show nothing yet
since no nodes are running).

CONNECTION TO SESSION 2: Same command you used in TurtleSim!
Remember 'ros2 topic list' showed /turtle1/cmd_vel?"

wait_for_user

execute_command "ros2 topic list"

show_command "ros2 node list"

show_explanation "Listing all active ROS nodes (programs).

EXPECTED: Empty (no nodes running yet)

CONNECTION TO SESSION 2: Like when you ran 'ros2 node list'
and saw /turtlesim and /teleop_turtle!"

wait_for_user

execute_command "ros2 node list"

#==============================================================================
# Installation Complete!
#==============================================================================
clear
echo "=========================================="
echo "   ✓ Installation Complete!"
echo "=========================================="
echo ""
echo -e "${GREEN}Congratulations! ROS 2 Humble is installed!${NC}"
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
echo "  ros2 command: $(which ros2)"
echo ""
echo "NEXT STEPS (Continue with Session 5):"
echo ""
echo "Part 2: Hardware Verification"
echo "  → Test motors and camera with Freenove code"
echo ""
echo "Part 3: Download ROS Node Code"
echo "  → git clone https://github.com/RuthraBellan/freenove-ros2-nodes.git"
echo ""
echo "Part 4: Create ROS Workspace"
echo "  → Build your freenove_car package"
echo ""
echo "Part 5-8: Run and test your autonomous robot!"
echo ""
echo "=========================================="
echo ""
echo "✍️  COMPLETE YOUR ACTIVITY SHEET:"
echo "  - Fill in any remaining explanations"
echo "  - Answer reflection questions"
echo "  - Get instructor sign-off"
echo ""
echo "Need help? Check:"
echo "  • Session 5 PDF for detailed instructions"
echo "  • GitHub repo: RuthraBellan/freenove-ros2-nodes"
echo "  • Ask your instructor or teammates"
echo ""
echo "=========================================="
echo ""
read -p "Press Enter to exit..."

exit 0