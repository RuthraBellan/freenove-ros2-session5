# Session 5: ROS 2 Integration - Activity Sheet

**Course:** Engineering Teamwork III - AI and Autonomous Systems Lab  
**Session:** 5 of 9  
**Date:** _______________

---

## Student Information

**Name:** ________________________________

**Group Number:** ______

**Team Members:**
1. _________________________________
2. _________________________________
3. _________________________________
4. _________________________________
5. _________________________________
6. _________________________________

**Robot ID:** _______________

---

## Learning Objectives

By the end of this session, you will be able to:
- [ ] Install and configure ROS 2 Humble on Raspberry Pi
- [ ] Explain what each installation command does
- [ ] Create a ROS 2 workspace and package
- [ ] Understand the three-node architecture
- [ ] Run and test an autonomous lane-following system
- [ ] Use ROS 2 debugging tools

---

# PART 1: Interactive Installation (Write as script runs)

## Installation Commands - Understanding What You're Doing

For each step, write in YOUR OWN WORDS what the command does and why it's needed.

---

### Step 1: `sudo apt update`

**What does this command do?**

________________________________________________________________

________________________________________________________________

**Why do we need it?**

________________________________________________________________

**Did it work?** ☐ Yes  ☐ No  ☐ With errors

---

### Step 2: `sudo apt install -y locales curl software-properties-common`

**What tools are being installed?**

1. locales: _____________________________________________________

2. curl: ________________________________________________________

3. software-properties-common: __________________________________

**Why do we need these before installing ROS?**

________________________________________________________________

________________________________________________________________

---

### Step 3: `sudo locale-gen en_US en_US.UTF-8`

**What does this command do?**

________________________________________________________________

________________________________________________________________

**Why does ROS need English language settings?**

________________________________________________________________

---

### Step 4: `sudo add-apt-repository universe -y`

**What is the "Universe" repository?**

________________________________________________________________

________________________________________________________________

**Why do we need to enable it?**

________________________________________________________________

---

### Step 5: Download ROS 2 Security Key

**Command:** `sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg`

**What is a GPG key?**

________________________________________________________________

________________________________________________________________

**Why is software security important?**

________________________________________________________________

________________________________________________________________

---

### Step 6: Add ROS 2 Repository

**What does this command tell Ubuntu?**

________________________________________________________________

________________________________________________________________

**What does "jammy" mean?**

________________________________________________________________

---

### Step 7: `sudo apt update` (again)

**Why do we run apt update again after adding the ROS repository?**

________________________________________________________________

________________________________________________________________

---

### Step 8: `sudo apt upgrade -y`

**What's the difference between `apt update` and `apt upgrade`?**

- `apt update`: ________________________________________________

- `apt upgrade`: _______________________________________________

---

### Step 9: `sudo apt install -y ros-humble-ros-base`

**What gets installed with ros-humble-ros-base?**

1. ____________________________________________________________

2. ____________________________________________________________

3. ____________________________________________________________

**Estimated download size:** ____________

**Time taken:** ____________ minutes

---

### Step 10: `sudo apt install -y ros-dev-tools python3-colcon-common-extensions`

**What is colcon?**

________________________________________________________________

________________________________________________________________

**When will you use colcon in this lab?**

________________________________________________________________

---

### Step 11: `source /opt/ros/humble/setup.bash`

**What does the 'source' command do?**

________________________________________________________________

________________________________________________________________

**What happens if you DON'T source this file?**

________________________________________________________________

---

### Step 12: `echo $ROS_DISTRO`

**What did this output?** _______________

**What is an environment variable?**

________________________________________________________________

________________________________________________________________

---

### Step 13: Install Computer Vision Packages

**Match each package to its purpose:**

| Package | Purpose |
|---------|---------|
| cv-bridge | A. _______________________________________ |
| vision-opencv | B. _______________________________________ |
| image-transport | C. _______________________________________ |
| camera-info-manager | D. _______________________________________ |

---

### Step 14: Install Python Libraries

**What is OpenCV used for?**

________________________________________________________________

________________________________________________________________

**What is NumPy used for in lane detection?**

________________________________________________________________

________________________________________________________________

---

### Step 15: Test ROS Commands

**Command:** `ros2 topic list`

**Output:** ____________________________________________________


**What are these system topics used for?**
________________________________________________________________

---

## Part 1 Checkpoint

Installation completed at: __________ (time)

**Verify:**
- [ ] `echo $ROS_DISTRO` shows "humble"
- [ ] `ros2 topic list` runs without errors
- [ ] `ros2 node list` runs without errors
- [ ] All commands executed successfully

---

# PART 2: Understanding ROS Workspace

## Workspace Structure

**Draw and label the ROS 2 workspace directory structure:**

```
~/ros2_ws/
├── _______________/
│   └── freenove_car/
│       ├── _______________/
│       ├── package.xml
│       ├── setup.py
│       └── resource/
├── _______________/
├── _______________/
└── log/
```

**What does each directory do?**

- `src/`: __________________________________________________________

- `build/`: ________________________________________________________

- `install/`: ______________________________________________________

- `log/`: __________________________________________________________

---

## Package Configuration

### Command: `ros2 pkg create freenove_car --build-type ament_python --dependencies rclpy std_msgs sensor_msgs geometry_msgs cv_bridge`

**What does this command create?**

________________________________________________________________

________________________________________________________________

**What are dependencies?**

________________________________________________________________

________________________________________________________________

**List the 5 dependencies and what each provides:**

1. rclpy: _________________________________________________________

2. std_msgs: ______________________________________________________

3. sensor_msgs: ___________________________________________________

4. geometry_msgs: _________________________________________________

5. cv_bridge: _____________________________________________________

---

## Building the Workspace

### Command: `colcon build --symlink-install`

**What does `colcon build` do?**

________________________________________________________________

________________________________________________________________

**What does the `--symlink-install` flag do?**

________________________________________________________________

________________________________________________________________

**Why is this useful during development?**

________________________________________________________________

---

## Part 2 Checkpoint

- [ ] Workspace created at `~/ros2_ws/`
- [ ] Package `freenove_car` created
- [ ] Three Python node files copied
- [ ] Workspace built successfully
- [ ] Workspace sourced

---

# PART 3: Understanding the Three-Node System

## Node Architecture Diagram

**Draw the ROS graph showing the three nodes and their communication:**

```
┌─────────────────┐
│                 │
│  Camera Node    │
│                 │
└────────┬────────┘
         │
         │ (publishes to)
         ▼
    _______________
         │
         │ (subscribes)
         ▼
┌─────────────────┐
│                 │
│ Lane Follower   │
│                 │
└────────┬────────┘
         │
         │ (publishes to)
         ▼
    _______________
         │
         │ (subscribes)
         ▼
┌─────────────────┐
│                 │
│ Motor Control   │
│                 │
└─────────────────┘
```

**Fill in the topic names above.**

---

## Camera Node

**What does camera_node do?**

________________________________________________________________

________________________________________________________________

**What topic does it publish to?**

Topic name: ______________________________

**What message type?** ______________________________

**Publishing frequency:** __________ Hz

---

## Lane Follower Node

**What does lane_follower_node do?**

________________________________________________________________

________________________________________________________________

**What topic does it subscribe to?**

Topic name: ______________________________

**What topic does it publish to?**

Topic name: ______________________________

**What message type does it publish?** ______________________________

**What computer vision techniques does it use? (from Session 3)**

1. ____________________________________________________________

2. ____________________________________________________________

3. ____________________________________________________________

---

## Motor Control Node

**What does motor_control_node do?**

________________________________________________________________

________________________________________________________________

**What topic does it subscribe to?**

Topic name: ______________________________

**Why do we need to run this node with `sudo -E`?**

________________________________________________________________

________________________________________________________________

**What does the `-E` flag do?**

________________________________________________________________

---

## Part 3 Checkpoint

- [ ] Understand what each node does
- [ ] Know the topic names
- [ ] Understand the data flow
- [ ] Can explain why three separate nodes

---

# PART 4: Running and Testing the System

## Starting the Nodes

**Terminal 1 - Camera Node:**

Command used: ____________________________________________________

**What should you see?**

________________________________________________________________

________________________________________________________________

**Terminal 2 - Lane Follower Node:**

Command used: ____________________________________________________

**What should you see?**

________________________________________________________________

________________________________________________________________

**Terminal 3 - Motor Control Node:**

Command used: ____________________________________________________

**What should you see?**

________________________________________________________________

________________________________________________________________

---

## Part 4 Checkpoint

- [ ] All three nodes running
- [ ] No error messages
- [ ] Camera publishing images
- [ ] Lane follower detecting lines
- [ ] Motors responding

---

# PART 5: Debugging Commands

**Write the command for each task:**

### 1. List all active nodes
```bash
________________________________________
```

**Output:**
```
________________________________________
________________________________________
________________________________________
```

---

### 2. List all active topics
```bash
________________________________________
```

**Output (list at least 3 topics):**
```
________________________________________
________________________________________
________________________________________
```

---

### 3. Check the frequency of camera images
```bash
________________________________________
```

**Output:** __________ Hz

---

### 4. Monitor velocity commands being sent
```bash
________________________________________
```

**Sample output (write one Twist message you observed):**
```
linear:
  x: __________
  y: __________
  z: __________
angular:
  x: __________
  y: __________
  z: __________
```

---

### 5. Get information about a specific topic
```bash
________________________________________
```

**What information does this show?**

________________________________________________________________

________________________________________________________________

---

### 6. Visualize the ROS graph
```bash
________________________________________
```

**Did you see the three nodes and topics?** ☐ Yes  ☐ No

---

## Part 5 Checkpoint

- [ ] Used `ros2 node list`
- [ ] Used `ros2 topic list`
- [ ] Used `ros2 topic hz`
- [ ] Used `ros2 topic echo`
- [ ] Used `ros2 topic info`
- [ ] Understand how to debug ROS systems

---

# PART 6: Testing and Observations

## Robot Behavior

**Describe what the robot did when you tested it:**

________________________________________________________________

________________________________________________________________

________________________________________________________________

________________________________________________________________

**Did the robot successfully follow lanes?** ☐ Yes  ☐ Partially  ☐ No

**If not, what issues did you observe?**

________________________________________________________________

________________________________________________________________

**What parameters might you adjust to improve performance?**

________________________________________________________________

________________________________________________________________

---

# REFLECTION QUESTIONS

## Connection to Session 2 (TurtleSim)

### 1. How is your lane-following system similar to TurtleSim?

**Similarities:**

________________________________________________________________

________________________________________________________________

________________________________________________________________

**Same commands used:**

________________________________________________________________

________________________________________________________________

**Same message types:**

________________________________________________________________

---

### 2. How is it different from TurtleSim?

**Differences:**

________________________________________________________________

________________________________________________________________

________________________________________________________________

---

## Understanding ROS Architecture

### 3. Why do we use separate nodes instead of one big program?

**Advantages of modular nodes:**

1. ____________________________________________________________

2. ____________________________________________________________

3. ____________________________________________________________

---

### 4. What would happen if the camera node crashed?

________________________________________________________________

________________________________________________________________

**Would the other nodes still run?** ☐ Yes  ☐ No

**Why?**

________________________________________________________________

________________________________________________________________

---

### 5. What is the advantage of using topics for communication?

________________________________________________________________

________________________________________________________________

________________________________________________________________

---

## Real-World Applications

### 6. Name three real-world applications that use similar ROS systems:

1. ____________________________________________________________

2. ____________________________________________________________

3. ____________________________________________________________

---

### 7. What other sensors could you add to make the robot more capable?

1. ____________________________________________________________

2. ____________________________________________________________

3. ____________________________________________________________

---

# CHALLENGES (Optional - If Time Permits)

## Challenge 1: Tune the Lane Follower

**Parameter to adjust:** ______________________________________

**Original value:** __________

**New value:** __________

**Result:** ____________________________________________________

________________________________________________________________

---

## Challenge 2: Add a Safety Feature

**What safety feature would you add?**

________________________________________________________________

________________________________________________________________

**How would you implement it?**

________________________________________________________________

________________________________________________________________

---

# FINAL CHECKLIST

## System Functionality
- [ ] ROS 2 Humble installed successfully
- [ ] All three nodes can start without errors
- [ ] Camera publishes images at ~30 Hz
- [ ] Lane follower processes images and publishes commands
- [ ] Motor control receives commands and drives wheels
- [ ] Robot attempts to follow lanes

## ROS 2 Knowledge
- [ ] Can use `ros2 node list` to see running nodes
- [ ] Can use `ros2 topic list` to see active topics
- [ ] Can use `ros2 topic echo` to monitor messages
- [ ] Can use `ros2 topic hz` to check message rates
- [ ] Understand the three-node architecture
- [ ] Can explain what each node does

## Conceptual Understanding
- [ ] Understand what a ROS 2 node is
- [ ] Understand what a ROS 2 topic is
- [ ] Understand publish/subscribe communication
- [ ] Know why we use modular design
- [ ] Can connect concepts to Session 2 (TurtleSim)

---

# INSTRUCTOR SIGN-OFF

**Installation completed successfully:** ☐ Yes  ☐ No

**All nodes running correctly:** ☐ Yes  ☐ No

**Robot demonstrates lane-following behavior:** ☐ Yes  ☐ No

**Student understands ROS concepts:** ☐ Yes  ☐ No

**Activity sheet completed thoroughly:** ☐ Yes  ☐ No

**Comments:**

________________________________________________________________

________________________________________________________________

________________________________________________________________

**Instructor Signature:** ________________________  **Date:** __________

---

**Session 5 Completed!** 

**Time finished:** __________

**What did you learn today?**

________________________________________________________________

________________________________________________________________

________________________________________________________________

**What was the most challenging part?**

________________________________________________________________

________________________________________________________________

**What would you like to explore further?**

________________________________________________________________

________________________________________________________________