# Camera Setup Recovery Guide
## For Students Who Skipped Camera Steps

---

## Situation

In the last class, some students encountered camera issues and were instructed to **skip the camera steps** (Steps 14a, 14b, and 18) and continue with the rest of the installation.

**Good news:** We've identified and fixed the camera issues! This guide will help you add the working camera setup to your existing installation.

---

## What Was the Problem?

### The Camera We Have:
**Microdia Innomaker-U20CAM-720P** - A USB webcam

### The Issue:
The installation script was written for **CSI cameras** (Raspberry Pi Camera Module with ribbon cable), but we're using **USB cameras**. They need different initialization code!

| Camera Type | CSI Camera | USB Camera (Ours) |
|-------------|------------|-------------------|
| **Connection** | Ribbon cable to board | USB port |
| **Library** | Picamera2 | OpenCV + V4L2 |
| **Config needed** | `start_x=1` in config.txt | None |
| **Format setting** | Automatic | Must set MJPEG explicitly |
| **Buffer handling** | Automatic | Must flush manually |

### What Went Wrong:

**Old camera_node.py tried to use Picamera2:**
```python
from picamera2 import Picamera2  # ← For CSI cameras only!
camera = Picamera2()
camera.start()
frame = camera.capture_array()  # ← Doesn't work with USB!
```

**Result:** Camera couldn't be opened, or opened but couldn't capture frames.

---

## The 3-Line Fix

The fix involves **3 critical changes** to how we initialize the camera:

### Change #1: Use V4L2 Backend
```python
# OLD (wrong):
cap = cv2.VideoCapture(0)

# NEW (correct):
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # ← Specify V4L2!
```
**Why:** Your USB camera uses the V4L2 (Video4Linux2) driver. Without specifying this, OpenCV might use the wrong backend.

### Change #2: Set MJPEG Format FIRST
```python
# Must set format BEFORE resolution!
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
```
**Why:** Your camera supports multiple formats (MJPEG and YUYV). MJPEG is compressed and more reliable at 30 FPS. 

**⚠️ CRITICAL:** Without setting MJPEG explicitly:
- Camera picks format randomly (MJPEG or YUYV)
- Some USB ports default to MJPEG → works by luck ✓
- Other USB ports default to YUYV → fails ✗

**Real Example from Your Class:**
Even when some groups tried USB camera scripts (with V4L2 but without MJPEG line), results were inconsistent:
- Group A: Camera worked! (Got lucky - defaulted to MJPEG)
- Group B: Camera failed! (Unlucky - defaulted to YUYV)
- **Same code, different results!**

**The Fix:** Explicitly setting MJPEG → always works consistently ✓
No more luck-based behavior!

### Change #3: Flush Camera Buffers
```python
# Discard first 10 frames (they're garbage)
for i in range(10):
    cap.read()
    time.sleep(0.05)
```
**Why:** USB cameras fill internal buffers with "warm-up" frames when first opened. These are often blank, corrupted, or have wrong exposure. By reading and discarding them, we ensure the next frames are valid.

### Visual Comparison:

**Before (Failed):**
```
Open Camera → Try Read → Get Garbage → ❌ ERROR!
```

**After (Works):**
```
Open Camera → Set V4L2 → Set MJPEG → Flush Buffer → Read → ✅ SUCCESS!
```

### Complete System Architecture:

```
┌─────────────────────────────────────────────┐
│  USB Camera (Microdia Innomaker)            │
│  - Device: /dev/video0                      │
│  - Format: MJPEG @ 30 FPS                   │
│  - Driver: uvcvideo (V4L2)                  │
└──────────────────┬──────────────────────────┘
                   ↓
         [FIX #1: V4L2 Backend]
         [FIX #2: Set MJPEG Format]
         [FIX #3: Flush Buffers]
                   ↓
┌─────────────────────────────────────────────┐
│  camera_node.py (FIXED for USB)             │
│  - cv2.VideoCapture(0, cv2.CAP_V4L2)       │
│  - Captures at 30 FPS                       │
│  - Publishes BGR images                     │
└──────────────────┬──────────────────────────┘
                   ↓
    /freenove/camera/image_raw (ROS Topic)
                   ↓
┌─────────────────────────────────────────────┐
│  lane_follower_node.py (NO CHANGES!)        │
│  - Receives images                          │
│  - Detects lanes                            │
│  - Publishes velocity commands              │
└──────────────────┬──────────────────────────┘
                   ↓
    /freenove/cmd_vel (ROS Topic)
                   ↓
┌─────────────────────────────────────────────┐
│  motor_control_node.py (NO CHANGES!)        │
│  - Receives velocity commands               │
│  - Controls robot motors                    │
└─────────────────────────────────────────────┘
```

**Key Insight:** Only the camera initialization changed. Everything else is identical!

---

## What We'll Do Today

✅ Add the fixed USB camera test script  
✅ Install required packages (if missing)  
✅ Update your camera node to USB version  
✅ Test the camera works  
✅ Verify your complete system  

**Time needed:** 20-30 minutes

---

## Prerequisites Check

Before starting, verify you have:

```bash
# 1. Check ROS workspace exists
ls ~/ros2_ws/src/freenove_car/freenove_car/

# 2. Check if camera node file exists
ls ~/ros2_ws/src/freenove_car/freenove_car/camera_node.py

# 3. Check other nodes work
ros2 run freenove_car lane_follower_node --help
# Should show help text (press Ctrl+C to exit)
```

**If any of these fail, raise your hand!** ✋

---

## Step 1: Install Missing Packages (5 minutes)

Even if you skipped camera steps, let's make sure all packages are installed:

```bash
# Install V4L2 tools
sudo apt install -y v4l-utils

# Install ROS camera packages (if not already installed)
sudo apt install -y ros-humble-v4l2-camera ros-humble-image-transport-plugins

# Install OpenCV for Python
sudo apt install -y python3-opencv

# Verify OpenCV installation
python3 -c "import cv2; print('OpenCV version:', cv2.__version__)"
```

**Expected output:** `OpenCV version: 4.x.x`

**Checkpoint:** ✅ All packages installed successfully

---

## Step 2: Add User to Video Group (2 minutes)

This gives you permission to access the camera:

```bash
# Add user to video group
sudo usermod -aG video $USER

# Verify (you should see "video" in the list)
groups $USER | grep video
```

**Important:** Changes take effect after logout/login or reboot.

**For now, we'll continue and test. If camera doesn't work, we'll logout and back in.**

---

## Step 3: Create Working Camera Test Script (3 minutes)

Create the fixed camera test script:

```bash
cd ~
nano test_camera_fixed.py
```

**Copy and paste this entire script:**

```python
#!/usr/bin/env python3
"""
Fixed USB Camera Test
Tests camera with proper V4L2 + MJPEG initialization
"""

import cv2
import time

print("="*60)
print("USB CAMERA TEST - V4L2 + MJPEG")
print("="*60)

# Open with V4L2 backend
print("\nOpening /dev/video0...")
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not cap.isOpened():
    print("❌ Failed to open camera")
    print("\nTroubleshooting:")
    print("  1. Check camera plugged in: ls -l /dev/video*")
    print("  2. Logout and login (for video group)")
    print("  3. Try different USB port")
    exit(1)

print("✅ Camera opened")

# CRITICAL: Set MJPEG format FIRST
print("\nSetting MJPEG format...")
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
actual_fps = cap.get(cv2.CAP_PROP_FPS)

print(f"Resolution: {actual_w}x{actual_h}")
print(f"FPS: {actual_fps}")

# CRITICAL: Flush buffers
print("\nFlushing camera buffers...")
for i in range(5):
    ret, frame = cap.read()
    if ret:
        print(f"  Buffer {i+1}/5: ✓")
    time.sleep(0.1)

# Test capture
print("\nCapturing 10 test frames...")
success = 0

for i in range(10):
    ret, frame = cap.read()
    
    if ret and frame is not None and frame.size > 0:
        print(f"  Frame {i+1}/10: ✓ ({frame.shape[1]}x{frame.shape[0]})")
        success += 1
        
        if i == 0:
            filename = f"camera_test_{int(time.time())}.jpg"
            cv2.imwrite(filename, frame)
            print(f"  📸 Saved: {filename}")
    else:
        print(f"  Frame {i+1}/10: ✗ Failed")
    
    time.sleep(0.05)

cap.release()

# Results
print("\n" + "="*60)
print(f"Success rate: {success}/10")

if success >= 8:
    print("✅ CAMERA WORKING PERFECTLY!")
    print("\nYour camera is ready for ROS!")
elif success >= 3:
    print("⚠️  CAMERA WORKING BUT UNSTABLE")
    print("\nCamera captures frames but may have issues.")
else:
    print("❌ CAMERA NOT WORKING")
    print("\nCamera cannot capture frames reliably.")
    print("Raise your hand for help!")

print("="*60)
```

**Save:** Press `Ctrl+X`, then `Y`, then `Enter`

Make it executable:

```bash
chmod +x test_camera_fixed.py
```

**Checkpoint:** ✅ Test script created

---

## Step 4: Test Your Camera (3 minutes)

Run the test:

```bash
python3 test_camera_fixed.py
```

### Expected Output (Good):

```
============================================================
USB CAMERA TEST - V4L2 + MJPEG
============================================================

Opening /dev/video0...
✅ Camera opened

Setting MJPEG format...
Resolution: 640x480
FPS: 30.0

Flushing camera buffers...
  Buffer 1/5: ✓
  Buffer 2/5: ✓
  Buffer 3/5: ✓
  Buffer 4/5: ✓
  Buffer 5/5: ✓

Capturing 10 test frames...
  Frame 1/10: ✓ (640x480)
  📸 Saved: camera_test_xxxxx.jpg
  Frame 2/10: ✓ (640x480)
  Frame 3/10: ✓ (640x480)
  ...
  Frame 10/10: ✓ (640x480)

============================================================
Success rate: 10/10
✅ CAMERA WORKING PERFECTLY!

Your camera is ready for ROS!
============================================================
```

### If Test Fails:

**Problem: "Failed to open camera"**

```bash
# Check camera device exists
ls -l /dev/video*

# If exists but can't open, logout and login
exit
# SSH back in
ssh pi@robot
```

**Problem: "0/10 or low success rate"**

```bash
# Try different USB port, then run test again
python3 test_camera_fixed.py
```

**If still failing:** Raise your hand! ✋

**Checkpoint:** ✅ Camera test shows 8/10 or better

---

## Step 5: Update Camera Node (5 minutes)

Now update your ROS camera node to use USB cameras:

```bash
# Go to your camera node location
cd ~/ros2_ws/src/freenove_car/freenove_car/

# Backup old version (just in case)
cp camera_node.py camera_node.py.backup

# Edit the camera node
nano camera_node.py
```

**Delete ALL the old content and replace with this:**

```python
#!/usr/bin/env python3
"""
Camera Node for USB Cameras
Compatible with V4L2 USB webcams
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera')
        
        self.publisher_ = self.create_publisher(Image, '/freenove/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Initialize USB camera
        try:
            self.get_logger().info('Initializing USB camera...')
            
            # Open with V4L2 backend
            self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
            
            if not self.camera.isOpened():
                raise RuntimeError("Failed to open camera")
            
            # CRITICAL: Set MJPEG format BEFORE resolution
            self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.camera.set(cv2.CAP_PROP_FPS, 30)
            
            actual_w = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
            
            self.get_logger().info('✓ USB camera initialized')
            self.get_logger().info(f'  Resolution: {actual_w}x{actual_h}')
            self.get_logger().info(f'  FPS: {actual_fps}')
            self.get_logger().info('  Format: MJPEG')
            
            # CRITICAL: Flush buffers
            self.get_logger().info('Flushing camera buffers...')
            for i in range(10):
                ret, frame = self.camera.read()
                if ret:
                    self.get_logger().info(f'  Buffer {i+1}/10: ✓')
                time.sleep(0.05)
            
            self.get_logger().info('✓ Camera ready!')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize camera: {e}')
            raise
        
        # Create timer (30 Hz)
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.frame_count = 0
        self.error_count = 0
        self.get_logger().info('Camera node started - publishing at 30 Hz')
    
    def timer_callback(self):
        try:
            ret, frame = self.camera.read()
            
            if not ret or frame is None or frame.size == 0:
                self.error_count += 1
                if self.error_count % 30 == 0:
                    self.get_logger().warn(f'Frame capture failed (errors: {self.error_count})')
                return
            
            if self.error_count > 0:
                self.get_logger().info('✓ Camera recovered')
                self.error_count = 0
            
            # Convert to ROS Image
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(ros_image)
            
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'Published {self.frame_count} frames',
                    throttle_duration_sec=1.0
                )
        
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
    
    def destroy_node(self):
        self.get_logger().info('Shutting down...')
        try:
            if hasattr(self, 'camera') and self.camera is not None:
                self.camera.release()
                self.get_logger().info('Camera released')
        except Exception as e:
            self.get_logger().warn(f'Error releasing camera: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        camera_node = CameraNode()
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Save:** Press `Ctrl+X`, then `Y`, then `Enter`

**Checkpoint:** ✅ Camera node updated

---

## Step 6: Rebuild Your Workspace (3 minutes)

Rebuild your ROS workspace with the new camera node:

```bash
cd ~/ros2_ws

# Clean build (ensures fresh compilation)
rm -rf build/ install/ log/

# Rebuild
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

**Expected output:**
```
Starting >>> freenove_car
Finished <<< freenove_car [X.XXs]

Summary: 1 package finished [X.XXs]
```

**Checkpoint:** ✅ Workspace rebuilt successfully

---

## Step 7: Test Camera Node with ROS (5 minutes)

Test that your camera node works with ROS:

### Terminal 1: Start Camera Node

```bash
source ~/ros2_ws/install/setup.bash
ros2 run freenove_car camera_node
```

**Expected output:**
```
[camera] Initializing USB camera...
[camera] ✓ USB camera initialized
[camera]   Resolution: 640x480
[camera]   FPS: 30.0
[camera]   Format: MJPEG
[camera] Flushing camera buffers...
[camera]   Buffer 1/10: ✓
[camera]   Buffer 2/10: ✓
...
[camera] ✓ Camera ready!
[camera] Camera node started - publishing at 30 Hz
[camera] Published 30 frames
[camera] Published 60 frames
...
```

### Terminal 2: Verify Images Being Published

Open a **new terminal:**

```bash
ssh pi@robot
source ~/ros2_ws/install/setup.bash

# Check publishing rate
ros2 topic hz /freenove/camera/image_raw
```

**Expected output:**
```
average rate: 29.xxx
  min: 0.030s max: 0.035s std dev: 0.00xxx s window: 30
```

**Should show ~30 Hz** ✅

Press `Ctrl+C` to stop.

**Checkpoint:** ✅ Camera node publishes images at ~30 Hz

---

## Step 8: Test Complete System (5 minutes)

Now test your complete autonomous system!

### Terminal 1: Camera Node (already running)
```bash
# Should still be running from Step 7
# You should see: "Published XXX frames"
```

### Terminal 2: Lane Follower
```bash
source ~/ros2_ws/install/setup.bash
ros2 run freenove_car lane_follower_node
```

**Expected output:**
```
[lane_follower] Lane follower node started
[lane_follower] Subscribing to: /freenove/camera/image_raw
[lane_follower] Publishing to: /freenove/cmd_vel
[lane_follower] Lane detected, steering_error: XX.XX, angular_vel: X.XXX
```

### Terminal 3: Motor Control
```bash
source ~/ros2_ws/install/setup.bash
sudo -E ros2 run freenove_car motor_control_node
```

**⚠️ SAFETY:** Make sure robot is on blocks or you're ready to catch it!

**Expected output:**
```
[motor_control] Motor control node started
[motor_control] Subscribing to: /freenove/cmd_vel
```

**The robot should now respond to the lane following!** 🤖

**Checkpoint:** ✅ All three nodes running, system operational

---

## Understanding What Changed (Technical Deep Dive)

### Your Camera Specifications:

From running `lsusb` and `v4l2-ctl`, we know your robot has:
- **Model:** Microdia Innomaker-U20CAM-720P
- **Connection:** USB (not CSI ribbon cable)
- **Formats:** MJPEG @ 30fps and YUYV @ 30fps
- **Driver:** uvcvideo (USB Video Class)

### The Code Comparison:

#### ❌ OLD camera_node.py (CSI Camera):
```python
from picamera2 import Picamera2

camera = Picamera2()
config = camera.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)
camera.configure(config)
camera.start()

# Capture frame
frame = camera.capture_array()  # Returns RGB
```

**Problems with USB camera:**
1. Picamera2 doesn't work with USB cameras
2. No V4L2 backend specified
3. No MJPEG format setting
4. No buffer flushing

#### ✅ NEW camera_node.py (USB Camera):
```python
import cv2
import time

# 1. Open with V4L2 backend
camera = cv2.VideoCapture(0, cv2.CAP_V4L2)

# 2. Set MJPEG format BEFORE resolution (critical!)
camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
camera.set(cv2.CAP_PROP_FPS, 30)

# 3. Flush buffers (discard first 10 frames)
for i in range(10):
    camera.read()
    time.sleep(0.05)

# 4. Now capture works!
ret, frame = camera.read()  # Returns BGR
```

**Why this works:**
1. ✅ OpenCV supports USB cameras via V4L2
2. ✅ MJPEG format explicitly set
3. ✅ Buffers flushed before real capture
4. ✅ Compatible with uvcvideo driver

### Why Order Matters:

**Wrong Order (Fails):**
```python
cap = cv2.VideoCapture(0)
cap.set(WIDTH, 640)
cap.set(HEIGHT, 480)
cap.set(FOURCC, 'MJPG')  # ← TOO LATE!
ret, frame = cap.read()  # ← Returns garbage
```

**Correct Order (Works):**
```python
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # ← Backend first
cap.set(FOURCC, 'MJPG')  # ← Format BEFORE resolution
cap.set(WIDTH, 640)       # ← Then resolution
cap.set(HEIGHT, 480)
for i in range(10): cap.read()  # ← Flush buffers
ret, frame = cap.read()  # ← Returns valid frame!
```

### Buffer Flushing Explained:

When a USB camera opens, it fills an internal buffer with frames. The first several frames are "warm-up" frames:

```
Frame 0: [Black/Empty] ← Exposure not adjusted
Frame 1: [Corrupted]   ← Sensor warming up
Frame 2: [Dark]        ← Auto-exposure adjusting
Frame 3: [Partial]     ← Buffer not full
Frame 4: [OK-ish]      ← Getting better
...
Frame 10: [Good!]      ← Finally stable ✓
```

By reading and discarding frames 0-9, we ensure frame 10+ are valid.

### What Didn't Change:

Everything else in your system stayed exactly the same:
- ✅ Lane follower node (still processes BGR images)
- ✅ Motor control node (still receives Twist messages)
- ✅ ROS topics (still uses /freenove/camera/image_raw)
- ✅ Message types (still sensor_msgs/Image)

**The fix was ONLY in the camera initialization!**

---

## Key Takeaway:

**3 lines of code made the difference:**
```python
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)                          # V4L2
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))  # MJPEG
for i in range(10): cap.read()                                    # Flush
```

These 3 lines are specifically for USB cameras. CSI cameras don't need them because Picamera2 handles everything automatically.

---

## Troubleshooting

Want to see what your robot sees? Add the web viewer:

```bash
# In Terminal 4 (new terminal)
cd ~
nano ros_camera_web_viewer.py
```

**Paste the web viewer code** (instructor will provide or see handout)

Then:
```bash
sudo pip3 install flask
python3 ~/ros_camera_web_viewer.py
```

**Open browser:** `http://YOUR_ROBOT_IP:5000`

You'll see live camera feed! 📹

---

## Troubleshooting

### Issue: Camera test works, but ROS node fails

**Solution:**
```bash
# Make sure workspace is sourced
source ~/ros2_ws/install/setup.bash

# Try rebuilding
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Issue: "Permission denied" on camera

**Solution:**
```bash
# Logout and login for group changes
exit
ssh pi@robot

# Or reboot
sudo reboot
```

### Issue: Lane follower says "No lane detected"

**Solution:**
- Check camera is pointing at lane
- Improve lighting
- Use web viewer to see what camera sees

---

## Summary Checklist

Mark each item as you complete it:

- [ ] Step 1: Installed packages
- [ ] Step 2: Added to video group
- [ ] Step 3: Created test script
- [ ] Step 4: Camera test passes (8/10+)
- [ ] Step 5: Updated camera_node.py
- [ ] Step 6: Rebuilt workspace
- [ ] Step 7: Camera node works with ROS (~30 Hz)
- [ ] Step 8: Complete system runs (all 3 nodes)
- [ ] Step 9: (Optional) Web viewer working

**When all checked:** ✅ You're ready to continue with the course!

---

## Quick Reference Commands

```bash
# Test camera hardware
python3 ~/test_camera_fixed.py

# Start camera node
ros2 run freenove_car camera_node

# Check image publishing
ros2 topic hz /freenove/camera/image_raw

# Start lane follower
ros2 run freenove_car lane_follower_node

# Start motor control
sudo -E ros2 run freenove_car motor_control_node

# View camera in browser (optional)
python3 ~/ros_camera_web_viewer.py
```

---

## What Changed?

**The Fix:** USB cameras need special initialization:
1. V4L2 backend
2. MJPEG format set FIRST
3. Camera buffers flushed at startup

**Your old camera node** → Tried to use Picamera2 (for CSI cameras)  
**Your new camera node** → Uses OpenCV with V4L2 (for USB cameras)

Everything else stays the same! ✅

---

## Questions?

If you encounter any issues:
1. Check the troubleshooting section
2. Verify each checkpoint was successful
3. Raise your hand for instructor help

**You've got this!** 💪

---

**Last Updated:** November 2025  
**Instructor:** Ruthra  
**Course:** Engineering Teamwork III
