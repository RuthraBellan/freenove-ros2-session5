# CAMERA NOT WORKING? - QUICK FIX

**For any student with camera issues, regardless of which script version you used**

---

## Step 1: Test Your Camera (2 minutes)

Run this test to see if your camera hardware works:

```bash
cd ~
cat > test_camera.py << 'EOF'
#!/usr/bin/env python3
import cv2, time

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
if not cap.isOpened():
    print("❌ Camera failed to open")
    exit(1)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

for i in range(5): cap.read()  # Flush buffers

success = sum([1 for i in range(10) if cap.read()[0]])
cap.release()

print(f"✅ WORKING! ({success}/10)" if success >= 8 else f"❌ FAILED ({success}/10)")
EOF

python3 test_camera.py
```

**Result:**
- ✅ Shows "WORKING" → Go to Step 2
- ❌ Shows "FAILED" → Run Step 1B below first

---

### Step 1B: Fix Hardware/Permissions (if test failed)

```bash
# Install packages
sudo apt install -y v4l-utils python3-opencv

# Add to video group
sudo usermod -aG video $USER

# Reboot (required!)
sudo reboot

# After reboot, run test_camera.py again
```

---

## Step 2: Fix Your ROS Camera Node (5 minutes)

Once test_camera.py works, update your ROS node:

```bash
# Go to your camera node
cd ~/ros2_ws/src/freenove_car/freenove_car/

# Backup old version
cp camera_node.py camera_node.py.old

# Create new USB camera version
cat > camera_node.py << 'EOF'
#!/usr/bin/env python3
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
        
        # Open camera with V4L2 + MJPEG
        self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.camera.isOpened():
            raise RuntimeError("Camera failed to open")
        
        # CRITICAL: Set MJPEG FIRST
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Flush buffers
        self.get_logger().info('Flushing camera buffers...')
        for i in range(10):
            self.camera.read()
            time.sleep(0.05)
        
        self.get_logger().info('✓ Camera ready!')
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.frame_count = 0
    
    def timer_callback(self):
        ret, frame = self.camera.read()
        if ret and frame is not None:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(ros_image)
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')
    
    def destroy_node(self):
        if hasattr(self, 'camera'):
            self.camera.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# Rebuild
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Step 3: Test ROS Node (1 minute)

```bash
# Terminal 1: Run camera node
ros2 run freenove_car camera_node

# Terminal 2: Check it's working
ros2 topic hz /freenove/camera/image_raw
```

**Expected output:**
```
[camera] Flushing camera buffers...
[camera] ✓ Camera ready!
[camera] Published 30 frames
[camera] Published 60 frames
...
```

---

## That's It!

**If Step 1 test works, Step 2 will work too.**

The key is:
1. V4L2 backend
2. MJPEG format set FIRST
3. Flush buffers at startup

---

## Still Not Working?

Share this information:
```bash
# Health check
ls -l /dev/video*
groups | grep video
v4l2-ctl --device=/dev/video0 --list-formats-ext | head -15
python3 -c "import cv2; print(cv2.__version__)"
```

---

**Save this file!** Print it or keep it handy during lab sessions.

**Time to fix: 10 minutes total** 
