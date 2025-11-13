# 📹 Add Camera Viewing to Your Robot

**For students using the interactive installation script**

This is an OPTIONAL add-on that lets you see what your robot sees in a web browser while it's running!

---

## What This Does

✅ View camera feed in browser during lane following  
✅ Works alongside all your ROS nodes  
✅ No changes to existing setup needed  
✅ See exactly what the robot sees in real-time  

---

## Quick Setup (5 minutes)

### Step 1: Create the Viewer Script

SSH to your robot:
```bash
ssh pi@robot
```

Create the file:
```bash
cd ~
nano ros_camera_web_viewer.py
```

**Paste this entire script:**

```python
#!/usr/bin/env python3
"""ROS Camera Web Viewer"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from flask import Flask, Response, render_template_string
import cv2, threading, numpy as np

app = Flask(__name__)

HTML = """
<!DOCTYPE html>
<html>
<head><title>Robot Camera</title>
<style>
body{background:#1a1a1a;color:white;text-align:center;padding:20px;font-family:Arial}
h1{color:#4CAF50}
#video-container{margin:20px auto;max-width:800px;border:3px solid #4CAF50;border-radius:10px}
img{width:100%;display:block}
.info{background:#2a2a2a;padding:15px;border-radius:5px;margin:20px auto;max-width:600px}
</style></head>
<body>
<h1>🤖 Robot Camera View</h1>
<div id="video-container"><img src="{{ url_for('video_feed') }}"></div>
<div class="info"><p style="color:#4CAF50;font-weight:bold">{{ status }}</p></div>
</body></html>
"""

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_web_viewer')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.subscription = self.create_subscription(Image, '/freenove/camera/image_raw', self.callback, 10)
        self.frame_count = 0
        self.get_logger().info('Web viewer started')
    
    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.frame_lock:
                self.latest_frame = cv_image.copy()
            self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
    
    def get_frame(self):
        with self.frame_lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

viewer = None

def generate():
    placeholder = np.zeros((480,640,3), dtype=np.uint8)
    cv2.putText(placeholder, "Waiting...", (200,240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    while True:
        frame = viewer.get_frame() if viewer else None
        if frame is None: frame = placeholder
        ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ret:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

@app.route('/')
def index():
    status = "✓ Live" if viewer and viewer.frame_count > 0 else "⚠️ Waiting"
    return render_template_string(HTML, status=status)

@app.route('/feed')
def feed():
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def main():
    global viewer
    import socket
    rclpy.init()
    viewer = CameraViewer()
    threading.Thread(target=lambda: rclpy.spin(viewer), daemon=True).start()
    ip = socket.gethostbyname(socket.gethostname())
    print(f"\n{'='*60}\n🎥 Open: http://{ip}:5000\n{'='*60}\n")
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Save:** Press `Ctrl+X`, then `Y`, then `Enter`

Make executable:
```bash
chmod +x ros_camera_web_viewer.py
```

---

### Step 2: Install Flask

```bash
sudo pip3 install flask
```

---

### Step 3: Get Your Robot's IP Address

```bash
hostname -I
```

**Note the first IP address** (e.g., `192.168.0.69`)

---

## How to Use It

### When Running Your Robot System:

You should already have these running:
- **Terminal 1:** `ros2 run freenove_car camera_node`
- **Terminal 2:** `ros2 run freenove_car lane_follower_node`  
- **Terminal 3:** `sudo -E ros2 run freenove_car motor_control_node`

### Now Add the Viewer:

**Terminal 4:** (new terminal)
```bash
ssh pi@robot
python3 ~/ros_camera_web_viewer.py
```

**Your Browser:** (on your laptop)
```
http://YOUR_ROBOT_IP:5000
```
Replace `YOUR_ROBOT_IP` with the IP from Step 3.

**Example:** `http://192.168.0.69:5000`

---

## What You'll See

🎥 **Live camera feed** showing:
- What the robot's camera sees
- The lane it's trying to follow
- Real-time updates as robot moves

---

## System Diagram

```
Your Setup Now:

Terminal 1  →  Camera Node        →  Captures images
Terminal 2  →  Lane Follower      →  Processes images  
Terminal 3  →  Motor Control      →  Drives robot
Terminal 4  →  Web Viewer         →  Shows you the feed
                                      ↓
Browser     →  http://robot:5000  →  YOU SEE WHAT ROBOT SEES!
```

---

## Benefits

✅ **Debug visually** - See if lane detection works  
✅ **Demo easily** - Show others what robot sees  
✅ **Understand behavior** - Know why robot turns/stops  
✅ **Multiple viewers** - Everyone can watch on their devices  

---

## Troubleshooting

### Problem: "No images yet"

**Solution:** Make sure Terminal 1 (camera_node) is running:
```bash
ros2 topic hz /freenove/camera/image_raw
```
Should show ~30 Hz

### Problem: Can't connect to browser

**Solution:** Check your laptop and robot are on same WiFi network

### Problem: "ModuleNotFoundError: flask"

**Solution:**
```bash
sudo pip3 install flask
```

---

## To Stop Viewing

Press `Ctrl+C` in Terminal 4

**Your robot keeps running!** Only the viewer stops.

---

## Summary

| What | Command |
|------|---------|
| Create script | `nano ~/ros_camera_web_viewer.py` |
| Install Flask | `sudo pip3 install flask` |
| Run viewer | `python3 ~/ros_camera_web_viewer.py` |
| View in browser | `http://YOUR_ROBOT_IP:5000` |

---

**This is completely optional** - your robot works without it. But it's really helpful for debugging and demos! 🚀

**Total time to set up:** 5 minutes  
**Once set up:** Use anytime with one command

---

**Questions?** Ask your instructor or refer to the full setup guide.
