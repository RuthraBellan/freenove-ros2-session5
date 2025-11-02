import cv2
import numpy as np

class Transform:
    def __init__(self, hflip=0, vflip=0):
        pass

class Picamera2:
    def __init__(self):
        print("✅ Mock Camera Initialized")
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("⚠️  No webcam, using test pattern")
            self.cap = None
    
    def create_preview_configuration(self, main=None, transform=None):
        return {}
    
    def configure(self, config):
        pass
    
    def start(self):
        pass
    
    def capture_array(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Test pattern with lane lines
        frame = np.ones((480, 640, 3), dtype=np.uint8) * 200
        cv2.line(frame, (200, 480), (250, 240), (0, 0, 0), 30)
        cv2.line(frame, (440, 480), (390, 240), (0, 0, 0), 30)
        return frame
    
    def stop(self):
        pass
    
    def close(self):
        if self.cap:
            self.cap.release()