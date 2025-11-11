#!/usr/bin/env python3
"""
Camera Node for Freenove 4WD Smart Car - USB Camera Version
Compatible with USB webcams using V4L2 backend

Course: Engineering Teamwork III - AI and Autonomous Systems Lab
Session 5: ROS 2 Integration

This version uses OpenCV with proper V4L2 + MJPEG initialization
for USB cameras.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time


class CameraNode(Node):
    """
    ROS 2 Node that captures images from USB camera
    and publishes them to /freenove/camera/image_raw topic
    
    Uses V4L2 backend with MJPEG format for reliable capture
    """
    
    def __init__(self):
        super().__init__('camera')
        
        # Create publisher for camera images
        self.publisher_ = self.create_publisher(
            Image, 
            '/freenove/camera/image_raw', 
            10
        )
        
        # Initialize CV Bridge (converts between OpenCV and ROS images)
        self.bridge = CvBridge()
        
        # Initialize USB camera with proper settings
        try:
            self.get_logger().info('Initializing USB camera...')
            
            # Open camera with V4L2 backend
            self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
            
            if not self.camera.isOpened():
                raise RuntimeError("Failed to open camera at /dev/video0")
            
            # CRITICAL: Set MJPEG format BEFORE resolution
            # This ensures reliable frame capture
            self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
            
            # Set resolution
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            # Set FPS
            self.camera.set(cv2.CAP_PROP_FPS, 30)
            
            # Verify settings
            actual_w = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
            
            self.get_logger().info('✓ USB camera initialized successfully')
            self.get_logger().info(f'  Resolution: {actual_w}x{actual_h}')
            self.get_logger().info(f'  FPS: {actual_fps}')
            self.get_logger().info('  Format: MJPEG')
            
            # CRITICAL: Flush camera buffers
            # First few frames are often empty/corrupted
            self.get_logger().info('Flushing camera buffers...')
            for i in range(10):
                ret, frame = self.camera.read()
                if ret:
                    self.get_logger().info(f'  Buffer frame {i+1}/10: ✓')
                time.sleep(0.05)
            
            self.get_logger().info('✓ Camera ready!')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize camera: {e}')
            self.get_logger().error('Make sure:')
            self.get_logger().error('  1. USB camera is plugged in')
            self.get_logger().error('  2. /dev/video0 exists: ls -l /dev/video*')
            self.get_logger().error('  3. User is in video group: groups $USER')
            self.get_logger().error('  4. Camera works: python3 test_camera_fixed.py')
            raise
        
        # Create timer to capture and publish images at 30 Hz
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        
        self.frame_count = 0
        self.error_count = 0
        self.get_logger().info('Camera node started - publishing to /freenove/camera/image_raw at 30 Hz')
    
    def timer_callback(self):
        """
        Timer callback that captures and publishes camera frames
        Called every 1/30 second (30 Hz)
        """
        try:
            # Capture frame
            ret, frame = self.camera.read()
            
            if not ret or frame is None or frame.size == 0:
                self.error_count += 1
                if self.error_count % 30 == 0:  # Log every second
                    self.get_logger().warn(
                        f'Failed to capture frame (error count: {self.error_count})'
                    )
                return
            
            # Reset error count on success
            if self.error_count > 0:
                self.get_logger().info('✓ Camera recovered')
                self.error_count = 0
            
            # Frame is already in BGR format (OpenCV default)
            # Convert OpenCV image (numpy array) to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            
            # Publish the image
            self.publisher_.publish(ros_image)
            
            # Log progress (once per second = every 30 frames at 30 Hz)
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'Published {self.frame_count} frames',
                    throttle_duration_sec=1.0
                )
        
        except Exception as e:
            self.get_logger().error(f'Error capturing/publishing frame: {e}')
    
    def destroy_node(self):
        """
        Clean up camera resources when node is destroyed
        """
        self.get_logger().info('Shutting down camera node...')
        try:
            # Release camera properly
            if hasattr(self, 'camera') and self.camera is not None:
                self.camera.release()
                self.get_logger().info('Camera released successfully')
        except Exception as e:
            self.get_logger().warn(f'Error releasing camera: {e}')
        
        super().destroy_node()


def main(args=None):
    """
    Main function to initialize and run the camera node
    """
    rclpy.init(args=args)
    
    try:
        camera_node = CameraNode()
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        print('\nShutting down camera node...')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
