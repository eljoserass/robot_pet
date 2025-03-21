#!/usr/bin/env python3
"""
Debug tool for the multi-view navigation system.
This script subscribes to all relevant topics and provides real-time monitoring
of the robot's state, decisions, and communications.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time
import os
import json
import numpy as np
import threading
import datetime

class NavigationDebugger(Node):
    def __init__(self):
        super().__init__('navigation_debugger')
        
        # Initialize variables
        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_cmd_vel = None
        self.latest_servo_cmd = None
        self.cmd_vel_time = None
        self.servo_cmd_time = None
        self.last_image_time = None
        
        # Create log directory
        self.log_dir = os.path.join(os.getcwd(), 'debug_logs')
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Create log file
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = os.path.join(self.log_dir, f'navigation_debug_{timestamp}.log')
        self.image_count = 0
        
        # Create subscribers
        self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )
        
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.create_subscription(
            String,
            'servo_command',
            self.servo_cmd_callback,
            10
        )
        
        # Create display timer
        self.display_timer = self.create_timer(0.5, self.display_status)
        
        # Create log timer
        self.log_timer = self.create_timer(1.0, self.log_status)
        
        # For saving images
        self.save_images = True
        self.debug_window = True
        
        # Initialize OpenCV window if enabled
        if self.debug_window:
            cv2.namedWindow('Camera Debug', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Camera Debug', 640, 480)
        
        self.log_message("Debugger started")
        
    def log_message(self, message):
        """Write a message to the log file with timestamp."""
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        with open(self.log_file, 'a') as f:
            f.write(f"[{timestamp}] {message}\n")
        print(f"[{timestamp}] {message}")
    
    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_image_time = time.time()
            
            # Save images periodically
            if self.save_images and self.image_count % 30 == 0:  # Save every 30th image
                img_path = os.path.join(self.log_dir, f'frame_{self.image_count}.jpg')
                cv2.imwrite(img_path, self.latest_frame)
                self.log_message(f"Saved image to {img_path}")
            
            self.image_count += 1
            
            # Update debug window if enabled
            if self.debug_window and self.latest_frame is not None:
                # Add overlay with current robot state
                debug_img = self.latest_frame.copy()
                
                # Add text overlays
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.6
                color = (0, 255, 0)  # Green
                thickness = 2
                
                # Add cmd_vel info
                if self.latest_cmd_vel is not None:
                    cmd_text = f"cmd_vel: linear={self.latest_cmd_vel.linear.x:.2f}, angular={self.latest_cmd_vel.angular.z:.2f}"
                    cv2.putText(debug_img, cmd_text, (10, 30), font, font_scale, color, thickness)
                
                # Add servo info
                if self.latest_servo_cmd is not None:
                    servo_text = f"servo: {self.latest_servo_cmd}"
                    cv2.putText(debug_img, servo_text, (10, 60), font, font_scale, color, thickness)
                
                # Display the image
                cv2.imshow('Camera Debug', debug_img)
                cv2.waitKey(1)
                
        except Exception as e:
            self.log_message(f"Error processing image: {e}")
    
    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg
        self.cmd_vel_time = time.time()
        
        # Log significant movement commands
        if abs(msg.linear.x) > 0.1 or abs(msg.angular.z) > 0.1:
            self.log_message(f"Movement: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")
    
    def servo_cmd_callback(self, msg):
        self.latest_servo_cmd = msg.data
        self.servo_cmd_time = time.time()
        self.log_message(f"Servo command: {msg.data}")
    
    def display_status(self):
        """Display current status information."""
        now = time.time()
        
        # Check if we're receiving camera frames
        if self.last_image_time is None:
            camera_status = "❌ NO CAMERA FEED"
        elif now - self.last_image_time > 1.0:
            camera_status = f"⚠️ Camera delay: {now - self.last_image_time:.1f}s"
        else:
            camera_status = "✅ Camera OK"
        
        # Check movement commands
        if self.cmd_vel_time is None:
            movement_status = "❓ No movement commands yet"
        elif now - self.cmd_vel_time > 5.0:
            movement_status = f"⚠️ No movement for {now - self.cmd_vel_time:.1f}s"
        else:
            if self.latest_cmd_vel is not None:
                if abs(self.latest_cmd_vel.linear.x) < 0.01 and abs(self.latest_cmd_vel.angular.z) < 0.01:
                    movement_status = "🛑 Robot stopped"
                else:
                    movement_status = "🚀 Robot moving"
            else:
                movement_status = "❓ Unknown movement state"
        
        # Check servo commands
        if self.servo_cmd_time is None:
            servo_status = "❓ No servo commands yet"
        elif now - self.servo_cmd_time > 10.0:
            servo_status = f"⚠️ No servo movement for {now - self.servo_cmd_time:.1f}s"
        else:
            servo_status = f"✅ Servo position: {self.latest_servo_cmd}"
        
        # Print status update to console
        os.system('clear')  # Clear terminal
        print("=" * 50)
        print("ROBOT NAVIGATION DEBUGGER")
        print("=" * 50)
        print(f"Camera: {camera_status}")
        print(f"Movement: {movement_status}")
        print(f"Servo: {servo_status}")
        print("-" * 50)
        
        if self.latest_cmd_vel is not None:
            print(f"Linear: x={self.latest_cmd_vel.linear.x:.3f}, y={self.latest_cmd_vel.linear.y:.3f}, z={self.latest_cmd_vel.linear.z:.3f}")
            print(f"Angular: x={self.latest_cmd_vel.angular.x:.3f}, y={self.latest_cmd_vel.angular.y:.3f}, z={self.latest_cmd_vel.angular.z:.3f}")
        print("-" * 50)
        print(f"Log file: {self.log_file}")
        print("=" * 50)
    
    def log_status(self):
        """Log current status to file."""
        if self.latest_cmd_vel is not None:
            self.log_message(
                f"STATUS - Linear: {self.latest_cmd_vel.linear.x:.3f}, "
                f"Angular: {self.latest_cmd_vel.angular.z:.3f}, "
                f"Servo: {self.latest_servo_cmd}"
            )

def main(args=None):
    rclpy.init(args=args)
    debugger = NavigationDebugger()
    
    try:
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        pass
    finally:
        # Close OpenCV windows
        cv2.destroyAllWindows()
        debugger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()