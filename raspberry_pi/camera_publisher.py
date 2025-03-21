#!/usr/bin/env python3
"""
Camera publisher node for the Raspberry Pi.
This script captures frames from the Raspberry Pi Camera and publishes them to a ROS2 topic.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Initialize camera
        self.get_logger().info('Initializing camera')
        self.camera = cv2.VideoCapture(0)  # Use 0 for first camera
        
        if not self.camera.isOpened():
            self.get_logger().error('Could not open camera!')
            raise RuntimeError('Failed to open camera')
            
        # Set camera properties (adjust as needed for your camera)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.camera.set(cv2.CAP_PROP_FPS, 15)
        
        # Initialize the CV bridge
        self.bridge = CvBridge()
        
        # Create the image publisher
        self.publisher = self.create_publisher(
            Image,
            '/image',
            10)
            
        # Create a timer to publish frames
        self.timer = self.create_timer(0.066, self.publish_frame)  # ~15 FPS
        
        self.get_logger().info('Camera publisher started')
        
    def publish_frame(self):
        """Capture and publish a frame from the camera."""
        ret, frame = self.camera.read()
        
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
            
        try:
            # Convert the OpenCV image to a ROS image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            
            # Publish the image
            self.publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')
    
    def __del__(self):
        """Clean up resources."""
        if hasattr(self, 'camera') and self.camera.isOpened():
            self.camera.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Camera publisher stopped by user')
    except Exception as e:
        node.get_logger().error(f'Error in camera publisher: {e}')
    finally:
        # Clean up
        if hasattr(node, 'camera') and node.camera.isOpened():
            node.camera.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()