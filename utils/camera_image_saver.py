#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import threading
from rclpy.executors import MultiThreadedExecutor

class CameraImageSaver(Node):
    def __init__(self):
        super().__init__('camera_image_saver')
        
        # Create output directory if it doesn't exist
        self.output_dir = '/home/smarthc/ros2_ws/src/utils/png'
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Keep track of which images we've received
        self.left_received = False
        self.right_received = False
        self.lock = threading.Lock()
        
        # Subscribe to both camera topics
        self.left_sub = self.create_subscription(
            Image,
            '/Camera_L',
            self.left_callback,
            10)
            
        self.right_sub = self.create_subscription(
            Image,
            '/Camera_R',
            self.right_callback,
            10)
            
        self.get_logger().info(f'Camera Image Saver node started. Images will be saved to {self.output_dir}')
    
    def left_callback(self, msg):
        if not self.left_received:
            with self.lock:
                if not self.left_received:  # Double-check under lock
                    try:
                        # Convert ROS Image to OpenCV image
                        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                        
                        # Save image to file
                        image_path = os.path.join(self.output_dir, 'Camera_L.png')
                        cv2.imwrite(image_path, cv_image)
                        
                        self.left_received = True
                        self.get_logger().info(f'Saved left camera image to {image_path}')
                        
                        # Check if we're done
                        self._check_if_done()
                        
                    except Exception as e:
                        self.get_logger().error(f'Error processing left image: {str(e)}')
    
    def right_callback(self, msg):
        if not self.right_received:
            with self.lock:
                if not self.right_received:  # Double-check under lock
                    try:
                        # Convert ROS Image to OpenCV image
                        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                        
                        # Save image to file
                        image_path = os.path.join(self.output_dir, 'Camera_R.png')
                        cv2.imwrite(image_path, cv_image)
                        
                        self.right_received = True
                        self.get_logger().info(f'Saved right camera image to {image_path}')
                        
                        # Check if we're done
                        self._check_if_done()
                        
                    except Exception as e:
                        self.get_logger().error(f'Error processing right image: {str(e)}')
    
    def _check_if_done(self):
        """Check if both images have been saved and shut down if done"""
        if self.left_received and self.right_received:
            self.get_logger().info('Both camera images have been saved, shutting down.')
            # Use a timer to properly shut down the node from a callback
            self.create_timer(1.0, self.shutdown)
    
    def shutdown(self):
        """Properly shut down the node"""
        self.get_logger().info('Shutting down camera_image_saver node')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    camera_image_saver = CameraImageSaver()
    
    # Use a MultiThreadedExecutor to handle callbacks for both subscriptions
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(camera_image_saver)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Explicit cleanup
        executor.remove_node(camera_image_saver)
        camera_image_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()