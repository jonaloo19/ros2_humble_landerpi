#!/usr/bin/env python3
"""
多颜色识别节点 - 修复版本
"""
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class MultiColorDetectorFixed(Node):
    def __init__(self):
        super().__init__('multi_color_detector_fixed')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/depth_cam/depth_cam',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(String, '/color_detection/colors_fixed', 10)
        self.get_logger().info('Multi Color Detector Fixed Started')
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 简单的占位处理
            result_msg = String()
            result_msg.data = "Fixed version: Ready"
            self.publisher.publish(result_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = MultiColorDetectorFixed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
