#!/usr/bin/env python3
"""
简化版颜色识别节点
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class SimpleColorDetector(Node):
    def __init__(self):
        super().__init__('simple_color_detector')
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 订阅摄像头图像
        self.subscription = self.create_subscription(
            Image,
            '/depth_cam/depth_cam',
            self.image_callback,
            10
        )
        
        # 发布处理后的图像
        self.processed_pub = self.create_publisher(
            Image,
            '/color_detection/processed_image',
            10
        )
        
        # 发布检测结果
        self.result_pub = self.create_publisher(
            String,
            '/color_detection/results',
            10
        )
        
        self.get_logger().info('简化版颜色识别节点已启动')
    
    def image_callback(self, msg):
        try:
            # 转换ROS图像为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 转换为HSV颜色空间
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # 红色范围（简化版，只用一个范围）
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            
            # 创建红色掩码
            mask = cv2.inRange(hsv, lower_red, upper_red)
            
            # 找到轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 绘制结果
            result_image = cv_image.copy()
            red_count = 0
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # 只处理面积大于500的区域
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(result_image, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    cv2.putText(result_image, "Red", (x, y-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    red_count += 1
            
            # 发布处理后的图像
            processed_msg = self.bridge.cv2_to_imgmsg(result_image, "bgr8")
            processed_msg.header = msg.header
            self.processed_pub.publish(processed_msg)
            
            # 发布检测结果
            result_msg = String()
            if red_count > 0:
                result_msg.data = f"检测到 {red_count} 个红色区域"
            else:
                result_msg.data = "未检测到红色"
            self.result_pub.publish(result_msg)
            
            self.get_logger().info(f'处理完成，检测到 {red_count} 个红色区域')
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
