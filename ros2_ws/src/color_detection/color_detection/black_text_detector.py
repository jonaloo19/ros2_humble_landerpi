#!/usr/bin/env python3
"""
黑色文字颜色识别器 - 彩色边框 + 黑色英文标签
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class BlackTextColorDetector(Node):
    def __init__(self):
        super().__init__('black_text_detector')
        
        # 颜色配置（英文，HSV范围）
        self.color_configs = {
            'RED': {
                'ranges': [
                    (np.array([0, 100, 100]), np.array([10, 255, 255])),
                    (np.array([160, 100, 100]), np.array([180, 255, 255]))
                ],
                'border_color': (0, 0, 255),  # 红色边框（BGR）
                'bg_color': (0, 0, 255)       # 红色标签背景
            },
            'GREEN': {
                'ranges': [
                    (np.array([40, 100, 100]), np.array([80, 255, 255]))
                ],
                'border_color': (0, 255, 0),  # 绿色边框
                'bg_color': (0, 255, 0)       # 绿色标签背景
            },
            'BLUE': {
                'ranges': [
                    (np.array([100, 100, 100]), np.array([130, 255, 255]))
                ],
                'border_color': (255, 0, 0),  # 蓝色边框（BGR中是红色）
                'bg_color': (255, 0, 0)       # 蓝色标签背景
            },
            'YELLOW': {
                'ranges': [
                    (np.array([20, 100, 100]), np.array([40, 255, 255]))
                ],
                'border_color': (0, 255, 255),  # 黄色边框
                'bg_color': (0, 255, 255)       # 黄色标签背景
            }
        }
        
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
        self.image_pub = self.create_publisher(
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
        
        # 发布详细检测信息
        self.info_pub = self.create_publisher(
            String,
            '/color_detection/detection_info',
            10
        )
        
        self.get_logger().info('黑色文字颜色识别器已启动')
        self.get_logger().info('边框: 彩色, 文字: 黑色, 标签: 英文')
        
        # 检测参数
        self.min_area = 500  # 最小检测面积（像素）
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 转换ROS图像为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 处理图像
            result_image, detections_info = self.process_image(cv_image)
            
            # 发布处理后的图像
            result_msg = self.bridge.cv2_to_imgmsg(result_image, "bgr8")
            result_msg.header = msg.header
            self.image_pub.publish(result_msg)
            
            # 发布检测结果
            if detections_info['detected_colors']:
                result_str = String()
                result_str.data = f"检测到: {detections_info['summary']}"
                self.result_pub.publish(result_str)
                
                # 发布详细检测信息
                info_str = String()
                info_str.data = f"检测到 {len(detections_info['detections'])} 个物体: {detections_info['summary']}"
                self.info_pub.publish(info_str)
                
                self.get_logger().info(f"检测到: {detections_info['summary']}")
            else:
                self.get_logger().info("未检测到颜色", throttle_duration_sec=2.0)
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')
    
    def process_image(self, image):
        """处理图像，检测颜色"""
        # 复制原始图像
        result_image = image.copy()
        
        # 转换为HSV颜色空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 模糊处理减少噪声
        hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
        
        # 存储所有检测结果
        all_detections = []
        detection_summary = {}
        
        # 检测每种颜色
        for color_name, config in self.color_configs.items():
            # 合并多个HSV范围
            mask = None
            for lower, upper in config['ranges']:
                range_mask = cv2.inRange(hsv, lower, upper)
                if mask is None:
                    mask = range_mask
                else:
                    mask = cv2.bitwise_or(mask, range_mask)
            
            if mask is not None:
                # 形态学操作去除噪声
                kernel = np.ones((5, 5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                
                # 找到轮廓
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                color_detections = []
                
                for contour in contours:
                    area = cv2.contourArea(contour)
                    
                    if area > self.min_area:
                        # 计算边界框
                        x, y, w, h = cv2.boundingRect(contour)
                        center_x = x + w // 2
                        center_y = y + h // 2
                        
                        # 存储检测信息
                        detection = {
                            'color': color_name,
                            'x': x,
                            'y': y,
                            'width': w,
                            'height': h,
                            'center_x': center_x,
                            'center_y': center_y,
                            'area': area
                        }
                        all_detections.append(detection)
                        color_detections.append(detection)
                        
                        # 边框颜色和背景颜色
                        border_color = config['border_color']
                        bg_color = config['bg_color']
                        
                        # 1. 绘制边框（3像素粗）
                        cv2.rectangle(result_image, (x, y), (x + w, y + h), border_color, 3)
                        
                        # 2. 绘制中心点
                        cv2.circle(result_image, (center_x, center_y), 5, border_color, -1)
                        
                        # 3. 绘制标签背景
                        label = f"{color_name} {int(area)}px"
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        font_scale = 0.6
                        thickness = 1
                        
                        # 计算文本大小
                        (text_width, text_height), baseline = cv2.getTextSize(
                            label, font, font_scale, thickness
                        )
                        
                        # 标签背景（彩色）
                        cv2.rectangle(
                            result_image,
                            (x, y - text_height - 10),
                            (x + text_width + 10, y),
                            bg_color,
                            -1  # 填充
                        )
                        
                        # 4. 绘制黑色文字（核心：黑色文字在彩色背景上）
                        cv2.putText(
                            result_image,
                            label,
                            (x + 5, y - 5),
                            font,
                            font_scale,
                            (0, 0, 0),  # 黑色文字
                            thickness,
                            cv2.LINE_AA
                        )
                        
                        # 5. 可选：显示中心坐标
                        coord_text = f"({center_x}, {center_y})"
                        cv2.putText(
                            result_image,
                            coord_text,
                            (center_x - 30, center_y + 20),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.4,
                            (255, 255, 255),  # 白色坐标文字
                            1,
                            cv2.LINE_AA
                        )
                
                # 统计当前颜色的检测数量
                if color_detections:
                    detection_summary[color_name] = len(color_detections)
        
        # 在图像顶部添加黑色文字统计信息
        if all_detections:
            # 创建统计字符串
            total_objects = len(all_detections)
            summary_parts = [f"{color}: {count}" for color, count in detection_summary.items()]
            summary_str = ", ".join(summary_parts)
            stats = f"Objects: {total_objects} ({summary_str})"
            
            # 计算统计文本大小
            (text_width, text_height), _ = cv2.getTextSize(
                stats, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2
            )
            
            # 统计信息背景（深灰色）
            cv2.rectangle(
                result_image,
                (10, 10),
                (text_width + 25, text_height + 25),
                (50, 50, 50),  # 深灰色背景
                -1
            )
            
            # 黑色统计文字
            cv2.putText(
                result_image,
                stats,
                (15, text_height + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 0),  # 黑色文字
                2,
                cv2.LINE_AA
            )
            
            # 添加边框使统计信息更突出
            cv2.rectangle(
                result_image,
                (10, 10),
                (text_width + 25, text_height + 25),
                (200, 200, 200),  # 浅灰色边框
                1
            )
        
        # 返回结果
        summary = ", ".join([f"{color}({count})" for color, count in detection_summary.items()]) if detection_summary else "无"
        
        detections_info = {
            'detections': all_detections,
            'detected_colors': list(detection_summary.keys()),
            'summary': summary
        }
        
        return result_image, detections_info


def main(args=None):
    rclpy.init(args=args)
    node = BlackTextColorDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
