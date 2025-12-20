#!/usr/bin/env python3
"""
多颜色识别节点 - 同时检测多种颜色并返回详细信息
简化版本：只做颜色检测，移除深度功能
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import yaml
import os
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


class MultiColorDetector(Node):
    def __init__(self):
        super().__init__('multi_color_detector')
        
        # 加载颜色配置
        self.load_color_config()
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 订阅摄像头图像
        self.subscription = self.create_subscription(
            Image,
            '/depth_cam/depth_cam',
            self.image_callback,
            10
        )
        
        # 发布处理后的图像（带检测框）
        self.image_pub = self.create_publisher(
            Image,
            '/color_detection/debug_image',
            10
        )
        
        # 发布检测到的颜色名称
        self.color_names_pub = self.create_publisher(
            String,
            '/color_detection/color_names',
            10
        )
        
        # 发布详细的检测信息（JSON格式）
        self.detection_info_pub = self.create_publisher(
            String,
            '/color_detection/detection_info',
            10
        )
        
        # 发布颜色位置（数组：x, y, width, height, color_id）
        self.position_pub = self.create_publisher(
            Float32MultiArray,
            '/color_detection/positions',
            10
        )
        
        self.get_logger().info('Multi Color Detector Started')
        self.get_logger().info(f'Supported colors: {list(self.colors.keys())}')
        
        # 创建调试窗口（可选）
        self.show_debug_window = True
        self.debug_window_name = "Color Detection"
        if self.show_debug_window:
            cv2.namedWindow(self.debug_window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.debug_window_name, 800, 600)
    
    def load_color_config(self):
        """从配置文件加载颜色信息"""
        try:
            # 获取包路径
            pkg_dir = get_package_share_directory('color_detection')
            config_path = os.path.join(pkg_dir, 'config/colors.yaml')
            
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            colors_config = config.get('colors', {})
            self.detection_params = config.get('detection', {})
            
            # 预处理颜色配置
            self.colors = {}
            self.color_hsv_ranges = {}
            self.color_names = {}
            
            for color_id, color_info in colors_config.items():
                # 将中文颜色名转换为英文大写
                original_name = color_info.get('name', color_id)
                
                # 中文到英文的映射
                chinese_to_english = {
                    '红色': 'RED',
                    '绿色': 'GREEN', 
                    '蓝色': 'BLUE',
                    '黄色': 'YELLOW',
                    '红': 'RED',
                    '绿': 'GREEN',
                    '蓝': 'BLUE',
                    '黄': 'YELLOW'
                }
                
                # 如果是中文，转换为英文；否则保持原样并大写
                if original_name in chinese_to_english:
                    color_info['name'] = chinese_to_english[original_name]
                else:
                    color_info['name'] = original_name.upper()
                
                self.color_names[color_id] = color_info['name']
                bgr_color = color_info.get('bgr_color', [0, 0, 255])
                
                # 处理HSV范围
                hsv_ranges = []
                if 'lower1' in color_info and 'upper1' in color_info:
                    lower1 = np.array(color_info['lower1'], dtype=np.uint8)
                    upper1 = np.array(color_info['upper1'], dtype=np.uint8)
                    hsv_ranges.append((lower1, upper1))
                    
                    if 'lower2' in color_info and 'upper2' in color_info:
                        lower2 = np.array(color_info['lower2'], dtype=np.uint8)
                        upper2 = np.array(color_info['upper2'], dtype=np.uint8)
                        hsv_ranges.append((lower2, upper2))
                elif 'lower' in color_info and 'upper' in color_info:
                    lower = np.array(color_info['lower'], dtype=np.uint8)
                    upper = np.array(color_info['upper'], dtype=np.uint8)
                    hsv_ranges.append((lower, upper))
                
                self.colors[color_id] = {
                    'name': self.color_names[color_id],
                    'bgr_color': tuple(bgr_color),
                    'hsv_ranges': hsv_ranges
                }
            
            self.get_logger().info(f'Loaded {len(self.colors)} color configurations')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load color config: {e}')
            # 默认配置（红色、绿色、蓝色、黄色）
            self.colors = {
                'red': {
                    'name': 'RED',
                    'bgr_color': (0, 0, 255),
                    'hsv_ranges': [
                        (np.array([0, 100, 100]), np.array([10, 255, 255])),
                        (np.array([160, 100, 100]), np.array([180, 255, 255]))
                    ]
                },
                'green': {
                    'name': 'GREEN',
                    'bgr_color': (0, 255, 0),
                    'hsv_ranges': [
                        (np.array([40, 100, 100]), np.array([80, 255, 255]))
                    ]
                },
                'blue': {
                    'name': 'BLUE',
                    'bgr_color': (255, 0, 0),
                    'hsv_ranges': [
                        (np.array([100, 100, 100]), np.array([130, 255, 255]))
                    ]
                }
            }
            self.detection_params = {
                'min_area': 500,
                'max_area': 50000,
                'blur_size': 5,
                'show_mask': True
            }
    
    def image_callback(self, msg):
        """图像回调函数 - 处理每帧图像"""
        try:
            # 转换ROS图像为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 处理图像，检测所有颜色
            debug_image, detection_info = self.process_image(cv_image)
            
            # 发布处理后的图像
            if debug_image is not None:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                debug_msg.header = msg.header
                self.image_pub.publish(debug_msg)
            
            # 发布检测信息
            if detection_info:
                self.publish_detection_info(detection_info)
            
            # 显示调试窗口（可选）
            if self.show_debug_window:
                cv2.imshow(self.debug_window_name, debug_image)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')
    
    def process_image(self, image):
        """处理图像，检测所有颜色"""
        # 复制原始图像用于绘制
        debug_image = image.copy()
        
        # 转换为HSV颜色空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 模糊处理减少噪声
        blur_size = self.detection_params.get('blur_size', 5)
        if blur_size > 0:
            hsv = cv2.GaussianBlur(hsv, (blur_size, blur_size), 0)
        
        # 存储所有检测结果
        all_detections = []
        min_area = self.detection_params.get('min_area', 500)
        max_area = self.detection_params.get('max_area', 50000)
        
        # 对每种颜色进行检测
        for color_id, color_info in self.colors.items():
            color_name = color_info['name']
            bgr_color = color_info['bgr_color']
            hsv_ranges = color_info['hsv_ranges']
            
            # 合并多个HSV范围
            mask = None
            for lower, upper in hsv_ranges:
                range_mask = cv2.inRange(hsv, lower, upper)
                if mask is None:
                    mask = range_mask
                else:
                    mask = cv2.bitwise_or(mask, range_mask)
            
            if mask is not None:
                # 形态学操作去噪
                kernel = np.ones((5, 5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                
                # 找到轮廓
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                # 处理每个检测到的区域
                for i, contour in enumerate(contours):
                    area = cv2.contourArea(contour)
                    
                    if min_area <= area <= max_area:
                        # 计算边界框
                        x, y, w, h = cv2.boundingRect(contour)
                        center_x = x + w // 2
                        center_y = y + h // 2
                        
                        # 存储检测信息
                        detection = {
                            'color_id': color_id,
                            'color_name': color_name,
                            'x': int(x),
                            'y': int(y),
                            'width': int(w),
                            'height': int(h),
                            'center_x': int(center_x),
                            'center_y': int(center_y),
                            'area': float(area)
                        }
                        all_detections.append(detection)
                        
                        # 在图像上绘制
                        # 1. 绘制边界框
                        thickness = 3
                        cv2.rectangle(debug_image, (x, y), (x + w, y + h), bgr_color, thickness)
                        
                        # 2. 绘制中心点
                        cv2.circle(debug_image, (center_x, center_y), 5, bgr_color, -1)
                        
                        # 3. 添加标签
                        label = f"{color_name}: {int(area)}px"
                        font_scale = 0.7
                        font_thickness = 2
                        (text_width, text_height), baseline = cv2.getTextSize(
                            label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness
                        )
                        
                        # 标签背景
                        cv2.rectangle(
                            debug_image,
                            (x, y - text_height - 10),
                            (x + text_width + 10, y),
                            bgr_color,
                            -1
                        )
                        
                        # 标签文字
                        cv2.putText(
                            debug_image,
                            label,
                            (x + 5, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            font_scale,
                            (255, 255, 255),  # 白色文字
                            font_thickness
                        )
                        
                        # 4. 显示中心坐标
                        coord_text = f"({center_x}, {center_y})"
                        cv2.putText(
                            debug_image,
                            coord_text,
                            (center_x - 25, center_y + 20),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (255, 255, 255),  # 白色
                            1
                        )
        
        # 在图像顶部添加统计信息
        if all_detections:
            total_detections = len(all_detections)
            stat_text = f"Detected: {total_detections} objects"
            cv2.putText(
                debug_image,
                stat_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 255, 255),  # 白色外框
                3
            )
            cv2.putText(
                debug_image,
                stat_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 0),  # 黑色文字
                2
            )
        
        return debug_image, all_detections
    
    def publish_detection_info(self, detections):
        """发布检测信息"""
        # 1. 发布检测到的颜色名称
        detected_colors = set([d['color_name'] for d in detections])
        if detected_colors:
            color_names_msg = String()
            color_names_msg.data = ", ".join(detected_colors)
            self.color_names_pub.publish(color_names_msg)
        
        # 2. 发布详细的检测信息（JSON格式）
        import json
        info_msg = String()
        
        # 构建信息字典
        info_dict = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'total_detections': len(detections),
            'detections': detections,
            'summary': {
                color_name: len([d for d in detections if d['color_name'] == color_name])
                for color_name in set([d['color_name'] for d in detections])
            }
        }
        
        info_msg.data = json.dumps(info_dict, ensure_ascii=False)
        self.detection_info_pub.publish(info_msg)
        
        # 3. 发布位置信息（数组）
        position_msg = Float32MultiArray()
        for detection in detections:
            # 添加：x, y, width, height, color_id（用数字表示）
            color_id_map = {'red': 1, 'green': 2, 'blue': 3, 'yellow': 4}
            color_num = color_id_map.get(detection['color_id'], 0)
            
            position_msg.data.extend([
                float(detection['center_x']),
                float(detection['center_y']),
                float(detection['width']),
                float(detection['height']),
                float(color_num)
            ])
        
        if position_msg.data:
            self.position_pub.publish(position_msg)
        
        # 4. 在终端显示简要信息
        self.get_logger().info(
            f"Detected {len(detections)} objects: {', '.join(detected_colors)}",
            throttle_duration_sec=1.0  # 每秒最多显示一次
        )
    
    def __del__(self):
        """清理函数"""
        if self.show_debug_window:
            cv2.destroyWindow(self.debug_window_name)


def main(args=None):
    rclpy.init(args=args)
    node = MultiColorDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理
        if node.show_debug_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
