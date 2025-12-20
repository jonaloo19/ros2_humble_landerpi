#!/usr/bin/env python3
"""
多颜色识别节点 - 同时检测多种颜色并返回详细信息
修复字符编码问题，添加RGB值和距离显示
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
                # ===== 修复：将中文颜色名转换为英文大写 =====
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
                # ============================================
                
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
                    
                    if area >= min_area:
                        # 计算边界框
                        x, y, w, h = cv2.boundingRect(contour)
                        center_x = x + w // 2
                        center_y = y + h // 2
                        
                        # 计算边界框的宽高比
                        aspect_ratio = w / h if h > 0 else 0
                        
                        # 估算距离：基于物体大小和面积的简单估算
                        # 假设标准物体大小（单位：像素）与距离成反比
                        standard_area_at_1m = 10000  # 在1米处的标准面积
                        estimated_distance = standard_area_at_1m / area if area > 0 else 0
                        
                        # 格式化RGB值
                        border_rgb = (bgr_color[2], bgr_color[1], bgr_color[0])  # BGR转RGB
                        
                        # 存储检测信息
                        detection = {
                            'color_id': color_id,
                            'color_name': color_name,
                            'rgb': border_rgb,  # 添加RGB值
                            'x': int(x),
                            'y': int(y),
                            'width': int(w),
                            'height': int(h),
                            'center_x': int(center_x),
                            'center_y': int(center_y),
                            'area': float(area),
                            'aspect_ratio': float(aspect_ratio),
                            'estimated_distance_m': float(estimated_distance)  # 估算距离（米）
                        }
                        all_detections.append(detection)
                        
                        # 在图像上绘制
                        # 1. 绘制边界框
                        thickness = 3
                        cv2.rectangle(debug_image, (x, y), (x + w, y + h), bgr_color, thickness)
                        
                        # 2. 绘制中心点
                        cv2.circle(debug_image, (center_x, center_y), 5, bgr_color, -1)
                        
                        # 3. 添加标签（显示颜色名称、RGB值和面积）
                        # ===== 修复：确保使用大写英文 =====
                        display_name = color_name.upper() if isinstance(color_name, str) else str(color_name).upper()
                        label1 = f"{display_name}"
                        label2 = f"RGB: {border_rgb}"
                        label3 = f"Area: {int(area)}px"
                        
                        font_scale = 0.5  # 稍微调小字体
                        font_thickness = 1
                        line_spacing = 5
                        
                        # 计算三行文本的大小
                        (text_width1, text_height1), _ = cv2.getTextSize(
                            label1, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness
                        )
                        (text_width2, text_height2), _ = cv2.getTextSize(
                            label2, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness
                        )
                        (text_width3, text_height3), _ = cv2.getTextSize(
                            label3, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness
                        )
                        
                        # 找到最宽的文本
                        max_width = max(text_width1, text_width2, text_width3)
                        total_height = text_height1 + text_height2 + text_height3 + 2 * line_spacing
                        
                        # 标签背景（上移一点，为三行文本留空间）
                        bg_y_start = y - total_height - 10
                        bg_y_end = y
                        bg_x_end = x + max_width + 15
                        
                        # 确保背景不会超出图像顶部
                        if bg_y_start < 0:
                            bg_y_start = 0
                            bg_y_end = total_height + 10
                        
                        cv2.rectangle(
                            debug_image,
                            (x, bg_y_start),
                            (bg_x_end, bg_y_end),
                            bgr_color,
                            -1
                        )
                        
                        # 绘制三行文字
                        cv2.putText(
                            debug_image,
                            label1,
                            (x + 5, bg_y_start + text_height1 + 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            font_scale,
                            (255, 255, 255),  # 白色文字
                            font_thickness
                        )
                        
                        cv2.putText(
                            debug_image,
                            label2,
                            (x + 5, bg_y_start + text_height1 + text_height2 + line_spacing + 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            font_scale,
                            (255, 255, 255),
                            font_thickness
                        )
                        
                        cv2.putText(
                            debug_image,
                            label3,
                            (x + 5, bg_y_start + text_height1 + text_height2 + text_height3 + 2*line_spacing + 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            font_scale,
                            (255, 255, 255),
                            font_thickness
                        )
                        
                        # 4. 显示中心坐标和估算距离
                        # 那个(,)显示的是中心点坐标 (x, y)
                        distance_text = f"Dist: ~{estimated_distance:.1f}m"
                        coord_text = f"({center_x}, {center_y})"
                        
                        # 显示坐标
                        cv2.putText(
                            debug_image,
                            coord_text,
                            (center_x - 25, center_y + 15),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.4,
                            (255, 255, 255),  # 白色
                            1,
                            cv2.LINE_AA
                        )
                        
                        # 显示估算距离（在坐标下方）
                        cv2.putText(
                            debug_image,
                            distance_text,
                            (center_x - 25, center_y + 35),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.4,
                            (200, 200, 0),  # 黄色
                            1,
                            cv2.LINE_AA
                        )
        
        # 在图像顶部添加统计信息
        if all_detections:
            total_detections = len(all_detections)
            # 按颜色分组统计
            color_counts = {}
            for detection in all_detections:
                color_name = detection['color_name']
                color_counts[color_name] = color_counts.get(color_name, 0) + 1
            
            # 创建统计字符串
            count_parts = [f"{color}: {count}" for color, count in color_counts.items()]
            stats_detail = ", ".join(count_parts)
            stat_text = f"Objects: {total_detections}"
            
            # 第一行：总数量
            cv2.putText(
                debug_image,
                stat_text,
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),  # 白色外框
                3
            )
            cv2.putText(
                debug_image,
                stat_text,
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 0),  # 黑色文字
                2
            )
            
            # 第二行：详细统计（如果有多行）
            if stats_detail:
                cv2.putText(
                    debug_image,
                    stats_detail,
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),  # 白色外框
                    3
                )
                cv2.putText(
                    debug_image,
                    stats_detail,
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (50, 150, 255),  # 橙色文字
                    2
                )
        else:
            # 如果没有检测到物体，显示提示
            stat_text = "No objects detected"
            cv2.putText(
                debug_image,
                stat_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),  # 红色
                2
            )
        
        return debug_image, all_detections
    
    def publish_detection_info(self, detections):
        """发布检测信息"""
        # 1. 发布检测到的颜色名称
        detected_colors = set([d['color_name'] for d in detections])
        if detected_colors:
            color_names_msg = String()
            # 确保颜色名是英文大写
            english_colors = [color.upper() for color in detected_colors]
            color_names_msg.data = ", ".join(english_colors)
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
        # ===== 修复：使用英文输出 =====
        english_colors = [color.upper() for color in detected_colors]
        self.get_logger().info(
            f"Detected {len(detections)} objects: {', '.join(english_colors)}",
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
