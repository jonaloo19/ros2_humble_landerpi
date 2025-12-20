#!/usr/bin/env python3
"""
仿真环境查看启动文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的share目录路径
    color_detection_path = get_package_share_directory('color_detection')
    
    # RViz2配置文件路径
    rviz_config = os.path.join(color_detection_path, 'config', 'rviz', 'gazebo_simulation.rviz')
    
    # 检查配置文件是否存在
    if not os.path.exists(rviz_config):
        print(f"警告：RViz配置文件不存在: {rviz_config}")
        # 使用默认配置
        rviz_config = ""
    
    # RViz2节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if rviz_config else [],
        parameters=[{'use_sim_time': True}],
    )
    
    # RQT图像查看器（彩色图像）
    color_image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='color_image_view',
        arguments=['/depth_cam/depth_cam'],
        output='screen',
    )
    
    # RQT图像查看器（检测结果）
    detection_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='detection_view',
        arguments=['/color_detection/debug_image'],
        output='screen',
    )
    
    # RQT工具查看所有话题
    topic_view = Node(
        package='rqt_topic',
        executable='rqt_topic',
        name='topic_view',
        output='screen',
    )
    
    return LaunchDescription([
        rviz_node,
        color_image_view,
        detection_view,
        topic_view,
    ])
