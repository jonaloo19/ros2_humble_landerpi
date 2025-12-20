#!/usr/bin/env python3
"""
深度信息查看启动文件 - 用于Gazebo仿真相机
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的share目录路径
    color_detection_path = get_package_share_directory('color_detection')
    
    # RViz2配置文件路径
    rviz_config = os.path.join(color_detection_path, 'config', 'rviz', 'simulation_depth_view.rviz')
    
    # 检查配置文件是否存在
    if not os.path.exists(rviz_config):
        print(f"警告：RViz配置文件不存在: {rviz_config}")
        print("将使用默认的RViz配置")
        rviz_config = None
    
    # 创建RViz2节点的参数
    rviz_params = {}
    if rviz_config:
        rviz_params['arguments'] = ['-d', rviz_config]
    
    # RViz2节点 - 查看深度信息
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{
            'use_sim_time': True  # 使用仿真时间
        }],
        **rviz_params
    )
    
    # 可选：图像查看器节点
    image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='depth_image_view',
        arguments=['/depth_cam/depth_cam/depth'],  # 查看深度图像
        output='screen',
    )
    
    # 可选：点云查看器
    pointcloud_view_node = Node(
        package='rqt_topic',
        executable='rqt_topic',
        name='topic_monitor',
        output='screen',
    )
    
    return LaunchDescription([
        rviz_node,
        # 根据需要启用或禁用下面的节点
        # image_view_node,
        # pointcloud_view_node,
    ])
