#!/usr/bin/env python3
"""
完整系统启动文件 - 同时启动Gazebo、颜色检测和深度查看
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取各个包的路径
    robot_gazebo_path = get_package_share_directory('robot_gazebo')
    color_detection_path = get_package_share_directory('color_detection')
    
    # 配置文件路径
    color_config = os.path.join(color_detection_path, 'config', 'colors.yaml')
    rviz_config = os.path.join(color_detection_path, 'config', 'rviz', 'simulation_depth_view.rviz')
    
    # 1. Gazebo仿真启动
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(robot_gazebo_path, 'launch', 'worlds.launch.py')
        ]),
        launch_arguments={'moveit_unite': 'true'}.items()
    )
    
    # 2. 颜色检测节点（延迟10秒启动，等待Gazebo完全启动）
    color_detector_node = Node(
        package='color_detection',
        executable='multi_color_detector',
        name='multi_color_detector',
        output='screen',
        parameters=[color_config],
    )
    
    # 3. RViz2深度查看器（延迟15秒启动）
    rviz_params = {}
    if os.path.exists(rviz_config):
        rviz_params['arguments'] = ['-d', rviz_config]
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        **rviz_params
    )
    
    # 4. 图像查看器（延迟20秒启动，查看深度图像）
    depth_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='depth_view',
        arguments=['/depth_cam/depth_cam/depth'],
        output='screen',
    )
    
    return LaunchDescription([
        # 首先启动Gazebo
        gazebo_launch,
        
        # 延迟10秒后启动颜色检测
        TimerAction(
            period=10.0,
            actions=[color_detector_node]
        ),
        
        # 延迟15秒后启动RViz2
        TimerAction(
            period=15.0,
            actions=[rviz_node]
        ),
        
        # 延迟20秒后启动深度图像查看器
        TimerAction(
            period=20.0,
            actions=[depth_view_node]
        ),
    ])

