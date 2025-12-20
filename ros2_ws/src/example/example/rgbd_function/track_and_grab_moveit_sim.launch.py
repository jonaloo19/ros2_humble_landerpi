#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # === 通用参数 ===
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name   = LaunchConfiguration('world_name',   default='robocup_home')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='robocup_home',
        description='World name for robot_gazebo/worlds.launch.py'
    )

    # === 1) Gazebo + 小车 + 深度相机（robot_gazebo） ===
    robot_gazebo_share = get_package_share_directory('robot_gazebo')
    worlds_launch = os.path.join(robot_gazebo_share, 'launch', 'worlds.launch.py')

    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(worlds_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_name':   world_name,
            'nav':          'false',       # 不启导航，只做抓取
        }.items()
    )

    # === 2) MoveIt 的 move_group（hiwonder_moveit_config） ===
    moveit_share = get_package_share_directory('hiwonder_moveit_config')
    move_group_launch = os.path.join(moveit_share, 'launch', 'move_group.launch.py')

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch)
    )

    # === 3) 仿真版 track_and_grab 节点 ===
    # 在这里把 Gazebo 的深度相机话题 remap 成 ascamera 风格，
    # 方便直接复用你原来的代码结构。
    grab_node = Node(
        package='example',
        executable='track_and_grab_moveit',      # 待会在 setup.py 里注册
        name='track_and_grab_moveit',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}],
        #remappings=[
        #    ('/ascamera/camera_publisher/depth0/image_raw',
        #     '/depth_cam/depth_cam'),
        #    ('/ascamera/camera_publisher/depth0/camera_info',
        #     '/depth_cam/rgb/camera_info'),
        #]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world_name,
        gazebo_world,
        move_group,
        grab_node,
    ])

