#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch file for LanderPi arm grasp action server
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('landerpi_arm'),
            'config',
            'grasp.yaml'
        ]),
        description='Path to grasp configuration YAML file'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )
    
    # Grasp action server node
    grasp_server_node = Node(
        package='landerpi_arm',
        executable='grasp_action_server',
        name='grasp_action_server',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        config_file_arg,
        log_level_arg,
        grasp_server_node,
    ])
