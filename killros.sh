#!/usr/bin/env bash

echo "🔪 Killing ROS 2, Gazebo, ros2_control, bridges..."

# Kill Gazebo / Ignition
pkill -f ign_gazebo
pkill -f ign
pkill -f gazebo

# Kill ros2_control + controllers
pkill -f controller_manager
pkill -f gz_ros2_control
pkill -f ign_ros2_control
pkill -f ros2_control

# Kill ROS-GZ bridges
pkill -f ros_gz_bridge
pkill -f parameter_bridge

# Kill common ROS 2 nodes
pkill -f robot_state_publisher
pkill -f static_transform_publisher
pkill -f joint_state_broadcaster
pkill -f joint_state_publisher
pkill -f spawner

# Nuclear fallback (comment out if you want gentler behaviour)
pkill -9 -f ign
pkill -9 -f gazebo

# Reset ROS daemon
ros2 daemon stop
sleep 1
ros2 daemon start

echo "✅ ROS / Gazebo reset complete"

