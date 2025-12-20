import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Get package paths
    robot_gazebo_path = get_package_share_directory('robot_gazebo')
    
    # URDF/Xacro file path
    xacro_file = os.path.join(robot_gazebo_path, 'urdf', 'robot.gazebo.xacro')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Robot State Publisher - publishes robot_description and TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro ', xacro_file,
                ' sim_ign:=false'  # Not using Ignition for real robot
            ]),
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
    ])
