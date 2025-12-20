from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="deptrum-ros-driver-aurora930",
            executable="aurora930_node",
            namespace="aurora",
            parameters=[
                {"rgb_enable": LaunchConfiguration('rgb_enable', default=True),
                 "ir_enable": LaunchConfiguration('ir_enable', default=True),
                 "depth_enable": LaunchConfiguration('depth_enable', default=True),
                 "rgbd_enable": LaunchConfiguration('rgbd_enable', default=True),
                 "point_cloud_enable": LaunchConfiguration('point_cloud_enable', default=True),
                 "boot_order": LaunchConfiguration('boot_order', default=1),
                 "ir_fps": LaunchConfiguration('ir_fps', default=15),
                 "rgb_fps": LaunchConfiguration('rgb_fps', default=15),
                 "exposure_enable": LaunchConfiguration('exposure_enable', default=True),
                 "exposure_time": LaunchConfiguration('exposure_time', default=10),
                 "gain_enable": LaunchConfiguration('gain_enable', default=True),
                 "gain_value": LaunchConfiguration('gain_value', default=10),
                 "usb_port_number": LaunchConfiguration('usb_port_number', default=""),
                 "threshold_size": LaunchConfiguration('threshold_size', default=30),
                 "depth_correction": LaunchConfiguration('depth_correction', default=True),
                 "align_mode": LaunchConfiguration('align_mode', default=True),
                 "laser_power": LaunchConfiguration('laser_power', default=3.0),
                 "minimum_filter_depth_value": LaunchConfiguration('minimum_filter_depth_value', default=150),
                 "maximum_filter_depth_value": LaunchConfiguration('maximum_filter_depth_value', default=4000),
                 "resolution_mode_index": LaunchConfiguration('resolution_mode_index', default=2),
                 "log_dir": LaunchConfiguration('log_dir', default="/tmp/"),
                 "stream_sdk_log_enable": LaunchConfiguration('stream_sdk_log_enable', default=False),
                 "heart_enable": LaunchConfiguration('heart_enable', default=False),
                 "update_file_path": LaunchConfiguration('update_file_path', default=""),
                 }
            ],
            remappings=[
                ("/aurora/depth/image_raw", "/ascamera/camera_publisher/depth0/image_raw"),
                ("/aurora/ir/camera_info", "/ascamera/camera_publisher/depth0/camera_info"),
                ("/aurora/ir/image_raw", "/ascamera/camera_publisher/ir0/image"),
                ("/aurora/points2", "/ascamera/camera_publisher/depth0/points"),
                ("/aurora/rgb/camera_info", "/ascamera/camera_publisher/rgb0/camera_info"),
                ("/aurora/rgb/image_raw", "/ascamera/camera_publisher/rgb0/image"),
            ],
            arguments=None,
            output="screen",
        ),
        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '-1.57', '0', '-1.57', 'depth_cam_link', 'depth_camera_link']
        ),
        

        
        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '-1.57', '0', '-1.57', 'depth_cam', 'depth_camera_link_1']
        ),
        
       	Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '-1.57', '0', '-1.57', 'depth_cam', 'rgb_camera_link_1']
        ),
        
    ])
