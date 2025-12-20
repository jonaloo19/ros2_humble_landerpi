import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    compiled = os.environ['need_compile']
    start = LaunchConfiguration('start', default='true')
    conf = LaunchConfiguration('conf', default=0.45)
    conf_arg = DeclareLaunchArgument('conf', default_value=conf)
    start_arg = DeclareLaunchArgument('start', default_value=start)
    debug = LaunchConfiguration('debug', default='false')
    debug_arg = DeclareLaunchArgument('debug', default_value=debug)
    broadcast = LaunchConfiguration('broadcast', default='false')
    broadcast_arg = DeclareLaunchArgument('broadcast', default_value=broadcast)
    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
        controller_package_path = get_package_share_directory('controller')
        example_package_path = get_package_share_directory('example')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/controller'
        example_package_path = '/home/ubuntu/ros2_ws/src/example'

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
    )
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')),
    )


    yolov8_node = Node(
        package='yolov8_detect',
        executable='yolov8_node',
        output='screen',
        parameters=[
                    {'classes': ['BananaPeel','BrokenBones','CigaretteEnd','DisposableChopsticks','Ketchup','Marker','OralLiquidBottle','Plate','PlasticBottle','StorageBattery','Toothbrush', 'Umbrella']},
                    {'model': 'garbage_classification','conf': conf}
                    ]
        )
    garbage_classification_node = Node(
        package='example',
        executable='garbage_classification',
        output='screen',
        parameters=[os.path.join(example_package_path, 'config/garbage_classification_roi.yaml'), {'start': start}, {'debug': debug}, {'broadcast': broadcast}],
    )

    return [start_arg,
            debug_arg,
            conf_arg,
            broadcast_arg,
            depth_camera_launch,
            controller_launch,
            yolov8_node,
            garbage_classification_node,
            ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(Create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

