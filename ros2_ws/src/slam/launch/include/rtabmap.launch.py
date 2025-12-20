from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch import LaunchService
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')

    parameters = {
        'frame_id': 'base_footprint',
        'use_sim_time': use_sim_time,
        'subscribe_rgbd': True,
        'subscribe_scan': True,
        'use_action_for_goal': True,
        'qos_scan': qos,
        'qos_image': qos,
        'qos_imu': qos,
        'queue_size': 50,  
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'Grid/RangeMin': '0.2',  
        'Optimizer/GravitySigma': '0',  # Disable imu constraints (we are already in 2D)
        'Grid/Sensor': '0',  # 0=laser, 1=camera, 2=both
        'Grid/FromDepth': 'false',  # 使用激光雷达生成2D栅格
        'Grid/GroundIsObstacle': 'false',  # 地面不是障碍物 
        #'Grid/3D': 'true',  # 启用3D栅格地图
        'Grid/MaxObstacleHeight': '2.0',  # 最大障碍物高度
        
        # 'Vis/MinInliers': '10',  # 视觉特征最小内点数
        # 'Vis/MaxDepth': '4.0',  # 视觉特征最大深度
        # 'Icp/VoxelSize': '0.05',  # ICP体素大小
        'Grid/MaxGroundHeight': '0.1',  # 地面高度阈值                                                                                                                                  │ │

        'Grid/NormalK': '10',  # 法线估计的K近邻数                            
        'Grid/ClusterRadius': '0.1',  # 聚类半径，用于分组3D物体               
        'Grid/MinClusterSize': '20',  # 最小聚类大小                         
        'Grid/DepthDecimation': '4',  # 深度图降采样以提高性能                 
        'Grid/DepthRoiRatios': '0.0 0.0 0.0 0.0',  # 使用完整深度图           


        # 视觉特征增强                                                       
        'Vis/MinInliers': '15',  # 增加视觉特征最小内点数                                             
        'Vis/FeatureType': '6',  # 使用ORB特征                               
        'Vis/MaxFeatures': '400',  # 最大特征数                             

        
        # ICP配置优化                                                        
        'Icp/VoxelSize': '0.03',  # 减小ICP体素大小以提高精度                  
        'Icp/PointToPlane': 'true',  # 使用点到平面ICP                       
        'Icp/Iterations': '10',  # ICP迭代次数                               
        'Icp/MaxCorrespondenceDistance': '0.1',  # 最大对应距离              


        # RTAB-Map内存和处理优化                                              
        'Rtabmap/TimeThr': '700',  # 时间阈值                                
        'Rtabmap/DetectionRate': '1',  # 检测率                             
        'RGBD/AngularUpdate': '0.05',  # 角度更新阈值                        
        'RGBD/LinearUpdate': '0.05',  # 线性更新阈值                         
        'RGBD/ProximityBySpace': 'true',  # 基于空间的邻近检测                
        'RGBD/ProximityMaxGraphDepth': '50',  # 最大图深度                   
        'RGBD/ProximityPathMaxNeighbors': '10',  # 路径最大邻居数             


        # 点云滤波和优化                                                      
        'Cloud/MaxDepth': '5.0',  # 点云最大深度                             
        'Cloud/VoxelSize': '0.03',  # 点云体素大小（更精细）                   
        'Cloud/FloorHeight': '0.0',  # 地板高度                              
        'Cloud/CeilingHeight': '2.5',  # 天花板高度                          
        'Cloud/NoiseFilteringRadius': '0.05',  # 噪声滤波半径                
        'Cloud/NoiseFilteringMinNeighbors': '2'  # 噪声滤波最小邻居数      

        #'approx_sync_max_interval': 0.02,  
        #'queue_size_imu': 300,  
    }

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('rgb/image', '/ascamera/camera_publisher/rgb0/image'),
        ('rgb/camera_info', '/ascamera/camera_publisher/rgb0/camera_info'),
        ('depth/image', '/ascamera/camera_publisher/depth0/image_raw'),
        ('odom', '/odom'),
        ('scan','/scan_raw'),
    ]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),

        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync': True, 'approx_sync_max_interval': 0.008, 'use_sim_time': use_sim_time, 'qos': qos}],
            remappings=remappings),

        # SLAM Mode:
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),
    ])

if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()



