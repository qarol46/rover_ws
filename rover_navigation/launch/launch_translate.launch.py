import os
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    
    translate = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('/cloud_in', '/velodyne_points_filtered'),  # Input pointcloud
                ('/scan', '/scan') # Output laserscan
            ],
            parameters=[{
                # # CRITICAL FIX: Override QoS to match RViz2 requirements
                # 'qos_overrides./scan.publisher.reliability': 'reliable',  # Force RELIABLE
                # 'qos_overrides./scan.publisher.durability': 'volatile',
                # 'qos_overrides./scan.publisher.history': 'keep_last',
                # 'qos_overrides./scan.publisher.depth': 10,
                'use_sim_time': LaunchConfiguration('sim'),  # Explicitly set
                'allow_undeclared_parameters': False,
                #'target_frame': 'laserscan',
                #'transform_tolerance': 0.01,
                'min_height': -0.45,  # Lowered to detect ground obstacles
                'max_height': 2.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.01545,  # ~1 degree resolution
                'scan_time': 0.005,
                'range_min': 0.1,
                'range_max': 15.0,
                'use_inf': False,
                'inf_epsilon': 1.0
            }]
            
    )
    ground_filter = Node(
        package='rover_navigation',
        executable='ground_filter_node',
        name='ground_filter',
        parameters=[{
            'max_distance': 0.3,                            # Ignore distanse from floor
            'angular_threshold': 7.0,                       # Угол отклонения от вертикали (градусы)
            'min_ground_points': 500,                       
            'use_imu': True,                                # Use imu
            'target_frame':'root_link',                     # Robot targer frame
            'subscribe_topic': '/velodyne_points',          # Input pcl
            'publish_topic': '/velodyne_points_filtered',   # Output pcl
            'number_of_points': 20,                         # Mininal number of points before vanish
            'distance_threshold_calculation': 0.9,          # Min distance between points
            #'line_min_length': 1.5,                         # Min lenght of points along a line
            #'line_point_density': 150,                       
            #'buffer_size': 3,                               # Размер буфера для временной фильтрации
            #'min_point_age': 2,
            'min_height': -0.3

        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),
        translate,
        ground_filter,

    ])