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
        # Максимальное расстояние от точки до плоскости, чтобы считаться частью земли (в метрах)
        # Меньшие значения делают фильтрацию более строгой
            'max_distance': 0.15,          # (по умолчанию: 0.15)
        
        # Минимальное количество точек, необходимых для определения плоскости земли
        # Если найдено меньше точек - весь облако считается препятствием
            'min_ground_points': 300,      # (по умолчанию: 500)
        
        # Использовать ли данные IMU для коррекции ориентации плоскости
        # Включение требует рабочей TF-трансформации от IMU
            'use_imu': False,              # (по умолчанию: False)
        
        # Целевая система координат для преобразования облака точек
        # Используется только если use_imu=True
            'target_frame': 'base_link',   # (по умолчанию: "base_link")
        
        # Количество соседних точек для анализа при удалении выбросов
        # Большие значения делают фильтрацию более плавной, но медленной
            'point_stack': 20,             # (по умолчанию: 50)
        
        # Стандартное отклонение для фильтрации выбросов
        # Меньшие значения удаляют больше точек (более агрессивная фильтрация)
            'dist_threshold': 0.9,         # (по умолчанию: 1.0)
        
        # Минимальная высота точки относительно робота, чтобы считаться препятствием (в метрах)
        # Отрицательные значения учитывают неровности ниже уровня робота
            'min_height': -0.3            # (по умолчанию: -0.3)
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