# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os, yaml
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


MAP_NAME='my_map' #change to the name of your own map here
package_name = 'rover_navigation'
def generate_launch_description():


    # Initializing LIDAR - set here for debugging cause there is no nedd to drive robot

    # Firstly point config file and start driver 
    lidar_parameters_file= os.path.join(get_package_share_directory("real_rover"), 'config', 'VLP16-velodyne_driver_node-params.yaml')
    VLP_driver= Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        output='both',
        parameters=[lidar_parameters_file]
    )
    # Secondly start trasform node using original launch file of veodyne project
    convert_share_dir = get_package_share_path('velodyne_pointcloud')
    convert_params_file = os.path.join(get_package_share_directory('velodyne_pointcloud'), 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(str(convert_params_file), 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = str(convert_share_dir / 'params' / 'VLP16db.yaml')
    VLP_pointcloud = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        output='both',
        parameters=[convert_params]
    )

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'rviz', 'rover_navigation.rviz']
    )

    # default_map_path = PathJoinSubstitution(
    #     [FindPackageShare(package_name), 'maps', f'{MAP_NAME}.yaml']
    # )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'navigation.yaml']
    )

    nav2_sim_config_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'navigation_sim.yaml']
    )

    slam_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'slam.yaml')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
            launch_arguments={'use_sim_time': LaunchConfiguration("sim"), 'params_file': slam_config_file}.items()
    )

    translate = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('/cloud_in', '/velodyne_points'),  # Input pointcloud
                ('/scan', '/scan') # Output laserscan
            ],
            parameters=[{
                # CRITICAL FIX: Override QoS to match RViz2 requirements
                'qos_overrides./scan.publisher.reliability': 'reliable',  # Force RELIABLE
                'qos_overrides./scan.publisher.durability': 'volatile',
                'qos_overrides./scan.publisher.history': 'keep_last',
                'qos_overrides./scan.publisher.depth': 10,
                'use_sim_time': False,  # Explicitly set
                'allow_undeclared_parameters': False,
                #'target_frame': 'laserscan',
                'transform_tolerance': 0.01,
                'min_height': -0.5,  # Lowered to detect ground obstacles
                'max_height': 2.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.01745,  # ~1 degree resolution
                'scan_time': 0.01,
                'range_min': 0.9,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }]
            
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

    #    DeclareLaunchArgument(
    #         name='map', 
    #         default_value=default_map_path,
    #         description='Navigation map path'
    #     ),
        VLP_driver,
        VLP_pointcloud,
        translate,

        slam,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            condition=UnlessCondition(LaunchConfiguration("sim")),
            launch_arguments={
                #'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            condition=IfCondition(LaunchConfiguration("sim")),
            launch_arguments={
                #'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_sim_config_path
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        )
    ])