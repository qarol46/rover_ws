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

from launch import LaunchDescription
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


package_name = 'rover_navigation'

def generate_launch_description():

    slam_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'slam.yaml')
    slam = Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    output='screen',
    parameters=[
        slam_config_file,
        {
            'use_sim_time': LaunchConfiguration("sim"),

        }
    ],
    remappings=[('scan', '/scan')]
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
                'use_sim_time': LaunchConfiguration("sim"),  # Explicitly set
                'allow_undeclared_parameters': False,
                #'target_frame': 'velodyne',
                #'transform_tolerance': 0.01,
                'min_height': -0.5,  # Lowered to detect ground obstacles
                'max_height': 2.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.01745,  # ~1 degree resolution
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 25.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }]
            
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='true',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

        #VLP_driver,
        #VLP_pointcloud,
        #translate,

        slam
    ])
