import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

package_name = 'rover_navigation'

def generate_launch_description():
   
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Run rviz'
    )
    
    nav2_params=os.path.join(get_package_share_directory(package_name), 'config', 'navigation_sim.yaml')
    
    # Nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        nav2_launch
    ])