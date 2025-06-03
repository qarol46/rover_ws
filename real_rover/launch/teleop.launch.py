import os
import yaml
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    joy_params = os.path.join(get_package_share_directory("real_rover"), 'config', 'joy_params.yaml')
    teleop_params = os.path.join(get_package_share_directory("real_rover"), 'config', 'teleop_params.yaml')

    # Node for joystick
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joy_params]
    )

    # Node for teleoperation
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[teleop_params],
        remappings=[('/cmd_vel','/diff_cont/cmd_vel_unstamped')]
    )

    return LaunchDescription([
        joy_node,
        teleop_node
    ])

    
