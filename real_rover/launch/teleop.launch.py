import os
import yaml
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError as e: 
        print(f"Error reading file {file_path}: {e}")
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except (EnvironmentError, yaml.YAMLError) as e: 
        print(f"Error reading YAML file {file_path}: {e}")
        return None

def generate_launch_description():
    joy_params = get_package_file('real_rover', 'config/joy_params.yaml')
    teleop_params = get_package_file('real_rover', 'config/teleop_params.yaml')

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
