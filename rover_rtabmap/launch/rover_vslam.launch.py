
# Similar to gazebo example on https://github.com/chvmp/champ/tree/ros2, we can do:
#
#   Run the Gazebo environment:
#     $ ros2 launch champ_config gazebo.launch.py 
#
#   Run Nav2's navigation and rtabmap:
#     $ ros2 launch rtabmap_demos champ_vslam.launch.py use_sim_time:=true rviz:=true rtabmap_viz:=true
#
#   When a map is already created using command above, we can re-launch in localization-only mode with:
#     $ ros2 launch rtabmap_demos champ_vslam.launch.py use_sim_time:=true rviz:=true rtabmap_viz:=true localization:=true
#
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    
    localization = LaunchConfiguration('localization')
    
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_ekf = LaunchConfiguration("ekf")

    package_name='rover_rtabmap'
    
    # With the simulator, the imu is not published fast enough 
    # and have a huge delay, disabling imu usage from VO
    use_imu = use_sim_time.perform(context) in ["false", "False"]

    rtabmap_params = os.path.join(get_package_share_directory('rover_rtabmap'),
        'config',
        'rtabmap.yaml'
    )

    vslam_remappings=[
            ('scan', '/scan'),  
            ('imu', '/imu/data/filtered')
    ]
    ekf_config = os.path.join(get_package_share_directory(package_name), 'config', 'ekf_rtabmap_config.yaml')

    return [
        
        # compute imu orientation
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False
            }],
            remappings=[
                ('imu/data_raw', 'imu/data/raw'),    # Input (what the node subscribes to)
                ('imu/data', 'imu/data/filtered')     # Output (what the node publishes)
            ]
        ),
        
        # VSLAM nodes:
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[rtabmap_params],
            remappings=[('rgb/image', '/camera/image_raw'),
                        ('rgb/camera_info', '/camera/camera_info'),
                        ('depth/image', '/camera/depth/image_raw')]),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
                parameters=[rtabmap_params, {
                    'odom_frame_id': 'vo',  # Publish to vo frame
                    'publish_tf_odom': False  # Disable TF publishing
                }],
            remappings=[('odom', '/vo')],
            arguments=["--ros-args", "--log-level", 'info'],
            ),

        # SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_params],
            remappings=vslam_remappings,
            #arguments=['--delete_db_on_start'],
            arguments=['-d']
        ),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_params, 
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=vslam_remappings
        ),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[rtabmap_params],
            remappings=vslam_remappings
        ),
        
        # Compute ground/obstacle clouds for nav2 voxel layers
        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[{'decimation': 2,
                         'max_depth': 3.0,
                         'voxel_size': 0.02}],
            remappings=[('depth/image', '/camera/depth/image_raw'),
                        ('depth/camera_info', '/camera/depth/camera_info'),
                        ('cloud', '/camera/cloud')]
            ),
        
        Node(
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[rtabmap_params],
            remappings=[('cloud', '/camera/cloud'),
                        ('obstacles', '/camera/obstacles'),
                        ('ground', '/camera/ground')]
            ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': LaunchConfiguration("use_sim_time")}],
            remappings=[('odom', 'odom')],
            condition = IfCondition(LaunchConfiguration("ekf")),
        ),
    ]

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),
        
        DeclareLaunchArgument(
            name='rtabmap_viz', 
            default_value='false',
            description='Run rtabmap_viz'
        ),

        DeclareLaunchArgument(
            'localization', default_value='false', choices=['true', 'false'],
            description='Launch rtabmap in localization mode (a map should have been already created).'),

        DeclareLaunchArgument(
            name = 'ekf',
            default_value = 'true',
            description = 'Use robot localisation package to use multiple odometry sources'
        ),
        
        
        OpaqueFunction(function=launch_setup)
    ])