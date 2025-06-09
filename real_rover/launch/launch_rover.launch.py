import os
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'real_rover'

    # Запуск rsp.launch.py для публикации robot_description
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    robot_description = Command(['xacro ', os.path.join(get_package_share_directory(package_name), 'urdf', 'trk211.xacro')])
    
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','controllers.yaml')

    # Запуск Controller Manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file],
    )
    #joystick = IncludeLaunchDescription(
    #            PythonLaunchDescriptionSource([os.path.join(
    #                get_package_share_directory(package_name),'launch','joystick.launch.py'
    #            )]), launch_arguments={'use_sim_time': 'true'}.items()
    #)

    rviz_config_file = os.path.join(get_package_share_directory("rover_navigation"), 'rviz', 'rover_navigation.rviz')
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    delay_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # Загрузка и запуск контроллера дифференциального привода
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    # Загрузка и запуск контроллера для публикации состояний суставов
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Задержка запуска контроллеров после спавна робота
    delay_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    delay_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )
    
    # I prefer to use singe launch file for project, so
    # teleop_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('real_rover'), 'launch', 'teleop.launch.py')])   
    # )


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

    # This block might be used to also convert VLP raw data to /sensor_msg/laserscan but we use translate node for this purpose

    # laserscan_params_file = get_package_share_path('velodyne_laserscan') / 'config' / 'default-velodyne_laserscan_node-params.yaml'
    # ld.add_action(Node(
    #     package='velodyne_laserscan',
    #     executable='velodyne_laserscan_node',
    #     output='both',
    #     parameters=[laserscan_params_file]
    # ))

    # slam_config_file = os.path.join(get_package_share_directory('rover_navigation'), 'config', 'slam.yaml')
    # slam = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
    #         launch_arguments={'params_file': slam_config_file}.items()
    # )
    #nav2_config_file = os.path.join(get_package_share_directory('rover_navigation'), 'config', 'navigation.yaml')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rover_navigation'), 'launch', 'navigation.launch.py')])
            #launch_arguments={'params_file': nav2_config_file}.items()
    )
    
    # translate = Node(
    #         package='pointcloud_to_laserscan',
    #         executable='pointcloud_to_laserscan_node',
    #         name='pointcloud_to_laserscan',
    #         remappings=[
    #             ('/cloud_in', '/velodyne_points'),  # Input pointcloud
    #             ('/scan', '/scan') # Output laserscan
    #         ],
    #         parameters=[{
    #             # CRITICAL FIX: Override QoS to match RViz2 requirements
    #             'qos_overrides./scan.publisher.reliability': 'reliable',  # Force RELIABLE
    #             'qos_overrides./scan.publisher.durability': 'volatile',
    #             'qos_overrides./scan.publisher.history': 'keep_last',
    #             'qos_overrides./scan.publisher.depth': 10,
    #             'use_sim_time': False,  # Explicitly set
    #             'allow_undeclared_parameters': False,
    #             #'target_frame': 'laserscan',
    #             'transform_tolerance': 0.01,
    #             'min_height': -0.5,  # Lowered to detect ground obstacles
    #             'max_height': 2.0,
    #             'angle_min': -1.5708,  # -M_PI/2
    #             'angle_max': 1.5708,  # M_PI/2
    #             'angle_increment': 0.01745,  # ~1 degree resolution
    #             'scan_time': 0.01,
    #             'range_min': 0.9,
    #             'range_max': 30.0,
    #             'use_inf': True,
    #             'inf_epsilon': 1.0
    #         }]
            
    # )
    return LaunchDescription([
        rsp,
        start_rviz_cmd,
        delay_controller_manager,
        delay_diff_drive_spawner,
        delay_joint_broad_spawner,
        #joystick,
        twist_mux,
        #VLP_driver,
        #VLP_pointcloud,
        #translate,
        #slam,
        #nav2
    ])