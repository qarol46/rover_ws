import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    package_name = 'rover_description'

    # Запуск rsp.launch.py для публикации robot_description

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    
    #joystick = IncludeLaunchDescription(
    #            PythonLaunchDescriptionSource([os.path.join(
    #                get_package_share_directory(package_name),'launch','joystick.launch.py'
    #            )]), launch_arguments={'use_sim_time': 'true'}.items()
    #)

    #Lead to velodyne queque overload - filter dropping message
    #ekf_config = os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')
    #robot_localization_node = Node(
    #    package='robot_localization',
    #    executable='ekf_node',
    #    name='ekf_filter_node',
    #    output='screen',
    #    parameters=[ekf_config, {'use_sim_time': True}],
    #    remappings=[('odometry/filtered', 'odom')]  # Перенаправляем выходной топик
    #)


    # Пути к файлам запуска

    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'description.rviz')
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
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )
    
    # Запуск Gazebo
    gazebo_world_file = os.path.join(get_package_share_directory(package_name),'worlds','gas_station.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'world': gazebo_world_file}.items()
    )
   
    slam_config_file = os.path.join(get_package_share_directory('rover_navigation'), 'config', 'slam.yaml')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
            launch_arguments={'params_file': slam_config_file}.items()
    )
    nav2_config_file = os.path.join(get_package_share_directory('rover_navigation'), 'config', 'navigation_sim.yaml')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rover_navigation'), 'launch', 'navigation.launch.py')]),
            launch_arguments={'params_file': nav2_config_file}.items()
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
                #'target_frame': 
                #'transform_tolerance': 0.01,
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

    # Спавн робота в Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Запуск Controller Manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(
                get_package_share_directory(package_name),
                "config", "my_controllers.yaml"
            ),
           {"use_sim_time": True}  # Использование симуляционного времени
        ],
        output="screen",
    )

    # Загрузка и запуск контроллера для публикации состояний суставов
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Загрузка и запуск контроллера дифференциального привода
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    # Задержка запуска контроллеров после спавна робота
    delay_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[diff_drive_spawner],
        )
    )

    delay_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[joint_broad_spawner],
       )
    )

    return LaunchDescription([
        rsp,
        gazebo,
        control_node,
        delay_diff_drive_spawner,
        delay_joint_broad_spawner,
        spawn_entity,
        #robot_localization_node,
        #start_rviz_cmd,
        #joystick,
        twist_mux,
        translate,
        slam,
        nav2
    ])