import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

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

    ekf_config = os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        #remappings=[('odometry/filtered', 'odom')]  # Перенаправляем выходной топик
    )

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
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
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
            target_action=spawn_entity,
            on_exit=[diff_drive_spawner],
        )
    )

    delay_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[joint_broad_spawner],
        )
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        robot_localization_node,
        start_rviz_cmd,
        control_node,
        delay_diff_drive_spawner,
        delay_joint_broad_spawner,
        #joystick,
        twist_mux,
    ])