import os, yaml
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    package_name = 'rover_description'
    
    # Запуск rsp.launch.py для публикации robot_description

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': LaunchConfiguration("use_sim_time"), 'use_ros2_control': 'true'}.items()
    )
    
    joystick = IncludeLaunchDescription(
               PythonLaunchDescriptionSource([os.path.join(
                   get_package_share_directory(package_name),'launch','joystick.launch.py'
               )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    ekf_config = os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': LaunchConfiguration("use_sim_time")}],
        remappings=[('odometry/filtered', 'odom')],
    )


    # Пути к файлам запуска

    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'description.rviz')
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}]
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': LaunchConfiguration("use_sim_time")}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )
    
    # Запуск Gazebo
    gazebo_world_file = os.path.join(get_package_share_directory(package_name),'worlds','gas_station.world')
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'world': gazebo_world_file, 'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
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
           {"use_sim_time": LaunchConfiguration("use_sim_time")}  # Использование симуляционного времени
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
        arguments=["diff_cont"]
    )

    odometry_fus_node = Node(
            package='sp_udp_communication',
            executable='odometry_fus_node',
            name='odometry_fus_node',        
            output='screen',            
    )

    # Initializing LIDAR - set here for debugging cause there is no nedd to drive robot

    # Firstly point config file and start driver 
    lidar_parameters_file= os.path.join(get_package_share_directory("real_rover"), 'config', 'VLP16-velodyne_driver_node-params.yaml')
    VLP_driver= Node(
        condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
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
        condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        output='both',
        parameters=[convert_params]
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
                'use_sim_time': LaunchConfiguration("use_sim_time"),  
                'allow_undeclared_parameters': False,
                #'target_frame': 'velodyne',
                #'transform_tolerance': 0.01,
                'min_height': -0.5,  # Lowered to detect ground obstacles
                'max_height': 2.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.01745,  # ~1 degree resolution
                'scan_time': 0.0333,
                'range_min': 0.1,
                'range_max': 25.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }]
            
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        rsp,
        gazebo,
        control_node,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        odometry_fus_node,
        #robot_localization_node,
        start_rviz_cmd,
        #joystick,
        twist_mux,
        VLP_driver,
        VLP_pointcloud,
        translate
    ])