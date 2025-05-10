from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable, IncludeLaunchDescription, GroupAction

import os
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'prius_gazebo'
    package_share = FindPackageShare(package=package_name).find(package_name)

    use_ekf_odom = LaunchConfiguration('use_ekf_odom')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_path_rel = LaunchConfiguration('world_path_rel')
    rviz_config_path = LaunchConfiguration('rviz_config_path')

    declare_use_ekf_odom = DeclareLaunchArgument(
        name='use_ekf_odom',
        default_value='true',
        description='Whether to start ekf filter for odometry'
    )

    declare_rviz_config_path = DeclareLaunchArgument(
        name='rviz_config_path',
        default_value=os.path.join(package_share, 'config', 'rviz_config.rviz'),
        description='Location of RViz config file'
    )

    declare_use_rviz = DeclareLaunchArgument(
        name='use_rviz',
        default_value='false',
        description='Whether to start RViz'
    )

    declare_world_path_rel = DeclareLaunchArgument(
        name='world_path_rel',
        default_value='parking.sdf',
        description='Location of world file for gazebo (relative to "worlds" folder)'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(package_share, 'worlds')
    )

    world = PathJoinSubstitution([package_share, 'worlds', world_path_rel])
    package_prius_bringup = FindPackageShare(package='prius_bringup').find('prius_bringup')
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_prius_bringup, 'launch', 'description.launch.py')),
            launch_arguments={
                'use_rviz': use_rviz,
                'rviz_config_path': rviz_config_path,
                'use_jsp_gui': 'false',
                'use_sim_time': use_sim_time
        }.items()
    )

    package_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': ['-r ', world],
                'on_exit_shutdown': 'true',
                'use_sim_time': use_sim_time
        }.items()
    )

    bridge_param_file = os.path.join(package_share, 'config', 'bridge.yaml')
    main_bridge = Node(
        condition=UnlessCondition(use_ekf_odom),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_param_file}],
        remappings=[
            ('/prius_hybrid/odom', '/odom'),
            ('/prius_hybrid/tf', '/tf')
        ],
        output='screen'
    )

    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/prius_hybrid/front_camera/image"]
    )

    package_libbot_localization = FindPackageShare(package='prius_localization').find('prius_localization')
    ekf_gazebo_group = GroupAction(
        actions=[
            Node(
                condition=IfCondition(use_ekf_odom),
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[{'config_file': bridge_param_file}],
                output='screen'
            ),

            Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                parameters=[{
                    'gain': 0.01,
                    'use_sim_time': use_sim_time,
                    "use_mag": True,
                    "publish_tf": False,
                    "world_frame": "enu",
                    "fixed_frame": "odom"
                }],
                remappings=[
                    ('/imu/data_raw', '/prius_hybrid/imu_raw'),
                    ('/imu/mag', '/prius_hybrid/mag_raw'),
                    ('/imu/data', '/prius_hybrid/imu'),
                ]
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_libbot_localization, 'launch', 'localization.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'run_global_ekf': "false",
                }.items(),
            )
        ],
        condition=IfCondition(use_ekf_odom)
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_ekf_odom)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_rviz_config_path)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world_path_rel)

    ld.add_action(set_env_vars_resources)
    ld.add_action(robot_description)
    ld.add_action(gz_sim)
    ld.add_action(main_bridge)
    ld.add_action(image_bridge)
    ld.add_action(ekf_gazebo_group)

    return ld