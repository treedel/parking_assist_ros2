from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

import os
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'prius_localization'
    package_share = FindPackageShare(package=package_name).find(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time')
    run_local_ekf = LaunchConfiguration('run_local_ekf')
    run_global_ekf = LaunchConfiguration('run_global_ekf')

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_run_local_ekf = DeclareLaunchArgument(
        name='run_local_ekf',
        default_value='true',
        description='Publish odom using wheel and imu data'
    )

    declare_run_global_ekf = DeclareLaunchArgument(
        name='run_global_ekf',
        default_value='true',
        description='Publish map-odom tf using gps data'
    )

    ekf_param_file = os.path.join(package_share, 'config', 'ekf.yaml')
    ekf_localization_odom = Node(
        condition=IfCondition(run_local_ekf),
        package="robot_localization",
        executable="ekf_node",
        name='ekf_filter_node_odom',
        parameters=[
            ekf_param_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/odometry/filtered', '/prius_hybrid/odom/local')
        ]
    )

    ekf_localization_map = Node(\
        condition=IfCondition(run_global_ekf),
        package="robot_localization",
        executable="ekf_node",
        name='ekf_filter_node_map',
        parameters=[
            ekf_param_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/odometry/filtered', '/prius_hybrid/odom/global')
        ]
    )

    navsat_transform = Node(
        condition=IfCondition(run_global_ekf),
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        parameters=[
            ekf_param_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/imu', '/prius_hybrid/imu'),
            ('/odometry/filtered', '/prius_hybrid/odom/local'),
            ('/gps/fix', '/prius_hybrid/gps_raw'),
            ('/odometry/gps', '/prius_hybrid/odom/gps'),
            ('/gps/filtered', '/prius_hybrid/gps'),
        ]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_run_local_ekf)
    ld.add_action(declare_run_global_ekf)
    
    ld.add_action(ekf_localization_odom)
    ld.add_action(ekf_localization_map)
    ld.add_action(navsat_transform)

    return ld