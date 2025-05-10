from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

map_name = "parking"

def generate_launch_description():
    package_name = 'prius_navigation'
    package_share = FindPackageShare(package=package_name).find(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_path = LaunchConfiguration('map_path')
    nav_config_path = LaunchConfiguration('nav_config_path')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_path = LaunchConfiguration('rviz_config_path')

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map_path = DeclareLaunchArgument(
        name='map_path',
        default_value=os.path.join(package_share, 'maps', f'{map_name}.yaml'),
        description='Location of map file for AMCL localization and navigation'
    )

    declare_nav_config_path = DeclareLaunchArgument(
        name='nav_config_path',
        default_value=os.path.join(package_share, 'config', 'navigation.yaml'),
        description='Location of "navigation.yaml" file for navigation parameters'
    )

    declare_use_rviz = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_rviz_config_path = DeclareLaunchArgument(
        name='rviz_config_path',
        default_value=os.path.join(package_share, 'config', 'navigation_launch.rviz'),
        description='Location of RViz config file'
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'nav2b_bringup.launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'params_file': nav_config_path,
            'use_sim_time': use_sim_time
        }.items()
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map_path)
    ld.add_action(declare_nav_config_path)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_rviz_config_path)

    ld.add_action(navigation)
    ld.add_action(rviz)

    return ld