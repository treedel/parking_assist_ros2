from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

map_name = "parking_updated"
keepout_mask = "keepout"

def generate_launch_description():
    package_name = 'prius_navigation'
    package_share = FindPackageShare(package=package_name).find(package_name)

    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    use_sim_time = LaunchConfiguration('use_sim_time')
    load_map_server = LaunchConfiguration('load_map_server')
    map_path = LaunchConfiguration('map_path')
    nav_config_path = LaunchConfiguration('nav_config_path')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_path = LaunchConfiguration('rviz_config_path')

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_load_map_server = DeclareLaunchArgument(
        name='load_map_server',
        default_value='true',
        description='Whether to load map servers'
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
            'use_sim_time': use_sim_time,
            'load_map_server': 'True',
        }.items(),
        condition=IfCondition(load_map_server)
    )

    navigation_nm = IncludeLaunchDescription(  
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'nav2b_bringup.launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'params_file': nav_config_path,
            'use_sim_time': use_sim_time,
            'load_map_server': 'False',
        }.items(),
        condition=UnlessCondition(load_map_server)
    )

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': os.path.join(package_share, 'maps', f'{keepout_mask}.yaml')
    }

    configured_params = RewrittenYaml(
        source_file=nav_config_path,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    keepout_map_server = Node(
        condition=IfCondition(load_map_server),
        package='nav2_map_server',
        executable='map_server',
        name="filter_mask_server",
        emulate_tty=True,
        parameters=[
            configured_params,
            {'use_sim_time': use_sim_time},
        ],
        output='screen'
    )

    keepout_cmap_info_server = Node(
        condition=IfCondition(load_map_server),
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name="costmap_filter_info_server",
        emulate_tty=True,
        parameters=[
            configured_params,
            {'use_sim_time': use_sim_time},
        ],
        output='screen'
    )

    keepout_nav2_lifecycle_manager = Node(
        condition=IfCondition(load_map_server),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name="lifecycle_manager_costmap_filters",
        emulate_tty=True,
        parameters=[
            configured_params,
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': lifecycle_nodes}
        ],
        output='screen'
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
    ld.add_action(declare_load_map_server)
    ld.add_action(declare_map_path)
    ld.add_action(declare_nav_config_path)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_rviz_config_path)

    ld.add_action(navigation)
    ld.add_action(navigation_nm)
    ld.add_action(keepout_map_server)
    ld.add_action(keepout_cmap_info_server)
    ld.add_action(keepout_nav2_lifecycle_manager)
    
    ld.add_action(rviz)

    return ld