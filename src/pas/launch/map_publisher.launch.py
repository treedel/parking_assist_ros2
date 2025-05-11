from launch import LaunchDescription
from launch.conditions import IfCondition
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
    package_name = 'pas'
    package_share = FindPackageShare(package=package_name).find(package_name)

    lifecycle_nodes = ['map_server', 'filter_mask_server', 'costmap_filter_info_server']

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_path = LaunchConfiguration('map_path')
    keepout_map_path = LaunchConfiguration('keepout_map_path')
    config_path = LaunchConfiguration('config_path')

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

    declare_keepout_map_path = DeclareLaunchArgument(
        name='keepout_map_path',
        default_value=os.path.join(package_share, 'maps', f'{keepout_mask}.yaml'),
        description='Location of keepout mask file for Keepout filter'
    )

    declare_config_path = DeclareLaunchArgument(
        name='config_path',
        default_value=os.path.join(package_share, 'config', 'map_publisher.yaml'),
        description='Location of "map_publisher.yaml" file for map publisher parameters'
    )

    param_substitutions = {
        'use_sim_time': use_sim_time,
    }

    configured_params = RewrittenYaml(
        source_file=config_path,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'yaml_filename': map_path}
        ],
    )

    keepout_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name="filter_mask_server",
        emulate_tty=True,
        parameters=[
            configured_params,
            {'yaml_filename': keepout_map_path},
        ],
        output='screen'
    )

    keepout_cmap_info_server = Node(
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

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map_path)
    ld.add_action(declare_keepout_map_path)
    ld.add_action(declare_config_path)

    ld.add_action(map_server)
    ld.add_action(keepout_map_server)
    ld.add_action(keepout_cmap_info_server)
    ld.add_action(keepout_nav2_lifecycle_manager)
    
    return ld