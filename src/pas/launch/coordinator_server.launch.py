from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

import os
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'pas'
    package_share = FindPackageShare(package=package_name).find(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    map_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'map_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    coordinator_server = Node(
        package=package_name,
        executable="coordinator_server",
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    
    ld.add_action(map_publisher)
    ld.add_action(coordinator_server)

    return ld