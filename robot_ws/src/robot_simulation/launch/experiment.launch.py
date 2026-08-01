#!/usr/bin/env python3

"""Launch the deterministic PyBullet control-and-sensor teaching slice."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_foxglove = LaunchConfiguration('use_foxglove')
    use_rviz = LaunchConfiguration('use_rviz')
    master_launch_file = PathJoinSubstitution(
        [FindPackageShare('robot_simulation'), 'launch', 'master.launch.py']
    )

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(master_launch_file),
        launch_arguments={
            'use_foxglove': use_foxglove,
            'use_rviz': use_rviz,
            'publish_initial_configuration': 'false',
        }.items(),
    )
    example_controller = Node(
        package='robot_controller',
        executable='example_stance_controller',
        name='example_stance_controller',
        output='screen',
    )
    interface_monitor = Node(
        package='robot_simulation',
        executable='state_interface_monitor',
        name='state_interface_monitor',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_foxglove',
            default_value='true',
            description='Start the Foxglove WebSocket bridge on port 8765.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz (requires a graphical display).',
        ),
        stack,
        example_controller,
        interface_monitor,
    ])
