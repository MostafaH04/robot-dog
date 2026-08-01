#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_foxglove = LaunchConfiguration('use_foxglove')
    use_rviz = LaunchConfiguration('use_rviz')
    publish_initial_configuration = LaunchConfiguration(
        'publish_initial_configuration'
    )

    visualization_launch_file = PathJoinSubstitution(
        [FindPackageShare('robot_desc'), 'launch', 'view_robot.launch.py']
    )

    sim_launch_file = PathJoinSubstitution(
        [FindPackageShare('robot_simulation'), 'launch', 'start_sim.launch.py']
    )

    controller_launch_file = PathJoinSubstitution(
        [FindPackageShare('robot_controller'), 'launch', 'quad_controller.launch.py']
    )

    visualization_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(visualization_launch_file),
        launch_arguments={
            'use_foxglove': use_foxglove,
            'use_rviz': use_rviz,
        }.items(),
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_file)
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_launch_file),
        launch_arguments={
            'publish_initial_configuration': publish_initial_configuration,
        }.items(),
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
        DeclareLaunchArgument(
            'publish_initial_configuration',
            default_value='true',
            description='Publish the controller adapter nominal stance at startup.',
        ),
        visualization_launch_file,
        sim_launch,
        controller_launch,
    ])
