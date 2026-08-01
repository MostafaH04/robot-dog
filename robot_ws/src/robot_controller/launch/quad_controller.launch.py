#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_initial_configuration = LaunchConfiguration(
        'publish_initial_configuration'
    )
    controller_launch = Node(
        package='robot_controller',
        executable='quad_controller',
        name='quad_controller',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'publish_initial_configuration': ParameterValue(
                publish_initial_configuration,
                value_type=bool,
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the fixed-step clock published by the simulator.',
        ),
        DeclareLaunchArgument(
            'publish_initial_configuration',
            default_value='true',
            description='Publish the built-in nominal stance once after startup.',
        ),
        controller_launch,
    ])
