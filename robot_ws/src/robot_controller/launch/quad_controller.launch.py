#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    controller_launch = Node(
        package='robot_controller',
        executable='quad_controller',
        name='quad_controller',
        output='screen',
    )

    return LaunchDescription([
        controller_launch,
    ])
