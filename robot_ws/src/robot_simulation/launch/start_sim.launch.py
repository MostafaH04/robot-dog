#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sim_launch = Node(
        package='robot_simulation',
        executable='quad_sim',
        name='quad_sim',
        output='screen',
    )

    return LaunchDescription([
        sim_launch,
    ])
