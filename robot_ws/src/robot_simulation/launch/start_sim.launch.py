#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    publish_ground_truth_tf = LaunchConfiguration('publish_ground_truth_tf')
    sim_launch = Node(
        package='robot_simulation',
        executable='quad_sim',
        name='quad_sim',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'publish_ground_truth_tf': ParameterValue(
                publish_ground_truth_tf,
                value_type=bool,
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_ground_truth_tf',
            default_value='false',
            description='Publish optional truth TF in distinct sim_ground_truth frames.',
        ),
        sim_launch,
    ])
