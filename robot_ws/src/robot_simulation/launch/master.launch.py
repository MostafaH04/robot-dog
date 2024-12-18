#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
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
        PythonLaunchDescriptionSource(visualization_launch_file)
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_file)
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_launch_file)
    )

    return LaunchDescription([
        visualization_launch_file,
        sim_launch,
        controller_launch
    ])
