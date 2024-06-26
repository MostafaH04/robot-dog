#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gazebo_launch_file = PathJoinSubstitution(
        [FindPackageShare('robot_description'), 'launch', 'gazebo.launch.py']
    )
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    rviz_launch_file = PathJoinSubstitution(
        [FindPackageShare('robot_description'), 'launch', 'view_robot.launch.py']
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_file)
    )

    controller_launch_file = PathJoinSubstitution(
        [FindPackageShare('robot_description'), 'launch', 'controller.launch.py']
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_launch_file)
    )

    return LaunchDescription([
        rviz_launch,
        gazebo_launch,
        controller_launch
    ])
