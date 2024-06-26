#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare('robot_description'), 'launch', 'gazebo.launch.py']
    )
    rviz_launch = PathJoinSubstitution(
        [FindPackageShare('robot_description'), 'launch', 'view_robot.launch.py']
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch)
        ),
    ])
