#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gazebo_params_file = str(PathJoinSubstitution(
        [
            FindPackageShare("robot_description"),
            "config",
            "gazebo_params.yaml",
        ]
    ))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']
            )]
        ),

        launch_arguments={
            'extra_gazebo_args':'--ros-args --params-file ' + gazebo_params_file
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'Quadruped','-x','0','-y','0','-z','1'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity
    ]) 
