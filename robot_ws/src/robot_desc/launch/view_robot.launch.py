#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_foxglove = LaunchConfiguration('use_foxglove')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('robot_desc'),
                    'urdf',
                    'quad.xacro',
                ]
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([FindPackageShare('robot_desc'), 'rviz', 'view_robot.rviz']),
        ],
        condition=IfCondition(use_rviz),
    )

    foxglove_launch_file = PathJoinSubstitution(
        [FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml']
    )

    foxglove_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(foxglove_launch_file),
        condition=IfCondition(use_foxglove),
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
        robot_state_pub,
        rviz_launch,
        foxglove_launch,
    ])
