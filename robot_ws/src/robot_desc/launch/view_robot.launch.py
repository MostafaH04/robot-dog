#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_desc"),
                    "urdf",
                    "quad.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare("robot_desc"), "rviz", "view_robot.rviz"])]
    )
    
    foxglove_launch_file = PathJoinSubstitution(
        [FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml']
    )

    foxglove_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(foxglove_launch_file)
    )

    return LaunchDescription([
        robot_state_pub,
        foxglove_launch
    ])
