
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
  joint_broad_spawner = Node(
    package = "controller_manager", executable = "spawner", arguments = ["joint_broad"]
  )

  joint_trajectory_spawner = Node(
    package = "controller_manager",executable = "spawner", arguments = ["joint_trajectory_controller"]
  )

  return LaunchDescription([
    joint_broad_spawner,
    joint_trajectory_spawner
  ])
