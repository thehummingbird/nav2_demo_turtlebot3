import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
  pkg_tb3_sim = get_package_share_directory('tb3_sim')

  # Spawn blocks
  block_spawner = Node(
      package="tb3_sim",
      executable="block_spawner",
      name="block_spawner",
      parameters=[{
          "location_file": os.path.join(pkg_tb3_sim, "config", "sim_house_locations.yaml")
      }]
  )

  ld = LaunchDescription()

  # Add the commands to the launch description
  ld.add_action(block_spawner)

  return ld
