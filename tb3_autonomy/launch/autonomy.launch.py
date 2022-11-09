import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
  pkg_tb3_sim = get_package_share_directory('tb3_sim')
  pkg_tb3_autonomy = get_package_share_directory('tb3_autonomy')

  autonomy_node_cmd = Node(
      package="tb3_autonomy",
      executable="autonomy_node",
      name="autonomy_node",
      parameters=[{
          "location_file": os.path.join(pkg_tb3_sim, "config", "sim_house_locations.yaml")
      }]
  )

  ld = LaunchDescription()

  # Add the commands to the launch description
  ld.add_action(autonomy_node_cmd)

  return ld
