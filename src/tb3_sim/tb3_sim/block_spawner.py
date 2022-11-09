import rclpy
from gazebo_msgs.srv import SpawnEntity
import yaml
import random
import os
import transforms3d
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

model_dir = os.path.join(get_package_share_directory("tb3_sim"), "models")


class BlockSpawner(Node):
  def __init__(self):
    super().__init__("block_spawner")
    self.client = self.create_client(
        SpawnEntity, '/spawn_entity')  # check this
    while not self.client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not available, waiting again...')

    self.declare_parameter("location_file")

  def spawn_blocks(self):
    block_location_file = self.get_parameter("location_file").value
    self.get_logger().info(f"Using location file: {block_location_file}")

    with open(block_location_file, "r") as file:
      locations = yaml.load(file, Loader=yaml.FullLoader)

    model_names = ["red_block", "green_block", "blue_block"]

    sampled_locs = random.sample(list(locations.keys()), len(model_names))

    for model_name, loc in zip(model_names, sampled_locs):
      x, y, theta = locations[loc]
      self.spawn_model(model_name, x, y, theta)

  def spawn_model(self, model_name, x, y, theta):
    model_file = os.path.join(model_dir, model_name, "model.sdf")

    with open(model_file, "r") as file:
      model_xml = file.read()

    request = SpawnEntity.Request()
    request.name = model_name
    request.xml = model_xml
    request.initial_pose.position.x = x
    request.initial_pose.position.y = y
    quat = transforms3d.euler.euler2quat(0, 0, theta)
    request.initial_pose.orientation.w = quat[0]
    request.initial_pose.orientation.x = quat[1]
    request.initial_pose.orientation.y = quat[2]
    request.initial_pose.orientation.z = quat[3]

    future = self.client.call_async(request)
    rclpy.spin_until_future_complete(self, future)
    return future.result()


def main(args=None):
  rclpy.init(args=args)

  spawner = BlockSpawner()
  spawner.spawn_blocks()

  rclpy.spin(spawner)  # why this needed?
  rclpy.shutdown()


if __name__ == "__main__":
  main()
