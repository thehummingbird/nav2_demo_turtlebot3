import time
import rclpy
from rclpy.node import Node
import transforms3d
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitAmclPosePublisher(Node):
  def __init__(self):
    super().__init__("init_amcl_pose_publisher")

    self.declare_parameter("x", value=0.0)
    self.declare_parameter("y", value=0.0)
    self.declare_parameter("theta", value=0.0)
    self.declare_parameter("cov", value=0.5**2)

    self.publisher = self.create_publisher(
        PoseWithCovarianceStamped,
        "/initialpose",
        10,
    )

    while(self.publisher.get_subscription_count() == 0):
      self.get_logger().info("Waiting for AMCL Initial Pose subscriber")
      time.sleep(1.0)

  def send_init_pose(self):
    x = self.get_parameter("x").value
    y = self.get_parameter("y").value
    theta = self.get_parameter("theta").value
    cov = self.get_parameter("cov").value

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    quat = transforms3d.euler.euler2quat(0, 0, theta)
    msg.pose.pose.orientation.w = quat[0]
    msg.pose.pose.orientation.x = quat[1]
    msg.pose.pose.orientation.y = quat[2]
    msg.pose.pose.orientation.z = quat[3]

    msg.pose.covariance = [
        cov, 0.0, 0.0, 0.0, 0.0, 0.0,  # Pos X
        0.0, cov, 0.0, 0.0, 0.0, 0.0,  # Pos Y
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Pos Z
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Rot X
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Rot Y
        0.0, 0.0, 0.0, 0.0, 0.0, cov   # Rot Z
    ]

    self.publisher.publish(msg)


def main(args=None):
  rclpy.init()
  initAmclPosePublisher = InitAmclPosePublisher()

  future = initAmclPosePublisher.send_init_pose()

  rclpy.spin(initAmclPosePublisher)

  initAmclPosePublisher.destroy_node()
  rclpy.shutdown()
