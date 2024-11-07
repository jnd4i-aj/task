#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import String


class GetTurtlePose(Node):
    def __init__(self):
        super().__init__("subscribe_to_data")
        self.pose_subs = self.create_subscription(String, "/read_data/csv", self.pose_callback, 10)

    def pose_callback(self, msg: String):
        self.get_logger().info(str(msg))
        print("Printing received msg", msg)


def main(args=None):
    rclpy.init(args=args)
    node = GetTurtlePose()
    rclpy.spin(node)
    rclpy.shutdown()


