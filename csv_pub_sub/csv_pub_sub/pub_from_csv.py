#! /usr/bin/env python3

import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import csv
import os

class PubSub(Node):

    def __init__(self):

        super().__init__("publish_data")

        self.vel_pub = self.create_publisher(String, "/read_data/csv", 10)
        self.timer = self.create_timer(0.5, self.pub_data)

        self.csv_path = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'flipkart_task.csv'
        )

        try:
            self.csv_file = open(self.csv_path, mode='r')
            self.csv_reader = csv.reader(self.csv_file)
            self.get_logger().info(f"Successfully opened {self.csv_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to open CSV file: {e}")


    def pub_data(self):

        try:
            row = next(self.csv_reader)
            data_str = ', '.join(row)

            str_msg = String()
            str_msg.data = f"CSV Row Data: {data_str}"
            self.vel_pub.publish(str_msg)

        except StopIteration:
            self.get_logger().info("Reached end of CSV file.")
            # self.csv_file.seek(0)

        except Exception as e:
            self.get_logger().error(f"Error reading from CSV file: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PubSub()
    rclpy.spin(node)
    rclpy.shutdown()