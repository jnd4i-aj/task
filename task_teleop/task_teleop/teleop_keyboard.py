#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

# ANSI color codes for terminal output
class Color:
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    RESET = '\033[0m'

# Key mappings
key_mapping = {
    'w': [1.0, 0.0],  # Move forward
    'x': [-1.0, 0.0], # Move backward
    'a': [0.0, 1.0],  # Turn left
    'd': [0.0, -1.0], # Turn right
    's': [0.0, 0.0]   # Stop
}

class AMRTeleopKeyboardNode(Node):

    def __init__(self):
        super().__init__('amr_teleop_keyboard')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 0.25
        self.angular_speed = 0.25
        self.max_speed = 1.0
        self.get_logger().info(self.__get_colored_message(self.__get_interface_message(), Color.GREEN))

    def run(self):
        try:
            while True:
                key = self.__get_key()
                if key in key_mapping.keys():
                    self.__send_velocity(key)
                elif KeyboardInterrupt:
                    self.get_logger().info(self.__get_colored_message('Keyboard Interrupt! Shutting down gracefully...', Color.RED))
                    break
                else:
                    self.get_logger().info(self.__get_colored_message("Invalid key pressed! Use 'W', 'A', 'S', 'D' to control the robot.", Color.YELLOW))
        except Exception as e:
            self.get_logger().error(self.__get_colored_message(f"An error occurred: {e}", Color.RED))
        finally:
            self.stop_robot()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def __get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def __send_velocity(self, key):
        twist = Twist()
        if key in ['w', 'x']:
            if key == 'w':
                self.speed = min(self.speed + 0.25, self.max_speed)
            elif key == 'x':
                self.speed = max(self.speed - 0.25, -self.max_speed)
            twist.linear.x = self.speed
        elif key in ['a', 'd']:
            if key == 'a':
                self.angular_speed = min(self.angular_speed + 0.25, self.max_speed)
            elif key == 'd':
                self.angular_speed = max(self.angular_speed - 0.25, -self.max_speed)
            twist.angular.z = self.angular_speed
        elif key == 's':
            self.speed = 0.0
            self.angular_speed = 0.0
            twist.linear.x = self.speed
            twist.angular.z = self.angular_speed

        self.get_logger().info(self.__get_colored_message(f'Current linear speed: {twist.linear.x}, angular speed: {twist.angular.z}', Color.CYAN))
        self.pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.pub.publish(twist)

    def __get_interface_message(self):
        return """
Control Your AMR With Your Keyboard Keys!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity by 0.25
a/d : increase/decrease angular velocity by 0.25

s : force stop

CTRL-C to quit
"""

    def __get_colored_message(self, message, color):
        return f"{color}{message}{Color.RESET}"

def main(args=None):
    rclpy.init(args=args)
    node = AMRTeleopKeyboardNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
