#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from angle_transformations import TransformAngle


class NAV2API():
    def __init__(self) -> None:
        pass

    def __set_pose(self, navigator: BasicNavigator, pose_x, pose_y, orientation_z):

        angle_in_quat = TransformAngle().euler_to_quaternion(roll=0.0,
                                                             pitch=0.0,
                                                             yaw=orientation_z)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = pose_x
        pose.pose.position.y = pose_y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = angle_in_quat[0]
        pose.pose.orientation.y = angle_in_quat[1]
        pose.pose.orientation.z = angle_in_quat[2]
        pose.pose.orientation.w = angle_in_quat[3]

        return pose

    def set_initial_pose(self):

        rclpy.init()
        nav = BasicNavigator()

        # setting initial pose i.e. 2D pose Estimate 
        # initial_pose = self.__set_pose(nav, -3.87, -0.35, 0.0)
        initial_pose = self.__set_pose(nav, -4.0, -1.0, 0.0)
        nav.setInitialPose(initial_pose)

        # wait for Nav2 to set inital pose
        nav.waitUntilNav2Active()

        # For sending one single goal
        goal4 = self.__set_pose(nav, 5.5, 2.0, 0.0)
        nav.goToPose(goal4)

        while not nav.isTaskComplete():
            status = nav.getFeedback()
            print(status)

        rclpy.shutdown()

if __name__ == '__main__':
    NAV2API().set_initial_pose()

