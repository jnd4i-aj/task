#!/usr/bin/env python3

import tf_transformations
import logging


# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class TransformAngle():
    def __init__(self) -> None:
        pass

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert a Quaternion to Euler angles (roll, pitch, yaw)
        And returns the angles in Euler in radions
        """
        logging.info(f"Converting Quaternion (x={x}, y={y}, z={z}, w={w}) to Euler angles.")
        try:
            euler = tf_transformations.euler_from_quaternion([x, y, z, w])
            logging.info(f"Converted to Euler angles: roll={euler[0]}, pitch={euler[1]}, yaw={euler[2]}")
            return euler
        except Exception as e:
            logging.error(f"Failed to convert quaternion to Euler angles: {e}")
            return None

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw in radions) to a Quaternion
        And returns a List of Angles in Quaternion in the order x, y, z, w
        """
        logging.info(f"Converting Euler angles (roll={roll}, pitch={pitch}, yaw={yaw}) to Quaternion.")
        try:
            quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            logging.info(f"Converted to Quaternion: x={quaternion[0]}, y={quaternion[1]}, z={quaternion[2]}, w={quaternion[3]}")
            return quaternion
        except Exception as e:
            logging.error(f"Failed to convert Euler angles to quaternion: {e}")
            return None

