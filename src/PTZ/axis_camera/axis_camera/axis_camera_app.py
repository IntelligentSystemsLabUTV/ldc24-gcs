"""
Axis Camera driver node application.

Lorenzo Bianchi <lnz.bnc@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

October 29, 2023
"""

import sys

import rclpy

from axis_camera.axis_camera_node import AxisCameraNode

def main():
    rclpy.init(args=sys.argv)
    axis_camera_node = AxisCameraNode()

    try:
        rclpy.spin(axis_camera_node)
    finally:
        axis_camera_node.vs.stop()
        axis_camera_node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
