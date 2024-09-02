"""
Axis camera driver launch file.

Lorenzo Bianchi <lnz.bnc@gmail.com>
Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

October 3, 2023
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Builds a LaunchDescription for the bottom Detector"""
    ld = LaunchDescription()

    # Build config file path
    # config = os.path.join(
    #     get_package_share_directory('axis_camera'),
    #     'config',
    #     'axis_camera_turin.yaml'
    # )
    config = '/home/neo/workspace/src/axis_camera_driver/src/axis_camera/config/axis_camera_turin.yaml'

    # Declare launch arguments
    ns = LaunchConfiguration('namespace')
    cf = LaunchConfiguration('cf')
    ns_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='axis_camera'
    )
    cf_launch_arg = DeclareLaunchArgument(
        'cf',
        default_value=config
    )
    ld.add_action(ns_launch_arg)
    ld.add_action(cf_launch_arg)

    # Create node launch description
    node = Node(
        package='axis_camera',
        executable='axis_camera',
        namespace=ns,
        shell=False,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[cf]
    )

    ld.add_action(node)

    return ld
