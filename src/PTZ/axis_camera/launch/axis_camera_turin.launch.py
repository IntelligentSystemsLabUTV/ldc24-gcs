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
    """Builds a LaunchDescription for the Axis Camera Driver"""
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('axis_camera'),
        'config',
        'axis_camera_turin.yaml'
    )

    # Declare launch arguments
    ns = LaunchConfiguration('namespace')
    cf = LaunchConfiguration('cf')
    ns_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value=''
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
        parameters=[cf],
        remappings=[('/axis_camera/stream', '/axis_camera/stream'),
                    ('/axis_camera/command', '/axis_camera/command'),
                    ('/axis_camera/stream_ptzf', '/axis_camera/stream_ptzf'),
                    ('/axis_camera/ptzf', '/axis_camera/ptzf')],
    )

    ld.add_action(node)

    return ld
