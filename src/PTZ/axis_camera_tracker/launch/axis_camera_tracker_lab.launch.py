"""
Axis Camera tracker launch file.

Lorenzo Bianchi <lnz.bnc@gmail.com>
Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

October 4, 2023
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Builds a LaunchDescription for the Aruco tracker"""
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('axis_camera_tracker'),
        'config',
        'axis_camera_tracker_lab.yaml'
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
        package='axis_camera_tracker',
        executable='axis_camera_tracker',
        namespace=ns,
        shell=False,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[cf],
        remappings=[('/command', '/axis_camera/command'),
                    ('/visual', '/hmi/found_target_visual'),
                    ('/axis_camera_tracker/selfie', '/axis_camera_tracker/selfie'),
                    ('/axis_image', '/axis_camera/axis_image'),
                    ('/axis_camera_tracker/image', '/axis_camera_tracker/image_color'),
                    ('/valid_targets', '/valid_targets'),
                    ('/targets', '/axis_camera/targets'),
                    ('/axis_error', '/axis_error'),
                    ('/search_arianna', '/search_arianna'),
                    ('/search_dottore', '/search_dottore'),
                    ('/stream_ptzf', '/axis_camera/stream_ptzf'),
                   ]
    )

    ld.add_action(node)

    return ld
