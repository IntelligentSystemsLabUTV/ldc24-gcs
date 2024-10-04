"""
GCS launch file.

October 4, 2024
"""

# Copyright 2024 dotX Automation s.r.l.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    # Build config files path
    config = os.path.join(
        get_package_share_directory('gcs_bringup'),
        'config',
        'gcs.yaml')
    arianna_desc = os.path.join(
        get_package_share_directory('arianna_description'),
        'launch',
        'arianna_description.launch.py')
    dottorcane_desc = os.path.join(
        get_package_share_directory('dottorcane_description'),
        'launch',
        'dottorcane_description.launch.py')

    # Set environment variables
    rmw_env_var = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION',
        value='rmw_cyclonedds_cpp')
    domain_id_env_var = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value='1')
    ld.add_action(rmw_env_var)
    ld.add_action(domain_id_env_var)

    # Include the descriptions launch files
    arianna_desc_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([arianna_desc]))
    ld.add_action(arianna_desc_ld)
    dottorcane_desc_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dottorcane_desc]))
    ld.add_action(dottorcane_desc_ld)

    # Composable node container
    container = ComposableNodeContainer(
        name='gcs_container',
        namespace='gcs',
        package='dua_app_management',
        executable='dua_component_container_mt',
        emulate_tty=True,
        output='both',
        log_cmd=True,
        composable_node_descriptions=[
            # ZED DRIVERS #
            ComposableNode(
                package='zed_driver',
                plugin='zed_drivers::ZEDDriverNode',
                name='arianna_zed2i_driver',
                namespace='gcs',
                parameters=[config]),
            ComposableNode(
                package='zed_driver',
                plugin='zed_drivers::ZEDDriverNode',
                name='dottorcane_zed2i_driver',
                namespace='gcs',
                parameters=[config])
        ]
    )
    ld.add_action(container)

    # GCS FSM
    fsm_node = Node(
        package='gcs_fsm',
        executable='gcs_fsm',
        exec_name='gcs_fsm_app',
        shell=False,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        namespace='gcs',
        parameters=[config]
    )
    ld.add_action(fsm_node)

    return ld
