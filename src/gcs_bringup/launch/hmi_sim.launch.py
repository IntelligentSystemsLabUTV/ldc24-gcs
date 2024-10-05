"""
HMI launch file for simulation.

October 5, 2024
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
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
        get_package_share_directory('arianna_sim_description'),
        'launch',
        'arianna_description.launch.py')
    dottorcane_desc = os.path.join(
        get_package_share_directory('dottorcane_sim_description'),
        'launch',
        'dottorcane_description.launch.py')

    # Set environment variables
    rmw_env_var = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION',
        value='rmw_cyclonedds_cpp')
    domain_id_env_var = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value='2')
    ld.add_action(rmw_env_var)
    ld.add_action(domain_id_env_var)

    # Include the descriptions launch files
    arianna_desc_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([arianna_desc]))
    ld.add_action(arianna_desc_ld)
    dottorcane_desc_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dottorcane_desc]))
    ld.add_action(dottorcane_desc_ld)

    # Rviz config
    rviz_config = os.path.join(
        get_package_share_directory('gcs_bringup'),
        'rviz',
        'hmi_sim.rviz'
    )
    rviz = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        emulate_tty=True,
        output='screen')
    ld.add_action(rviz)

    return ld
