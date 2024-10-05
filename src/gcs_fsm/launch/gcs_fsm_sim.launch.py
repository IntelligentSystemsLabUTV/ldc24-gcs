"""
GCS FSM app launch file for simulation.

Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

October 5, 2024
"""

# Copyright 2024 Intelligent Systems Lab
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
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(get_package_share_directory('gcs_fsm'), 'config', 'gcs_fsm_sim.yaml')

    # Set environment variables
    rmw_env_var = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION',
        value='rmw_cyclonedds_cpp')
    domain_id_env_var = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value='1')
    ld.add_action(rmw_env_var)
    ld.add_action(domain_id_env_var)

    # Create node launch description
    node = Node(
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

    ld.add_action(node)

    return ld
