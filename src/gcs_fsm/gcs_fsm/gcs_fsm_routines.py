"""
GCS FSM state routines.

Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

September 29, 2024
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

import time

from action_msgs.msg import GoalStatus
from dua_interfaces.msg import CommandResultStamped

from gcs_fsm.gcs_fsm_node import *


def init_routine(node: GCSFSMNode) -> str:
    """
    Get and parse mission start data.

    :param node: ROS 2 node.
    :return: Next trigger.
    """
    node.update_fsm_state('INIT')
    node.get_logger().info('Waiting for mission start command...')
    while len(node.valid_ids) == 0:
        # Wait for updates
        node.wait_spinning()

    # Do it again to update detectors
    node.wait_spinning()

    return 'start'


def explore_routine(node: GCSFSMNode) -> str:
    """
    Explore the environment.

    :param node: ROS 2 node.
    :return: Next trigger.
    """
    node.update_fsm_state('EXPLORE')

    # Wait for exploration to complete
    next_trigger = ''
    while True:
        # Check exit conditions
        if len(node.valid_ids) == 0:
            next_trigger = 'stop'
            break
        if node.followme:
            next_trigger = 'time_for_followme'
            break
        if node.rtb:
            next_trigger = 'found_rtb'
            break
        if node.emergency_landing:
            next_trigger = 'found_emergency_landing'
            break

        # Wait for updates
        try:
            node.wait_spinning()
        except KeyboardInterrupt:
            next_trigger = 'stop'
            break

    node.get_logger().info('Exploration stopped')
    return next_trigger


def followme_routine(node: GCSFSMNode) -> str:
    """
    Implements FollowMe logic.

    :param node: ROS 2 node.
    :return: Next trigger.
    """
    node.update_fsm_state('FOLLOW_ME')
    pass


def emergency_landing_routine(node: GCSFSMNode) -> str:
    """
    Triggers emergency landing in UAV.

    :param node: ROS 2 node.
    :return: Next trigger.
    """
    node.update_fsm_state('EMERGENCY_LANDING')
    pass


def rtb_routine(node: GCSFSMNode) -> str:
    """
    Implements RTB logic.

    :param node: ROS 2 node.
    :return: Next trigger.
    """
    node.update_fsm_state('RTB')
    pass


def completed_routine(node: GCSFSMNode) -> str:
    """
    Mission completed.

    :param node: ROS 2 node.
    :return: Next trigger.
    """
    node.update_fsm_state('COMPLETED')
    node.stop_pub.publish(Empty())
    node.wait_spinning()
    return ''


def abort_routine(node: GCSFSMNode) -> str:
    """
    Abort mission.

    :param node: ROS 2 node.
    :return: Next trigger.
    """
    node.update_fsm_state('ABORT')
    node.wait_spinning()
    return ''
