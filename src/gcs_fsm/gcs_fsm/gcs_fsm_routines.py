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

import math
import subprocess
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

    # Download mission data from REST server
    try:
        subprocess.run(
            [
                "wget",
                "-O",
                "/home/neo/workspace/logs/mission.json",
                "http://172.16.0.18:8080"
            ],
            check=True)
    except subprocess.CalledProcessError as e:
        node.get_logger().fatal(f"Error downloading mission data: {e}")
        return 'error'

    # Display mission data and await user confirmation
    with open('/home/neo/workspace/logs/mission.json', 'r') as f:
        mission_data = json.load(f)
    mission_data_hr = json.dumps(mission_data, indent=4)
    print(mission_data_hr)
    print('Press Enter to confirm mission data...')
    input()
    with open('/home/neo/workspace/logs/mission.json', 'r') as f:
        mission_data = json.load(f)

    # Parse manche
    manche = int(mission_data['manche'])

    # Parse ArUco targets
    if 'markers' in mission_data:
        if 'uav' in mission_data['markers']:
            for id in mission_data['markers']['uav']:
                node.valid_ids.append("ArUco " + str(id))
        if 'ugv' in mission_data['markers']:
            for id in mission_data['markers']['ugv']:
                node.valid_ids.append("ArUco " + str(id))
        if 'uxv' in mission_data['markers']:
            for id in mission_data['markers']['uxv']:
                node.valid_ids.append("ArUco " + str(id))
    else:
        node.get_logger().warn("No ArUco markers specified")

    # Parse object targets
    if 'object_detection' in mission_data:
        key = 'classes' if 'classes' in mission_data['object_detection'] else 'class'
        for obj in mission_data['object_detection'][key]:
            if str(obj) not in node.COCO_CLASSES:
                node.get_logger().fatal(f"Unknown object class: {str(obj)}")
                return 'error'
            node.valid_ids.append(str(obj))
    else:
        node.get_logger().warn("No object classes specified")

    # Parse FollowMe waypoints
    if 'follow_me' in mission_data:
        node.followme_start_time = mission_data['follow_me']['time']
        for wp in mission_data['follow_me']['wps_coordinates_world']:
            node.followme_waypoints.append((wp[0], wp[1]))
    else:
        node.get_logger().warn("No FollowMe waypoints specified")

    # Parse emergency tasks
    key = 'emergency' if 'emergency' in mission_data else 'emergency_task'
    if key in mission_data:
        if 'landing' in mission_data[key]:
            node.emergency_landing_id = "ArUco " + str(mission_data[key]['landing'])
        if 'RTB' in mission_data[key]:
            node.rtb_id = "ArUco " + str(mission_data[key]['RTB'])
    else:
        node.get_logger().warn("No emergency tasks specified")

    # Wait second confirmation
    print('Press Enter to confirm mission start...')
    input()

    # Open log file
    node.open_log(manche)
    node.log("Mission started")

    # Send mission data to agents
    node.mission_data_pub.publish(String(data=json.dumps(mission_data)))

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
        if len(node.valid_ids) == 0 and node.followme_done:
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
    node.close_log()
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
