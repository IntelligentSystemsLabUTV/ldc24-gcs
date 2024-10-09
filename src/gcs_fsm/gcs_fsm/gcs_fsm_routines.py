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
                "http://172.16.0.19:8080"
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
    with open('/home/neo/workspace/logs/mission.json', 'r') as f:
        mission_data = json.load(f)

    # Parse manche
    manche = int(mission_data['manche'])

    # Parse ArUco targets
    if 'markers' in mission_data:
        if 'uav' in mission_data['markers']:
            for id in mission_data['markers']['uav']:
                if int(id) not in range(101, 500):
                    node.get_logger().warn(f'ID out of legal range: {id}')
                else:
                    node.valid_ids.append("ArUco " + str(id))
        if 'ugv' in mission_data['markers']:
            for id in mission_data['markers']['ugv']:
                if int(id) not in range(101, 500):
                    node.get_logger().warn(f'ID out of legal range: {id}')
                else:
                    node.valid_ids.append("ArUco " + str(id))
        if 'uxv' in mission_data['markers']:
            for id in mission_data['markers']['uxv']:
                if int(id) not in range(101, 500):
                    node.get_logger().warn(f'ID out of legal range: {id}')
                else:
                    # These must be found two times!
                    node.valid_ids.append("ArUco " + str(id))
                    node.valid_ids.append("ArUco " + str(id))
    else:
        node.get_logger().warn("No ArUco markers specified")

    # Parse object targets
    if 'object_detection' in mission_data:
        key = 'classes' if 'classes' in mission_data['object_detection'] else 'class'
        for obj in mission_data['object_detection'][key]:
            if str(obj).replace('_', ' ') not in node.COCO_CLASSES:
                node.get_logger().fatal(f"Unknown object class: {str(obj).replace('_', ' ')}")
                return 'error'
            node.valid_ids.append(str(obj).replace('_', ' '))
    else:
        node.get_logger().warn("No object classes specified")

    # Parse FollowMe waypoints
    if 'follow_me' in mission_data:
        followme_start_time = float(mission_data['follow_me']['time'])
        node.followme_orig_start_time = float(mission_data['follow_me']['time'])
        followme_start_time = max(followme_start_time, 60.0 * 5.0)
        node.followme_start_time = followme_start_time
        followme_lerciata = []
        for wp in mission_data['follow_me']['wps_coordinates_world']:
            followme_lerciata.append((float(wp[0]), float(wp[1])))
        node.followme_waypoints.append(followme_lerciata[0])
        for i in range(len(followme_lerciata) - 1):
            mid_point = ((followme_lerciata[i][0] + followme_lerciata[i + 1][0]) / 2,
                         (followme_lerciata[i][1] + followme_lerciata[i + 1][1]) / 2)
            node.followme_waypoints.append(mid_point)
            node.followme_waypoints.append(followme_lerciata[i + 1])
    else:
        node.get_logger().warn("No FollowMe waypoints specified")

    # Parse emergency tasks
    key = 'emergency' if 'emergency' in mission_data else 'emergency_task'
    if key in mission_data:
        if 'landing' in mission_data[key]:
            if int(mission_data[key]['landing']['marker']) not in range(101, 500):
                node.get_logger().warn(f'Emergency task ID out of legal range: {mission_data[key]["landing"]["marker"]}')
            else:
                node.emergency_landing_id = "ArUco " + str(mission_data[key]['landing']['marker'])
        if 'RTB' in mission_data[key]:
            if int(mission_data[key]['RTB']['marker']) not in range(101, 500):
                node.get_logger().warn(f'Emergency task ID out of legal range: {mission_data[key]["RTB"]["marker"]}')
            else:
                node.rtb_id = "ArUco " + str(mission_data[key]['RTB']['marker'])
    else:
        node.get_logger().warn("No emergency tasks specified")

    # Wait second confirmation
    print('Press START button on HMI to confirm mission start...')
    node.start = False
    while not node.start:
        node.wait_spinning()

    # Open log file
    node.open_log(manche)
    node.log("--- START OF MISSION ---")

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
        if node.stop:
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
        node.wait_spinning()

    node.get_logger().info('Exploration stopped')
    return next_trigger


def followme_routine(node: GCSFSMNode) -> str:
    """
    Implements FollowMe logic.

    :param node: ROS 2 node.
    :return: Next trigger.
    """
    node.update_fsm_state('FOLLOW_ME')
    node.followme = False
    node.followme_done = True
    on_req: SetBool.Request = SetBool.Request(data=True)
    off_req: SetBool.Request = SetBool.Request(data=False)
    delta_y = node.followme_waypoints[1][1] - node.followme_waypoints[0][1]
    delta_x = node.followme_waypoints[1][0] - node.followme_waypoints[0][0]
    start_heading = math.atan2(delta_y, delta_x)

    # Get control of agents
    while True:
        res1_future = node.arianna_followme_client.call_async(on_req)
        tic = node.get_clock().now()
        while not res1_future.done():
            node.wait_spinning()
            toc = node.get_clock().now()
            diff: Duration = toc - tic
            if diff.nanoseconds > 1e9:
                break
        if not res1_future.done():
            node.get_logger().error('Failed to get control of UAV')
            continue
        res1 = res1_future.result()
        if res1.success:
            break
    node.get_logger().info('UAV in FOLLOW_ME')
    while True:
        res2_future = node.dottorcane_followme_client.call_async(on_req)
        tic = node.get_clock().now()
        while not res2_future.done():
            node.wait_spinning()
            toc = node.get_clock().now()
            diff: Duration = toc - tic
            if diff.nanoseconds > 1e9:
                break
        if not res2_future.done():
            node.get_logger().error('Failed to get control of UGV')
            continue
        res2 = res2_future.result()
        if res2.success:
            break
    time.sleep(2.0)
    node.get_logger().info('UGV in FOLLOW_ME')

    # Send agents to FollowMe start point
    arianna_nav_goal = Navigate.Goal(
        header=Header(
            stamp=node.get_clock().now().to_msg(),
            frame_id='map'
        ),
        target=Point(
            x=node.followme_waypoints[0][0],
            y=node.followme_waypoints[0][1],
            z=2.0
        ),
        heading=start_heading
    )
    dottorcane_nav_goal = Navigate.Goal(
        header=Header(
            stamp=node.get_clock().now().to_msg(),
            frame_id='map'
        ),
        target=Point(
            x=node.followme_waypoints[0][0],
            y=node.followme_waypoints[0][1],
            z=0.33
        ),
        heading=start_heading
    )
    arianna_goal_handle = node.arianna_navigate_client.send_goal_sync(arianna_nav_goal)
    dottorcane_goal_handle = node.dottorcane_navigate_client.send_goal_sync(dottorcane_nav_goal)
    if arianna_goal_handle is None or dottorcane_goal_handle is None:
        node.get_logger().error('Failed to send FollowMe start goals')
        if arianna_goal_handle is not None:
            node.arianna_navigate_client.cancel_sync(arianna_goal_handle)
            node.arianna_navigate_client.get_result_sync(arianna_goal_handle)
        if dottorcane_goal_handle is not None:
            node.dottorcane_navigate_client.cancel_sync(dottorcane_goal_handle)
            node.dottorcane_navigate_client.get_result_sync(dottorcane_goal_handle)
        # Give back control of agents
        while True:
            res1_future = node.arianna_followme_client.call_async(off_req)
            tic = node.get_clock().now()
            while not res1_future.done():
                node.wait_spinning()
                toc = node.get_clock().now()
                diff: Duration = toc - tic
                if diff.nanoseconds > 1e9:
                    break
            if not res1_future.done():
                node.get_logger().error('Failed to give back control of UAV')
                continue
            res1 = res1_future.result()
            if res1.success:
                break
        node.get_logger().info('UAV back in control')
        while True:
            res2_future = node.dottorcane_followme_client.call_async(off_req)
            tic = node.get_clock().now()
            while not res2_future.done():
                node.wait_spinning()
                toc = node.get_clock().now()
                diff: Duration = toc - tic
                if diff.nanoseconds > 1e9:
                    break
            if not res2_future.done():
                node.get_logger().error('Failed to give back control of UGV')
                continue
            res2 = res2_future.result()
            if res2.success:
                break
        node.get_logger().info('Agents back in control')
        return 'followme_done'
    node.get_logger().info('FollowMe start goals sent')

    # Wait for agents to get to starting point
    dottorcane_res: Navigate.Result = node.dottorcane_navigate_client.get_result_sync(dottorcane_goal_handle)
    arianna_res: Navigate.Result = node.arianna_navigate_client.get_result_sync(arianna_goal_handle)
    if dottorcane_res.result.result.result != CommandResultStamped.SUCCESS or arianna_res.result.result.result != CommandResultStamped.SUCCESS:
        node.get_logger().error('Error executing FollowMe')
        # Give back control of agents
        while True:
            res1_future = node.arianna_followme_client.call_async(off_req)
            tic = node.get_clock().now()
            while not res1_future.done():
                node.wait_spinning()
                toc = node.get_clock().now()
                diff: Duration = toc - tic
                if diff.nanoseconds > 1e9:
                    break
            if not res1_future.done():
                node.get_logger().error('Failed to give back control of UAV')
                continue
            res1 = res1_future.result()
            if res1.success:
                break
        node.get_logger().info('UAV back in control')
        while True:
            res2_future = node.dottorcane_followme_client.call_async(off_req)
            tic = node.get_clock().now()
            while not res2_future.done():
                node.wait_spinning()
                toc = node.get_clock().now()
                diff: Duration = toc - tic
                if diff.nanoseconds > 1e9:
                    break
            if not res2_future.done():
                node.get_logger().error('Failed to give back control of UGV')
                continue
            res2 = res2_future.result()
            if res2.success:
                break
        node.get_logger().info('Agents back in control')
        return 'followme_done'
    node.get_logger().info('Agents at FollowMe start point')

    # Start UAV collimation over target
    collimator_goal = PrecisionLanding.Goal(
        altitude=2.0,
        land=False,
        align=True
    )
    collimator_goal_handle = node.arianna_collimate_client.send_goal_sync(collimator_goal)
    if collimator_goal_handle is None:
        node.get_logger().error('Error sending collimator goal')
        # Give back control of agents
        while True:
            res1_future = node.arianna_followme_client.call_async(off_req)
            tic = node.get_clock().now()
            while not res1_future.done():
                node.wait_spinning()
                toc = node.get_clock().now()
                diff: Duration = toc - tic
                if diff.nanoseconds > 1e9:
                    break
            if not res1_future.done():
                node.get_logger().error('Failed to give back control of UAV')
                continue
            res1 = res1_future.result()
            if res1.success:
                break
        node.get_logger().info('UAV back in control')
        while True:
            res2_future = node.dottorcane_followme_client.call_async(off_req)
            tic = node.get_clock().now()
            while not res2_future.done():
                node.wait_spinning()
                toc = node.get_clock().now()
                diff: Duration = toc - tic
                if diff.nanoseconds > 1e9:
                    break
            if not res2_future.done():
                node.get_logger().error('Failed to give back control of UGV')
                continue
            res2 = res2_future.result()
            if res2.success:
                break
        node.get_logger().info('Agents back in control')
        return 'followme_done'
    node.get_logger().info('UAV collimation started')

    # Pre-set UGV gains
    node.set_followme_parameters(True)

    # Wait for UAV stabilization over target
    time.sleep(3.0)
    node.log('FollowMe started')

    # Send UGV
    collimate_done = False
    collimate_future = node.arianna_collimate_client.get_result(collimator_goal_handle)
    for i in range(1, len(node.followme_waypoints)):
        node.get_logger().info(f'Going to waypoint ({i+1})')
        nav_goal = Navigate.Goal(
            header=Header(
                stamp=node.get_clock().now().to_msg(),
                frame_id='map'
            ),
            target=Point(
                x=node.followme_waypoints[i][0],
                y=node.followme_waypoints[i][1],
                z=0.33
            ),
            heading=6.28
        )
        node.dottorcane_navigate_client.call(nav_goal)
        time.sleep(1.0)
        if collimate_future.done():
            # Try to recover collimation: send UAV to next waypoint
            curr_pose = node.get_agent_pose('arianna')
            reach_goal: Reach.Goal = Reach.Goal(
                target_pose=PoseStamped(
                    header=Header(
                        stamp=node.get_clock().now().to_msg(),
                        frame_id='map'
                    ),
                    pose=Pose(
                        position=Point(
                            x=nav_goal.target.x,
                            y=nav_goal.target.y,
                            z=2.0
                        ),
                        orientation=curr_pose.pose.orientation
                    )
                ),
                reach_radius=0.1,
                stop_at_target=True
            )
            node.arianna_reach_client.call(reach_goal)
            collimator_goal_handle = node.arianna_collimate_client.send_goal_sync(collimator_goal)
            if collimator_goal_handle is None:
                collimate_done = True
                node.get_logger().error('UAV re-collimation rejected')
                break
            collimate_future = node.arianna_collimate_client.get_result(collimator_goal_handle)
            time.sleep(3.0)
            if collimate_future.done():
                collimate_done = True
                node.get_logger().error('UAV collimation failed')
                break

    # Stop collimation
    if not collimate_done:
        node.arianna_collimate_client.cancel_sync(collimator_goal_handle)
        node.arianna_collimate_client.get_result_sync(collimator_goal_handle)
        node.get_logger().info('UAV collimation stopped')

    # Post-set UGV gains
    node.set_followme_parameters(False)

    # Give back control of agents
    node.log('FollowMe completed')
    node.get_logger().info('FollowMe completed')
    while True:
        res1_future = node.arianna_followme_client.call_async(off_req)
        tic = node.get_clock().now()
        while not res1_future.done():
            node.wait_spinning()
            toc = node.get_clock().now()
            diff: Duration = toc - tic
            if diff.nanoseconds > 1e9:
                break
        if not res1_future.done():
            node.get_logger().error('Failed to give back control of UAV')
            continue
        res1 = res1_future.result()
        if res1.success:
            break
    node.get_logger().info('UAV back in control')
    while True:
        res2_future = node.dottorcane_followme_client.call_async(off_req)
        tic = node.get_clock().now()
        while not res2_future.done():
            node.wait_spinning()
            toc = node.get_clock().now()
            diff: Duration = toc - tic
            if diff.nanoseconds > 1e9:
                break
        if not res2_future.done():
            node.get_logger().error('Failed to give back control of UGV')
            continue
        res2 = res2_future.result()
        if res2.success:
            break
    node.get_logger().info('UGV back in control')

    return 'followme_done'


def emergency_landing_routine(node: GCSFSMNode) -> str:
    """
    Triggers emergency landing in UAV.

    :param node: ROS 2 node.
    :return: Next trigger.
    """
    node.update_fsm_state('EMERGENCY_LANDING')
    node.emergency_landing = False

    # Trigger UAV emergency landing
    for i in range(10):
        time.sleep(0.1)
        node.arianna_emergency_landing_pub.publish(Empty())
    node.get_logger().info('UAV emergency landing triggered')
    node.log("UAV emergency landing triggered")
    return 'emergency_landing_done'


def rtb_routine(node: GCSFSMNode) -> str:
    """
    Implements RTB logic.

    :param node: ROS 2 node.
    :return: Next trigger.
    """
    node.update_fsm_state('RTB')
    node.rtb = False

    # Trigger RTB in agents
    for i in range(10):
        time.sleep(0.1)
        node.rtb_pub.publish(Empty())
    node.get_logger().info('RTB triggered')
    node.log('RTB triggered')
    return 'rtb_done'


def completed_routine(node: GCSFSMNode) -> str:
    """
    Mission completed.

    :param node: ROS 2 node.
    :return: Next trigger.
    """
    node.update_fsm_state('COMPLETED')
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
