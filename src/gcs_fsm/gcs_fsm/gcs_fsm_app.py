"""
GCS FSM application.

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

import sys

import rclpy

from transitions_ros.machine import Machine

from gcs_fsm.gcs_fsm_node import GCSFSMNode
from gcs_fsm.gcs_fsm_routines import *

def main():
    # Create states table
    states_table = [
        {'name': 'INIT',                'routine': init_routine},
        {'name': 'EXPLORE',             'routine': explore_routine},
        {'name': 'FOLLOW_ME',           'routine': followme_routine},
        {'name': 'EMERGENCY_LANDING',   'routine': emergency_landing_routine},
        {'name': 'RTB',                 'routine': rtb_routine},
        {'name': 'COMPLETED',           'routine': completed_routine},
        {'name': 'ABORT',               'routine': abort_routine}
    ]

    # Create transitions table
    transitions_table = [
        ['start',                   'INIT',                 'EXPLORE'],
        ['time_for_followme',       'EXPLORE',              'FOLLOW_ME'],
        ['found_emergency_landing', 'EXPLORE',              'EMERGENCY_LANDING'],
        ['found_rtb',               'EXPLORE',              'RTB'],
        ['stop',                    'EXPLORE',              'COMPLETED'],
        ['followme_done',           'FOLLOW_ME',            'EXPLORE'],
        ['emergency_landing_done',  'EMERGENCY_LANDING',    'EXPLORE'],
        ['rtb_done',                'RTB',                  'EXPLORE'],
        ['error',                   '*',                    'ABORT']
    ]

    # Initialize ROS 2 context and node
    rclpy.init(args=sys.argv)
    gcs_node = GCSFSMNode()

    # Initialize FSM
    gcs_fsm = Machine(
        node=gcs_node,
        states_table=states_table,
        transitions_table=transitions_table,
        initial_state='INIT')

    # Run the FSM
    try:
        gcs_fsm.run()
    finally:
        gcs_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
