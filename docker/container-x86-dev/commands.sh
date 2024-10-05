#!/usr/bin/env bash

# Project-specific shell functions and commands.
#
# Roberto Masocco <r.masocco@dotxautomation.com>
#
# June 13, 2024

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

# Add yours, some convenient ones are provided below.
# You can also source other files from sub-units included by this project.

# shellcheck disable=SC1090

# Source DUA commands
source ~/.dua_submod.sh
source ~/.dua_subtree.sh

# Routine to convert an angle in degrees [-180° +180°] to radians [-PI +PI].
function degrad {
  local angle_in_degrees="$1"
  angle_in_radians=$(python3 -c "import sys, math; angle=float(sys.argv[1]); print(math.radians((angle + 180) % 360 - 180))" "$angle_in_degrees")
  echo "$angle_in_radians"
}

# Routine to update dua-utils.
function utils-update {
  CURR_SHELL=$(ps -p $$ | awk 'NR==2 {print $4}')

  pushd || return
  cd /opt/ros/dua-utils || return
  git pull
  git submodule update --init --recursive
  rm -rf install
  colcon build --merge-install
  rm -rf build log
  source "install/local_setup.$CURR_SHELL"
  popd || return
}

# Moves the drone to a target position
function reach {
  # Check input arguments
  if [[ $# -ne 5 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    reach X Y Z YAW RADIUS"
    echo >&2 "XYZ must be w.r.t. a NWU reference frame, YAW must be in [-180° +180°], RADIUS is absolute."
    return 1
  fi

  local yaw_rad
  yaw_rad="$(degrad "$4")"

  ros2 action send_goal -f \
    "$NAMESPACE"/flight_stack/flight_control/reach \
    dua_interfaces/action/Reach \
      "{ \
        target_pose: { \
          header: {frame_id: map}, \
          pose: { \
            position: {x: $1, y: $2, z: $3}, \
            orientation: { \
              w: $(python3 -c "import math; print(math.cos($yaw_rad/2.0))"), \
              x: 0.0, \
              y: 0.0, \
              z: $(python3 -c "import math; print(math.sin($yaw_rad/2.0))") \
            } \
          } \
       },
       reach_radius: $5, \
       stop_at_target: true \
      }"
}

# Performs a turn to the desired heading
function turn {
  # Check input arguments
  if [[ $# -ne 1 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    turn YAW"
    echo >&2 "YAW must be in [-180° +180°]."
    return 1
  fi

  ros2 action send_goal -f "$NAMESPACE"/flight_stack/flight_control/turn dua_interfaces/action/Turn "{header: {frame_id: map}, heading: $(degrad "$1")}"
}

# Requests a navigation to reach a target position
function navigate {
  # Check input arguments
  if [[ $# -ne 3 && $# -ne 4 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    navigate X Y Z [YAW]"
    echo >&2 "XYZ must be w.r.t. WORLD reference frame"
    return 1
  fi

  local yaw_rad
  if [[ $# -eq 4 ]]; then
    yaw_rad="$(degrad "$4")"
  else
    yaw_rad=6.28
  fi

  ros2 action send_goal -f "$NAMESPACE"/navigation_stack/navigator/navigate dua_interfaces/action/Navigate "{header: {frame_id: map}, target: {x: $1, y: $2, z: $3}, heading: $yaw_rad}"
}

# Requests an exploration of the environment
function explore {
  # Check input arguments
  if [[ $# -ne 1 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    explore true/false"
    return 1
  fi
  ros2 action send_goal -f "$NAMESPACE"/navigation_stack/explorer/explore dua_interfaces/action/Explore "{first: $1}"
}

# Send a go point for the exploration
function go_point {
  # Check input arguments
  if [[ $# -ne 4 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    go_point X Y Z YAW"
    return 1
  fi

  local yaw_rad
  yaw_rad="$(degrad "$4")"

  ros2 topic pub --once \
    "$NAMESPACE"/navigation_stack/explorer/go_point \
    geometry_msgs/msg/PoseStamped \
      "{ \
        header: {frame_id: map}, \
        pose: { \
          position: {x: $1, y: $2, z: $3}, \
          orientation: { \
            w: $(python3 -c "import math; print(math.cos($yaw_rad/2.0))"), \
            x: 0.0, \
            y: 0.0, \
            z: $(python3 -c "import math; print(math.sin($yaw_rad/2.0))") \
          } \
        } \
      }"
}

# Requests a tracking
function track {
  # Check input arguments
  if [[ $# -ne 3 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    track ID STOP SIDE"
    return 1
  fi
  ros2 action send_goal -f "$NAMESPACE"/navigation_stack/tracker/track dua_interfaces/action/Track "{target_id: $1, stop_when_centered: $2, start_side: $3}"
}

# Requests a tracking
function aruco_track {
  # Check input arguments
  if [[ $# -ne 3 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    aruco_track ID STOP SIDE"
    return 1
  fi
  ros2 action send_goal -f "$NAMESPACE"/navigation_stack/aruco_tracker/track dua_interfaces/action/Track "{target_id: $1, stop_when_centered: $2, start_side: $3}"
}

# Requests a tracking
function object_track {
  # Check input arguments
  if [[ $# -ne 3 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    object_track ID STOP SIDE"
    return 1
  fi
  ros2 action send_goal -f "$NAMESPACE"/navigation_stack/object_tracker/track dua_interfaces/action/Track "{target_id: $1, stop_when_centered: $2, start_side: $3}"
}

# Requests a tracking
function collimate {
  # Check input arguments
  if [[ $# -ne 4 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    collimate ALIGN LAND ALTITUDE MINIMUMS"
    return 1
  fi
  ros2 action send_goal -f "$NAMESPACE"/collimator/precision_landing dua_interfaces/action/PrecisionLanding "{align: $1, land: $2, altitude: $3, minimums: $4}"
}

function visual_targets {

  if [[ $# -ne 5 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    taregts AGENT ID X Y Z"
    return 1
  fi

  ros2 topic pub --once /visual_targets dua_interfaces/msg/VisualTargets "{
    targets: {
      header: {
        stamp: {sec: 0, nanosec: 0},
        frame_id: '$1'
      },
      detections: [
        {
          header: {
            stamp: {sec: 0, nanosec: 0},
            frame_id: 'map'
          },
          results: [
            {
              hypothesis: {
                class_id: '$2',
                score: 0.95
              },
              pose: {
                pose: {
                  position: { x: $3, y: $4, z: $5 },
                  orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
                }
              }
            }
          ],
          bbox: {
            center: {
              position: { x: 5.0, y: 5.0 },
              theta: 0.0
            },
            size_x: 2.0,
            size_y: 2.0
          },
          id: '$1'
        }
      ]
    },
    image: {
      header: {
        stamp: {sec: 0, nanosec: 0},
        frame_id: 'map'
      },
      height: 100,
      width: 100,
      encoding: 'bgr8',
      is_bigendian: 0,
      step: 300,
      data: [$(python3 -c 'print(", ".join(["0"]*(100*100*3)))')]
    }
  }"
}
