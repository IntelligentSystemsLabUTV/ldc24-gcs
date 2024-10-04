"""
GCS FSM ROS 2 node implementation.

Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

September 28, 2024
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

import json

import cv2 as cv
from cv_bridge import CvBridge

import rclpy
import rclpy.time
from rclpy.duration import Duration
from rclpy.node import Node

from dua_interfaces.msg import VisualTargets
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, TransformStamped, Vector3
from rcl_interfaces.msg import Parameter, ParameterDescriptor, ParameterType
from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Header, String
from vision_msgs.msg import Detection2D, Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from dua_interfaces.srv import TransformPose, GetTransform, SafeLanding
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import SetBool, Trigger

from dua_interfaces.action import Reach, Landing, Navigate, PrecisionLanding

import dua_qos_py.dua_qos_reliable as dua_qos

from simple_serviceclient_py.simple_serviceclient import Client as ServiceClient
from simple_actionclient_py.simple_actionclient import Client as ActionClient


class GCSFSMNode(Node):
    """
    GCS FSM ROS 2 node implementation.
    """

    TARGET_COLORS = [
        ("Red", (1.0, 0.0, 0.0)),        # Pure Red
        ("Green", (0.0, 1.0, 0.0)),      # Pure Green
        ("Blue", (0.0, 0.0, 1.0)),       # Pure Blue
        ("Yellow", (1.0, 1.0, 0.0)),     # Bright Yellow
        ("Cyan", (0.0, 1.0, 1.0)),       # Cyan
        ("Magenta", (1.0, 0.0, 1.0)),    # Magenta
        ("Orange", (1.0, 0.5, 0.0)),     # Orange
        ("Purple", (0.5, 0.0, 0.5)),     # Dark Purple
        ("Pink", (1.0, 0.75, 0.8)),      # Light Pink
        ("Lime", (0.75, 1.0, 0.0))       # Lime Green
    ]


    COCO_CLASSES = ['airplane', 'apple', 'backpack', 'banana', 'baseball bat', 'baseball glove',
                    'bear', 'bed', 'bench', 'bicycle', 'bird', 'boat', 'book', 'bottle', 'bowl',
                    'broccoli', 'bus', 'cake', 'car', 'carrot', 'cat', 'cell phone', 'chair',
                    'clock', 'couch', 'cow', 'cup', 'dining table', 'dog', 'donut', 'elephant',
                    'fire hydrant', 'fork', 'frisbee', 'giraffe', 'hair drier', 'handbag', 'horse',
                    'hot dog', 'keyboard', 'kite', 'knife', 'laptop', 'microwave', 'motorcycle',
                    'mouse', 'orange', 'oven', 'parking meter', 'person', 'pizza', 'potted plant',
                    'refrigerator', 'remote', 'sandwich', 'scissors', 'sheep', 'sink', 'skateboard',
                    'skis', 'snowboard', 'spoon', 'sports ball', 'stop sign', 'suitcase',
                    'surfboard', 'teddy bear', 'tennis racket', 'tie', 'toaster', 'toilet',
                    'toothbrush', 'traffic light', 'train', 'truck', 'tv', 'umbrella', 'vase',
                    'wine glass', 'zebra']

    def __init__(self) -> None:
        """
        Initialize the node.
        """
        super().__init__('fsm')

        # Set internal data
        self.rtb = False
        self.emergency_landing = False
        self.followme = False
        self.followme_done = False
        self.valid_ids = []
        self.rtb_id = ''
        self.emergency_landing_id = ''
        self.followme_start_time = 0.0
        self.followme_waypoints = []
        self._color_counter = 0
        self._log_file = None
        self._start_time = 0
        self._markers = [
            Marker(
                header=Header(
                    frame_id='map',
                    stamp=self.get_clock().now().to_msg()
                ),
                action=Marker.DELETEALL
            )
        ]
        self._cv_bridge = CvBridge()

        # Initialize ROS 2 entities
        self._init_parameters()
        self._init_publishers()
        self._init_service_clients()
        self._init_action_clients()
        self._init_subscribers()

        self.get_logger().info('Node initialized')

    @property
    def rtb(self) -> bool:
        return self._rtb

    @rtb.setter
    def rtb(self, value: bool) -> None:
        if not isinstance(value, bool):
            raise TypeError('rtb must be a boolean')
        self._rtb = value

    @property
    def emergency_landing(self) -> bool:
        return self._emergency_landing

    @emergency_landing.setter
    def emergency_landing(self, value: bool) -> None:
        if not isinstance(value, bool):
            raise TypeError('emergency_landing must be a boolean')
        self._emergency_landing = value

    @property
    def followme(self) -> bool:
        return self._followme

    @followme.setter
    def followme(self, value: bool) -> None:
        if not isinstance(value, bool):
            raise TypeError('followme must be a boolean')
        self._followme = value

    @property
    def followme_done(self) -> bool:
        return self._followme_done

    @followme_done.setter
    def followme_done(self, value: bool) -> None:
        if not isinstance(value, bool):
            raise TypeError('followme_done must be a boolean')
        self._followme_done = value

    @property
    def valid_ids(self) -> list:
        return self._valid_ids

    @valid_ids.setter
    def valid_ids(self, value: list) -> None:
        if not isinstance(value, list):
            raise TypeError('valid_ids must be a list')
        self._valid_ids = value

    @property
    def rtb_id(self) -> str:
        return self._rtb_id

    @rtb_id.setter
    def rtb_id(self, value: str) -> None:
        if not isinstance(value, str):
            raise TypeError('rtb_id must be a string')
        self._rtb_id = value

    @property
    def emergency_landing_id(self) -> str:
        return self._emergency_landing_id

    @emergency_landing_id.setter
    def emergency_landing_id(self, value: str) -> None:
        if not isinstance(value, str):
            raise TypeError('emergency_landing_id must be a string')
        self._emergency_landing_id = value

    @property
    def followme_start_time(self) -> float:
        return self._followme_start_time

    @followme_start_time.setter
    def followme_start_time(self, value: float) -> None:
        if not isinstance(value, float):
            raise TypeError('followme_start_time must be a float')
        self._followme_start_time = value

    @property
    def followme_waypoints(self) -> list:
        return self._followme_waypoints

    @followme_waypoints.setter
    def followme_waypoints(self, value: list) -> None:
        if not isinstance(value, list):
            raise TypeError('followme_waypoints must be a list')
        self._followme_waypoints = value

    @property
    def event_polling_period(self) -> float:
        return self._event_polling_period

    @event_polling_period.setter
    def event_polling_period(self, value: float) -> None:
        if not isinstance(value, float):
            raise TypeError('event_polling_period must be a float')
        self._event_polling_period = value

    @property
    def landing_altitude(self) -> float:
        return self._landing_altitude

    @landing_altitude.setter
    def landing_altitude(self, value: float) -> None:
        if not isinstance(value, float):
            raise TypeError('landing_altitude must be a float')
        self._landing_altitude = value

    @property
    def tf_timeout(self) -> Duration:
        return self._tf_timeout

    @tf_timeout.setter
    def tf_timeout(self, value: float) -> None:
        if not isinstance(value, float):
            raise TypeError('tf_timeout must be a float')
        self._tf_timeout = Duration(nanoseconds=value * 1000000.0)

    @property
    def wait_servers(self) -> bool:
        return self._wait_servers

    @wait_servers.setter
    def wait_servers(self, value: bool) -> None:
        if not isinstance(value, bool):
            raise TypeError('wait_servers must be a bool')
        self._wait_servers = value

    def _init_parameters(self) -> None:
        """
        Initialize node parameters.
        """
        # do_emergency_landing
        do_emergency_landing_descriptor = ParameterDescriptor(
            name='do_emergency_landing',
            type=ParameterType.PARAMETER_BOOL,
            description='Triggers emergency landing.',
            read_only=False,
            dynamic_typing=False)
        self.declare_parameter(
            'do_emergency_landing',
            False,
            do_emergency_landing_descriptor)

        # event_polling_period
        event_polling_descriptor = ParameterDescriptor(
            name='event_polling_period',
            type=ParameterType.PARAMETER_DOUBLE,
            description='Event polling period [s].',
            read_only=True,
            dynamic_typing=False,
            floating_point_range=[FloatingPointRange(
                from_value=0.0,
                to_value=1.0,
                step=0.0)])
        self.declare_parameter(
            'event_polling_period',
            0.2,
            event_polling_descriptor)
        self.event_polling_period = self.get_parameter(
            'event_polling_period').get_parameter_value().double_value
        self.get_logger().info(
            f'Event polling period: {self.event_polling_period} s')

        # followme_kp_yaw
        followme_kp_yaw_descriptor = ParameterDescriptor(
            name='followme_kp_yaw',
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            description='FollowMe DottorCane yaw PID controller gains (1: normal, 2: followme).',
            read_only=False,
            dynamic_typing=False)
        self.declare_parameter(
            'followme_kp_yaw',
            [2.0, 2.0],
            followme_kp_yaw_descriptor)

        # followme_reach_radius
        followme_reach_radius_descriptor = ParameterDescriptor(
            name='followme_reach_radius',
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            description='FollowMe DottorCane reach radius (1: normal, 2: followme).',
            read_only=False,
            dynamic_typing=False)
        self.declare_parameter(
            'followme_reach_radius',
            [1.0, 1.0],
            followme_reach_radius_descriptor)

        # landing_altitude
        landing_altitude_descriptor = ParameterDescriptor(
            name='landing_altitude',
            type=ParameterType.PARAMETER_DOUBLE,
            description='Landing altitude [m].',
            read_only=True,
            dynamic_typing=False)
        self.declare_parameter(
            'landing_altitude',
            1.0,
            landing_altitude_descriptor)
        self.landing_altitude = self.get_parameter(
            'landing_altitude').get_parameter_value().double_value

        # tf_timeout
        tf_timeout_descriptor = ParameterDescriptor(
            name='tf_timeout',
            type=ParameterType.PARAMETER_DOUBLE,
            description='TF lookup timeout [ms].',
            read_only=True,
            dynamic_typing=False)
        self.declare_parameter(
            'tf_timeout',
            500.0,
            tf_timeout_descriptor)
        tf_timeout_ms = self.get_parameter(
            'tf_timeout').get_parameter_value().double_value
        self.tf_timeout = tf_timeout_ms
        self.get_logger().info(f'TF lookup timeout: {tf_timeout_ms} ms')

        # wait_servers
        wait_servers_descriptor = ParameterDescriptor(
            name='wait_servers',
            type=ParameterType.PARAMETER_BOOL,
            description='Wait for servers to be ready.',
            read_only=True,
            dynamic_typing=False)
        self.declare_parameter(
            'wait_servers',
            False,
            wait_servers_descriptor)
        self.wait_servers = self.get_parameter(
            'wait_servers').get_parameter_value().bool_value
        self.get_logger().info(f'Wait for servers: {self.wait_servers}')

    def _init_publishers(self) -> None:
        """
        Initialize topic publishers.
        """
        # mission_data
        self.mission_data_pub = self.create_publisher(
            String,
            '/mission_data',
            dua_qos.get_datum_qos())

        # gcs/fsm_state
        self._fsm_state_pub = self.create_publisher(
            String,
            '/gcs/fsm_state',
            dua_qos.get_datum_qos())

        # hmi/markers
        self.markers_pub = self.create_publisher(
            MarkerArray,
            '/hmi/markers',
            dua_qos.get_datum_qos())

        # hmi/pictures
        self.pictures_pub = self.create_publisher(
            Image,
            '/hmi/pictures',
            dua_qos.get_image_qos())

    def _init_service_clients(self) -> None:
        """
        Initialize service clients.
        """
        # arianna/navigation_stack/navigator/safe_landing
        self.arianna_safe_landing_client = ServiceClient(
            self,
            SafeLanding,
            '/arianna/navigation_stack/navigator/safe_landing',
            self.wait_servers)

        # arianna/emergency_landing
        self.arianna_emergency_landing_client = ServiceClient(
            self,
            Trigger,
            '/arianna/emergency_landing',
            self.wait_servers)

        # arianna/rtb
        self.arianna_rtb_client = ServiceClient(
            self,
            SetBool,
            '/arianna/rtb',
            self.wait_servers)

        # arianna/followme
        self.arianna_followme_client = ServiceClient(
            self,
            SetBool,
            '/arianna/followme',
            self.wait_servers)

        # dottorcane/rtb
        self.dottorcane_rtb_client = ServiceClient(
            self,
            SetBool,
            '/dottorcane/rtb',
            self.wait_servers)

        # dottorcane/followme
        self.dottorcane_followme_client = ServiceClient(
            self,
            SetBool,
            '/dottorcane/followme',
            self.wait_servers)

        # dottorcane/navigation_stack/navigator/set_parameters
        self._dottorcane_navigator_set_parameters_client = ServiceClient(
            self,
            SetParameters,
            '/dottorcane/navigation_stack/navigator/set_parameters',
            self.wait_servers)

        # dottorcane/position_controller/set_parameters
        self._dottorcane_pos_controller_set_parameters_client = ServiceClient(
            self,
            SetParameters,
            '/dottorcane/position_controller/set_parameters',
            self.wait_servers)

        # dottorcane/navigation_stack/navigator/transform_pose
        self.transform_pose_client = ServiceClient(
            self,
            TransformPose,
            '/dottorcane/navigation_stack/navigator/transform_pose',
            self.wait_servers)

        # dottorcane/navigation_stack/navigator/get_transform
        self.get_transform_client = ServiceClient(
            self,
            GetTransform,
            '/dottorcane/navigation_stack/navigator/get_transform',
            self.wait_servers)

    def _init_action_clients(self) -> None:
        """
        Initialize action clients.
        """
        # arianna/flight_stack/flight_control/landing
        self.arianna_landing_client = ActionClient(
            self,
            Landing,
            '/arianna/flight_stack/flight_control/landing',
            None,
            self.wait_servers)

        # arianna/flight_stack/flight_control/reach
        self.arianna_reach_client = ActionClient(
            self,
            Reach,
            '/arianna/flight_stack/flight_control/reach',
            None,
            self.wait_servers)

        # arianna/navigation_stack/navigator/navigate
        self.arianna_navigate_client = ActionClient(
            self,
            Navigate,
            '/arianna/navigation_stack/navigator/navigate',
            None,
            self.wait_servers)

        # arianna/collimator/collimate
        self.arianna_collimate_client = ActionClient(
            self,
            PrecisionLanding,
            '/arianna/collimator/collimate',
            None,
            self.wait_servers)

        # dottorcane/navigation_stack/navigator/navigate
        self.dottorcane_navigate_client = ActionClient(
            self,
            Navigate,
            '/dottorcane/navigation_stack/navigator/navigate',
            None,
            self.wait_servers)

    def _init_subscribers(self) -> None:
        """
        Initialize topic subscribers.
        """
        # hmi/found_targets/visual
        self._found_targets_visual_sub = self.create_subscription(
            VisualTargets,
            '/hmi/found_targets/visual',
            self._found_targets_visual_callback,
            dua_qos.get_datum_qos())

    def _found_targets_visual_callback(self, msg: VisualTargets) -> None:
        """
        Process notifications of targets found by the agents.

        :param msg: Message to parse.
        """
        # Log target data
        self.log_target(msg)

        # Check what kind of target was found
        target_id: str = msg.targets.detections[0].results[0].hypothesis.class_id
        if target_id in self.valid_ids:
            self.valid_ids.remove(target_id)
        elif target_id == self.rtb_id:
            self.rtb = True
        elif target_id == self.emergency_landing_id:
            self.emergency_landing = True
        else:
            # Should never happen, but let us check
            self.get_logger().error(f'Unknown target ID: {target_id}')

    def update_fsm_state(self, state: str) -> None:
        """
        Update the FSM state.

        :param state: New FSM state.
        """
        self.get_logger().warn(state)
        if state == 'EXPLORE':
            self.exploring = True
        else:
            self.exploring = False
        for i in range(10):
            self._fsm_state_pub.publish(String(data=state))

    def wait_spinning(self) -> None:
        """
        Processes background jobs while waiting for events.

        :raises RuntimeError: If the ROS 2 context is unavailable.
        """
        # Do a round of spin
        executor = rclpy.get_global_executor()
        if not executor._context.ok() or executor._is_shutdown:
            raise RuntimeError('wait_spinning: context unavailable')
        executor.add_node(self)
        try:
            executor.spin_once(timeout_sec=self.event_polling_period)
        finally:
            executor.remove_node(self)

        # Check FollowMe time
        if not self.followme and not self.followme_done:
            curr_time = self.get_clock().now()
            elapsed_time: Duration = curr_time - self._start_time
            if float(elapsed_time.nanoseconds) / float(1e6) >= (self.followme_start_time * 1000.0):
                self.followme = True

    def get_agent_pose(self, agent: str) -> PoseStamped:
        """
        Returns current agent pose in global frame.

        :param agent: Agent name.
        :return: Current pose.
        """
        tf_req: GetTransform.Request = GetTransform.Request(
            frame='map',
            child_frame= agent + '/base_link',
            time=self.get_clock().now().to_msg()
        )
        tf_resp: GetTransform.Response = self.get_transform_client.call_sync(tf_req)
        transform: TransformStamped = tf_resp.transform

        curr_pose = PoseStamped(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id='map'),
            pose=Pose(
                position=Point(
                    x=transform.transform.translation.x,
                    y=transform.transform.translation.y,
                    z=transform.transform.translation.z),
                orientation=transform.transform.rotation))
        return curr_pose

    def get_target_pose(self, target_data: Detection2D) -> PoseStamped:
        """
        Returns the pose of a target in the global frame.

        :param target_data: Target data.
        :return: Target pose.
        """
        tgt_frame = target_data.header.frame_id
        tgt_pose: Pose = target_data.results[0].pose.pose
        # self.get_logger().info(f'Target frame: {tgt_frame}, x: {tgt_pose.position.x}, y: {tgt_pose.position.y}, z: {tgt_pose.position.z}')
        tgt_pose_stamped = PoseStamped(
            header=Header(
                stamp=target_data.header.stamp,
                frame_id=tgt_frame
            ),
            pose=tgt_pose
        )
        req: TransformPose.Request = TransformPose.Request(
            pose=tgt_pose_stamped,
            target_frame='map'
        )
        res: TransformPose.Response = self.transform_pose_client.call_sync(req)
        # self.get_logger().info(f'Target transformed to {res.transformed_pose.header.frame_id}: x: {res.transformed_pose.pose.position.x}, y: {res.transformed_pose.pose.position.y}, z: {res.transformed_pose.pose.position.z}')
        return res.transformed_pose

    def set_followme_parameters(self, pre: bool) -> bool:
        """
        Sets relevant parameters for FollowMe, before or after.

        :param param: Pre/post FollowMe.
        :return: True if the service call succeeded, False otherwise.
        """
        # Get parameters values to set
        kp_yaws = self.get_parameter(
            'followme_kp_yaw').get_parameter_value().double_array_value
        reach_radii = self.get_parameter(
            'followme_reach_radius').get_parameter_value().double_array_value
        if pre:
            kp_yaw = kp_yaws[1]
            reach_radius = reach_radii[1]
        else:
            kp_yaw = kp_yaws[0]
            reach_radius = reach_radii[0]

        # Update yaw PID controller gain
        params_msg = SetParameters.Request()
        param_msg = Parameter()
        param_msg.name = 'ctrl_angular_err_gain'
        param_msg.value.type = ParameterType.PARAMETER_DOUBLE
        param_msg.value.double_value = kp_yaw
        params_msg.parameters.append(param_msg)
        res1: SetParametersResult = self._dottorcane_pos_controller_set_parameters_client.call_sync(
            params_msg).results[0]
        if not res1.successful:
            self.get_logger().error(
                f'DottorCane kp_yaw update failed: {res1.reason}')
        else:
            self.get_logger().info(f'DottorCane kp_yaw set to {kp_yaw}')

        # Update reach radius
        params_msg = SetParameters.Request()
        param_msg = Parameter()
        param_msg.name = 'reach_radius'
        param_msg.value.type = ParameterType.PARAMETER_DOUBLE
        param_msg.value.double_value = reach_radius
        params_msg.parameters.append(param_msg)
        res2: SetParametersResult = self._dottorcane_navigator_set_parameters_client.call_sync(
            params_msg).results[0]
        if not res2.successful:
            self.get_logger().error(
                f'DottorCane reach_radius update failed: {res2.reason}')
        else:
            self.get_logger().info(
                f'DottorCane reach_radius set to {reach_radius}')

        return res1.successful and res2.successful

    def open_log(self, manche: int) -> None:
        """
        Opens the log file.

        :param manche: Manche number.
        """
        self._log_file = open('/home/neo/workspace/logs/mission_log.txt', 'w')
        self._log_file.write(f'--- UNIVERSITY OF ROME TOR VERGATA | MANCHE #{manche} ---\n\n')
        self._start_time = self.get_clock().now()

    def close_log(self) -> None:
        """
        Closes the log file.
        """
        self.log("--- END OF MISSION ---")
        self._log_file.close()

    def log(self, msg: str) -> None:
        """
        Logs a message to the log file.

        :param msg: Message to log.
        """
        elapsed_time: Duration = self.get_clock().now() - self._start_time
        log_str = f'[{elapsed_time.to_msg().sec}]: {msg}\n'
        self._log_file.write(log_str)

    def log_target(self, target_data: VisualTargets) -> None:
        """
        Logs target data locally and to HMI.

        :param target_data: Target data.
        """
        color = self.TARGET_COLORS[self._color_counter]
        self._color_counter = (self._color_counter + 1) % len(self.TARGET_COLORS)
        agent_frame_id = target_data.targets.header.frame_id
        agent = 'UAV' if agent_frame_id.find('arianna') != -1 else 'UGV'
        target_picture = target_data.image
        target_position: Point = target_data.targets.detections[0].results[0].pose.pose.position
        target_id_orig: str = target_data.targets.detections[0].results[0].hypothesis.class_id
        target_id = target_id_orig.replace(' ', '_')

        # Log target data to file
        self.log(f'Target ({target_id}, {color[0]}) found by {agent} at ({target_position.x}, {target_position.y}, {target_position.z})')

        # Save new marker
        marker_scale = 0.3
        marker_msg = Marker()
        marker_msg.header.frame_id = 'map'
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD
        marker_msg.pose.position = target_position
        marker_msg.scale = Vector3(x=marker_scale, y=marker_scale, z=marker_scale)
        marker_msg.color.r = color[1][0]
        marker_msg.color.g = color[1][1]
        marker_msg.color.b = color[1][2]
        marker_msg.color.a = 1.0
        self._markers.append(marker_msg)

        # Publish target data to HMI
        marker_array_msg = MarkerArray()
        marker_array_msg.markers = self._markers
        self.markers_pub.publish(marker_array_msg)

        # Publish target picture to HMI
        self.pictures_pub.publish(target_picture)

        # Save target picture
        cv_image = self._cv_bridge.imgmsg_to_cv2(target_picture, 'bgr8')
        cv.imwrite(f'/home/neo/workspace/logs/target_{target_id}_{agent}.png', cv_image)

        self.get_logger().warn(f'{target_id_orig} found by {agent} at ({target_position.x}, {target_position.y}, {target_position.z})')
