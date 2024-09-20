"""
Axis Camera driver node implementation.

Lorenzo Bianchi <lnz.bnc@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

October 29, 2023
"""

import time
from math import pi
import dua_qos_py.dua_qos_besteffort as dua_qos_besteffort
import dua_qos_py.dua_qos_reliable as dua_qos_reliable
import requests
from imutils.video import VideoStream
from requests.auth import HTTPDigestAuth
from scipy.spatial.transform import Rotation

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tf2_ros import TransformBroadcaster

from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped

from axis_camera_interfaces.msg import ImagePTZF, PTZF


class AxisCameraNode(Node):
    """
     Axis Caemra driver ROS 2 node implementation.
    """

    def __init__(self):
        """
        Initialize the node.
        """
        super().__init__('axis_camera')

        self.get_namespace()

        self.init_parameters()
        self.init_publishers()
        self.init_timers()
        self.init_subscribers()
        self.init_tf2()
        self.init_video_stream()

        # Initialize variables
        self.pan = 0.0
        self.tilt = 0.0
        self.zoom = 0.0
        self.focus = 0.0

        self.get_logger().info('Node initialized')

    def init_parameters(self):
        """
        Init parameters
        """
        self.declare_parameters(
            namespace='',
            parameters=[('ip_address', '0.0.0.0'),
                        ('username', 'username'),
                        ('password', 'password'),
                        ('pan0', 0.0),
                        ('publish_split', True),
                        ('timer_period', 1.0),
                       ])

        self.ip_address = self.get_parameter('ip_address').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value
        self.pan0 = self.get_parameter('pan0').value
        self.publish_split = self.get_parameter('publish_split').value
        self.timer_period = self.get_parameter('timer_period').value

        self.get_logger().info(f'ip_address: {self.ip_address}')
        self.get_logger().info(f'username: {self.username}')
        self.get_logger().info(f'password: {self.password}')
        self.get_logger().info(f'pan0: {self.pan0}°')
        self.get_logger().info(f'publish_split: {self.publish_split}')
        self.get_logger().info(f'timer_period: {self.timer_period}')

    def init_publishers(self):
        """
        Init publishers
        """
        self.publisher_ = self.create_publisher(
            ImagePTZF,
            "~/stream_ptzf",
            dua_qos_reliable.get_image_qos())

        if self.publish_split:
            self.publisher_image = self.create_publisher(
                Image,
                "~/stream",
                dua_qos_reliable.get_image_qos())

            self.publisher_ptz = self.create_publisher(
                PTZF,
                '~/ptzf',
                dua_qos_reliable.get_image_qos())

    def init_timers(self):
        """
        Init timers
        """
        self.cgroup_timer = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(
            self.timer_period,
            self.timer_callback,
            callback_group=self.cgroup_timer)

    def init_subscribers(self):
        """
        Init subscribers
        """
        self.cgroup_sub = MutuallyExclusiveCallbackGroup()
        self.subscriber = self.create_subscription(
            PTZF,
            "~/command",
            self.command_callback,
            dua_qos_reliable.get_datum_qos(),
            callback_group=self.cgroup_sub)

    def init_tf2(self):
        """
        Init tf2
        """
        self.tf_broadcaster = TransformBroadcaster(self)

    def init_video_stream(self):
        """
        Init video stream
        """
        # initialize video stream and allow the camera sensor to warm up
        self.get_logger().info('Starting video stream...')
        self.url = f'http://{self.username}:{self.password}@{self.ip_address}//mjpg/video.mjpg'
        self.vs = VideoStream(src=self.url).start()

        time.sleep(2.0)

        self.camera_n = 1
        self.camera_url = f'http://{self.ip_address}//axis-cgi/com/ptz.cgi'

    def camera_cmd(self, *q_cmd):
        """
        Send command to camera
        """
        resp_data = {}
        q_args = {
            'camera': self.camera_n,
            'html': 'no',
            'timestamp': int(time.time())
        }
        for d in q_cmd:
            q_args.update(d)

        resp = requests.get(self.camera_url, params=q_args,
                            auth=HTTPDigestAuth(self.username, self.password), timeout=5)
        if resp.text.startswith('Error'):
            self.get_logger().error(f'{resp.text}')
        else:
            for line in resp.text.splitlines():
                (name, var) = line.split('=', 2)
                try:
                    resp_data[name.strip()] = float(var)
                except ValueError:
                    resp_data[name.strip()] = var

        return resp_data

    def command_callback(self, msg):
        """
        Callback for the command topic
        """
        cmds = {}

        if msg.pan_global:
            cmds['pan'] = msg.pan + self.pan0
        else:
            cmds['rpan'] = msg.pan

        if msg.tilt_global:
            cmds['tilt'] = msg.tilt
        else:
            cmds['rtilt'] = msg.tilt

        if msg.zoom_global:
            cmds['zoom'] = msg.zoom
        else:
            cmds['rzoom'] = msg.zoom

        if abs(msg.focus) > 0.5:
            cmds['autofocus'] = 'off'
            if msg.focus_global:
                cmds['focus'] = msg.focus
            else:
                cmds['rfocus'] = msg.focus
        else:
            cmds['autofocus'] = 'on'

        self.camera_cmd(cmds)

    def timer_callback(self):
        """
        Callback for the timer
        """
        # grab the frame from the threaded video stream
        frame = self.vs.read()

        try:
            ptzf_dict = self.camera_cmd({'query': 'position'})
        except:
            return

        self.pan = ptzf_dict['pan'] - self.pan0
        if self.pan > -(self.pan0 + 180) and self.pan < self.pan0 - 270:
            self.pan += 360
        self.tilt = ptzf_dict['tilt']
        self.zoom = ptzf_dict['zoom']
        self.focus = ptzf_dict['focus']

        # Prepare TF2 messages
        stamp = self.get_clock().now().to_msg()

        # Create pan transformation
        t_pan = TransformStamped()

        t_pan.header.stamp = stamp
        t_pan.header.frame_id = 'ptz/wall_link'
        t_pan.child_frame_id = 'ptz/pan_link'

        r_pan = Rotation.from_euler('xyz', [0, 0, -self.pan*pi/180])
        q_pan = r_pan.as_quat()

        t_pan.transform.rotation.x = q_pan[0]
        t_pan.transform.rotation.y = q_pan[1]
        t_pan.transform.rotation.z = q_pan[2]
        t_pan.transform.rotation.w = q_pan[3]

        # Create tilt transformation
        t_tilt = TransformStamped()

        t_tilt.header.stamp = stamp
        t_tilt.header.frame_id = 'ptz/pan_link'
        t_tilt.child_frame_id = 'ptz/tilt_link'

        r_tilt = Rotation.from_euler('xyz', [0, -self.tilt*pi/180, 0])
        q_tilt = r_tilt.as_quat()

        t_tilt.transform.rotation.x = q_tilt[0]
        t_tilt.transform.rotation.y = q_tilt[1]
        t_tilt.transform.rotation.z = q_tilt[2]
        t_tilt.transform.rotation.w = q_tilt[3]

        # Send transformations
        self.tf_broadcaster.sendTransform([t_pan, t_tilt])

        # Prepare PTZF message
        ptzf_msg = PTZF()
        ptzf_msg.pan = self.pan
        ptzf_msg.tilt = self.tilt
        ptzf_msg.zoom = self.zoom
        ptzf_msg.focus = self.focus

        # Prepare Image message
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.height = frame.shape[0]
        img_msg.width = frame.shape[1]

        img_msg.encoding = 'bgr8'

        cvim = frame
        if cvim.dtype.byteorder == '>':
            img_msg.is_bigendian = True
        img_msg.data.frombytes(cvim.tobytes())
        img_msg.step = len(img_msg.data) // img_msg.height

        # Publish split messages if required
        if self.publish_split:
            self.publisher_ptz.publish(ptzf_msg)
            self.publisher_image.publish(img_msg)

        # Prepare and publish ImagePTZF message
        msg = ImagePTZF()
        msg.image = img_msg
        msg.ptzf = ptzf_msg

        self.publisher_.publish(msg)
