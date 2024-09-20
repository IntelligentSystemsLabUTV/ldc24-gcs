import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, qos_profile_sensor_data

from axis_camera_interfaces.msg import ImagePTZF, PTZF
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, JointState

from threading import Thread, Lock
import time

from math import pi

class AxisCamera(Node):

    def __init__(self):
        super().__init__('axis_camera')

        self.group = MutuallyExclusiveCallbackGroup()

        self.qos_profile = QoSProfile(depth = 0)
        self.qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        self.qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        self.qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.subscriber_gazebo_image_ = self.create_subscription(Image,
                                                                 '/axis_camera/camera/image_raw',
                                                                 self.listener_callback_image,
                                                                 self.qos_profile,
                                                                 callback_group=self.group)
        self.subscriber_gazebo_image_  # prevent unused variable warning

        self.publisher_imagePTZF_ = self.create_publisher(ImagePTZF,
                                                          '/axis_camera/stream_ptzf',
                                                          self.qos_profile,
                                                          callback_group=self.group)

        self.publisher_image_ = self.create_publisher(
            Image,
            '/axis/stream',
            qos_profile_sensor_data)

        self.pan = 0.0
        self.tilt = 0.0
        self._lock = Lock()

        self.subscriber_gazebo_state_ = self.create_subscription(JointState,
                                                                 '/joint_states',
                                                                 self.listener_callback_state,
                                                                 self.qos_profile,
                                                                 callback_group=self.group)
        self.subscriber_gazebo_state_  # prevent unused variable warning

        self.subscriber_command_ = self.create_subscription(
            PTZF,
            '/axis_camera/command',
            self.listener_callback_command,
            0,
            callback_group=self.group)
        self.subscriber_command_  # prevent unused variable warning

        self.publisher_errors_ = self.create_publisher(Float64MultiArray,
                                                       '/position_controller/commands',
                                                       self.qos_profile)

        self.get_logger().info("[INFO] starting video stream...")

    def listener_callback_command(self, msg):
        with self._lock:
            curr_pan = self.pan
            curr_tilt = self.tilt

        pan = tilt = 0.0
        if msg.pan_global:
            pan = -msg.pan
        else:
            pan = curr_pan - msg.pan
        if msg.tilt_global:
            tilt = -msg.tilt
        else:
            tilt = curr_tilt - msg.tilt

        cmds = Float64MultiArray()
        cmds.data = [pan*pi/180, tilt*pi/180]

        self.publisher_errors_.publish(cmds)

    def listener_callback_image(self, msg_image):
        msg = ImagePTZF()
        msg.image = msg_image
        with self._lock:
            msg.ptzf.pan = -self.pan
            msg.ptzf.tilt = -self.tilt
            msg.ptzf.zoom = 9999.0

        self.publisher_imagePTZF_.publish(msg)
        self.publisher_image_.publish(msg_image)

    def listener_callback_state(self, msg_state):
        with self._lock:
            self.pan = msg_state.position[0]*180/pi
            self.tilt = msg_state.position[1]*180/pi


def main(args=None):
    rclpy.init(args=args)

    axis_camera = AxisCamera()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(axis_camera)

    executor.spin()

    executor.shutdown()
    axis_camera.destroy_node()

    axis_camera.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
