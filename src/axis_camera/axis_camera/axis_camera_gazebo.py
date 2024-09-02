import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, qos_profile_sensor_data

from axis_camera_interfaces.msg import ImagePT, ImageError
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, JointState

from threading import Thread, Lock
import time

from math import pi

class AxisController(Node):

    def __init__(self, axis_camera):
        super().__init__('axis_controller')

        self.group = MutuallyExclusiveCallbackGroup()

        self.qos_profile = QoSProfile(depth = 0)
        self.qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        self.qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.subscriber = self.create_subscription(
            ImageError,
            'axis_errors',
            self.listener_callback,
            0,
            callback_group=self.group)
        self.subscriber  # prevent unused variable warning

        self.axis_camera = axis_camera

        self.publisher_errors_ = self.create_publisher(Float64MultiArray,
                                                       '/velocity_controller/commands',
                                                       self.qos_profile,
                                                       callback_group=self.group)

        self.time_to_wait = 5
        self.kill_thread = False
        self.timeout_counter = 0
        self.timeout_lock = Lock()
        self.sgn = 1

    def listener_callback(self, msg):
        #joint_names = ["chassis_to_pan", "pan_to_tilt"]
        with self.timeout_lock:
            self.timeout_counter += 1

        cmds = Float64MultiArray()
        kp = 0.3
        cmds.data = [kp*msg.ex, kp*msg.ey]

        self.publisher_errors_.publish(cmds)

    def timeout(self):
        while not self.kill_thread:
            time.sleep(self.time_to_wait)
            with self.timeout_lock:
                if self.timeout_counter > 0:
                    self.timeout_counter = 0
                    self.time_to_wait = 5
                else:

                    with self.axis_camera._lock:
                        pan = self.axis_camera.pan
                        tilt = self.axis_camera.tilt

                    if pan > 20*pi/180:
                        self.sgn = -1
                    elif pan < -20*pi/180:
                        self.sgn = 1

                    tilt_des = 35*pi/180

                    cmds = Float64MultiArray()
                    cmds.data = [self.sgn*0.1, 0.3*(tilt_des-tilt)]

                    self.publisher_errors_.publish(cmds)

                    self.time_to_wait = 1


class AxisCamera(Node):

    def __init__(self):
        super().__init__('axis_camera')

        self.group = MutuallyExclusiveCallbackGroup()

        self.qos_profile = QoSProfile(depth = 0)
        self.qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        self.qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.subscriber_gazebo_image_ = self.create_subscription(Image,
                                                                 '/axis_camera/image_raw',
                                                                 self.listener_callback_image,
                                                                 self.qos_profile,
                                                                 callback_group=self.group)
        self.subscriber_gazebo_image_  # prevent unused variable warning

        self.publisher_imagePT_ = self.create_publisher(ImagePT,
                                                      'axis_image',
                                                      self.qos_profile,
                                                      callback_group=self.group)

        self.publisher_image_ = self.create_publisher(
            Image,
            '/axis/image_color',
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

        self.get_logger().info("[INFO] starting video stream...")

    def listener_callback_image(self, msg_image):
        msg = ImagePT()

        msg.image = msg_image
        with self._lock:
            msg.pan = self.pan
            msg.tilt = self.tilt

        self.publisher_imagePT_.publish(msg)

        self.publisher_image_.publish(msg_image)

    def listener_callback_state(self, msg_state):
        with self._lock:
            self.pan = msg_state.position[0]
            self.tilt = msg_state.position[1]


def main(args=None):
    rclpy.init(args=args)

    axis_camera = AxisCamera()
    axis_controller = AxisController(axis_camera)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(axis_camera)
    executor.add_node(axis_controller)

    timeout_thread = Thread(target=axis_controller.timeout)
    timeout_thread.start()

    executor.spin()

    executor.shutdown()
    axis_camera.destroy_node()
    axis_controller.destroy_node()

    axis_camera.destroy_node()

    axis_controller.kill_thread = True
    timeout_thread.join()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
