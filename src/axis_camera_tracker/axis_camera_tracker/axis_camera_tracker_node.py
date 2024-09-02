import dua_qos_py.dua_qos_besteffort
import dua_qos_py.dua_qos_reliable
import cv2, yaml, time
import numpy as np
from math import pi, atan
from scipy.spatial.transform import Rotation
from threading import Lock

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

import tf2_ros
from tf2_ros import TransformException

from cv_bridge import CvBridge

from axis_camera_interfaces.msg import ImagePTZF, PTZF
from sensor_msgs.msg import Image
from dua_interfaces.msg import Target, TargetIDArray, VisualTargets

import dua_qos_py

from std_srvs.srv import Trigger

from geometry_msgs.msg import Point


class AxisCameraTrackerNode(Node):
    """
    Axis Camera Tracker ROS 2 node implementation.
    """

    def __init__(self):
        super().__init__('axis_camera_tracker')

        self.init_parameters()
        self.init_calibration()
        self.init_publishers()
        self.init_subscribers()
        self.init_services()
        self.init_arucos()
        self.init_states()
        self.init_locks()

        # Initialize other variables
        self.bridge = CvBridge()
        self.found_arucos = {}
        self.sgn = 1
        self.current_tilt = self.tilt_des_up
        self.counter = 0
        self.pan = 0.0
        self.tilt = 0.0
        self.zoom = 0.0
        self.focus = 0.0
        self.frame = 0
        self.first_time = True
        self.t0 = 0.0
        self.special_aruco = -1
        self.last_pan = 0.0
        self.last_tilt = 0.0
        self.n_segments = 0
        self.iso_fix_cam = np.eye(4)
        self.wait_after_selfie = 10

        # Move camera to home position
        command_msg = PTZF()
        command_msg.pan_global = True
        command_msg.pan = self.pan_l
        command_msg.tilt_global = True
        command_msg.tilt = self.current_tilt
        command_msg.zoom_global = True
        command_msg.zoom = self.zoom_search
        self.command_pub.publish(command_msg)

        time.sleep(5)

        self.get_logger().info('Node initialized')

    def init_parameters(self):
        """
        Init parameters
        """
        self.declare_parameters(
            namespace='',
            parameters=[('aruco_numbers', [1]),
                        ('aruco_size', 0.1),
                        ('calibration_file_path', 'path'),
                        ('err_thr', 0.5),
                        ('err_thr_zoom', 0.5),
                        ('frame_fixed', 'fixed_frame'),
                        ('frame_camera', 'camera_link'),
                        ('frames_track_to_search', 1),
                        ('frames_wait_after_selfie', 1),
                        ('kpx', 1.0),
                        ('kpy', 1.0),
                        ('k_tilt', 1.0),
                        ('max_distance', 1.0),
                        ('n_loops', 1),
                        ('padding', 1),
                        ('pan_l', 1.0),
                        ('pan_r', 1.0),
                        ('pan_step', 0.1),
                        ('thr_sharpness', 1.0),
                        ('thr_time', 1.0),
                        ('tilt_des_down', 1.0),
                        ('tilt_des_up', 1.0),
                        ('topic_command', '/topic_command'),
                        ('topic_hmi_aruco', '/aruco'),
                        ('topic_hmi_selfie', '/selfie'),
                        ('topic_image_ptzf', '/axis_image'),
                        ('topic_image_target', '/axis_camera_tracker/image_color'),
                        ('topic_search_seppia', '/search_seppia'),
                        ('topic_search_stanis', '/search_stanis'),
                        ('topic_start', '/start'),
                        ('topic_target', '/targets'),
                        ('topic_tracker', '/topic_tracker'),
                        ('zoom_des', 1.0),
                        ('zoom_search', 1.0),
                        ('zoom_selfie', 1.0),
                        ('zoom_step_high', 1.0),
                        ('zoom_step_low', 1.0),
                       ])

        self.err_thr = self.get_parameter('err_thr').value
        self.err_thr_zoom = self.get_parameter('err_thr_zoom').value
        self.aruco_size = self.get_parameter('aruco_size').value
        self.kpx = self.get_parameter('kpx').value
        self.kpy = self.get_parameter('kpy').value
        self.topic_image_target = self.get_parameter('topic_image_target').value
        self.topic_target = self.get_parameter('topic_target').value
        self.topic_image_ptzf = self.get_parameter('topic_image_ptzf').value
        self.calibration_file_path = self.get_parameter('calibration_file_path').value
        self.pan_l = self.get_parameter('pan_l').value
        self.pan_r = self.get_parameter('pan_r').value
        self.pan_step = self.get_parameter('pan_step').value
        self.tilt_des_down = self.get_parameter('tilt_des_down').value
        self.tilt_des_up = self.get_parameter('tilt_des_up').value
        self.zoom_des = self.get_parameter('zoom_des').value
        self.zoom_search = self.get_parameter('zoom_search').value
        self.k_tilt = self.get_parameter('k_tilt').value
        self.topic_tracker = self.get_parameter('topic_tracker').value
        self.topic_command = self.get_parameter('topic_command').value
        self.topic_start = self.get_parameter('topic_start').value
        self.zoom_step_low = self.get_parameter('zoom_step_low').value
        self.zoom_step_high = self.get_parameter('zoom_step_high').value
        self.zoom_selfie = self.get_parameter('zoom_selfie').value
        self.topic_hmi_selfie = self.get_parameter('topic_hmi_selfie').value
        self.frames_track_to_search = self.get_parameter('frames_track_to_search').value
        self.frames_wait_after_selfie = self.get_parameter('frames_wait_after_selfie').value
        self.thr_sharpness = self.get_parameter('thr_sharpness').value
        self.thr_time = self.get_parameter('thr_time').value
        self.padding = self.get_parameter('padding').value
        self.topic_hmi_aruco = self.get_parameter('topic_hmi_aruco').value
        self.frame_fixed = self.get_parameter('frame_fixed').value
        self.frame_camera = self.get_parameter('frame_camera').value
        self.n_loops = self.get_parameter('n_loops').value
        self.max_distance = self.get_parameter('max_distance').value
        self.topic_search_seppia = self.get_parameter('topic_search_seppia').value
        self.topic_search_stanis = self.get_parameter('topic_search_stanis').value

        self.get_logger().info(f'aruco_size: {self.aruco_size}')
        self.get_logger().info(f'calibration_file_path: {self.calibration_file_path}')
        self.get_logger().info(f'err_thr: {self.err_thr}')
        self.get_logger().info(f'err_thr_zoom: {self.err_thr_zoom}')
        self.get_logger().info(f'frame_camera: {self.frame_camera}')
        self.get_logger().info(f'frame_fixed: {self.frame_fixed}')
        self.get_logger().info(f'frames_track_to_search: {self.frames_track_to_search}')
        self.get_logger().info(f'frames_wait_after_selfie: {self.frames_wait_after_selfie}')
        self.get_logger().info(f'k_tilt: {self.k_tilt}')
        self.get_logger().info(f'kpx: {self.kpx}')
        self.get_logger().info(f'kpy: {self.kpy}')
        self.get_logger().info(f'max_distance: {self.max_distance}')
        self.get_logger().info(f'n_loops: {self.n_loops}')
        self.get_logger().info(f'padding: {self.padding} px')
        self.get_logger().info(f'pan_l: {self.pan_l}°')
        self.get_logger().info(f'pan_r: {self.pan_r}°')
        self.get_logger().info(f'pan_step: {self.pan_step}°')
        self.get_logger().info(f'thr_sharpness: {self.thr_sharpness}')
        self.get_logger().info(f'thr_time: {self.thr_time} s')
        self.get_logger().info(f'tilt_des_down: {self.tilt_des_down}°')
        self.get_logger().info(f'tilt_des_up: {self.tilt_des_up}°')
        self.get_logger().info(f'topic_command: {self.topic_command}')
        self.get_logger().info(f'topic_hmi_aruco: {self.topic_hmi_aruco}')
        self.get_logger().info(f'topic_hmi_selfie: {self.topic_hmi_selfie}')
        self.get_logger().info(f'topic_image_ptzf: {self.topic_image_ptzf}')
        self.get_logger().info(f'topic_image_target: {self.topic_image_target}')
        self.get_logger().info(f'topic_search_seppia: {self.topic_search_seppia}')
        self.get_logger().info(f'topic_search_stanis: {self.topic_search_stanis}')
        self.get_logger().info(f'topic_start: {self.topic_start}')
        self.get_logger().info(f'topic_target: {self.topic_target}')
        self.get_logger().info(f'topic_tracker: {self.topic_tracker}')
        self.get_logger().info(f'zoom_des: {self.zoom_des}')
        self.get_logger().info(f'zoom_search: {self.zoom_search}')
        self.get_logger().info(f'zoom_selfie: {self.zoom_selfie}')
        self.get_logger().info(f'zoom_step_high: {self.zoom_step_high}')
        self.get_logger().info(f'zoom_step_low: {self.zoom_step_low}')

    def init_calibration(self):
        """
        Init calibration matrix and vector
        """
        # Get matrices from calibration file
        with open(self.calibration_file_path, 'r') as file:
            data = yaml.safe_load(file)
        self.camera_matrix = np.array(data['camera_matrix']['data'], dtype=np.float32).reshape((3,3))
        self.dist_coeffs = np.array(data['distortion_coefficients']['data'], dtype=np.float32)

        # Set points for PnP function
        self.objPoints = np.array([
            [-self.aruco_size/2,  self.aruco_size/2, 0.0],
            [ self.aruco_size/2,  self.aruco_size/2, 0.0],
            [ self.aruco_size/2, -self.aruco_size/2, 0.0],
            [-self.aruco_size/2, -self.aruco_size/2, 0.0]
        ])

    def init_publishers(self):
        """
        Init publishers
        """
        self.target_img_pub = self.create_publisher(
            Image,
            self.topic_image_target,
            dua_qos_py.dua_qos_besteffort())

        self.hmi_image_pub = self.create_publisher(
            Image,
            self.topic_hmi_selfie,
            dua_qos_py.dua_qos_besteffort())

        self.publisher_target = self.create_publisher(
            Target,
            self.topic_target,
            dua_qos_py.dua_qos_reliable())

        self.publisher_target_seppia = self.create_publisher(
            Point,
            self.topic_search_seppia,
            dua_qos_py.dua_qos_reliable())

        self.publisher_target_stanis = self.create_publisher(
            Point,
            self.topic_search_stanis,
            dua_qos_py.dua_qos_reliable())

        self.command_pub = self.create_publisher(
            PTZF,
            self.topic_command,
            dua_qos_py.dua_qos_reliable())

        self.hmi_aruco_pub = self.create_publisher(
            VisualTargets,
            self.topic_hmi_aruco,
            dua_qos_py.dua_qos_reliable())

    def init_subscribers(self):
        """
        Init subscribers
        """
        start_sub_cgroup = MutuallyExclusiveCallbackGroup()
        self.start_sub = self.create_subscription(
            TargetIDArray,
            self.topic_start,
            self.start_callback,
            dua_qos_py.dua_qos_reliable(),
            callback_group=start_sub_cgroup)

    def init_services(self):
        """
        Init services
        """
        selfie_srv_cgroup = MutuallyExclusiveCallbackGroup()
        self.selfie_srv = self.create_service(
            Trigger,
            'selfie',
            self.selfie_callback,
            callback_group=selfie_srv_cgroup)

    def init_tf2(self):
        """
        Init tf2
        """
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer, self)

    def init_arucos(self):
        """
        Init aruco dictionary and parameters
        """
        # Load the ArUCo dictionary and grab the ArUCo parameters
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.arucoParams = cv2.aruco.DetectorParameters()

    def init_states(self):
        """
        Init states
        """
        self.SEARCH = 0
        self.TRACK = 1
        self.SELFIE = 2
        self.status = self.SEARCH

    def init_locks(self):
        """
        Init locks
        """
        self.image_lock = Lock()
        self.status_lock = Lock()
        self.ptzf_lock = Lock()

    def reset_zoom(self):
        """
        Reset zoom to search value
        """
        command_msg = PTZF()
        command_msg.zoom_global = True
        command_msg.zoom = self.zoom_search
        command_msg.pan_global = True
        command_msg.pan = self.last_pan
        command_msg.tilt_global = True
        command_msg.tilt = self.last_tilt

        self.command_pub.publish(command_msg)
        time.sleep(3)

    def get_aruco_numbers(self):
        """
        Get aruco numbers from parameter server
        """
        aruco_numbers = self.get_parameter('aruco_numbers').value
        if self.special_aruco not in aruco_numbers:
            aruco_numbers.append(self.special_aruco)
        return aruco_numbers

    def start_callback(self, msg):
        """
        Callback for start subscriber
        """
        self.init_tf2()

        # Check if at least two values were sent
        if len(msg.target_ids) < 2:
            self.get_logger().error(f'Not enough arucos sent. Expected at least 2, got {len(msg.target_ids)}')
            return

        aruco_numbers = []
        for target_id in msg.target_ids:
            if target_id.str_id == 'SELFIE':
                self.special_aruco = target_id.int_id
            aruco_numbers.append(target_id.int_id)

        self.set_parameters([rclpy.parameter.Parameter(
            'aruco_numbers',
            rclpy.Parameter.Type.INTEGER_ARRAY,
            aruco_numbers)])

        self.get_logger().info(f'special_aruco: {self.special_aruco}')
        self.get_logger().info(f'aruco_numbers: {aruco_numbers}')

        # Create ImagePTZF subscriber
        self.image_ptzf_sub = self.create_subscription(
            ImagePTZF,
            self.topic_image_ptzf,
            self.image_ptzf_callback,
            dua_qos_visualization.get_image_qos())

        # Remove start subscriber
        self.destroy_subscription(self.start_sub)
        self.get_logger().info('Subscriber removed')

    def selfie_callback(self, request, response):
        """
        Callback for selfie service
        """
        self.get_logger().info('Selfie request received')
        if self.special_aruco in self.found_arucos:
            with self.status_lock:
                self.status = self.SELFIE

            pan_des = self.found_arucos[self.special_aruco]['pan']
            tilt_des = self.found_arucos[self.special_aruco]['tilt']
            with self.ptzf_lock:
                pan = self.pan
                tilt = self.tilt

            # Check if camera is already in posiion
            if abs(pan - pan_des) > 0.1 or abs(tilt - tilt_des) > 0.1:
                self.get_logger().info('Camera not in position, moving...')
                command_msg = PTZF()
                command_msg.pan_global = True
                command_msg.pan = pan_des
                command_msg.tilt_global = True
                command_msg.tilt = tilt_des
                command_msg.zoom_global = True
                command_msg.zoom = self.zoom_selfie
                self.command_pub.publish(command_msg)

                time.sleep(3)

            if self.n_loops * 2 > self.n_segments:
                with self.status_lock:
                    self.status = self.SEARCH

            with self.image_lock:
                frame = self.frame.copy()

            # Publish frame to HMI
            msg_image = Image()
            msg_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg_image.header.stamp = self.get_clock().now().to_msg()
            self.hmi_image_pub.publish(msg_image)

            response.success = True
            self.get_logger().info('Selfie completed')

            self.wait_after_selfie = 0
        else:
            response.success = False
            self.get_logger().info('Selfie aborted, special aruco not found yet')

        return response

    def image_ptzf_callback(self, msg_image):
        """
        Callback for ImagePTZF subscriber
        """
        if self.wait_after_selfie < self.frames_wait_after_selfie:
            self.wait_after_selfie += 1
            return

        with self.ptzf_lock:
            self.pan = msg_image.ptzf.pan
            self.tilt = msg_image.ptzf.tilt
            self.zoom = msg_image.ptzf.zoom
            self.focus = msg_image.ptzf.focus

        # Get image from message
        with self.image_lock:
            self.frame = self.bridge.imgmsg_to_cv2(msg_image.image)

        with self.status_lock:
            status = self.status

        if status == self.SEARCH:
            self.search(self.frame.copy())
        elif status == self.TRACK:
            self.track(self.frame.copy())
        elif status == self.SELFIE:
            pass

    def search(self, frame):
        """
        Searching routine
        """
        # detect ArUco markers in the input frame
        (corners, ids, _) = cv2.aruco.detectMarkers(
            frame,
            self.arucoDict,
            parameters=self.arucoParams)

        if len(corners) > 0:
            self.counter = 0
            ids = ids.flatten()
            for markerID in ids:
                if markerID in self.get_aruco_numbers() and markerID not in self.found_arucos:
                    with self.ptzf_lock:
                        self.last_pan = self.pan
                        self.last_tilt = self.tilt
                    with self.status_lock:
                        if self.status != self.SELFIE:
                            self.status = self.TRACK
                        else:
                            return
                    self.track(frame)
                    return

        with self.ptzf_lock:
            pan = self.pan
            tilt = self.tilt

        # create message to be populated
        command_msg = PTZF()

        command_msg.zoom_global = True
        command_msg.zoom = self.zoom_search

        if pan > self.pan_r:
            if self.sgn == 1:
                self.n_segments = self.n_segments + 1

            self.sgn = -1
            self.current_tilt = self.tilt_des_down
            command_msg.pan_global = True
            command_msg.pan = self.pan_r - 0.2
            command_msg.tilt_global = True
            command_msg.tilt = self.tilt_des_down
        elif pan < self.pan_l:
            if self.sgn == -1:
                self.n_segments = self.n_segments + 1

            if self.n_loops * 2 <= self.n_segments and self.special_aruco in self.found_arucos:
                with self.status_lock:
                    self.status = self.SELFIE
                command_msg = PTZF()
                command_msg.pan_global = True
                command_msg.pan = self.found_arucos[self.special_aruco]['pan']
                command_msg.tilt_global = True
                command_msg.tilt = self.found_arucos[self.special_aruco]['tilt']
                command_msg.zoom_global = True
                command_msg.zoom = self.zoom_selfie
                self.command_pub.publish(command_msg)
                return

            self.sgn = 1
            self.current_tilt = self.tilt_des_up
            command_msg.pan_global = True
            command_msg.pan = self.pan_l + 0.2
            command_msg.tilt_global = True
            command_msg.tilt = self.tilt_des_up
        else:
            command_msg.pan_global = False
            command_msg.pan = self.sgn*self.pan_step
            command_msg.tilt_global = False
            command_msg.tilt = self.k_tilt*(self.current_tilt-tilt)

        self.command_pub.publish(command_msg)

    def track(self, frame):
        """
        Tracking routine
        """
        frame_orig = frame.copy()

        self.counter += 1
        # If no aruco is found for a while, go back to search
        if self.counter > self.frames_track_to_search:
            with self.status_lock:
                if self.status != self.SELFIE:
                    self.status = self.SEARCH
                    self.first_time = True
                else:
                    return
            self.reset_zoom()
            return

        with self.ptzf_lock:
            pan = self.pan
            tilt = self.tilt
            zoom = self.zoom

        img_w = frame.shape[1]
        img_h = frame.shape[0]

        # detect ArUco markers in the input frame
        (corners, ids, _) = cv2.aruco.detectMarkers(
            frame,
            self.arucoDict,
            parameters=self.arucoParams)

        # sort corners and ids according to ids
        if len(corners) > 0:
            corners = [corners[i] for i in np.argsort(ids.flatten())]
            ids = np.sort(ids.flatten())

        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            self.counter = 0
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            back_to_search = True
            for (markerCorner, markerID) in zip(corners, ids):
                if markerID in self.get_aruco_numbers() and markerID not in self.found_arucos:
                    back_to_search = False
                    # extract marker corners (top-left, top-right, bottom-right, bottom-left)
                    markerCorner_42 = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = markerCorner_42

                    # convert each of the (x, y)-coordinate pairs to integers
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))

                    # draw the bounding box of the ArUCo detection
                    cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

                    # compute and draw the center (x, y)-coordinates of the ArUco marker
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

                    # draw the ArUco marker ID on the frame
                    cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # publish command to move camera
                    command_msg = PTZF()

                    cX -= img_w/2
                    cY -= img_h/2

                    exp = 2
                    f = 1 + 9 * ((zoom-1)/9998) ** (1/exp)
                    fov_h = 2*2*atan(6.058/2/f)
                    fov_v = 2*2*atan(4.415/2/f)

                    # Move camera
                    scale_factor = 1
                    if zoom > self.zoom_des/2:  # 5000
                        scale_factor = 3

                    ex =  fov_h*cX/img_w*self.kpx/scale_factor
                    ey = -fov_v*cY/img_h*self.kpy/scale_factor

                    command_msg.pan_global = False
                    command_msg.pan = ex
                    command_msg.tilt_global = False
                    command_msg.tilt = ey

                    if (zoom < 0.8*self.zoom_des):
                        command_msg.zoom_global = False
                        if (abs(ex) < self.err_thr_zoom and abs(ey) < self.err_thr_zoom):
                            command_msg.zoom = self.zoom_step_high
                        else:
                            command_msg.zoom = self.zoom_step_low
                    else:
                        command_msg.zoom_global = True
                        command_msg.zoom = self.zoom_des+10

                    self.command_pub.publish(command_msg)

                    if (abs(zoom-self.zoom_des) < 100 and
                        abs(ex) < self.err_thr and abs(ey) < self.err_thr):

                        if self.first_time:
                            self.first_time = False
                            self.t0 = time.time()

                        # Check focus
                        min_x = min(topLeft[0], bottomLeft[0], topRight[0], bottomRight[0]) - self.padding
                        max_x = max(topLeft[0], bottomLeft[0], topRight[0], bottomRight[0]) + self.padding
                        min_y = min(topLeft[1], bottomLeft[1], topRight[1], bottomRight[1]) - self.padding
                        max_y = max(topLeft[1], bottomLeft[1], topRight[1], bottomRight[1]) + self.padding

                        min_x = max(0, min_x)
                        max_x = min(img_w, max_x)
                        min_y = max(0, min_y)
                        max_y = min(img_h, max_y)

                        # Get rectangle around aruco
                        gray = cv2.cvtColor(frame_orig[min_y:max_y, min_x:max_x], cv2.COLOR_BGR2GRAY)
                        #show gray image
                        # cv2.imshow('gray', gray)
                        # cv2.waitKey(1)

                        # Compute sharpness
                        sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()
                        # print(sharpness)

                        if (sharpness < self.thr_sharpness) and (time.time() - self.t0 < self.thr_time):
                            return

                        self.get_logger().info(f'Found aruco {markerID}')

                        # Aruco is valid, publish data
                        # Get aruco's relative pose
                        _, rvecs, vec_aruco = cv2.solvePnP(
                            self.objPoints,
                            markerCorner,
                            self.camera_matrix,
                            self.dist_coeffs,
                            flags = cv2.SOLVEPNP_IPPE_SQUARE
                        )

                        rot_aruco, _ = cv2.Rodrigues(rvecs)
                        iso_cam_aruco = np.eye(4)
                        iso_cam_aruco[:3, :3] = rot_aruco
                        iso_cam_aruco[:3, 3] = vec_aruco[:, 0]

                        while True:
                            # Get camera tf
                            self.get_logger().info(
                                f'Waiting for transform from {self.frame_fixed} to {self.frame_camera}')
                            try:
                                transform = self.tfBuffer.lookup_transform(
                                    self.frame_fixed,              # origin
                                    self.frame_camera,             # destination
                                    rclpy.time.Time())
                                break
                            except TransformException as ex:
                                self.get_logger().error(
                                    f'Could not get transform from {self.frame_fixed} to {self.frame_camera}: {ex}')
                                time.sleep(1)

                        q_tf = [transform.transform.rotation.x,
                                transform.transform.rotation.y,
                                transform.transform.rotation.z,
                                transform.transform.rotation.w]
                        rot_tf = Rotation.from_quat(q_tf).as_matrix()
                        vec_tf = np.array([transform.transform.translation.x,
                                            transform.transform.translation.y,
                                            transform.transform.translation.z])
                        self.iso_fix_cam[:3, :3] = rot_tf
                        self.iso_fix_cam[:3, 3] = vec_tf

                        # Get camera's absolute pose
                        iso_global = np.matmul(self.iso_fix_cam, iso_cam_aruco)
                        quat = Rotation.from_matrix(iso_global[:3, :3]).as_quat()

                        # Target msg with global pose
                        target = Target()
                        target.header.stamp = self.get_clock().now().to_msg()
                        target.header.frame_id = 'ptz/camera_link'

                        target.target_id.int_id = int(markerID)

                        target.pose.position.x = iso_global[0, 3]
                        target.pose.position.y = iso_global[1, 3]
                        target.pose.position.z = iso_global[2, 3]
                        target.pose.orientation.x = quat[0]
                        target.pose.orientation.y = quat[1]
                        target.pose.orientation.z = quat[2]
                        target.pose.orientation.w = quat[3]

                        aruco_position = np.array([iso_global[0, 3],
                                                   iso_global[1, 3],
                                                   iso_global[2, 3]])

                        # check if aruco is too distant
                        if np.linalg.norm(vec_aruco) > self.max_distance and markerID != self.special_aruco:
                            self.get_logger().info(f'Aruco {markerID} too distant, sending agent to search')

                            # Get stanis tf
                            while True:
                                frame_agent = 'stanis/base_link'
                                self.get_logger().info(
                                    f'Waiting for transform from {self.frame_fixed} to {frame_agent}')
                                try:
                                    transform_stanis = self.tfBuffer.lookup_transform(
                                        self.frame_fixed,              # origin
                                        frame_agent,                   # destination
                                        rclpy.time.Time())
                                    break
                                except TransformException as ex:
                                    self.get_logger().error(
                                        f'Could not get transform from {self.frame_fixed} to {frame_agent}: {ex}')

                            # Get seppia tf
                            while True:
                                frame_agent = 'seppia/base_footprint'
                                self.get_logger().info(
                                    f'Waiting for transform from {self.frame_fixed} to {frame_agent}')
                                try:
                                    transform_seppia = self.tfBuffer.lookup_transform(
                                        self.frame_fixed,              # origin
                                        frame_agent,                   # destination
                                        rclpy.time.Time())
                                    break
                                except TransformException as ex:
                                    self.get_logger().error(
                                        f'Could not get transform from {self.frame_fixed} to {frame_agent}: {ex}')

                            # Get distance from stanis to aruco
                            stanis_position = np.array([transform_stanis.transform.translation.x,
                                                        transform_stanis.transform.translation.y,
                                                        transform_stanis.transform.translation.z])
                            stanis_distance = np.linalg.norm(stanis_position - aruco_position)

                            # Get distance from seppia to aruco
                            seppia_position = np.array([transform_seppia.transform.translation.x,
                                                        transform_seppia.transform.translation.y,
                                                        transform_seppia.transform.translation.z])
                            seppia_distance = np.linalg.norm(seppia_position - aruco_position)

                            # Send agent to search
                            point = Point()
                            point.x = aruco_position[0]
                            point.y = aruco_position[1]
                            point.z = aruco_position[2]
                            if stanis_distance < seppia_distance:
                                self.get_logger().info(f'Sending stanis to search')
                                self.publisher_target_stanis.publish(point)
                            else:
                                self.get_logger().info(f'Sending seppia to search')
                                self.publisher_target_seppia.publish(point)
                        else:
                            self.publisher_target.publish(target)

                            # VisualTargets msg
                            vt_msg = VisualTargets()
                            vt_msg.targets.append(target)
                            vt_msg.image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                            self.hmi_aruco_pub.publish(vt_msg)

                        self.found_arucos[markerID] = {'pan': pan, 'tilt': tilt}

                        frame = cv2.aruco.drawDetectedMarkers(frame, (markerCorner,))
                        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rot_aruco, vec_aruco, 0.1)

                        with self.status_lock:
                            if self.status != self.SELFIE:
                                self.status = self.SEARCH
                            else:
                                return

                        self.first_time = True
                        self.reset_zoom()

            if back_to_search:
                with self.status_lock:
                    self.status = self.SEARCH

            # Publish image with ArUco markers
            msg_imageCV = Image()
            msg_imageCV = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg_imageCV.header.stamp = self.get_clock().now().to_msg()

            self.target_img_pub.publish(msg_imageCV)
