import cv2, yaml, time, json, glob
import numpy as np
from math import sin, cos, atan, atan2, floor
from scipy.spatial.transform import Rotation
from threading import Lock, Thread

from cv_bridge import CvBridge

import dua_qos_py.dua_qos_besteffort as dua_qos_besteffort
import dua_qos_py.dua_qos_reliable as dua_qos_reliable

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

import tf2_ros
from tf2_ros import TransformException

from axis_camera_interfaces.msg import ImagePTZF, PTZF
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

from axis_camera_interfaces.srv import Selfie

from ultralytics import YOLO


class AxisCameraTrackerNode(Node):
    """
    Axis Camera Tracker ROS 2 node implementation.
    """

    def __init__(self):
        super().__init__('axis_camera_tracker')

        self.init_parameters()
        self.init_calibration()
        self.init_states()
        self.init_locks()
        self.init_arucos()
        self.init_tf2()
        self.init_network()
        self.init_publishers()
        self.init_subscribers()
        self.init_services()

        # Initialize other variables
        self.bridge = CvBridge()
        self.found_arucos = []
        self.found_objects = []
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
        self.last_pan = 0.0
        self.last_tilt = 0.0
        self.n_segments = 0
        self.iso_fix_cam = np.eye(4)
        self.wait_after_selfie = 10
        self.started_subscriber = False

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

    # Init functions
    def init_network(self):
        self.model_coco = YOLO(self.pt_path, verbose=False)
        # self.model = YOLO('', verbose=False)

        self.target_classes_ids = [i for i in self.model_coco.names if self.model_coco.names[i] in self.classes_targets]

    def init_parameters(self):
        """
        Init parameters
        """
        self.declare_parameters(
            namespace='',
            parameters=[('aruco_numbers_uav', [1]),
                        ('aruco_numbers_ugv', [1]),
                        ('aruco_selfie', 0),
                        ('aruco_size', 0.1),
                        ('calibration_file_path', 'path'),
                        ('classes_targets', ['']),
                        ('err_thr', 0.5),
                        ('err_thr_zoom', 0.5),
                        ('focal_x_m', 0.0),
                        ('focal_x_x0', 0.0),
                        ('focal_x_y0', 0.0),
                        ('focal_y_m', 0.0),
                        ('focal_y_x0', 0.0),
                        ('focal_y_y0', 0.0),
                        ('frame_camera', 'camera_link'),
                        ('frame_fixed', 'fixed_frame'),
                        ('frame_wall', 'fixed_footprint'),
                        ('frames_track_to_search', 1),
                        ('frames_wait_after_selfie', 1),
                        ('kpx', 1.0),
                        ('kpy', 1.0),
                        ('k_tilt', 1.0),
                        ('max_distance', 1.0),
                        ('model_score_threshold', 0.1),
                        ('n_loops', 1),
                        ('padding', 1),
                        ('pan_l', 1.0),
                        ('pan_r', 1.0),
                        ('pan_step', 0.1),
                        ('pt_path', ''),
                        ('selfie_offset', 0.0),
                        ('selfie_save_path', ''),
                        ('thr_sharpness', 1.0),
                        ('thr_time', 1.0),
                        ('tilt_des_down', 1.0),
                        ('tilt_des_up', 1.0),
                        ('track_aruco_backwards_distance', 0.0),
                        ('track_object_forwards_distance', 0.0),
                        ('zoom_des', 1.0),
                        ('zoom_num_pics', 1),
                        ('zoom_search', 1.0),
                        ('zoom_search_step', 1.0),
                        ('zoom_selfie_start', 1.0),
                        ('zoom_selfie_end', 1.0),
                        ('zoom_step_high', 1.0),
                        ('zoom_step_low', 1.0),
                       ])

        self.aruco_selfie = self.get_parameter('aruco_selfie').value
        self.aruco_size = self.get_parameter('aruco_size').value
        self.calibration_file_path = self.get_parameter('calibration_file_path').value
        self.err_thr = self.get_parameter('err_thr').value
        self.err_thr_zoom = self.get_parameter('err_thr_zoom').value
        self.focal_x_m = self.get_parameter('focal_x_m').value
        self.focal_x_x0 = self.get_parameter('focal_x_x0').value
        self.focal_x_y0 = self.get_parameter('focal_x_y0').value
        self.focal_y_m = self.get_parameter('focal_y_m').value
        self.focal_y_x0 = self.get_parameter('focal_y_x0').value
        self.focal_y_y0 = self.get_parameter('focal_y_y0').value
        self.frame_camera = self.get_parameter('frame_camera').value
        self.frame_fixed = self.get_parameter('frame_fixed').value
        self.frame_wall = self.get_parameter('frame_wall').value
        self.frames_track_to_search = self.get_parameter('frames_track_to_search').value
        self.frames_wait_after_selfie = self.get_parameter('frames_wait_after_selfie').value
        self.k_tilt = self.get_parameter('k_tilt').value
        self.kpx = self.get_parameter('kpx').value
        self.kpy = self.get_parameter('kpy').value
        self.max_distance = self.get_parameter('max_distance').value
        self.model_score_threshold = self.get_parameter('model_score_threshold').value
        self.n_loops = self.get_parameter('n_loops').value
        self.padding = self.get_parameter('padding').value
        self.pan_l = self.get_parameter('pan_l').value
        self.pan_r = self.get_parameter('pan_r').value
        self.pan_step = self.get_parameter('pan_step').value
        self.pt_path = self.get_parameter('pt_path').value
        self.selfie_offset = self.get_parameter('selfie_offset').value
        self.selfie_save_path = self.get_parameter('selfie_save_path').value
        self.thr_sharpness = self.get_parameter('thr_sharpness').value
        self.thr_time = self.get_parameter('thr_time').value
        self.tilt_des_down = self.get_parameter('tilt_des_down').value
        self.tilt_des_up = self.get_parameter('tilt_des_up').value
        self.track_aruco_backwards_distance = self.get_parameter('track_aruco_backwards_distance').value
        self.track_object_forwards_distance = self.get_parameter('track_object_forwards_distance').value
        self.zoom_des = self.get_parameter('zoom_des').value
        self.zoom_num_pics = self.get_parameter('zoom_num_pics').value
        self.zoom_search = self.get_parameter('zoom_search').value
        self.zoom_search_step = self.get_parameter('zoom_search_step').value
        self.zoom_selfie_start = self.get_parameter('zoom_selfie_start').value
        self.zoom_selfie_end = self.get_parameter('zoom_selfie_end').value
        self.zoom_step_low = self.get_parameter('zoom_step_low').value
        self.zoom_step_high = self.get_parameter('zoom_step_high').value

        self.get_logger().info(f'aruco_size: {self.aruco_selfie}')
        self.get_logger().info(f'aruco_size: {self.aruco_size}')
        self.get_logger().info(f'calibration_file_path: {self.calibration_file_path}')
        self.get_logger().info(f'err_thr: {self.err_thr}')
        self.get_logger().info(f'err_thr_zoom: {self.err_thr_zoom}')
        self.get_logger().info(f'focal_x_m: {self.focal_x_m}')
        self.get_logger().info(f'focal_x_x0: {self.focal_x_x0}')
        self.get_logger().info(f'focal_x_y0: {self.focal_x_y0}')
        self.get_logger().info(f'focal_y_m: {self.focal_y_m}')
        self.get_logger().info(f'focal_y_x0: {self.focal_y_x0}')
        self.get_logger().info(f'focal_y_y0: {self.focal_y_y0}')
        self.get_logger().info(f'frame_camera: {self.frame_camera}')
        self.get_logger().info(f'frame_fixed: {self.frame_fixed}')
        self.get_logger().info(f'frame_wall: {self.frame_wall}')
        self.get_logger().info(f'frames_track_to_search: {self.frames_track_to_search}')
        self.get_logger().info(f'frames_wait_after_selfie: {self.frames_wait_after_selfie}')
        self.get_logger().info(f'k_tilt: {self.k_tilt}')
        self.get_logger().info(f'kpx: {self.kpx}')
        self.get_logger().info(f'kpy: {self.kpy}')
        self.get_logger().info(f'max_distance: {self.max_distance}')
        self.get_logger().info(f'model_score_threshold: {self.model_score_threshold}')
        self.get_logger().info(f'n_loops: {self.n_loops}')
        self.get_logger().info(f'padding: {self.padding} px')
        self.get_logger().info(f'pan_l: {self.pan_l}°')
        self.get_logger().info(f'pan_r: {self.pan_r}°')
        self.get_logger().info(f'pan_step: {self.pan_step}°')
        self.get_logger().info(f'pt_path: {self.pt_path}')
        self.get_logger().info(f'selfie_offset: {self.selfie_offset}')
        self.get_logger().info(f'selfie_save_path: {self.selfie_save_path}')
        self.get_logger().info(f'thr_sharpness: {self.thr_sharpness}')
        self.get_logger().info(f'thr_time: {self.thr_time} s')
        self.get_logger().info(f'tilt_des_down: {self.tilt_des_down}°')
        self.get_logger().info(f'tilt_des_up: {self.tilt_des_up}°')
        self.get_logger().info(f'track_aruco_backwards_distance: {self.track_aruco_backwards_distance}')
        self.get_logger().info(f'track_object_forwards_distance: {self.track_object_forwards_distance}')
        self.get_logger().info(f'zoom_des: {self.zoom_des}')
        self.get_logger().info(f'zoom_num_pics: {self.zoom_num_pics}')
        self.get_logger().info(f'zoom_search: {self.zoom_search}')
        self.get_logger().info(f'zoom_search_step: {self.zoom_search_step}')
        self.get_logger().info(f'zoom_selfie_start: {self.zoom_selfie_start}')
        self.get_logger().info(f'zoom_selfie_end: {self.zoom_selfie_end}')
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
            '~/image',
            dua_qos_besteffort.get_image_qos())

        self.hmi_image_pub = self.create_publisher(
            Image,
            '/selfie',
            dua_qos_besteffort.get_image_qos())

        self.publisher_target_arianna = self.create_publisher(
            PoseStamped,
            '/search_arianna',
            dua_qos_reliable.get_datum_qos())

        self.publisher_target_dottore = self.create_publisher(
            PoseStamped,
            '/search_dottore',
            dua_qos_reliable.get_datum_qos())

        self.command_pub = self.create_publisher(
            PTZF,
            '/command',
            dua_qos_reliable.get_datum_qos())

    def init_subscribers(self):
        """
        Init subscribers
        """
        start_sub_cgroup = MutuallyExclusiveCallbackGroup()
        self.start_sub = self.create_subscription(
            String,
            '/valid_targets',
            self.start_callback,
            dua_qos_reliable.get_datum_qos(),
            callback_group=start_sub_cgroup)

    def init_services(self):
        """
        Init services
        """
        selfie_srv_cgroup = MutuallyExclusiveCallbackGroup()
        self.selfie_srv = self.create_service(
            Selfie,
            '~/selfie',
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
        self.TRACK_ARUCO = 1
        self.TRACK_OBJECT = 2
        self.SELFIE = 3
        self.status = self.SEARCH

    def init_locks(self):
        """
        Init locks
        """
        self.image_lock = Lock()
        self.status_lock = Lock()
        self.ptzf_lock = Lock()

    # Callbacks
    def selfie_callback(self, request, response):
        """
        Callback for selfie service
        """
        self.get_logger().info('Selfie request received')

        if not self.started_subscriber:
            response.success = False
            self.get_logger().info('Selfie must be called after sending targets')
            return response

        with self.status_lock:
            self.status = self.SELFIE

        time.sleep(2)

        # Get transform from frame_fixed to frame_wall
        iso_fix_foot = self.get_transform(self.frame_fixed, self.frame_wall)
        x_ptz = iso_fix_foot[0, 3]
        y_ptz = iso_fix_foot[1, 3]
        z_ptz = iso_fix_foot[2, 3]

        frame_id = request.point.header.frame_id
        point_des = np.array([request.point.point.x, request.point.point.y, 0.0])
        iso_frameid_footprint = self.get_transform(frame_id, self.frame_wall)

        point_des_local = np.matmul(np.linalg.inv(iso_frameid_footprint), np.append(point_des, 1))[:3]

        pan_des = -atan2(point_des_local[1], point_des_local[0]) * 180 / np.pi

        # Send camera to approximate position
        command_msg = PTZF()
        command_msg.pan_global = True
        command_msg.pan = pan_des
        command_msg.tilt_global = True
        command_msg.tilt = -atan2(z_ptz, np.linalg.norm(point_des_local[:2])) * 180 / np.pi
        command_msg.zoom_global = True
        command_msg.zoom = self.zoom_selfie_start
        self.command_pub.publish(command_msg)

        time.sleep(1)

        for zoom in range(int(self.zoom_selfie_start), \
                          int(self.zoom_selfie_end),   \
                          int((self.zoom_selfie_end - self.zoom_selfie_start) / self.zoom_num_pics)):
            command_msg = PTZF()
            command_msg.zoom_global = True
            command_msg.zoom = float(zoom)
            self.command_pub.publish(command_msg)

            time.sleep(1)

            with self.image_lock:
                frame = self.frame.copy()

            # Save image
            cv2.imwrite(f'{self.selfie_save_path}/selfie_{zoom}.png', frame)

            self.get_logger().info(f'Taken selfie with zoom: {zoom}')

        if self.n_loops * 2 > self.n_segments:
            with self.status_lock:
                self.status = self.SEARCH

        self.wait_after_selfie = 0

        # Start thread
        self.thread = Thread(target=self.analysis)


        response.success = True
        self.get_logger().info('Selfie completed')
        return response

    def analysis(self):
        paths = glob.glob(f'{self.selfie_save_path}/selfie_*.png')
        for path in paths:
            img = cv2.imread(path)
            results = self.model(img, verbose=False, conf=0.5)
            print(results)


    def image_ptzf_callback(self, msg):
        """
        Callback for ImagePTZF subscriber
        """
        if self.wait_after_selfie < self.frames_wait_after_selfie:
            self.wait_after_selfie += 1
            return

        # Get info from message
        with self.ptzf_lock:
            self.pan = msg.ptzf.pan
            self.tilt = msg.ptzf.tilt
            self.zoom = msg.ptzf.zoom
            self.focus = msg.ptzf.focus

        with self.image_lock:
            self.frame = self.bridge.imgmsg_to_cv2(msg.image)

        with self.status_lock:
            status = self.status

        # Update state
        if status == self.SEARCH:
            self.search(self.frame.copy())
        elif status == self.TRACK_ARUCO:
            self.track_aruco(self.frame.copy())
        elif status == self.TRACK_OBJECT:
            self.track_object(self.frame.copy())
        elif status == self.SELFIE:
            pass

    def start_callback(self, msg):
        """
        Callback after receiving Detection2DArray
        """
        json_dict = json.loads(msg.data)

        aruco_numbers_uav = json_dict['uav']
        aruco_numbers_ugv = json_dict['ugv']

        self.set_parameters([rclpy.parameter.Parameter(
            'aruco_numbers_uav',
            rclpy.Parameter.Type.INTEGER_ARRAY,
            aruco_numbers_uav)])

        self.set_parameters([rclpy.parameter.Parameter(
            'aruco_numbers_ugv',
            rclpy.Parameter.Type.INTEGER_ARRAY,
            aruco_numbers_ugv)])

        self.get_logger().info(f'aruco_numbers_uav: {aruco_numbers_uav}')
        self.get_logger().info(f'aruco_numbers_ugv: {aruco_numbers_ugv}')

        # Create ImagePTZF subscriber
        self.image_ptzf_sub = self.create_subscription(
            ImagePTZF,
            '/stream_ptzf',
            self.image_ptzf_callback,
            dua_qos_reliable.get_image_qos())
        self.started_subscriber = True

        # Remove start subscriber
        self.destroy_subscription(self.start_sub)
        self.get_logger().info('Subscriber removed')

    # Utils functions
    def get_ptzf(self):
        with self.ptzf_lock:
            pan = self.pan
            tilt = self.tilt
            zoom = self.zoom
            focus = self.focus

        return pan, tilt, zoom, focus

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
        aruco_numbers_uav = self.get_parameter('aruco_numbers_uav').value
        aruco_numbers_ugv = self.get_parameter('aruco_numbers_ugv').value

        return aruco_numbers_uav, aruco_numbers_ugv

    def get_classes_ids(self):
        """
        Get classes names from parameter server
        """
        classes_targets = self.get_parameter('classes_targets').value
        classes_targets_ids = [i for i in self.model_coco.names if self.model_coco.names[i] in classes_targets]

        return classes_targets_ids

    def get_transform(self, frame_origin, frame_dest):
        """
        Get TF from frame_origin to frame_dest
        """
        while True:
            # Get camera tf
            self.get_logger().info(
                f'Waiting for transform from {frame_origin} to {frame_dest}')
            try:
                transform = self.tfBuffer.lookup_transform(
                    frame_origin,                  # origin
                    frame_dest,                    # destination
                    rclpy.time.Time())
                break
            except TransformException as ex:
                self.get_logger().error(
                    f'Could not get transform from {frame_origin} to {frame_dest}: {ex}')
                time.sleep(1)

        q_tf = [transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w]
        rot_tf = Rotation.from_quat(q_tf).as_matrix()
        vec_tf = np.array([transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z])

        isometry = np.eye(4)
        isometry[:3, :3] = rot_tf
        isometry[:3, 3] = vec_tf

        return isometry

    # Routines
    def search(self, frame):
        """
        Searching routine
        """
        with self.status_lock:
            if self.status != self.SEARCH:
                return

        # Detect targets in the input frame
        nn_results = self.model_coco(frame,
                                     stream=True,
                                     verbose=False,
                                     conf=self.model_score_threshold,
                                     classes=self.get_classes_ids())

        for nn_result in nn_results:
            detections = len(nn_result)

            if detections > 0:
                with self.status_lock:
                    if self.status != self.SELFIE:
                        self.status = self.TRACK
                    else:
                        return
                self.track_object(frame)
                return

        # Detect ArUco markers in the input frame
        (corners, ids, _) = cv2.aruco.detectMarkers(
            frame,
            self.arucoDict,
            parameters=self.arucoParams)

        if len(corners) > 0:
            self.counter = 0
            ids = ids.flatten()
            for markerID in ids:
                aruco_numbers_uav, aruco_numbers_ugv = self.get_aruco_numbers()

                if (markerID in aruco_numbers_uav or markerID in aruco_numbers_ugv) and \
                    markerID not in self.found_arucos:

                    self.last_pan, self.last_tilt, _, _ = self.get_ptzf()

                    with self.status_lock:
                        if self.status != self.SELFIE:
                            self.status = self.TRACK
                        else:
                            return
                    self.track_aruco(frame)
                    return

        # Keep searching
        pan, tilt, _, _ = self.get_ptzf()

        command_msg = PTZF()
        command_msg.zoom_global = True
        command_msg.zoom = self.zoom_search + self.zoom_search_step * floor(self.n_segments / 2)

        if pan > self.pan_r:
            if self.sgn == 1:
                self.n_segments += 1

            self.sgn = -1
            self.current_tilt = self.tilt_des_down
            command_msg.pan_global = True
            command_msg.pan = self.pan_r - 0.2
            command_msg.tilt_global = True
            command_msg.tilt = self.tilt_des_down
        elif pan < self.pan_l:
            if self.sgn == -1:
                self.n_segments += 1

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

    def track_aruco(self, frame):
        """
        Tracking routine for Aruco markers
        """
        print('Tracking Aruco')
        frame_orig = frame.copy()

        _, _, zoom, _ = self.get_ptzf()

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

        img_w = frame.shape[1]
        img_h = frame.shape[0]

        # Detect ArUco markers in the input frame
        (corners, ids, _) = cv2.aruco.detectMarkers(
            frame,
            self.arucoDict,
            parameters=self.arucoParams)

        # Verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            print('Found aruco')
            # Sort corners and ids according to ids
            corners = [corners[i] for i in np.argsort(ids.flatten())]
            ids = np.sort(ids.flatten()).flatten()

            self.counter = 0
            back_to_search = True

            for (markerCorner, markerID) in zip(corners, ids):
                aruco_numbers_uav, aruco_numbers_ugv = self.get_aruco_numbers()

                if (markerID in aruco_numbers_uav or markerID in aruco_numbers_ugv) and \
                    markerID not in self.found_arucos:

                    back_to_search = False
                    markerCorner_42 = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = markerCorner_42

                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))

                    cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

                    cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    command_msg = PTZF()

                    cX -= img_w/2
                    cY -= img_h/2

                    time.sleep(0.3)

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

                        self.iso_wall_cam = self.get_transform(self.frame_wall, self.frame_camera)

                        # Get camera's absolute pose
                        iso_global = np.matmul(self.iso_wall_cam, iso_cam_aruco)

                        aruco_position = np.array([iso_global[0, 3],
                                                   iso_global[1, 3],
                                                   0.0])
                        aruco_xy_dist = np.linalg.norm(aruco_position)
                        if aruco_xy_dist > 2 * self.track_aruco_backwards_distance:
                            aruco_position *= (aruco_xy_dist - self.track_aruco_backwards_distance) / aruco_xy_dist
                        else:
                            aruco_position *= 0.8

                        aruco_angle = atan2(aruco_position[1], aruco_position[0])

                        # Send pose to robots
                        point = PoseStamped()
                        point.header.stamp = self.get_clock().now().to_msg()
                        point.header.frame_id = self.frame_wall
                        point.pose.position.x = aruco_position[0]
                        point.pose.position.y = aruco_position[1]
                        point.pose.position.z = 0.0                         # FIXME
                        q = Rotation.from_euler('z', aruco_angle).as_quat()
                        point.pose.orientation.x = q[0]
                        point.pose.orientation.y = q[1]
                        point.pose.orientation.z = q[2]
                        point.pose.orientation.w = q[3]

                        if markerID in aruco_numbers_uav:
                            self.get_logger().info(f'Sending arianna to search')
                            self.publisher_target_arianna.publish(point)
                        if markerID in aruco_numbers_ugv:
                            self.get_logger().info(f'Sending dottore to search')
                            self.publisher_target_dottore.publish(point)

                        self.found_arucos.append(markerID)

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

    def track_object(self, frame):
        """
        Tracking routine for COCO objects
        """
        print('Tracking object')
        pan, tilt, zoom, _ = self.get_ptzf()

        self.counter += 1
        # If no object is found for a while, go back to search
        if self.counter > self.frames_track_to_search:
            with self.status_lock:
                if self.status != self.SELFIE:
                    self.status = self.SEARCH
                    self.first_time = True
                else:
                    return
            self.reset_zoom()
            return

        img_w = frame.shape[1]
        img_h = frame.shape[0]

        # Detect targets in the input frame
        classes_ids = self.get_classes_ids()
        classes_ids = [id_ for id_ in classes_ids if id_ not in self.found_objects]
        nn_results = self.model_coco(frame,
                                     stream=True,
                                     verbose=False,
                                     conf=self.model_score_threshold,
                                     classes=classes_ids)

        for nn_result in nn_results:
            detections = len(nn_result)

            if detections > 0:
                self.counter = 0

                for i in range(detections):
                    box = nn_result.boxes[i]
                    conf = box.conf[0].cpu().item()
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls = int(box.cls[0])
                    label = f'{self.model_coco.names[cls]} {conf:.2f}'

                    # Draw bbox and label
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    cX = (x1 + x2) / 2.0 - img_w / 2.0
                    cY = (y1 + y2) / 2.0 - img_h / 2.0

                    exp = 2
                    f = 1 + 9 * ((zoom-1)/9998) ** (1/exp)
                    fov_h = 2*2*atan(6.058/2/f)
                    fov_v = 2*2*atan(4.415/2/f)

                    # Move camera
                    ex =  fov_h*cX/img_w*self.kpx
                    ey = -fov_v*cY/img_h*self.kpy

                    command_msg = PTZF()
                    command_msg.pan_global = False
                    command_msg.pan = ex
                    command_msg.tilt_global = False
                    command_msg.tilt = ey
                    self.command_pub.publish(command_msg)

                    time.sleep(0.3)

                    if abs(ex) < self.err_thr:
                        self.get_logger().info(f'Found object with id {cls}')

                        # Send pose to robots
                        point = PoseStamped()
                        point.header.stamp = self.get_clock().now().to_msg()
                        point.header.frame_id = self.frame_wall
                        point.pose.position.x = self.track_object_forwards_distance * cos(-pan)
                        point.pose.position.y = self.track_object_forwards_distance * sin(-pan)
                        point.pose.position.z = 0.0                      # FIXME
                        q = Rotation.from_euler('z', -pan).as_quat()     # FIXME: -pan?
                        point.pose.orientation.x = q[0]
                        point.pose.orientation.y = q[1]
                        point.pose.orientation.z = q[2]
                        point.pose.orientation.w = q[3]

                        self.get_logger().info(f'Sending robots to search')
                        self.publisher_target_arianna.publish(point)
                        self.publisher_target_dottore.publish(point)

                        self.found_objects.append(cls)

                        with self.status_lock:
                            if self.status != self.SELFIE:
                                self.status = self.SEARCH
                            else:
                                return

                        self.reset_zoom()

                # Publish image with ArUco markers
                msg_imageCV = Image()
                msg_imageCV = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                msg_imageCV.header.stamp = self.get_clock().now().to_msg()

                self.target_img_pub.publish(msg_imageCV)

    def get_focal(self, zoom, axis):
        if axis == 'x':
            return self.focal_x_m * (zoom - self.focal_x_x0) + self.focal_x_y0
        elif axis == 'y':
            return self.focal_y_m * (zoom - self.focal_y_x0) + self.focal_y_y0
        else:
            return 0.0
