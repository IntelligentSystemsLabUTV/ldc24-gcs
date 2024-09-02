import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy

from axis_camera_interfaces.msg import ImagePT, ImageError

from imutils.video import VideoStream
from requests.auth import HTTPDigestAuth
from cv_bridge import CvBridge
import imutils, requests, time, cv2

from .config import username, password, ip_address

class AxisController(Node):

    def __init__(self, axis_camera):
        super().__init__('axis_controller')

        self.group = MutuallyExclusiveCallbackGroup()

        self.subscriber = self.create_subscription(
            ImageError,
            'axis_errors',
            self.listener_callback,
            0,
            callback_group=self.group)
        self.subscriber  # prevent unused variable warning

        self.axis_camera = axis_camera

    def listener_callback(self, msg):
        self.axis_camera.cameraPanRelative(msg.ex)
        self.axis_camera.cameraTiltRelative(msg.ey)


class AxisCamera(Node):

    def merge_dicts(self, *dict_args):
        result = {}
        for dictionary in dict_args:
            result.update(dictionary)
        return result

    def cameraCmd(self, q_cmd):
        resp_data = {}
        base_q_args = { 
            'camera': self.camera_n, 'imagerotation': self.camera_ir, 'html': 'no', 'timestamp': int(time.time()) 
        }
        q_args = self.merge_dicts(q_cmd, base_q_args)
        resp = requests.get(self.camera_url, params=q_args, auth=HTTPDigestAuth(username, password))
        if resp.text.startswith('Error'):
            print(resp.text)
        else:
            try:
                for line in resp.text.splitlines():
                    (name, var) = line.split("=", 2)
                    try:
                        resp_data[name.strip()] = float(var)
                    except ValueError:
                        resp_data[name.strip()] = var
            except:
                pass

        return resp_data
        
    def cameraGet(self, query):	
        return self.cameraCmd({ 'query': query })

    def cameraSet(self, group, val):	
        return self.cameraCmd({ group: val })

    def cameraGetPTZ(self):
        return self.cameraGet('position')

    def cameraPan(self, value):
        return self.cameraSet('pan', value)

    def cameraTilt(self, value):
        return self.cameraSet('tilt', value)

    def cameraZoom(self, value):
        return self.cameraSet('zoom', value)

    def cameraPanRelative(self, value):
        return self.cameraSet('rpan', value)

    def cameraTiltRelative(self, value):
        return self.cameraSet('rtilt', value)

    def cameraZoomRelative(self, value):
        return self.cameraSet('rzoom', value)

    def __init__(self):
        super().__init__('axis_camera')

        self.qos_profile = QoSProfile(depth = 0)
        self.qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        self.qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.publisher_ = self.create_publisher(ImagePT, 'axis_image', self.qos_profile)

        # self.publisher_ = self.create_publisher(ImagePT, 'axis_image', 0)
        
        timer_period = 0.01  # seconds
        self.group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.group)
        
        # load the ArUCo dictionary and grab the ArUCo parameters
        #self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        #self.arucoParams = cv2.aruco.DetectorParameters_create()

        # initialize the video stream and allow the camera sensor to warm up
        self.get_logger().info("[INFO] starting video stream...")
        self.url = f"http://{username}:{password}@{ip_address}//mjpg/video.mjpg"
        self.vs = VideoStream(src=self.url).start()
        self.bridge = CvBridge()
        time.sleep(2.0)

        self.camera_n = 1
        self.camera_url = f"http://{ip_address}//axis-cgi/com/ptz.cgi"
        #self.camera_ir = 180        # upside down 
        self.camera_ir = 0

        self.w = 1000
        self.h = 562
        
    def timer_callback(self):
        # grab the frame from the threaded video stream and resize it to have a maximum width of 1000 pixels
        frame = self.vs.read()
        frame = imutils.resize(frame, width=1000)

        PTZ = self.cameraGetPTZ()
        pan = PTZ['pan']
        tilt = PTZ['tilt']

        msg = ImagePT()
        msg.image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        msg.image.header.stamp = self.get_clock().now().to_msg()

        msg.pan = pan
        msg.tilt = tilt
        
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    axis_camera = AxisCamera()
    axis_controller = AxisController(axis_camera)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(axis_camera)
    executor.add_node(axis_controller)

    executor.spin()
            
    executor.shutdown()
    axis_camera.destroy_node()
    axis_controller.destroy_node()

    cv2.destroyAllWindows()
    axis_camera.vs.stop()

    axis_camera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

