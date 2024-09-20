import rclpy
from rclpy.node import Node
from axis_camera_interfaces.msg import ImagePT
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy

import cv2
from cv_bridge import CvBridge

import numpy as np

# limiti camera
# pan -49° -> 0 -> 30 -> -360 -> -313°
# tilt -18° -> -48°

n_img = 459
n_frame = 0

class Visualizer(Node):

    def __init__(self):
        super().__init__('visualizer')

        self.qos_profile = QoSProfile(depth = 1)
        self.qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        self.qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.subscriber_ = self.create_subscription(ImagePT, '/axis_image', self.listener_callback, self.qos_profile)

        self.subscriber_  # prevent unused variable warning
        self.bridge = CvBridge()

        # Load the ArUCo dictionary and grab the ArUCo parameters
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.arucoParams = cv2.aruco.DetectorParameters()

        self.aruco_bad = [0, 1023]

    def listener_callback(self, msg_image):
        global n_frame, n_img
        frame = self.bridge.imgmsg_to_cv2(msg_image.image)

        # detect ArUco markers in the input frame
        (corners, ids, _) = cv2.aruco.detectMarkers(
            frame,
            self.arucoDict,
            parameters=self.arucoParams)

        # if len(corners) == 1:
        #     corn = corners[0].reshape((4, 2))
        #     (topLeft, topRight, bottomRight, bottomLeft) = corn

        #     # convert each of the (x, y)-coordinate pairs to integers
        #     topRight = (int(topRight[0]), int(topRight[1]))
        #     bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        #     bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        #     topLeft = (int(topLeft[0]), int(topLeft[1]))
            
        #     min_x = min(topLeft[0], bottomLeft[0], topRight[0], bottomRight[0]) - 20
        #     max_x = max(topLeft[0], bottomLeft[0], topRight[0], bottomRight[0]) + 20
        #     min_y = min(topLeft[1], bottomLeft[1], topRight[1], bottomRight[1]) - 20
        #     max_y = max(topLeft[1], bottomLeft[1], topRight[1], bottomRight[1]) + 20
            
        #     print(min_x, max_x, min_y, max_y)
            
        #     # get part of image around aruco
        #     frame_little = frame[min_y:max_y, min_x:max_x]
        #     gray = cv2.cvtColor(frame_little, cv2.COLOR_BGR2GRAY)

        #     # Calcola il gradiente usando il filtro di Sobel
        #     sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()
            
        #     # gradient_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        #     # gradient_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)

        #     # # Calcola la variazione media del gradiente
        #     # gradient_magnitude = np.sqrt(gradient_x**2 + gradient_y**2)
        #     # sharpness = np.mean(gradient_magnitude)
        #     print(sharpness)

        #     cv2.imshow("Frame_little", gray)

        if len(corners) > 0:
            for (markerCorner, markerID) in zip(corners, ids):
                    if markerID not in self.aruco_bad:
                        frame = cv2.aruco.drawDetectedMarkers(frame, (markerCorner,))
        frame = cv2.resize(frame, (1080, 720))

        if n_frame%40 == 0:
            cv2.imwrite(f'/home/ros/workspace/pics/{n_img}.jpg', frame)
            n_img += 1
        n_frame += 1
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF


def main(args=None):
    rclpy.init(args=args)

    visualizer = Visualizer()

    rclpy.spin(visualizer)

    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

