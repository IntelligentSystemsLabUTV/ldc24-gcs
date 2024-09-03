import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor
from axis_camera_tracker.axis_camera_tracker_node import AxisCameraTrackerNode

def main():
    rclpy.init(args=sys.argv)

    axis_camera_tracker_node = AxisCameraTrackerNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(axis_camera_tracker_node)

    try:
        executor.spin()
    finally:
        axis_camera_tracker_node.destroy_node()
        executor.shutdown()


if __name__ == '__main__':
    main()
