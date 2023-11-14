import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2

class Feedback(Node):
    def __init__(self):
        super().__init__('feedback')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Pose2D, '/detected_aruco', 10)
        self.cv_bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.convert_ros_to_cv(msg)
        detected_pose = self.detect_aruco(cv_image)
        self.publish_detected_pose(detected_pose)

    def convert_ros_to_cv(self, ros_image_msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding='bgr8')
        return cv_image

    def detect_aruco(self, cv_image):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)

        if ids is not None and len(ids) > 0:
            x, y, theta = 0.0, 0.0, 0.0  # Replace with the actual values
            detected_pose = Pose2D()
            detected_pose.x = x
            detected_pose.y = y
            detected_pose.theta = theta
            return detected_pose
        else:
            return None

    def publish_detected_pose(self, pose):
        if pose is not None:
            self.publisher_.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    feedback = Feedback()
    rclpy.spin(feedback)
    feedback.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
