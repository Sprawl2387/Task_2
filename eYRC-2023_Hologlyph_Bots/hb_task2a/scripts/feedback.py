import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math as m 
from geometry_msgs.msg import Pose2D
class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('ar_uco_detector')
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Pose2D, '/detected_aruco', 10)
        self.pose = Pose2D()
        self.cv_bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.param1 =  cv2.aruco.DetectorParameters()
        self.param1.adaptiveThreshWinSizeMin = 3
        self.param1.adaptiveThreshWinSizeMax = 13
        self.param1.adaptiveThreshWinSizeStep = 2
        self.param1.adaptiveThreshConstant = 13
        self.param2 = cv2.aruco.DetectorParameters()
        self.hb_x = 0.0
        self.hb_y = 0.0
        self.hb_theta = 0.0

    def create_perspective_image(self, img):
        detect = cv2.aruco.ArucoDetector(self.dictionary, self.param1)
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detect.detectMarkers(gray_img)
        if ids is not None and len(ids) == 5:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            ids = ids.flatten().tolist()
            center_lst = []
            for index in range(len(ids)):
                aruco_x = int((corners[index][0][0][0] + corners[index][0][1][0] + corners[index][0][2][0] + corners[index][0][3][0])/4)
                aruco_y = int((corners[index][0][0][1] + corners[index][0][1][1] + corners[index][0][2][1] + corners[index][0][3][1])/4)
                center_lst.append((aruco_x, aruco_y))
            ul_index = ids.index(8)
            ur_index = ids.index(10)
            lr_index = ids.index(12)
            ll_index = ids.index(4)
            width_1 = m.sqrt((center_lst[ul_index][0] - center_lst[ll_index][0])**2 + (center_lst[ul_index][1] - center_lst[ll_index][1])**2)
            width_2 = m.sqrt((center_lst[ur_index][0] - center_lst[lr_index][0])**2 + (center_lst[ur_index][1] - center_lst[lr_index][1])**2)
            print(f'width_1: {width_1}')
            print(f'width_2: {width_2}')
        cv2.imshow('d', img)
        
    def image_callback(self, msg):
        cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_img_zoomed = cv2.resize(cv_img, (2000,2000))
        gray_img = cv2.cvtColor(cv_img_zoomed, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(self.dictionary, self.param2)
        corners, ids, _ = detector.detectMarkers(gray_img)
        if ids is not None:
            ids = ids.flatten().tolist()
            if 1 in ids:
                robot_corners = corners[ids.index(1)]
                robot_corners = robot_corners.reshape((4, 2))
                (tl, tr, br, bl) = robot_corners
                p_x = int((tr[0] + br[0]) / 2.0)
                p_y = int((tr[1] + br[1]) / 2.0)
                c_x = int((tl[0] + br[0]) / 2.0)
                c_y = int((tl[1] + br[1]) / 2.0)
                self.hb_x = c_x/4
                self.hb_y = c_y/4
                if c_x == p_x:
                    if p_y-c_y < 0:
                        self.hb_theta = 0.5*m.pi
                    else:
                        self.hb_theta = -0.5*m.pi
                else:
                    if p_y-c_y < 0:
                        if p_x > c_x:
                            self.hb_theta = m.atan(abs(p_y-c_y)/abs(p_x-c_x))
                        else:
                            self.hb_theta = m.pi - m.atan(abs(p_y-c_y)/abs(p_x-c_x))
                    else:
                        if p_x > c_x:
                            self.hb_theta = -m.atan(abs(p_y-c_y)/abs(p_x-c_x))
                        else:
                            self.hb_theta = -(m.pi - m.atan(abs(p_y-c_y)/abs(p_x-c_x)))
                self.hb_theta = round(self.hb_theta + 0.09, 6)

        self.pose.x = self.hb_x
        self.pose.y = self.hb_y
        self.pose.theta = self.hb_theta
        self.pub.publish(self.pose)
        
def main(args=None):
    rclpy.init(args=args)
    aruco_detect = ArUcoDetector()
    rclpy.spin(aruco_detect)
    aruco_detect.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
