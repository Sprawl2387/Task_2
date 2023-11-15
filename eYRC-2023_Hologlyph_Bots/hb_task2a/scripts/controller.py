import rclpy
from rclpy.node import Node
import math as m
from my_robot_interfaces.srv import NextGoal   
from geometry_msgs.msg import Pose2D  
from geometry_msgs.msg import Wrench      
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2   

class HBController(Node):
    def __init__(self):
        super().__init__('bot')
        self.subscriber = self.create_subscription(Pose2D, '/detected_aruco', self.pose_cb, 10)
        self.frame_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.l_wheel_publisher = self.create_publisher(Wrench, '/hb_bot_1/left_wheel_force', 10)
        self.r_wheel_publisher = self.create_publisher(Wrench, '/hb_bot_1/right_wheel_force', 10)
        self.b_wheel_publisher = self.create_publisher(Wrench, '/hb_bot_1/rear_wheel_force', 10)
        self.l_wheel = Wrench()
        self.r_wheel = Wrench()
        self.b_wheel = Wrench()
        self.rate = self.create_rate(25)
        self.hb_x = 0
        self.hb_y = 0
        self.hb_theta = 0
        self.x_goal = 0
        self.y_goal = 0
        self.theta_goal = 0
        self.proportional = [5 , 5, 30.0]
        self.cli = self.create_client(NextGoal, 'next_goal')      
        self.req = NextGoal.Request() 
        self.index = 0
        self.future = None
        self.cv_bridge = CvBridge()
        
    def send_request(self):
        self.req.request_goal = self.index
        self.future = self.cli.call_async(self.req)
        
    def pose_cb(self, msg):
        self.hb_x = msg.x
        self.hb_y = msg.y
        self.hb_theta = msg.theta
    
    def rtn_mtrx(self, theta):
        return [[m.cos(theta), -m.sin(theta)],
                [m.sin(theta), m.cos(theta)]]

    def inverse_kinematics(self, vx, vy, w):
        self.l_wheel.force.y = -vx - vy + w
        self.r_wheel.force.y = -vx + vy + w
        self.b_wheel.force.y = vx + w

    def image_cb(self, msg):
        global pt_lst
        self.cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        for pt in pt_lst:
            cv2.circle(self.cv_img, pt, 2, (0,255,255), -1)
        cv2.putText(self.cv_img, 'Angle = ' + str(self.hb_theta),
            (20,40), cv2.FONT_HERSHEY_SIMPLEX,
            1, (255, 0, 255), 2)
        cv2.circle(self.cv_img, (int(self.hb_x), int(self.hb_y)), 2, (0,0,255), -1)
        cv2.putText(self.cv_img, '(' + str(self.hb_x) + ',' + str(self.hb_y) + ')',
            (int(self.hb_x), int(self.hb_y)), cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (255, 0, 0), 2)
        cv2.imshow('img', self.cv_img)
        cv2.waitKey(1)

pt_lst = []

def main(args=None):
    global pt_lst
    rclpy.init(args=args)    
    bot = HBController()   
    bot.send_request()    
    while rclpy.ok():
        if bot.future.done():
            try:
                resp = bot.future.result()
            except Exception as e:
                bot.get_logger().infselfo(
                    'Service call failed %r' % (e,))
            else:
                bot.x_goal      = resp.x_goal + 250
                bot.y_goal      = resp.y_goal + 250
                bot.theta_goal  = resp.theta_goal
                bot.flag = resp.end_of_list
                gf_x_err = bot.x_goal - bot.hb_x
                gf_y_err = -(bot.y_goal - bot.hb_y)   
                gf_theta_err = bot.theta_goal - bot.hb_theta
                rot_mtx = bot.rtn_mtrx(bot.hb_theta)
                bf_x_err = rot_mtx[0][0]*gf_x_err + rot_mtx[0][1]*gf_y_err
                bf_y_err = rot_mtx[1][0]*gf_x_err + rot_mtx[1][1]*gf_y_err
                vx = bot.proportional[0]*bf_x_err
                vy = bot.proportional[1]*bf_y_err
                w = bot.proportional[2]*gf_theta_err
                bot.inverse_kinematics(vx, vy, w)
                bot.l_wheel_publisher.publish(bot.l_wheel)
                bot.r_wheel_publisher.publish(bot.r_wheel)
                bot.b_wheel_publisher.publish(bot.b_wheel)
                if abs(bf_x_err) < 1 and abs(bf_y_err) < 1 :        
                    pt_lst.append((int(bot.hb_x), int(bot.hb_y)))
                    bot.index += 1
                    if bot.flag == 1 :
                        bot.index = 0
                    bot.send_request()
        rclpy.spin_once(bot)
    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
