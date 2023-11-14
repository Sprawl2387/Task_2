#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''


# Team ID:		[ 3695 ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node

#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal

class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')

        self.left_wheel_force_pub = self.create_publisher(Twist, '/hb_bot_1/left_wheel_force', 10)
        self.right_wheel_force_pub = self.create_publisher(Twist, '/hb_bot_1/right_wheel_force', 10)
        self.back_wheel_force_pub = self.create_publisher(Twist, '/hb_bot_1/back_wheel_force', 10)

        self.rate = self.create_rate(100)
        self.cli = self.create_client(NextGoal, 'next_goal')      
        self.req = NextGoal.Request() 
        self.index = 0

    def send_request(self, request_goal):
        self.req.request_goal = request_goal
        self.future = self.cli.call_async(self.req)

    def inverse_kinematics(self, target_velocity):
        # Assume a simple proportional control for inverse kinematics
        left_force = 0.5 * target_velocity
        right_force = 0.5 * target_velocity
        back_force = 0.5 * target_velocity

        # Publish the calculated efforts to actuate the robot
        left_msg = Twist()
        left_msg.linear.x = left_force
        self.left_wheel_force_pub.publish(left_msg)

        right_msg = Twist()
        right_msg.linear.x = right_force
        self.right_wheel_force_pub.publish(right_msg)

        back_msg = Twist()
        back_msg.linear.x = back_force
        self.back_wheel_force_pub.publish(back_msg)

    def calculate_error(self, x_goal, y_goal, theta_goal):
        # Assume you have access to the current robot pose (replace this with your actual pose retrieval logic)
        current_pose = self.get_current_pose()

        # Calculate the error based on the goal and current pose
        error_x = x_goal - current_pose.x
        error_y = y_goal - current_pose.y
        error_theta = theta_goal - current_pose.theta

        # Return the calculated error
        return error_x, error_y, error_theta

    def calculate_target_velocity(self, error):
        # Assume proportional control for simplicity (replace this with your actual control algorithm)
        proportional_gain = 0.1
        target_velocity = proportional_gain * error

        # Return the calculated target velocity
        return target_velocity

    def get_current_pose(self):
        # Assume a simple way to get the current pose (replace this with your actual pose retrieval logic)
        # For example, you may be subscribing to the Odometry topic
        # Here, I'm creating a hypothetical Pose object with some values
        current_pose = Pose(x=1.0, y=2.0, theta=0.5)
        return current_pose

    def main_loop(self):
        while rclpy.ok():
            if self.future.done():
                try:
                    response = self.future.result()
                except Exception as e:
                    self.get_logger().info('Service call failed %r' % (e,))
                else:
                    x_goal = response.x_goal
                    y_goal = response.y_goal
                    theta_goal = response.theta_goal
                    self.flag = response.end_of_list

                    # Calculate Error from feedback
                    error = self.calculate_error(x_goal, y_goal, theta_goal)

                    # Calculate the required velocity of the robot for the next iteration(s)
                    target_velocity = self.calculate_target_velocity(error)

                    # Apply inverse kinematics to get wheel forces
                    self.inverse_kinematics(target_velocity)

                    # Modify the condition to Switch to Next goal
                    self.index += 1
                    if self.flag == 1:
                        self.index = 0
                    self.send_request(self.index)

            # Spin once to process callbacks
            rclpy.spin_once(self)
            self.rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    
    hb_controller = HBController()
    hb_controller.send_request(hb_controller.index)

    # Main loop
    hb_controller.main_loop()

    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
