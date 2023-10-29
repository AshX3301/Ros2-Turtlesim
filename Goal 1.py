#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
import random
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_control')
        self.cmd_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.goal = Point()
        self.goal.x = 11.0  
        self.goal.y = 11.0  
        self.kp = 1.0  
        self.ki = 0.0  
        self.kd = 0.0  
        self.prev_error = 0.0
        self.integral = 0.0

    def pose_callback(self, msg):
        # error
        error = math.sqrt((self.goal.x - msg.x) ** 2 + (self.goal.y - msg.y) ** 2)
        
        # control input using PID
        linear_vel = self.kp * error + self.ki * self.integral + self.kd * (error - self.prev_error)
        
        # angular velocity
        angle_to_goal = math.atan2(self.goal.y - msg.y, self.goal.x - msg.x)
        angular_vel = 4 * (angle_to_goal - msg.theta)
        
        # Publish control input
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        self.cmd_pub.publish(cmd_vel)

        # Update integral and previous error
        self.integral += error
        self.prev_error = error

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
