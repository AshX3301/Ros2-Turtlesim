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
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.goal_reached = True
        self.environment_x_max = 11.0  # maximum X coordinate for the environment
        self.environment_y_max = 11.0  # maximum Y coordinate for the environment

    def pose_callback(self, msg):
        self.get_logger().info(f'Turtle Pose: x={msg.x}, y={msg.y}, theta={msg.theta}')

        if self.goal_reached:
            self.set_random_goal()
            self.goal_reached = False

        error = math.sqrt((self.goal.x - msg.x) ** 2 + (self.goal.y - msg.y) ** 2)
        
        # PID
        linear_vel = self.kp * error + self.ki * self.integral + self.kd * (error - self.prev_error)
        
        # Angular velocity
        angle_to_goal = math.atan2(self.goal.y - msg.y, self.goal.x - msg.x)
        angular_vel = 4 * (angle_to_goal - msg.theta)
        
        
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        self.cmd_pub.publish(cmd_vel)

        self.integral += error
        self.prev_error = error

    def set_random_goal(self):
       
        self.goal.x = random.uniform(1.0, self.environment_x_max)
        self.goal.y = random.uniform(1.0, self.environment_y_max)
        
        self.get_logger().info(f'Random Goal: x={self.goal.x}, y={self.goal.y}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
