#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
import random
import math
import time
import csv

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
        self.environment_x_max = 11.0
        self.environment_y_max = 11.0

        self.csv_file = open('/home/ashwin/ros2_ws/turtle_data.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time', 'Linear Velocity', 'Angular Velocity', 'X', 'Y', 'Error', 'Integral of Error'])

    def pose_callback(self, msg):
        time_now = time.time()
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

        self.get_logger().info(f'Time: {time_now}')
        self.get_logger().info(f'Control Inputs - Linear Velocity: {linear_vel}, Angular Velocity: {angular_vel}')
        self.get_logger().info(f'Turtle Pose - X: {msg.x}, Y: {msg.y}')
        self.get_logger().info(f'Error: {error}, Integral of Error: {self.integral}')

        self.csv_writer.writerow([time_now, linear_vel, angular_vel, msg.x, msg.y, error, self.integral])

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
