#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_control')
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.goal = Point()
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.goal_reached = True
        self.path = [(0, 0), (11, 0), (11, 1), (0, 1), (0, 2), (11, 2), (11, 3), (0, 3), (0, 4), (11, 4), (11, 5), (5, 5)]
        self.path_index = 0
        self.acceleration_limit = 0.2
        self.deceleration_limit = -0.2

    def pose_callback(self, msg):
        if self.goal_reached:
            self.set_next_goal()
            self.goal_reached = False

        current_goal = self.path[self.path_index]
        error = math.sqrt((current_goal[0] - msg.x) ** 2 + (current_goal[1] - msg.y) ** 2)

        linear_vel = self.kp * error + self.ki * self.integral + self.kd * (error - self.prev_error)
        linear_vel = self.limit_velocity(linear_vel, self.acceleration_limit, self.deceleration_limit)

        angle_to_goal = math.atan2(current_goal[1] - msg.y, current_goal[0] - msg.x)
        angular_vel = 4 * (angle_to_goal - msg.theta)

        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        self.cmd_pub.publish(cmd_vel)

        self.integral += error
        self.prev_error = error

        if error < 0.1:
            self.get_logger().info(f'Turtle reached the goal at ({msg.x}, {msg.y})')
            self.goal_reached = True
            self.path_index += 1

    def set_next_goal(self):
        if self.path_index < len(self.path):
            next_goal = self.path[self.path_index]
            self.goal.x = next_goal[0]
            self.goal.y = next_goal[1]
            self.get_logger().info(f'New goal set at ({self.goal.x}, {self.goal.y})')

    def limit_velocity(self, velocity, acceleration_limit, deceleration_limit):
        if velocity > self.acceleration_limit:
            velocity = self.acceleration_limit
        elif velocity < self.deceleration_limit:
            velocity = self.deceleration_limit
        return velocity

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
