#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn
from geometry_msgs.msg import Twist
import math
import random


class TurtleControl(Node):
    def __init__(self):
        super().__init__("turtle_control")
        self.kill_turtle_service = self.create_client(Kill, 'kill')
        self.spawn_turtle_service = self.create_client(Spawn, 'spawn')
        while not self.kill_turtle_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('turtle not killed')

        while not self.spawn_turtle_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('waiting for turtle spawn')
        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, 'turtle2/pose', self.callback_pose, 10)
        self.target_x = random.uniform(0.0, 11.0)
        self.target_y = random.uniform(0.0, 11.0)
        self.kp_linear = 1.0
        self.ki_linear = 0.0
        self.kd_linear = 0.0
        self.distance_threshold = 0.1
        self.kp_angular = 1.0
        self.ki_angular = 0.0
        self.kd_angular = 0.0

        self.prev_error_linear = 0.0
        self.integrated_error_linear = 0.0
        self.prev_error_angular = 0.0
        self.integrated_error_angular = 0.0

        self.max_linear_velocity = 2.0
        self.min_linear_velocity = -2.0
        self.max_angular_velocity = 2.0
        self.min_angular_velocity = -2.0

    def callback_pose(self, msg):

        distance = math.sqrt((self.target_x - msg.x) **
                             2 + (self.target_y - msg.y) ** 2)
        angle = math.atan2(self.target_y - msg.y,
                           self.target_x - msg.x) - msg.theta
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi

        error_linear = distance
        error_angular = angle
        self.integrated_error_linear += error_linear
        self.integrated_error_angular += error_angular
        control_linear = self.kp_linear * error_linear + self.ki_linear * \
            self.integrated_error_linear + self.kd_linear * \
            (error_linear - self.prev_error_linear)
        control_angular = self.kp_angular * error_angular + self.ki_angular * \
            self.integrated_error_angular + self.kd_angular * \
            (error_angular - self.prev_error_angular)
        self.prev_error_linear = error_linear
        self.prev_error_angular = error_angular

        control_linear = max(self.min_linear_velocity, min(
            self.max_linear_velocity, control_linear))
        control_angular = max(self.min_angular_velocity, min(
            self.max_angular_velocity, control_angular))

        cmd_vel = Twist()
        cmd_vel.linear.x = control_linear
        cmd_vel.angular.z = control_angular

        if distance < self.distance_threshold:
            self.get_logger().info('Reached the target')
        self.publisher.publish(cmd_vel)

    def kill_default_turtle(self):
        request = Kill.Request()
        request.name = 'turtle1'
        future = self.kill_turtle_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Killed the default turtle')
        else:
            self.get_logger().error('Failed to kill the default turtle')

    def spawn_turtle(self):
        self.kill_default_turtle()
        request = Spawn.Request()
        request.x = random.uniform(0.0, 11.0)
        request.y = random.uniform(0.0, 11.0)
        request.theta = random.uniform(0.0, 0.0)
        request.name = 'turtle2'
        future = self.spawn_turtle_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControl()
    node.spawn_turtle()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
