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
        self.distance_threshold = 0.1
        self.linear_speed = 1.0

    def callback_pose(self, msg):

        distance = math.sqrt((self.target_x - msg.x) **
                             2 + (self.target_y - msg.y) ** 2)
        angle = math.atan2(self.target_y - msg.y,
                           self.target_x - msg.x) - msg.theta
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        cmd_vel = Twist()
        if distance > self.distance_threshold:
            cmd_vel.linear.x = self.linear_speed
            cmd_vel.angular.z = angle
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
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
