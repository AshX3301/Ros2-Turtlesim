#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, PoseStamped
import random
import math

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("turtle_circle")
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.real_pose_pub = self.create_publisher(PoseStamped, "/rt_real_pose", 10)
        self.noisy_pose_pub = self.create_publisher(PoseStamped, "/rt_noisy_pose", 10)
        self.timer = self.create_timer(0.1, self.send_velocity_command)
        self.current_angle = 0.0
        self.radius = 1.0
        self.angular_speed = 1.0
        self.linear_speed = 2.0  
        self.noise_std_dev = 10.0
        self.get_logger().info("Circle drawing started")

    def send_velocity_command(self):
        msg = Twist()

        
        self.current_angle += self.angular_speed
        x = self.radius * math.cos(self.current_angle)
        y = self.radius * math.sin(self.current_angle)

        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed

        self.cmd_vel_pub.publish(msg)

        # Publish real pose
        real_pose = PoseStamped()
        real_pose.header.stamp = self.get_clock().now().to_msg()
        real_pose.pose.position.x = x
        real_pose.pose.position.y = y
        real_pose.pose.orientation.w = 1.0  
        self.real_pose_pub.publish(real_pose)
        self.get_logger().info(f'Real Pose: x={real_pose.pose.position.x}, y={real_pose.pose.position.y}, theta={real_pose.pose.orientation.w}')

        # Publish noisy pose
        noisy_x = x + random.gauss(0, self.noise_std_dev)
        noisy_y = y + random.gauss(0, self.noise_std_dev)

        noisy_pose = PoseStamped()
        noisy_pose.header.stamp = self.get_clock().now().to_msg()
        noisy_pose.pose.position.x = noisy_x
        noisy_pose.pose.position.y = noisy_y
        noisy_pose.pose.orientation.w = random.gauss(0, self.noise_std_dev)
        self.noisy_pose_pub.publish(noisy_pose)
        self.get_logger().info(f'Noisy Pose: x={noisy_pose.pose.position.x}, y={noisy_pose.pose.position.y}, theta={noisy_pose.pose.orientation.w}')

def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
