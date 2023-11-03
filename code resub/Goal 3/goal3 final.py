import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn
from geometry_msgs.msg import Twist, Quaternion
from geometry_msgs.msg import PoseStamped
import math
import random
import numpy as np

class TurtleControlCircle(Node):
    def __init__(self):
        super().__init__("turtle_control_circle")
        self.kill_turtle_service = self.create_client(Kill, 'kill')
        self.spawn_turtle_service = self.create_client(Spawn, 'spawn')
        while not self.kill_turtle_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('turtle not killed')

        while not self.spawn_turtle_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('waiting for turtle spawn')
        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 10)
        self.real_pose_publisher = self.create_publisher(PoseStamped, '/rt_real_pose', 1)
        self.noisy_pose_publisher = self.create_publisher(PoseStamped, '/rt_noisy_pose', 1)
        self.pose_subscriber = self.create_subscription(
            Pose, 'turtle2/pose', self.callback_pose, 10)
        self.target_x = random.uniform(0.0, 11.0)
        self.target_y = random.uniform(0.0, 11.0)
        self.kp_linear = 2.0
        self.ki_linear = 0.0
        self.kd_linear = 0.0
        self.distance_threshold = 0.1
        self.kp_angular = 2.0
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
        self.max_linear_acceleration = 1.0
        self.max_linear_deceleration = -1.0
        self.circle_radius = 3.0  # Set your desired circle radius

    def spawn_turtle(self):
        self.kill_default_turtle()
        request = Spawn.Request()
        request.x = random.uniform(0.0, 11.0)
        request.y = random.uniform(0.0, 11.0)
        request.theta = random.uniform(0.0, 0.0)
        request.name = 'turtle2'
        future = self.spawn_turtle_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def kill_default_turtle(self):
        request = Kill.Request()
        request.name = 'turtle1'
        future = self.kill_turtle_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def callback_pose(self, msg):
        # Calculate the error for a circular path
        error_linear = self.circle_radius - self.distance_to_target(msg)
        error_angular = math.pi / 2  # Keep the angle constant to maintain a circular path
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
        if error_linear < self.distance_threshold:
            control_linear = max(self.min_linear_velocity, min(
                control_linear, self.max_linear_deceleration))
        else:
            control_linear = self.limit_acceleration(
                control_linear, self.prev_error_linear)

        control_angular = max(self.min_angular_velocity, min(
            self.max_angular_velocity, control_angular))

        cmd_vel = Twist()
        cmd_vel.linear.x = control_linear
        cmd_vel.angular.z = control_angular

        self.publisher.publish(cmd_vel)
        self.publish_real_pose(msg)
        self.publish_noisy_pose(msg, noise_std_dev=10)

    def distance_to_target(self, pose):
        return math.sqrt((self.target_x - pose.x) ** 2 + (self.target_y - pose.y) ** 2)

    def limit_acceleration(self, current_value, prev_value):
        delta_value = current_value - prev_value
        if delta_value > self.max_linear_acceleration:
            current_value = prev_value + self.max_linear_acceleration
        elif delta_value < self.max_linear_deceleration:
            current_value = prev_value + self.max_linear_deceleration
        return current_value

    def publish_real_pose(self, pose):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'  # Set the frame ID accordingly
        pose_msg.pose.position.x = pose.x
        pose_msg.pose.position.y = pose.y
        pose_msg.pose.orientation = Quaternion(w=1.0)
        self.real_pose_publisher.publish(pose_msg)
        self.get_logger().info(f'Real Pose: x={pose.x}, y={pose.y}')

    def publish_noisy_pose(self, pose, noise_std_dev):
        noisy_pose_msg = PoseStamped()
        noisy_pose_msg.header.stamp = self.get_clock().now().to_msg()
        noisy_pose_msg.header.frame_id = 'map'  # Set the frame ID accordingly
        noisy_pose_msg.pose.position.x = pose.x + random.gauss(0, noise_std_dev)
        noisy_pose_msg.pose.position.y = pose.y + random.gauss(0, noise_std_dev)
        noisy_pose_msg.pose.orientation = Quaternion(w=1.0)
        self.noisy_pose_publisher.publish(noisy_pose_msg)
        self.get_logger().info(f'Noisy Pose: x={noisy_pose_msg.pose.position.x}, y={noisy_pose_msg.pose.position.y}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControlCircle()
    node.spawn_turtle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
