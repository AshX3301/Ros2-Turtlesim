import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn
from geometry_msgs.msg import Twist
import math
import random
import numpy as np
import time


class TurtleCircleChaseNoisy(Node):
    def __init__(self):
        super().__init__('turtle_chase_noisy')
        self.publisher = self.create_publisher(
            Twist, 'police_turtle/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, 'police_turtle/pose', self.police_turtle, 10)
        self.subscriber_np = self.create_subscription(
            Pose, '/rt_noisy_pose', self.robberturtle_pose, 10)

        self.kp_linear = 2.0
        self.ki_linear = 0.0
        self.kd_linear = 0.0
        self.distance_threshold = 0.5

        self.kp_angular = 2.0
        self.ki_angular = 0.0
        self.kd_angular = 0.0

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0

        self.prev_error_linear = 0.0
        self.integrated_error_linear = 0.0
        self.prev_error_angular = 0.0
        self.integrated_error_angular = 0.0

        self.max_acceleration = 2.0
        self.max_deceleration = 2.0

        self.max_linear_velocity = 2.5
        self.min_linear_velocity = -2.0
        self.max_angular_velocity = 10.0
        self.min_angular_velocity = -2.0

        self.current_linear_velocity = 0.0
        self.prediction_steps = 5
        self.prediction_interval = 0.1
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0

        # Kalman filter initialization
        self.kalman_filter = KalmanFilter()

    def robberturtle_pose(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_theta = msg.theta
        self.kalman_filter.update_state(
            np.array([[self.target_x], [self.target_y], [self.target_theta]]))

    def police_turtle(self, msg):
        self.kalman_filter.predict_state()

        real_pose = self.kalman_filter.get_estimated_state()
        #print(real_pose[0])
        predicted_x = real_pose[0]
        predicted_y = real_pose[1]
        predicted_theta = real_pose[2]

        for _ in range(self.prediction_steps):  # Update the predicted position
            predicted_x += self.target_linear_velocity * \
                math.cos(predicted_theta) * self.prediction_interval
            predicted_y += self.target_linear_velocity * \
                math.sin(predicted_theta) * self.prediction_interval
            predicted_theta += self.target_angular_velocity * self.prediction_interval

        distance = math.sqrt((predicted_x - msg.x) **
                             2 + (predicted_y - msg.y) ** 2)
        self.get_logger().info(f'{distance}')
        angle = math.atan2(predicted_y - msg.y,
                           predicted_x - msg.x) - msg.theta
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi

        error_linear = distance
        error_angular = 2.0 * angle
        self.integrated_error_linear += error_linear
        self.integrated_error_angular += error_angular

        desired_linear_velocity = self.max_linear_velocity
        acceleration = self.kp_linear * \
            (desired_linear_velocity - self.current_linear_velocity)
        if acceleration > self.max_acceleration:
            acceleration = self.max_acceleration
        if acceleration < -self.max_deceleration:
            acceleration = -self.max_deceleration

        self.current_linear_velocity += acceleration
        self.current_linear_velocity = max(self.min_linear_velocity, min(
            self.max_linear_velocity, self.current_linear_velocity))

        control_linear = self.kp_linear * error_linear + self.ki_linear * \
            self.integrated_error_linear + self.kd_linear * \
            (error_linear - self.prev_error_linear)
        control_linear = max(self.min_linear_velocity, min(
            self.current_linear_velocity, control_linear))

        control_angular = self.kp_angular * error_angular + self.ki_angular * \
            self.integrated_error_angular + self.kd_angular * \
            (error_angular - self.prev_error_angular)

        self.prev_error_linear = error_linear
        self.prev_error_angular = error_angular

        cmd_vel = Twist()
        cmd_vel.linear.x = control_linear
        cmd_vel.angular.z = control_angular

        if distance == self.distance_threshold:
            self.get_logger().info('Got em!')
            self.kill_turtle()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.current_linear_velocity = 0.0

        self.publisher.publish(cmd_vel)

    def kill_turtle(self):
        self.kill_turtle_service = self.create_client(Kill, 'kill')
        request = Kill.Request()
        request.name = 'robber_turtle'
        future = self.kill_turtle_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Killed the turtle')
        else:
            self.get_logger().error('Failed to kill the default turtle')

    def spawn_turtle(self):
        self.spawn_turtle_service = self.create_client(Spawn, 'spawn')
        request = Spawn.Request()
        request.x = 0.0
        request.y = 0.0
        request.theta = random.uniform(0.0, 2*math.pi)
        request.name = 'police_turtle'
        future = self.spawn_turtle_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)


class KalmanFilter:
    def __init__(self):
        self.dt = 0.1  # Your time step
        self.A = np.array([[1, 0, self.dt],
                           [0, 1, self.dt],
                           [0, 0, 1]])  # State transition matrix
        self.H = np.eye(3)  # Observation matrix
        self.Q = np.eye(3) * 0.01  # Process noise covariance matrix
        self.R = np.eye(3) * 10  # Measurement noise covariance matrix

        self.x = np.zeros((3, 1))  # State estimate [x, y, theta]
        self.P = np.eye(3)  # State covariance matrix

    def predict_state(self):
        # Prediction step
        self.x = np.dot(self.A, self.x)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

    def update_state(self, z):
        # Update step
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(
            np.dot(np.dot(self.H, self.P), self.H.T) + self.R))
        self.x = self.x + np.dot(K, (z - np.dot(self.H, self.x)))
        self.P = np.dot((np.eye(3) - np.dot(K, self.H)), self.P)

    def get_estimated_state(self):
        return self.x.flatten()


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCircleChaseNoisy()
    #time.sleep(10.0)
    node.spawn_turtle()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
