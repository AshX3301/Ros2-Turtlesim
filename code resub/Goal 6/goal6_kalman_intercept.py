import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn
from geometry_msgs.msg import Twist
import math
import random
import numpy as np
import time
from filterpy.kalman import KalmanFilter
from scipy.interpolate import interp1d


class TurtleCircleChaseNoisy(Node):
    def __init__(self):
        super().__init__('turtle_chase_noisy')
        self.publisher = self.create_publisher(
            Twist, 'police_turtle/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, 'police_turtle/pose', self.police_turtle, 10)
        self.pose_subscriber_robber = self.create_subscription(
            Pose, 'robber_turtle/pose', self.turtle_capture_control, 10)
        self.subscriber_np = self.create_subscription(
            Pose, '/rt_noisy_pose', self.robberturtle_pose, 10)

        self.kp_linear = 2.0
        self.ki_linear = 0.0
        self.kd_linear = 0.0
        self.distance_threshold = 0.55

        self.kp_angular = 2.0
        self.ki_angular = 0.0
        self.kd_angular = 0.0

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.predicted_x = 0.0
        self.predicted_y = 0.0
        self.predicted_theta = 0.0

        self.police_x = 0.0
        self.police_y = 0.0
        self.police_theta = 0.0

        self.prev_error_linear = 0.0
        self.integrated_error_linear = 0.0
        self.prev_error_angular = 0.0
        self.integrated_error_angular = 0.0

        self.max_acceleration = 2.0
        self.max_deceleration = 2.0

        self.max_linear_velocity = 2.5
        self.min_linear_velocity = -2.0
        self.max_angular_velocity = 5.0
        self.min_angular_velocity = -2.0

        self.current_linear_velocity = 0.0
        self.intercept_time = 5.0
        self.robber_passed_point = False
        self.kalman_filter = PoseEstimationKalman()

    ''' def predict_future_pose(self):
        radius = self.linear_velocity / (2 * math.pi)
        angular_displacement = self.angular_velocity * self.intercept_time
        self.predicted_x = self.real_pose_x + radius * \
            math.cos(self.real_pose_theta + angular_displacement)
        self.predicted_y = self.real_pose_y + radius * \
            math.sin(self.real_pose_theta + angular_displacement)
        self.predicted_theta = self.real_pose_theta + angular_displacement
        #self.get_logger().info(f'FilterOut: ({real_pose[0]},{real_pose[1]},{real_pose[2]})')
        return self.predicted_x, self.predicted_y, self.predicted_theta'''

    def robberturtle_pose(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_theta = msg.theta
        self.linear_velocity = msg.linear_velocity
        self.angular_velocity = msg.angular_velocity
        noisy_measurement = np.array(
            [[self.target_x], [self.target_y], [self.target_theta]])
        self.estimated_original_pose = self.kalman_filter.update_pose(
            noisy_measurement)
        self.get_logger().info(f'{self.estimated_original_pose}')
        self.predicted_x, self.predicted_y, self.predicted_theta = self.estimated_original_pose

    def police_turtle(self, msg):
        self.police_x = msg.x
        self.police_y = msg.y
        self.police_theta = msg.theta
        distance = math.sqrt((self.predicted_x - msg.x) **
                             2 + (self.predicted_y - msg.y) ** 2)

        angle = math.atan2(self.predicted_y - msg.y,
                           self.predicted_x - msg.x) - msg.theta
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi

        error_linear = distance
        error_angular = angle
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

        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = control_linear
        self.cmd_vel.angular.z = control_angular

        if distance <= self.distance_threshold:
            # Set the flag to indicate that the robber turtle passed through the predicted point
            self.robber_passed_point = True

        # Check if the robber turtle has passed through the predicted point
        if self.robber_passed_point:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.current_linear_velocity = 0.0
            self.robber_passed_point = False  # Reset the flag

        self.publisher.publish(self.cmd_vel)

    def turtle_capture_control(self, msg):
        distancex = math.sqrt((msg.x - self.police_x) **
                              2 + (msg.y - self.police_y) ** 2)
        # self.get_logger().info(f'distance :{distancex}')
        if distancex <= self.distance_threshold:
            self.kill_turtle()
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.current_linear_velocity = 0.0

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
        time.sleep(9.9)
        self.spawn_turtle_service = self.create_client(Spawn, 'spawn')
        request = Spawn.Request()
        request.x = 0.0
        request.y = 0.0
        request.theta = random.uniform(0.0, 2*math.pi)
        request.name = 'police_turtle'
        future = self.spawn_turtle_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)


class PoseEstimationKalman:
    def __init__(self):
        self.dt = 5.0  # Time step
        self.kf = KalmanFilter(dim_x=3, dim_z=3)

        # state transition matrix
        self.kf.F = np.array([[1, 0, self.dt],
                              [0, 1, self.dt],
                              [0, 0, 1]])

        # observation matrix
        self.kf.H = np.array([[1, 0, 0],
                              [0, 1, 0],
                              [0, 0, 1]])

        # process noise covariance matrix
        self.kf.Q = np.eye(3) * (0.01 * self.dt)

        # measurement noise covariance matrix
        self.kf.R = np.array([[10, 0, 0],
                              [0, 10, 0],
                              [0, 0, 10]])

        # state estimate [x, y, theta]
        self.kf.x = np.array([[0], [0], [0]])

        # state covariance matrix
        self.kf.P = np.eye(3) * 1000

    def update_pose(self, noisy_measurement):
        # predict
        self.kf.predict()
        self.kf.update(noisy_measurement)

        # estimated state
        estimated_pose = self.kf.x.flatten()
        return estimated_pose


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCircleChaseNoisy()
    node.spawn_turtle()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
