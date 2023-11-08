import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn
from geometry_msgs.msg import Twist
import math
import random
import time


class TurtleCircleChasePredictive(Node):

    def __init__(self):
        super().__init__('turtle_chase_predictive')
        self.publisher = self.create_publisher(
            Twist, 'police_turtle/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, 'police_turtle/pose', self.police_turtle, 10)
        self.subscriber_rp = self.create_subscription(
            Pose, '/rt_real_pose', self.robberturtle_pose, 10)

        self.kp_linear = 2.0
        self.ki_linear = 0.0
        self.kd_linear = 0.0
        self.distance_threshold = 0.3

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
        self.max_angular_velocity = 5.0
        self.min_angular_velocity = -2.0

        self.current_linear_velocity = 0.0

        # Predictive control parameters
        self.prediction_steps = 5
        self.prediction_interval = 0.1  # Time interval for predictions

        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0

    def robberturtle_pose(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_theta = msg.theta

    def police_turtle(self, msg):  # Predict the future position of the robber turtle
        predicted_x = self.target_x
        predicted_y = self.target_y
        predicted_theta = self.target_theta

        for _ in range(self.prediction_steps):  # Update the predicted position
            predicted_x += self.target_linear_velocity * \
                math.cos(predicted_theta) * self.prediction_interval
            predicted_y += self.target_linear_velocity * \
                math.sin(predicted_theta) * self.prediction_interval
            predicted_theta += self.target_angular_velocity * self.prediction_interval

        distance = math.sqrt((predicted_x - msg.x) **
                             2 + (predicted_y - msg.y) ** 2)
        # self.get_logger().info(f'ETA: {distance}')
        angle = math.atan2(predicted_y - msg.y,
                           predicted_x - msg.x) - msg.theta

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

        cmd_vel = Twist()
        cmd_vel.linear.x = control_linear
        cmd_vel.angular.z = control_angular

        if distance <= self.distance_threshold:
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
        self.get_logger().info('Got em!')
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


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCircleChasePredictive()
    time.sleep(10.0)  # 10 second delay
    node.spawn_turtle()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
