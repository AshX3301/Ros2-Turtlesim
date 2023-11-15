import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn
from geometry_msgs.msg import Twist
import math
import random
import time


class TurtleCircleChase(Node):

    def __init__(self):
        super().__init__('turtle_chase')
        self.publisher = self.create_publisher(
            Twist, 'police_turtle/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, 'police_turtle/pose', self.police_turtle, 10)
        self.pose_subscriber_robber = self.create_subscription(
            Pose, 'robber_turtle/pose', self.turtle_capture_control, 10)
        self.subscriber_rp = self.create_subscription(
            Pose, '/rt_real_pose', self.robberturtle_pose, 1) 

        self.kp_linear = 2.0
        self.ki_linear = 0.0
        self.kd_linear = 0.0
        self.distance_threshold = 0.1

        self.kp_angular = 2.0
        self.ki_angular = 0.0
        self.kd_angular = 0.0

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0


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
        self.intercept_time = 10.0
        

    def predict_future_pose(self):
        radius = self.linear_velocity / (2*math.pi)
        angular_displacement = self.angular_velocity * self.intercept_time
        predicted_x = self.target_x + radius * math.cos(self.target_theta + angular_displacement)
        predicted_y = self.target_y + radius * math.sin(self.target_theta + angular_displacement)
        predicted_theta = self.target_theta + angular_displacement
        #self.target_x, self.target_y, self.target_theta = predicted_x, predicted_y, predicted_theta
        return predicted_x, predicted_y, predicted_theta

    def robberturtle_pose(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_theta = msg.theta
        self.linear_velocity = msg.linear_velocity
        self.angular_velocity = msg.angular_velocity
        self.predict_future_pose()

    def police_turtle(self, msg):
        self.police_x = msg.x
        self.police_y = msg.y
        self.police_theta = msg.theta
        predicted_x, predicted_y, predicted_theta = self.predict_future_pose()
        self.target_x, self.target_y, self.target_theta = predicted_x, predicted_y, predicted_theta
        distance = math.sqrt((self.target_x - msg.x) **
                             2 + (self.target_y - msg.y) ** 2)
        #self.get_logger().info(f'distance to intercept :{distance}')

        angle = math.atan2(self.target_y - msg.y,
                           self.target_x - msg.x) - msg.theta
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi

        error_linear = 4.5 * distance
        error_angular = 5.5 * angle
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
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.current_linear_velocity = 0.0
        self.publisher.publish(self.cmd_vel)
        

    def turtle_capture_control(self, msg):
        distancex = math.sqrt((msg.x - self.police_x) **
                             2 + (msg.y - self.police_y) ** 2)
        self.get_logger().info(f'distance :{distancex}')
        if distancex <= self.distance_threshold:
            self.kill_turtle()
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.current_linear_velocity = 0.0

    def spawn_turtle(self):
        self.spawn_turtle_service = self.create_client(Spawn, 'spawn')
        request = Spawn.Request()
        request.x = 0.0
        request.y = 0.0
        request.theta = random.uniform(0.0, 2*math.pi)
        request.name = 'police_turtle'
        future = self.spawn_turtle_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Spawned the turtle')
        else:
            self.get_logger().error('Failed to spawn the turtle')

    def kill_turtle(self):
        self.kill_turtle_service = self.create_client(Kill, 'kill')
        request = Kill.Request()
        request.name = 'robber_turtle'
        future = self.kill_turtle_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Killed the turtle')
        else:
            self.get_logger().error('Failed to kill the turtle')


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCircleChase()
    #time.sleep(10.0)
    node.spawn_turtle()  # Spawn turtle
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
