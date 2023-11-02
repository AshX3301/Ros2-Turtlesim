import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn
from geometry_msgs.msg import Twist
import math
import random


class TurtleControlGrid(Node):
    def __init__(self):
        super().__init__("turtle_control_grid")
        self.kill_turtle_service = self.create_client(Kill, 'kill')
        self.spawn_turtle_service = self.create_client(Spawn, 'spawn')
        while not self.kill_turtle_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('turtle not killed')

        while not self.spawn_turtle_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('waiting for turtle spawn')
        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, 'turtle2/pose', self.callback_pose, 10)

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

        self.max_linear_velocity = 5.0
        self.min_linear_velocity = -2.0
        self.max_angular_velocity = 2.0
        self.min_angular_velocity = -2.0

        # Acceleration and deceleration limits
        self.max_linear_acceleration = 1.0
        self.max_linear_deceleration = -1.0

        # grid pattern waypoints
        self.grid_pattern = [
            (0, 0), (11, 0), (11, 1), (0, 1), (0, 2), (11, 2),
            (11, 3), (0, 3), (0, 4), (11, 4), (11, 5), (0, 5),
            (0, 6), (11, 6), (11, 7), (0, 7), (0, 8), (11, 8),
            (11, 9), (0, 9), (0, 10), (11, 10), (11, 11)
        ]
        self.current_waypoint_index = 0

        self.target_x = self.grid_pattern[self.current_waypoint_index][0]
        self.target_y = self.grid_pattern[self.current_waypoint_index][1]
        self.goal_orientation = 0.0

        self.reached_goal = False

    def callback_pose(self, msg):
        x, y = self.grid_pattern[self.current_waypoint_index]
        distance = math.sqrt((x - msg.x) ** 2 + (y - msg.y) ** 2)

        if not self.reached_goal:
            goal_angle = math.atan2(y - msg.y, x - msg.x)
            error_angular = goal_angle - msg.theta

            if error_angular > math.pi:
                error_angular -= 2 * math.pi
            elif error_angular < -math.pi:
                error_angular += 2 * math.pi

            control_angular = self.kp_angular * error_angular + self.ki_angular * \
                self.integrated_error_angular + self.kd_angular * \
                (error_angular - self.prev_error_angular)
            self.prev_error_angular = error_angular
            control_angular = max(self.min_angular_velocity, min(
                self.max_angular_velocity, control_angular))

            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = control_angular
            self.publisher.publish(cmd_vel)
            if abs(error_angular) < 0.01:
                self.reached_goal = True
        else:
            error_linear = distance
            self.integrated_error_linear += error_linear
            control_linear = self.kp_linear * error_linear + self.ki_linear * \
                self.integrated_error_linear + self.kd_linear * \
                (error_linear - self.prev_error_linear)
            self.prev_error_linear = error_linear
            control_linear = max(self.min_linear_velocity, min(
                self.max_linear_velocity, control_linear))

            if distance < self.distance_threshold:
                control_linear = max(self.min_linear_velocity, min(
                    control_linear, self.max_linear_deceleration))
            else:
                control_linear = self.limit_acceleration(
                    control_linear, self.prev_error_linear)

            control_angular = 0.0
            cmd_vel = Twist()
            cmd_vel.linear.x = control_linear
            cmd_vel.angular.z = control_angular
            self.publisher.publish(cmd_vel)
            if distance < self.distance_threshold:
                self.get_logger().info('Reached the waypoint')
                self.current_waypoint_index = (
                    self.current_waypoint_index + 1) % len(self.grid_pattern)
                if self.current_waypoint_index == 0:
                    self.goal_orientation += math.pi / 2.0
                self.reached_goal = False

    def limit_acceleration(self, current_value, prev_value):
        delta_value = current_value - prev_value
        if delta_value > self.max_linear_acceleration:
            current_value = prev_value + self.max_linear_acceleration
        elif delta_value < self.max_linear_deceleration:
            current_value = prev_value + self.max_linear_deceleration
        return current_value

    def kill_default_turtle(self):
        request = Kill.Request()
        request.name = 'turtle1'
        future = self.kill_turtle_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

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
    node = TurtleControlGrid()
    node.spawn_turtle()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
