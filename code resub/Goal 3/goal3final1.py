import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn
from geometry_msgs.msg import Twist
import math
import random


class TurtleCircleChase(Node):

    def __init__(self):
        super().__init__('robber_turtle')
        self.publisher = self.create_publisher(
            Twist, 'robber_turtle/cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.publish_velocity)
        self.pose_subscriber = self.create_subscription(
            Pose, 'robber_turtle/pose', self.robberturtle_pose, 10)
        self.publisher_rp = self.create_publisher(Pose, '/rt_real_pose', 10)
        self.publisher_np = self.create_publisher(Pose, '/rt_noisy_pose', 10)
        self.rt_linear_velocity = 5.0  
        self.radius = 3.0  
        self.current_angle = 0.0  
        self.turtle2_spawned = False
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

        self.max_linear_velocity = 5.0
        self.min_linear_velocity = -2.0
        self.max_angular_velocity = 5.0
        self.min_angular_velocity = -2.0

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.rt_linear_velocity
        msg.angular.z = self.rt_linear_velocity / self.radius
        self.current_angle += msg.angular.z * 0.1
        if self.current_angle >= 2.0 * math.pi:
            # self.radius = random.uniform(1.0, 3.0)
            self.current_angle = 0.0
        self.publisher.publish(msg)
# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    def robberturtle_pose(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_theta = msg.theta
        real_pose = Pose()
        real_pose.x = msg.x
        real_pose.y = msg.y
        real_pose.theta = msg.theta
        self.publisher_rp.publish(real_pose)

        noisy_pose = Pose()
        noisy_pose.x = msg.x + random.gauss(0, 10)
        noisy_pose.y = msg.y + random.gauss(0, 10)
        noisy_pose.theta = msg.theta + random.gauss(0, 10)
        self.publisher_np.publish(noisy_pose)

    def kill_default_turtle(self):
        self.kill_turtle_service = self.create_client(Kill, 'kill')
        request = Kill.Request()
        request.name = 'turtle1'
        future = self.kill_turtle_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Killed the turtle')
        else:
            self.get_logger().error('Failed to kill the default turtle')

    def spawn_turtle(self):
        self.kill_default_turtle()
        self.spawn_turtle_service = self.create_client(Spawn, 'spawn')
        request = Spawn.Request()
        request.x = random.uniform(0.0, 11.0)
        request.y = random.uniform(0.0, 11.0)
        request.theta = random.uniform(0.0, 2*math.pi)
        request.name = 'robber_turtle'
        future = self.spawn_turtle_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCircleChase()
    node.spawn_turtle()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
