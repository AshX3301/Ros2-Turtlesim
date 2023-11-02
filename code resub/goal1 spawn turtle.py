#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Kill, Spawn
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
