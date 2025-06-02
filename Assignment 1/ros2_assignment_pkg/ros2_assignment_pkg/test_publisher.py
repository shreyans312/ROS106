#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros2_assignment_pkg.msg import RobotPose2D
import math

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(RobotPose2D, '/robot_a/raw_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)
        self.count = 0
        self.get_logger().info('Test Publisher started, publishing to /robot_a/raw_pose')

    def publish_pose(self):
        msg = RobotPose2D()
        msg.x = 1.0 + 0.1 * math.sin(self.count * 0.1)
        msg.y = 2.0 + 0.1 * math.cos(self.count * 0.1)
        msg.theta = (self.count * 0.1) % (2 * math.pi)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()