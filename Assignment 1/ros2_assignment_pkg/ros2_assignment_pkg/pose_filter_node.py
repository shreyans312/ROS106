#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from collections import deque
import math

from ros2_assignment_pkg.msg import RobotPose2D

class PoseFilterNode(Node):
    def __init__(self):
        super().__init__('pose_filter_node')
        self.subscription = self.create_subscription(
            RobotPose2D,
            '/robot_a/raw_pose',
            self.raw_pose_callback,
            10
        )
        self.publisher = self.create_publisher(
            RobotPose2D,
            '/robot_a/filtered_pose',
            10
        )

        self.pose_buffer = deque(maxlen = 10)
        self.timer = self.create_timer(0.1, self.publish_filtered_pose)
        self.get_logger().info('Pose Filter Node has started')
        self.get_logger().info('Subscribing to: /robot_a/raw_pose')
        self.get_logger().info('Publishing to: /robot_a/filtered_pose')

    def raw_pose_callback(self, msg):
        pose_data = {'x':msg.x, 'y':msg.y, 'theta':msg.theta,}
        self.pose_buffer.append(pose_data)
        self.get_logger().debug(f'Received pose: x ={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}')
        self.get_logger().debug(f'Buffer size: {len(self.pose_buffer)}/10')

    def average_angles(self, angles):
        sin_sum = sum(math.sin(angle) for angle in angles)
        cos_sum = sum(math.cos(angle) for angle in angles)
        mean_sin = sin_sum / len(angles)
        mean_cos = cos_sum / len(angles)

        return math.atan2(mean_sin, mean_cos)
    
    def publish_filtered_pose(self):
        if len(self.pose_buffer) < 10:
            self.get_logger().debug(f'Buffer is not full yet: {len(self.pose_buffer)}/10')
            return
        
        x_values = [pose['x'] for pose in self.pose_buffer]
        y_values = [pose['y'] for pose in self.pose_buffer]
        theta_values = [pose['theta'] for pose in self.pose_buffer]

        avg_x = sum(x_values) / len(x_values)
        avg_y = sum(y_values) / len(y_values)
        avg_theta = self.average_angles(theta_values)

        filtered_msg = RobotPose2D()
        filtered_msg.x = avg_x
        filtered_msg.y = avg_y
        filtered_msg.theta = avg_theta

        self.publisher.publish(filtered_msg)
        self.get_logger().info(f'Published filtered pose: x={avg_x:.2f}, y={avg_y:.2f}, theta={avg_theta:.2f}')

def main(args=None):
    rclpy.init(args=args)

    pose_filter_node = PoseFilterNode()
    rclpy.spin(pose_filter_node)
    pose_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()