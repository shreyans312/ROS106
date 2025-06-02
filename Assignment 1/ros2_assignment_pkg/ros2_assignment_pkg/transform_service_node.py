#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from ros2_assignment_pkg.msg import RobotPose2D
from ros2_assignment_pkg.srv import GetRobotPosition

class TransformServiceNode(Node):
    def __init__(self):
        super().__init__('transform_service_node')

        self.subscription = self.create_subscription(
            RobotPose2D,
            '/robot_a/filtered_pose',
            self.filtered_pose_callback,
            10
        )

        self.service = self.create_service(
            GetRobotPosition,
            '/get_robot_position_in_professor_frame',
            self.get_robot_position_callback
        )

        # Professor fixed pose with assumed values as per the assginment
        self.professor_x = 4.0
        self.professor_y = 3.0
        self.professor_theta = math.pi / 4
        self.latest_robot_pose = None

        self.get_logger().info('Transform Service Node has started')
        self.get_logger().info(f'Professor pose: x={self.professor_x}, y={self.professor_y}, theta={self.professor_theta:.2f}')
        self.get_logger().info('Subscribing to: /robot_a/filtered_pose')
        self.get_logger().info('Service available at: /get_robot_position_in_professor_frame')

    def filtered_pose_callback(self, msg):
        self.latest_robot_pose = {'x':msg.x, 'y':msg.y, 'theta':msg.theta,}
        self.get_logger().debug(f'Updated robot pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        
        while angle < -math.pi:
            angle += 2*math.pi

        return angle
    
    def get_robot_position_callback(self, request, response):
        self.get_logger().info('Service called: get_robot_position_in_professor_frame')

        if self.latest_robot_pose is None:
            self.get_logger().warn('No filtered pose data is available yet !')
            response.x = 0.0
            response.y = 0.0
            response.theta = 0.0
            return response
        
        x_r = self.latest_robot_pose['x']
        y_r = self.latest_robot_pose['y']
        theta_r = self.latest_robot_pose['theta']

        x_p = self.professor_x
        y_p = self.professor_y
        theta_p = self.professor_theta

        dx = x_r - x_p
        dy = y_r - y_p
        
        cos_neg_theta_p = math.cos(-theta_p)
        sin_neg_theta_p = math.sin(-theta_p)

        x_prof = cos_neg_theta_p*dx - sin_neg_theta_p*dy
        y_prof = sin_neg_theta_p*dx + cos_neg_theta_p*dy

        theta_prof = theta_r - theta_p
        theta_prof = self.normalize_angle(theta_prof)

        response.x = x_prof
        response.y = y_prof
        response.theta = theta_prof

        self.get_logger().info(f'Robot in world frame: x={x_r:.2f}, y={y_r:.2f}, theta={theta_r:.2f}')
        self.get_logger().info(f'Robot in professor frame: x={x_prof:.2f}, y={y_prof:.2f}, theta={theta_prof:.2f}')

        return response
    

def main(args=None):
    rclpy.init(args=args)
    transform_service_node = TransformServiceNode()
    rclpy.spin(transform_service_node)
    transform_service_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()