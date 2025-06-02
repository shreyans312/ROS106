#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros2_assignment_pkg.srv import GetRobotPosition

class TestClient(Node):
    def __init__(self):
        super().__init__('test_client')
        self.client = self.create_client(GetRobotPosition, '/get_robot_position_in_professor_frame')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.get_logger().info('Test Client started, calling /get_robot_position_in_professor_frame')
        self.timer = self.create_timer(1.0, self.call_service)

    def call_service(self):
        request = GetRobotPosition.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f'Received response: x={response.x:.2f}, y={response.y:.2f}, theta={response.theta:.2f}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TestClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()