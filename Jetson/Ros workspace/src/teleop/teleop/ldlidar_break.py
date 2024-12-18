#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanSubscriber(Node):

    def __init__(self):
        super().__init__('scan_subscriber')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("ScanSubscriber node has been started")

    def scan_callback(self, scan_msg):
        self.get_logger().info("scan_callback called")

        # Print the raw data received from the LiDAR
        raw_data = scan_msg.ranges
        self.get_logger().info(f"Raw LiDAR data: {raw_data}")

        # Extract a specific range of data
        start_mid = 100
        end_mid = 128
        extracted_data = raw_data[start_mid:end_mid]

        # Print the extracted distances
        self.get_logger().info(f"Extracted distances: {extracted_data}")

        # Find the minimum value in the extracted data
        min_distance = min(extracted_data)

        # Find the index of the minimum distance
        min_index = extracted_data.index(min_distance)
        angle_of_min_distance = start_mid + min_index

        # Determine action based on the minimum value
        if min_distance < 1:
            action = 'Obstacle detected'
        else:
            action = 'Path clear'

        # Print the result
        self.get_logger().info(f"Min distance: {min_distance:.2f} m at angle index: {angle_of_min_distance} - {action}")


def main(args=None):
    rclpy.init(args=args)
    node = ScanSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.scan_callback()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()