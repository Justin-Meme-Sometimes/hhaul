#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarPrinter(Node):
    def __init__(self):
        super().__init__('lidar_printer')
        
        # Wait until /scan is available
        self.get_logger().info('Waiting for /scan topic...')
        while not self.count_publishers('/scan'):
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info('/scan found, starting!')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        print(f'\n--- Scan ---')
        print(f'Total points: {len(msg.ranges)}')
        for i, r in enumerate(msg.ranges):
            if r == float('inf') or r == 0.0:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = angle * 180.0 / 3.14159
            print(f'  angle={angle_deg:.1f}deg  dist={r:.3f}m')

def main():
    rclpy.init()
    node = LidarPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
