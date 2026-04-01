#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class MappingDriver(Node):
    def __init__(self):
        super().__init__('mapping_driver')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.timer = self.create_timer(0.1, self.tick)

        self.obstacle = False
        self.min_dist = 0.4  # meters, stop if anything closer than this

        self.sequence = [
            (0.15, 0.0,  3.0),
            (0.0,  0.5,  3.14),
            (0.15, 0.0,  3.0),
            (0.0,  0.5,  3.14),
            (0.15, 0.0,  6.0),
            (0.0,  0.5,  3.14),
            (0.15, 0.0,  3.0),
            (0.0,  0.5,  3.14),
            (0.15, 0.0,  6.0),
            (0.0,  0.5,  3.14),
            (0.15, 0.0,  6.0),
            (0.0,  0.5,  3.14),
            (0.15, 0.0,  6.0),
            (0.0,  0.5,  3.14),
            (0.0,  0.0,  1.0),
        ]

        self.step = 0
        self.step_start = self.get_clock().now()
        self.get_logger().info('Mapping driver started')

    def scan_cb(self, msg: LaserScan):
        # Check front-facing arc: -30 to +30 degrees
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        ranges = msg.ranges

        front_indices = []
        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_inc
            if -math.radians(30) <= angle <= math.radians(30):
                front_indices.append(r)

        valid = [r for r in front_indices if not math.isnan(r) and not math.isinf(r)]
        if valid and min(valid) < self.min_dist:
            self.obstacle = True
        else:
            self.obstacle = False

    def tick(self):
        if self.step >= len(self.sequence):
            self.pub.publish(Twist())
            self.get_logger().info('Mapping sequence complete, save your map now')
            return

        linear, angular, duration = self.sequence[self.step]
        elapsed = (self.get_clock().now() - self.step_start).nanoseconds / 1e9

        if elapsed >= duration:
            self.step += 1
            self.step_start = self.get_clock().now()
            return

        msg = Twist()

        if self.obstacle and linear > 0:
            # stop forward motion and turn in place
            self.get_logger().warn('Obstacle detected, turning')
            msg.linear.x = 0.0
            msg.angular.z = 0.5
        else:
            msg.linear.x = linear
            msg.angular.z = angular

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MappingDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
