#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MappingDriver(Node):
    def __init__(self):
        super().__init__('mapping_driver')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)

        # (linear, duration_seconds)
        self.sequence = []
        for _ in range(20):
            self.sequence.append((1, 0.15))   # forward
            self.sequence.append((0.0,  15))   # stop

        self.step = 0
        self.step_start = self.get_clock().now()
        self.get_logger().info('Mapping driver started')

    def tick(self):
        if self.step >= len(self.sequence):
            self.pub.publish(Twist())
            self.get_logger().info('Done')
            return

        linear, duration = self.sequence[self.step]
        elapsed = (self.get_clock().now() - self.step_start).nanoseconds / 1e9

        if elapsed >= duration:
            self.step += 1
            self.step_start = self.get_clock().now()
            return

        msg = Twist()
        msg.linear.x = linear
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MappingDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
