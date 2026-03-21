#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from gpiozero import Servo
from gpiozero.pins.lgpio import LGPIOFactory
import time

class PanTrackerNode(Node):
    def __init__(self):
        super().__init__('pan_tracker_node')

        PAN_PIN = 12

        factory = LGPIOFactory()
        self.servo = Servo(PAN_PIN, pin_factory=factory)

        self.PAN_ANGLES = {
            "FRONT": 90,
            "LEFT":  180,
            "BACK":  0,
            "RIGHT": 0,
        }

        self.MOTION_THRESHOLD = 0.3
        self.MIN_CHANGED_POINTS = 5
        self.COOLDOWN_SEC = 1.0

        self.prev_ranges = None
        self.current_quadrant = "FRONT"
        self.last_move_time = 0.0

        self.set_pan(self.PAN_ANGLES["FRONT"])

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info("Pan tracker started. Facing FRONT.")

    def set_pan(self, angle):
        value = (angle / 90.0) - 1.0
        value = max(-1.0, min(1.0, value))
        self.servo.value = value
        time.sleep(0.3)

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
        ranges = np.where(np.isnan(ranges), msg.range_max, ranges)

        if self.prev_ranges is None:
            self.prev_ranges = ranges.copy()
            return

        if len(ranges) != len(self.prev_ranges):
            self.prev_ranges = ranges.copy()
            return

        now = time.time()
        if now - self.last_move_time < self.COOLDOWN_SEC:
            self.prev_ranges = ranges.copy()
            return

        diff = np.abs(ranges - self.prev_ranges)
        num_points = len(ranges)
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        quadrant_motion = {"FRONT": 0, "LEFT": 0, "BACK": 0, "RIGHT": 0}

        for i in range(num_points):
            if diff[i] > self.MOTION_THRESHOLD:
                angle = angle_min + i * angle_increment
                angle_deg = np.degrees(angle) % 360

                if angle_deg < 45 or angle_deg >= 315:
                    quadrant_motion["FRONT"] += 1
                elif 45 <= angle_deg < 135:
                    quadrant_motion["LEFT"] += 1
                elif 135 <= angle_deg < 225:
                    quadrant_motion["BACK"] += 1
                elif 225 <= angle_deg < 315:
                    quadrant_motion["RIGHT"] += 1

        self.prev_ranges = ranges.copy()

        max_quadrant = max(quadrant_motion, key=quadrant_motion.get)
        max_count = quadrant_motion[max_quadrant]

        if max_count < self.MIN_CHANGED_POINTS:
            return

        if max_quadrant == self.current_quadrant:
            return

        self.get_logger().info(
            f"Motion: F={quadrant_motion['FRONT']} "
            f"L={quadrant_motion['LEFT']} "
            f"B={quadrant_motion['BACK']} "
            f"R={quadrant_motion['RIGHT']}"
        )
        self.get_logger().info(f"Panning to {max_quadrant}")

        self.set_pan(self.PAN_ANGLES[max_quadrant])
        self.current_quadrant = max_quadrant
        self.last_move_time = now

    def destroy_node(self):
        self.servo.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PanTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()