#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from gpiozero import Servo
from gpiozero.pins.lgpio import LGPIOFactory


PAN_PIN = 18

# Direct servo values matching what servo_test.py confirmed works
PAN_VALUES = {
    "LEFT":   1.0,
    "CENTER": 0.0,
    "RIGHT": -1.0,
}

MOTION_THRESHOLD  = 0.3   # meters change to count as motion
MIN_POINTS        = 15    # minimum changed points to act
COOLDOWN_SEC      = 2.0   # seconds between pans


class PanTrackerNode(Node):
    def __init__(self):
        super().__init__('pan_tracker_node')

        factory = LGPIOFactory()
        self.servo = Servo(PAN_PIN, min_pulse_width=0.4/1000, max_pulse_width=2.5/1000, pin_factory=factory)

        self.prev_ranges  = None
        self.current_dir  = "CENTER"
        self.last_move    = self.get_clock().now()
        self._detach_timer = None

        self._set_servo("CENTER")

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info("Pan tracker ready.")

    # ------------------------------------------------------------------ servo

    def _set_servo(self, direction):
        if self._detach_timer is not None:
            self._detach_timer.cancel()
            self._detach_timer.destroy()
            self._detach_timer = None
        self.servo.value = PAN_VALUES[direction]
        self._detach_timer = self.create_timer(0.6, self._detach_servo)

    def _detach_servo(self):
        self.servo.value = None
        self._detach_timer.cancel()
        self._detach_timer.destroy()
        self._detach_timer = None

    # ------------------------------------------------------------------ scan

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)
        np.nan_to_num(ranges, nan=msg.range_max, posinf=msg.range_max, copy=False)

        if self.prev_ranges is None or len(ranges) != len(self.prev_ranges):
            self.prev_ranges = ranges.copy()
            return

        now = self.get_clock().now()
        if (now - self.last_move).nanoseconds < COOLDOWN_SEC * 1e9:
            self.prev_ranges = ranges.copy()
            return

        changed = np.abs(ranges - self.prev_ranges) > MOTION_THRESHOLD
        self.prev_ranges = ranges.copy()

        if not np.any(changed):
            return

        # Split scan into LEFT / RIGHT halves
        # Lidar 0° = front, positive angles = left, negative = right
        n = len(ranges)
        half = n // 2
        left_count  = int(np.sum(changed[:half]))   # first half = left arc
        right_count = int(np.sum(changed[half:]))   # second half = right arc

        total = left_count + right_count
        if total < MIN_POINTS:
            if self.current_dir != "CENTER":
                self.get_logger().info("No motion — returning to CENTER")
                self._set_servo("CENTER")
                self.current_dir = "CENTER"
                self.last_move   = now
            return

        balance = abs(left_count - right_count) / total
        if balance < 0.3:
            direction = "CENTER"
        elif left_count > right_count:
            direction = "RIGHT"
        else:
            direction = "LEFT"

        if direction == self.current_dir:
            return

        self.get_logger().info(
            f"Motion → {direction}  (L={left_count} R={right_count})"
        )

        self._set_servo(direction)
        self.current_dir = direction
        self.last_move   = now

    # ------------------------------------------------------------------ cleanup

    def destroy_node(self):
        self.servo.value = None
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
