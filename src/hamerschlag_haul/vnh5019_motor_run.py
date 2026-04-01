import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio

LEFT_CS   = 26
RIGHT_CS  = 16
LEFT_PWM  = 13
RIGHT_PWM = 12
LEFT_INA  = 17;  LEFT_INB  = 27
RIGHT_INA = 22;  RIGHT_INB = 23

PWM_FREQ = 1000

class VNH5019MotorBridge(Node):
    def __init__(self):
        super().__init__('vnh5019_motor_bridge')
        self.declare_parameter('wheel_base', 0.30)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 3.5)
        self.declare_parameter('min_duty', 18.0)   # percent -- below this motors don't move
        self.declare_parameter('left_trim', 0.0)   # left is reference
        self.declare_parameter('right_trim', 10.0) # right significantly slower, needs extra duty

        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.min_duty = self.get_parameter('min_duty').value
        self.left_trim = self.get_parameter('left_trim').value
        self.right_trim = self.get_parameter('right_trim').value

        self.h = lgpio.gpiochip_open(4)

        for pin in [LEFT_INA, LEFT_INB, RIGHT_INA, RIGHT_INB]:
            lgpio.gpio_claim_output(self.h, pin, 0)

        lgpio.gpio_claim_output(self.h, LEFT_PWM, 0)
        lgpio.gpio_claim_output(self.h, RIGHT_PWM, 0)
        lgpio.tx_pwm(self.h, LEFT_PWM, PWM_FREQ, 0)
        lgpio.tx_pwm(self.h, RIGHT_PWM, PWM_FREQ, 0)

        self.get_logger().info('VNH5019 motors initialized')
        self.sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.5, self.timeout_cb)

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        linear = msg.linear.x
        angular = msg.angular.z

        lin_norm = linear / self.max_linear
        ang_norm = angular / self.max_angular

        left = lin_norm - ang_norm
        right = lin_norm + ang_norm

        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        self._drive_motor(LEFT_PWM, LEFT_INA, LEFT_INB, left, self.left_trim)
        self._drive_motor(RIGHT_PWM, RIGHT_INA, RIGHT_INB, right, self.right_trim)
        self.get_logger().info(
            f'lin={linear:.3f} ang={angular:.3f} -> '
            f'L={left:.3f} R={right:.3f}')

    def _drive_motor(self, pwm_pin, ina_pin, inb_pin, speed: float, trim: float = 0.0):
        duty_frac = abs(speed)

        if duty_frac > 0.01:
            # scale [0,1] into [min_duty, 100] then apply per-motor trim proportionally
            base = self.min_duty + duty_frac * (100.0 - self.min_duty)
            duty = base * (1.0 + trim / 100.0)
            duty = max(0.0, min(100.0, duty))
        else:
            duty = 0.0

        if speed > 0.01:
            lgpio.gpio_write(self.h, ina_pin, 1)
            lgpio.gpio_write(self.h, inb_pin, 0)
        elif speed < -0.01:
            lgpio.gpio_write(self.h, ina_pin, 0)
            lgpio.gpio_write(self.h, inb_pin, 1)
        else:
            lgpio.gpio_write(self.h, ina_pin, 0)
            lgpio.gpio_write(self.h, inb_pin, 0)
            duty = 0.0

        lgpio.tx_pwm(self.h, pwm_pin, PWM_FREQ, duty)

    def _stop_motors(self):
        for pin in [LEFT_INA, LEFT_INB, RIGHT_INA, RIGHT_INB]:
            lgpio.gpio_write(self.h, pin, 0)
        lgpio.tx_pwm(self.h, LEFT_PWM, PWM_FREQ, 0)
        lgpio.tx_pwm(self.h, RIGHT_PWM, PWM_FREQ, 0)

    def timeout_cb(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5:
            self._stop_motors()

    def destroy_node(self):
        self._stop_motors()
        lgpio.gpiochip_close(self.h)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VNH5019MotorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
