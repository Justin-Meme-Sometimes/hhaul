import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pigpio

# VNH5019 pin mapping for Pi 5
# Left motor
LEFT_PWM = 12      # GPIO 12, hardware PWM
LEFT_INA = 23      # Direction A
LEFT_INB = 24      # Direction B

# Right motor
RIGHT_PWM = 18     # GPIO 18, hardware PWM
RIGHT_INA = 25     # Direction A
RIGHT_INB = 16     # Direction B

PWM_FREQ = 20000   # 20kHz


class VNH5019MotorBridge(Node):
    def __init__(self):
        super().__init__('vnh5019_motor_bridge')

        self.declare_parameter('wheel_base', 0.30)   # Adjust for your robot
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_duty', 255)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_duty = self.get_parameter('max_duty').value

        # Init pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error('Failed to connect to pigpiod')
            raise RuntimeError('pigpiod not running')

        # Setup direction pins
        for pin in [LEFT_INA, LEFT_INB, RIGHT_INA, RIGHT_INB]:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, 0)

        # Setup hardware PWM (duty 0-1000000)
        self.pi.hardware_PWM(LEFT_PWM, PWM_FREQ, 0)
        self.pi.hardware_PWM(RIGHT_PWM, PWM_FREQ, 0)

        self.get_logger().info('VNH5019 motors initialized')

        self.sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_cb, 10)

        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.5, self.timeout_cb)

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive kinematics
        left = linear - (angular * self.wheel_base / 2.0)
        right = linear + (angular * self.wheel_base / 2.0)

        self._drive_motor(LEFT_PWM, LEFT_INA, LEFT_INB, left)
        self._drive_motor(RIGHT_PWM, RIGHT_INA, RIGHT_INB, right)

        self.get_logger().info(
            f'lin={linear:.3f} ang={angular:.3f} -> '
            f'L={left:.3f} R={right:.3f}')

    def _drive_motor(self, pwm_pin, ina_pin, inb_pin, speed: float):
        # Map speed to duty cycle (0-1000000 for pigpio hardware_PWM)
        duty_frac = abs(speed) / self.max_speed
        duty_frac = min(1.0, duty_frac)
        duty = int(duty_frac * 1000000)

        if speed > 0:
            # Forward: INA=HIGH, INB=LOW
            self.pi.write(ina_pin, 1)
            self.pi.write(inb_pin, 0)
        elif speed < 0:
            # Reverse: INA=LOW, INB=HIGH
            self.pi.write(ina_pin, 0)
            self.pi.write(inb_pin, 1)
        else:
            # Brake: INA=LOW, INB=LOW
            self.pi.write(ina_pin, 0)
            self.pi.write(inb_pin, 0)
            duty = 0

        self.pi.hardware_PWM(pwm_pin, PWM_FREQ, duty)

    def _stop_motors(self):
        self.pi.write(LEFT_INA, 0)
        self.pi.write(LEFT_INB, 0)
        self.pi.write(RIGHT_INA, 0)
        self.pi.write(RIGHT_INB, 0)
        self.pi.hardware_PWM(LEFT_PWM, PWM_FREQ, 0)
        self.pi.hardware_PWM(RIGHT_PWM, PWM_FREQ, 0)

    def timeout_cb(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5:
            self._stop_motors()

    def destroy_node(self):
        self._stop_motors()
        self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VNH5019MotorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()