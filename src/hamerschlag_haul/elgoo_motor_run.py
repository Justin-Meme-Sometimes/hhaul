import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class ElegooMotorBridge(Node):
    def __init__(self):
        super().__init__('elegoo_motor_bridge')

        # Parameters - declare so you can tune from launch file
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_base', 0.145)  # ~14.5cm for Elegoo V4
        self.declare_parameter('max_speed', 0.3)      # m/s estimate at PWM 255
        self.declare_parameter('max_pwm', 255)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_pwm = self.get_parameter('max_pwm').value

        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.get_logger().info(f'Connected to Arduino on {port}')

        self.sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_cb, 10)

        # Safety timeout on Pi side too
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.5, self.timeout_cb)

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive kinematics
        left  = linear - (angular * self.wheel_base / 2.0)
        right = linear + (angular * self.wheel_base / 2.0)

        left_pwm  = self._speed_to_pwm(left)
        right_pwm = self._speed_to_pwm(right)

        cmd = f'L{left_pwm},R{right_pwm}\n'
        self.ser.write(cmd.encode())

    def _speed_to_pwm(self, speed: float) -> int:
        pwm = int(speed / self.max_speed * self.max_pwm)
        return max(-255, min(255, pwm))

    def timeout_cb(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5:
            self.ser.write(b'L0,R0\n')

def main(args=None):
    rclpy.init(args=args)
    node = ElegooMotorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
