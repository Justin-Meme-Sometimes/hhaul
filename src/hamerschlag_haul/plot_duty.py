#!/usr/bin/env python3
"""
Listen to /cmd_vel and plot the left/right duty cycles the motor driver would produce.
Run alongside your nav2 launch to see what duty cycles are being commanded.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading

# Match these to vnh5019_motor_run.py
MAX_LINEAR  = 0.5
MAX_ANGULAR = 3.5
MIN_DUTY    = 18.0
LEFT_TRIM   = 0.0
RIGHT_TRIM  = 10.0

MAX_POINTS = 200

times  = deque(maxlen=MAX_POINTS)
left_duties  = deque(maxlen=MAX_POINTS)
right_duties = deque(maxlen=MAX_POINTS)
t = 0.0

def vel_to_duty(speed, trim):
    frac = abs(speed)
    if frac > 0.01:
        duty = MIN_DUTY + frac * (100.0 - MIN_DUTY) + trim
        duty = max(0.0, min(100.0, duty))
        return duty if speed > 0 else -duty
    return 0.0

class DutyListener(Node):
    def __init__(self):
        super().__init__('duty_plotter')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.start_time = self.get_clock().now()

    def cb(self, msg):
        global t
        lin = msg.linear.x
        ang = msg.angular.z
        lin_norm = lin / MAX_LINEAR
        ang_norm = ang / MAX_ANGULAR
        left  = max(-1.0, min(1.0, lin_norm - ang_norm))
        right = max(-1.0, min(1.0, lin_norm + ang_norm))
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        times.append(t)
        left_duties.append(vel_to_duty(left, LEFT_TRIM))
        right_duties.append(vel_to_duty(right, RIGHT_TRIM))

def ros_thread():
    rclpy.init()
    node = DutyListener()
    rclpy.spin(node)

threading.Thread(target=ros_thread, daemon=True).start()

fig, ax = plt.subplots()
line_l, = ax.plot([], [], label='Left duty %',  color='blue')
line_r, = ax.plot([], [], label='Right duty %', color='red')
ax.set_ylim(-105, 105)
ax.set_ylabel('Duty cycle %')
ax.set_xlabel('Time (s)')
ax.set_title('Motor duty cycles from /cmd_vel')
ax.axhline(0, color='black', linewidth=0.5)
ax.legend()

def update(_):
    if not times:
        return line_l, line_r
    ax.set_xlim(max(0, t - 30), t + 1)
    line_l.set_data(list(times), list(left_duties))
    line_r.set_data(list(times), list(right_duties))
    return line_l, line_r

ani = animation.FuncAnimation(fig, update, interval=100, blit=True)
plt.tight_layout()
plt.show()
