#!/usr/bin/env python3
"""
Spin calibration for OTOS angular scalar.
Spins the robot counterclockwise. Press Enter when it's back to the
starting orientation. Outputs the angular scalar to put in otos_odom.py.
"""
import lgpio
import time
import threading
import qwiic_otos

LEFT_PWM  = 13
RIGHT_PWM = 12
LEFT_INA  = 17;  LEFT_INB  = 27
RIGHT_INA = 22;  RIGHT_INB = 23
PWM_FREQ  = 1000
SPIN_DUTY = 70

h = lgpio.gpiochip_open(4)
for pin in [LEFT_INA, LEFT_INB, RIGHT_INA, RIGHT_INB]:
    lgpio.gpio_claim_output(h, pin, 0)
lgpio.gpio_claim_output(h, LEFT_PWM, 0)
lgpio.gpio_claim_output(h, RIGHT_PWM, 0)

def stop():
    lgpio.tx_pwm(h, LEFT_PWM, PWM_FREQ, 0)
    lgpio.tx_pwm(h, RIGHT_PWM, PWM_FREQ, 0)
    for pin in [LEFT_INA, LEFT_INB, RIGHT_INA, RIGHT_INB]:
        lgpio.gpio_write(h, pin, 0)

def spin():
    # Left wheel forward, right wheel backward = counterclockwise
    lgpio.gpio_write(h, LEFT_INA, 1);  lgpio.gpio_write(h, LEFT_INB, 0)
    lgpio.gpio_write(h, RIGHT_INA, 0); lgpio.gpio_write(h, RIGHT_INB, 1)
    lgpio.tx_pwm(h, LEFT_PWM,  PWM_FREQ, SPIN_DUTY)
    lgpio.tx_pwm(h, RIGHT_PWM, PWM_FREQ, SPIN_DUTY)

otos = qwiic_otos.QwiicOTOS()
if not otos.is_connected():
    print("OTOS not connected!")
    lgpio.gpiochip_close(h)
    exit(1)

otos.begin()
otos.calibrateImu()
otos.resetTracking()
print("OTOS ready.\n")
print("Put a piece of tape on the floor to mark the robot's starting orientation.")
input("Press Enter to start spinning...")

otos.resetTracking()
time.sleep(0.05)

# Read the initial heading before spinning
prev_h = otos.getPosition().h
accumulated = 0.0
running = True

def track_rotation():
    """Accumulate total rotation, handling the ±180° heading wraparound."""
    global accumulated, prev_h
    while running:
        pos = otos.getPosition()
        delta = pos.h - prev_h
        # Unwrap: if the heading jumped more than 180° it crossed the wrap boundary
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        accumulated += delta
        prev_h = pos.h
        time.sleep(0.02)   # 50 Hz — fast enough to catch the wraparound

spin()
print("Spinning... press Enter when robot is back to the tape mark.")

t = threading.Thread(target=track_rotation, daemon=True)
t.start()

try:
    input()
finally:
    stop()
    running = False
    time.sleep(0.1)

measured = abs(accumulated)
print(f"\n\nAccumulated rotation: {accumulated:.2f} deg  (absolute: {measured:.2f} deg)")

SCALAR_MIN = 0.872
SCALAR_MAX = 1.127

if measured < 10:
    print("WARNING: very small reading — OTOS may not have tracked the spin.")
    print("  Check wiring, or increase SPIN_DUTY.")
elif measured < 300 or measured > 420:
    print(f"WARNING: {measured:.1f} deg is too far from 360 to give a reliable scalar.")
    print("  Make sure the robot completes exactly one full rotation before pressing Enter.")
else:
    scalar = 360.0 / measured
    print(f"Angular scalar: {scalar:.4f}")

    if scalar < SCALAR_MIN or scalar > SCALAR_MAX:
        print(f"ERROR: {scalar:.4f} is outside the hardware limit [{SCALAR_MIN}, {SCALAR_MAX}].")
        print("Run again and make sure the rotation is closer to exactly 360°.")
    else:
        print(f"--> in otos_odom.py: self.otos.setAngularScalar({scalar:.4f})")
        if scalar < 0.95 or scalar > 1.05:
            print("NOTE: outside typical ±5% range — run 2-3 times and average.")

lgpio.gpiochip_close(h)
