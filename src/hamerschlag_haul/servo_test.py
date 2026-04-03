#!/usr/bin/env python3
"""Quick servo test — sweeps LEFT, CENTER, RIGHT with 2s pauses."""

import time
from gpiozero import Servo
from gpiozero.pins.lgpio import LGPIOFactory

PAN_PIN = 18

factory = LGPIOFactory()
servo = Servo(PAN_PIN, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)

def move(name, value):
    print(f"→ {name} (value={value})")
    servo.value = value
    time.sleep(2)

try:
    move("CENTER", 0.0)
    move("LEFT",   1.0)
    move("CENTER", 0.0)
    move("RIGHT",  -1.0)
    move("CENTER", 0.0)
    print("Done.")
finally:
    servo.value = None
    servo.close()
