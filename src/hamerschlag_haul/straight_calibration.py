#!/usr/bin/env python3
import lgpio, time

LEFT_PWM  = 13
RIGHT_PWM = 12
LEFT_INA  = 17;  LEFT_INB  = 27
RIGHT_INA = 22;  RIGHT_INB = 23
PWM_FREQ  = 1000
LEFT_DUTY  = 19
RIGHT_DUTY = 19
RUN_TIME   = 17  # seconds forward before reversing

h = lgpio.gpiochip_open(4)

for pin in [LEFT_INA, LEFT_INB, RIGHT_INA, RIGHT_INB]:
    lgpio.gpio_claim_output(h, pin, 0)
lgpio.gpio_claim_output(h, LEFT_PWM,  0)
lgpio.gpio_claim_output(h, RIGHT_PWM, 0)

def forward():
    lgpio.gpio_write(h, LEFT_INA,  1); lgpio.gpio_write(h, LEFT_INB,  0)
    lgpio.gpio_write(h, RIGHT_INA, 1); lgpio.gpio_write(h, RIGHT_INB, 0)
    lgpio.tx_pwm(h, LEFT_PWM,  PWM_FREQ, LEFT_DUTY)
    lgpio.tx_pwm(h, RIGHT_PWM, PWM_FREQ, RIGHT_DUTY)

def reverse():
    lgpio.gpio_write(h, LEFT_INA,  0); lgpio.gpio_write(h, LEFT_INB,  1)
    lgpio.gpio_write(h, RIGHT_INA, 0); lgpio.gpio_write(h, RIGHT_INB, 1)
    lgpio.tx_pwm(h, LEFT_PWM,  PWM_FREQ, LEFT_DUTY)
    lgpio.tx_pwm(h, RIGHT_PWM, PWM_FREQ, RIGHT_DUTY)

def stop():
    lgpio.tx_pwm(h, LEFT_PWM,  PWM_FREQ, 0)
    lgpio.tx_pwm(h, RIGHT_PWM, PWM_FREQ, 0)
    for pin in [LEFT_INA, LEFT_INB, RIGHT_INA, RIGHT_INB]:
        lgpio.gpio_write(h, pin, 0)

try:
    print(f"Forward for {RUN_TIME}s...")
    forward()
    time.sleep(RUN_TIME)

    print("Stopping...")
    stop()
    time.sleep(1)

    print(f"Reverse for {RUN_TIME}s...")
    reverse()
    time.sleep(RUN_TIME)

    print("Stopping.")
    stop()

except KeyboardInterrupt:
    pass
finally:
    stop()
    lgpio.gpiochip_close(h)
    print("Done.")
