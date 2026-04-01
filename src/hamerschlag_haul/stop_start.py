#!/usr/bin/env python3
import lgpio
import time

LEFT_CS   = 26
RIGHT_CS  = 16
LEFT_PWM  = 13
RIGHT_PWM = 12
LEFT_INA  = 17;  LEFT_INB  = 27
RIGHT_INA = 22;  RIGHT_INB = 23
PWM_FREQ  = 1000
DUTY      = 60.0  # adjust as needed

def set_motor(h, ina, inb, pwm_pin, duty, forward=True):
    lgpio.gpio_write(h, ina, 1 if forward else 0)
    lgpio.gpio_write(h, inb, 0 if forward else 1)
    lgpio.tx_pwm(h, pwm_pin, PWM_FREQ, duty)

def stop_motor(h, ina, inb, pwm_pin):
    lgpio.gpio_write(h, ina, 0)
    lgpio.gpio_write(h, inb, 0)
    lgpio.tx_pwm(h, pwm_pin, PWM_FREQ, 0)

def main():
    h = lgpio.gpiochip_open(4)

    for pin in [LEFT_INA, LEFT_INB, RIGHT_INA, RIGHT_INB]:
        lgpio.gpio_claim_output(h, pin, 0)
    lgpio.gpio_claim_output(h, LEFT_PWM, 0)
    lgpio.gpio_claim_output(h, RIGHT_PWM, 0)
    lgpio.tx_pwm(h, LEFT_PWM, PWM_FREQ, 0)
    lgpio.tx_pwm(h, RIGHT_PWM, PWM_FREQ, 0)

    print("Starting motors...")
    start = time.time()

    set_motor(h, LEFT_INA, LEFT_INB, LEFT_PWM, DUTY)
    set_motor(h, RIGHT_INA, RIGHT_INB, RIGHT_PWM, DUTY)

    # stop whichever finishes first (left at 1.0s)
    while True:
        elapsed = time.time() - start
        if elapsed >= 1.0:
            stop_motor(h, LEFT_INA, LEFT_INB, LEFT_PWM)
            print("Left stopped")
            break
        time.sleep(0.01)

    # right runs until 1.2s total
    while True:
        elapsed = time.time() - start
        if elapsed >= 1.30:
            stop_motor(h, RIGHT_INA, RIGHT_INB, RIGHT_PWM)
            print("Right stopped")
            break
        time.sleep(0.01)

    lgpio.gpiochip_close(h)
    print("Done")

if __name__ == '__main__':
    main()
