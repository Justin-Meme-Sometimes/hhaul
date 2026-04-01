#!/usr/bin/env python3
import lgpio, time

LEFT_CS   = 26
RIGHT_CS  = 16
LEFT_PWM  = 13
RIGHT_PWM = 12
LEFT_INA  = 17;  LEFT_INB  = 27
RIGHT_INA = 22;  RIGHT_INB = 23
PWM_FREQ  = 1000
LEFT_DUTY  = 19  # percent -- max safe speed for Cartographer mapping (~0.2 m/s)
RIGHT_DUTY = 21  # percent

h = lgpio.gpiochip_open(4)

for pin in [LEFT_INA, LEFT_INB, RIGHT_INA, RIGHT_INB]:
    lgpio.gpio_claim_output(h, pin, 0)

lgpio.gpio_claim_output(h, LEFT_PWM,  0)
lgpio.gpio_claim_output(h, RIGHT_PWM, 0)

# Forward direction
lgpio.gpio_write(h, LEFT_INA,  1); lgpio.gpio_write(h, LEFT_INB,  0)
lgpio.gpio_write(h, RIGHT_INA, 1); lgpio.gpio_write(h, RIGHT_INB, 0)

lgpio.tx_pwm(h, LEFT_PWM,  PWM_FREQ, LEFT_DUTY)
lgpio.tx_pwm(h, RIGHT_PWM, PWM_FREQ, RIGHT_DUTY)

print(f"Running both motors forward at L:{LEFT_DUTY}% R:{RIGHT_DUTY}% duty / {PWM_FREQ}Hz -- Ctrl+C to stop")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    for pin in [LEFT_INA, LEFT_INB, RIGHT_INA, RIGHT_INB]:
        lgpio.gpio_write(h, pin, 0)
    lgpio.tx_pwm(h, LEFT_PWM,  PWM_FREQ, 0)
    lgpio.tx_pwm(h, RIGHT_PWM, PWM_FREQ, 0)
    lgpio.gpiochip_close(h)
    print("Stopped.")
