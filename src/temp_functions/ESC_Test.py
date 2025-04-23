# WORKING SAMPLE CODE TO MAKE TIRES ROTATE

import RPi.GPIO as GPIO
import time

ESC_GPIO = 14  # GPIO pin connected to ESC signal wire

GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_GPIO, GPIO.OUT)

pwm = GPIO.PWM(ESC_GPIO, 50)  # 50Hz for ESCs
pwm.start(0)

def set_pulse(pulse_us):
    # pulse_us in range ~1000 to ~2000
    duty_cycle = (pulse_us / 20000) * 100  # 20ms period
    pwm.ChangeDutyCycle(duty_cycle)

try:
    print("Arming ESC...")
    set_pulse(1000)
    time.sleep(3)

    print("Slow speed...")
    set_pulse(1200)
    time.sleep(5)

    print("Mid speed...")
    set_pulse(1500)
    time.sleep(5)

    print("High speed...")
    set_pulse(1800)
    time.sleep(3)

    print("Stopping...")
    set_pulse(1000)
    time.sleep(2)

finally:
    pwm.stop()
    GPIO.cleanup()