import RPi.GPIO as GPIO
import time

ESC_GPIO = 14  # GPIO pin connected to ESC signal wire
SERVO_GPIO = 15 # GPIO pin connected to the servo motor

GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_GPIO, GPIO.OUT)
GPIO.setup(SERVO_GPIO, GPIO.OUT)

pwm = GPIO.PWM(ESC_GPIO, 50)  # 50Hz for ESCs
pwm.start(0)

def set_speed(speed):
    '''
    With the ECS, we take values from 5.0 - 10.0 and set them as our duty cycle
    these represent the  low and high ends of speed (where 5.0 is stopped, and  10.0 is fastest)
    This will be implemented for reversing as well soon

    The user will input a number from -1 to 1, and we will map that to a speed value on the duty cycle scale
    '''

    if speed > 1.0 or speed < -1.0:
        speed/=abs(speed)

    speed = 7.5 + (speed * 2.5) # THIS WILL BE CHANGED WHEN REVERSAL IS FIXED
    pwm.ChangeDutyCycle(speed)

def set_angle(angle):
    '''
    Given a positional angle between -30 and 30, the car will rotate its front axels to face that direction
    for turning.

    -30 Degrees => 6.0% Duty
    0 Degrees   => 7.5% Duty
    +30 Degrees => 9.0% Duty

    THIS FUNCTION IS STILL BEING WORKED ON, AND HASNT BEEN TESTED WITH THE CAR YET
    '''

    angle = max(-30, min(30, angle)) # CLAMPING INPUT TO -30 to 30

    duty_cycle = 7.5 + (angle_deg / 30.0) * 1.5
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(0)

try:
    print("Arming ESC...")
    set_speed(0)
    time.sleep(3)

    print("Slow speed...")
    set_speed(0.2)
    time.sleep(5)

    print("Mid speed...")
    set_speed(0.5)
    time.sleep(5)

    print("High speed...")
    set_speed(0.9)
    time.sleep(3)

    print("Stopping...")
    set_speed(0)
    time.sleep(3)

finally:
    pwm.stop()
    GPIO.cleanup()