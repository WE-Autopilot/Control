import RPi.GPIO as GPIO
import time

_ESC_GPIO = 14  # GPIO pin connected to ESC signal wire
_SERVO_GPIO = 15 # GPIO pin connected to the servo motor

class Actuator:
    def __init__(self, name: str):
        self.name = name

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(_ESC_GPIO, GPIO.OUT)
        GPIO.setup(_SERVO_GPIO, GPIO.OUT)

        self.pwm = GPIO.PWM(_ESC_GPIO, 50)  # 50Hz for ESCs
        self.pwm.start(0)

    def apply(self, speed: float, theta: float):
        """
        Not yet implemented.

        Args:
            speed (float): _description_
            theta (float): _description_
        """
        self.set_angle(theta)
        self.set_speed(speed)

    def apply_with_pass_filter(self, speed: float, theta: float):
        """
        Apply the speed and theta to the actuator with a pass filter.
        """
        # get the current speed and theta
        # if the speed and theta are within the pass filter range,
        # apply the speed and theta to the actuator
        self.apply(speed, theta)

    def set_speed(self, speed):
        '''
        With the ECS, we take values from 5.0 - 10.0 and set them as our duty cycle
        these represent the  low and high ends of speed (where 5.0 is stopped, and  10.0 is fastest)
        This will be implemented for reversing as well soon

        The user will input a number from -1 to 1, and we will map that to a speed value on the duty cycle scale
        '''

        if speed > 1.0 or speed < -1.0:
            speed/=abs(speed)

        speed = 7.5 + (speed * 2.5) # THIS WILL BE CHANGED WHEN REVERSAL IS FIXED
        self.pwm.ChangeDutyCycle(speed)

    def set_angle(self, angle):
        '''
        Given a positional angle between -30 and 30, the car will rotate its front axels to face that direction
        for turning.

        -30 Degrees => 6.0% Duty
        0 Degrees   => 7.5% Duty
        +30 Degrees => 9.0% Duty

        THIS FUNCTION IS STILL BEING WORKED ON, AND HASNT BEEN TESTED WITH THE CAR YET
        '''

        angle = max(-30, min(30, angle)) # CLAMPING INPUT TO -30 to 30

        duty_cycle = 7.5 + (angle / 30.0) * 1.5
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.5)
        self.pwm.ChangeDutyCycle(0)

    def cleanup(self):
        """Stop PWM and clean up GPIO pins."""
        self.pwm.stop()
        GPIO.cleanup()

    def __del__(self):
        # Fallback in case cleanup() was never called
        try:
            self.cleanup()
        except Exception:
            pass