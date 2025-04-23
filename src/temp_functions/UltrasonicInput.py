# THIS CODE ISN'T FINALIZED, STILL NEEDS TESTING DONE

import RPi.GPIO as GPIO
import time

# Pin numbers
TRIG_1 = 22
ECHO_1 = 23
TRIG_2 = 24
ECHO_2 = 25

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_1, GPIO.OUT)
GPIO.setup(ECHO_1, GPIO.IN)
GPIO.setup(TRIG_2, GPIO.OUT)
GPIO.setup(ECHO_2, GPIO.IN)

def get_distance(trigger_pin, echo_pin):
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(echo_pin) == 0:
        start_time = time.time()

    while GPIO.input(echo_pin) == 1:
        stop_time = time.time()

    elapsed = stop_time - start_time
    distance_cm = (elapsed * 34300) / 2
    return round(distance_cm, 2)

try:
    while True:
        dist1 = get_distance(TRIG_1, ECHO_1)
        dist2 = get_distance(TRIG_2, ECHO_2)

        print(f"Ultrasonic 1: {dist1} cm | Ultrasonic 2: {dist2} cm")
        time.sleep(1)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    GPIO.cleanup()