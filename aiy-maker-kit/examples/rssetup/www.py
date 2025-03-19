import RPi.GPIO as GPIO
import math
import time
from CRobot import CRobot

# ====== ENCODER SETUP ======
ENCODER_LEFT = 4
ENCODER_RIGHT = 17

# ====== ROBOT METRICS ======
WHEEL_DIAMETER = 0.065  # 65mm wheels
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER  # Wheel circumference in meters
ENCODER_TICKS_PER_REV = 20  # Encoder pulses per full wheel rotation
DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REV  # Distance per tick

# ====== ENCODER COUNTERS ======
encoder_ticks_left = 0
encoder_ticks_right = 0

# ====== ENCODER CALLBACKS ======
def encoder_left_callback(channel):
    global encoder_ticks_left
    encoder_ticks_left += 1

def encoder_right_callback(channel):
    global encoder_ticks_right
    encoder_ticks_right += 1

# ====== SETUP GPIO ======
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(ENCODER_LEFT, GPIO.RISING, callback=encoder_left_callback)
GPIO.add_event_detect(ENCODER_RIGHT, GPIO.RISING, callback=encoder_right_callback)

# ====== INITIALIZE ROBOT ======
LMPins = (8, 11)
RMPins = (10, 12)
PWMPins = (7, 9)
Robot = CRobot(LMPins, RMPins, PWMPins)

# ====== MOVE FORWARD 0.1m WITH SPEED CONTROL ======
try:
    encoder_ticks_left = 0
    encoder_ticks_right = 0
    target_ticks = int(0.2 / DISTANCE_PER_TICK)  # Expected encoder ticks for 0.1m

    Robot.forward(0.4)  # Start at high speed

    while encoder_ticks_left < target_ticks and encoder_ticks_right < target_ticks:
        # Slow down at 75% of target distance
        if encoder_ticks_left >= 0.6 * target_ticks or encoder_ticks_right >= 0.6 * target_ticks:
            Robot.forward(0.2)  # Reduce speed for precise stopping

    Robot.stop()

    # ====== PRINT RESULTS ======
    print(f"Left Encoder Ticks: {encoder_ticks_left}, Right Encoder Ticks: {encoder_ticks_right}")
    print(f"Distance Left Wheel: {encoder_ticks_left * DISTANCE_PER_TICK:.3f}m")
    print(f"Distance Right Wheel: {encoder_ticks_right * DISTANCE_PER_TICK:.3f}m")

finally:
    GPIO.cleanup()