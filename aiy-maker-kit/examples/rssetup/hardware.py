# hardware.py
import RPi.GPIO as GPIO
import math

# ====== ENCODER SETUP ======
el = 4   # Left encoder pin
er = 26  # Right encoder pin

GPIO.setmode(GPIO.BCM)
GPIO.setup(el, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(er, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ====== ENCODER COUNTERS ======
countl, countr = 0, 0

def encoder_callback_left(channel):
    global countl
    countl += 1

def encoder_callback_right(channel):
    global countr
    countr += 1

GPIO.add_event_detect(el, GPIO.RISING, callback=encoder_callback_left)
GPIO.add_event_detect(er, GPIO.RISING, callback=encoder_callback_right)

# ====== ROBOT METRICS ======
WHEEL_DIAMETER = 0.065  # 65cm wheels (in meters)
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER  # in meters
ENCODER_TICKS_PER_REV = 20  # pulses per full wheel rotation
DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REV
