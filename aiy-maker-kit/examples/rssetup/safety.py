# safety.py
import sys
import signal
import cv2
from CRobot import CRobot
from vision import pipeline
import RPi.GPIO as GPIO
from robot_control import Robot

def emergency_cleanup():
    print("\nEMERGENCY STOP INITIATED")
    global Robot
    Robot.stop()
    pipeline.stop()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    sys.exit(0)

def signal_handler(sig, frame):
    emergency_cleanup()

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)
