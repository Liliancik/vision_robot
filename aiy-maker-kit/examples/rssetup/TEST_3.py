import time
import math
from CRobot import CRobot
import RPi.GPIO as GPIO

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
robot = CRobot(LMPins, RMPins, PWMPins)

# Function to calculate distance from encoder ticks
def get_distance_from_encoders():
    # Calculate the average distance traveled by both wheels
    avg_ticks = (encoder_ticks_left + encoder_ticks_right) / 2
    return avg_ticks * DISTANCE_PER_TICK

# Function to calculate angle to target
def get_angle(x1, y1, x2, y2):
    return math.degrees(math.atan2(y2 - y1, x2 - x1))

# Function to move towards a target point using encoder feedback
def move_to_target(robot, target_x, target_y, current_x, current_y, robot_heading):
    angle_threshold = 4  # Reduce threshold for finer adjustments
    distance_threshold = 4  # Reduce for more precise waypoint following
    target_distance = get_distance_from_encoders()  # Target distance based on encoder feedback

    while get_distance(current_x, current_y, target_x, target_y) > distance_threshold:
        # Calculate the required angle
        angle_to_target = get_angle(current_x, current_y, target_x, target_y)
        angle_diff = angle_to_target - robot_heading

        # Normalize angle difference to [-180, 180] range
        angle_diff = (angle_diff + 180) % 360 - 180

        if abs(angle_diff) > angle_threshold:
            # Adjust turning speed based on angle difference
            turn_speed = max(0.4, min(0.8, abs(angle_diff) / 90))  # Scale speed with angle
            if angle_diff > 0:
                print("Turning LEFT")
                robot.left(0.8)
            else:
                print("Turning RIGHT")
                robot.right(0.8)
            time.sleep(0.5)  # Small turn duration
        else:
            print("Moving FORWARD")
            robot.forward(0.4)  # Move forward at moderate speed
            time.sleep(0.5)

        # Update estimated position (simple model)
        move_step = 5
        current_x += move_step * math.cos(math.radians(angle_to_target))
        current_y += move_step * math.sin(math.radians(angle_to_target))

        # Update robot heading to last movement direction
        robot_heading = angle_to_target

        # Stop if the distance traveled exceeds the encoder threshold
        if get_distance_from_encoders() >= target_distance:
            break

    robot.stop()
    return current_x, current_y, robot_heading  # Return updated position and heading

# Path following logic
file_path = "/home/pi/Downloads/waypoints.txt"
x_vals, y_vals = [], []

# Read waypoints
with open(file_path, "r") as file:
    for line in file:
        x, y = map(int, line.strip().replace(" ", "").split(","))  # Remove spaces and split
        x_vals.append(x/100)
        y_vals.append(y/100)

waypoints = list(zip(x_vals, y_vals))  # Pair x, y coordinates as waypoints
print("Loaded waypoints:", waypoints)

# Follow the path
current_x, current_y = 0, 0  # Start position (adjust based on reality)
robot_heading = 1  # Start heading (adjust if known)

for target_x, target_y in waypoints:
    print(f"Moving to waypoint: {target_x}, {target_y}")
    current_x, current_y, robot_heading = move_to_target(robot, target_x, target_y, current_x, current_y, robot_heading)

print("Path completed!")
robot.stop()
GPIO.cleanup()