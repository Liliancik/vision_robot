import RPi.GPIO as GPIO
import math
import time
from CRobot import CRobot
from collections import deque
import mpu6050
import pyrealsense2 as rs
import numpy as np
import cv2
from aiymakerkit import vision, utils
import models

# Global flag for avoidance direction (None until set)
avoidance_flag = None

# ====== ENCODER SETUP ======
el = 4  # Left encoder pin
er = 26  # Right encoder pin

# ====== ROBOT METRICS ======
WHEEL_DIAMETER = 0.065               # 65cm wheels
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
ENCODER_TICKS_PER_REV = 20           # Encoder pulses per full wheel rotation
DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REV

# ====== ENCODER COUNTERS ======
countl, countr = 0, 0

# ====== ENCODER CALLBACKS ======
def encoder_callback_left(channel):
    global countl
    countl += 1

def encoder_callback_right(channel):
    global countr
    countr += 1

# ====== SETUP GPIO ======
GPIO.setmode(GPIO.BCM)
GPIO.setup(el, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(er, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(el, GPIO.RISING, callback=encoder_callback_left)
GPIO.add_event_detect(er, GPIO.RISING, callback=encoder_callback_right)

# ====== INITIALIZE ROBOT & SENSORS ======
robot = CRobot(LMPins=(8, 11), RMPins=(10, 18), PWMPins=(7, 9))
mpu = mpu6050.mpu6050(0x68)  # MPU6050 Gyroscope Initialization

# ----- CONFIGURE REALSENSE PIPELINE (Depth & Color) -----
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)
FRAME_WIDTH = 640

# Create a named window for camera display
cv2.namedWindow("Navigation View", cv2.WINDOW_NORMAL)

# ----- INITIALIZE OBJECT DETECTOR -----
detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)

# ====== GYROSCOPE CALIBRATION ======
def gyro_cal(samples=150):
    """Calibrates the gyroscope by averaging samples."""
    gyro_z = 0
    for _ in range(samples):
        gyro_z += mpu.get_gyro_data()['z']
        time.sleep(0.002)
    return gyro_z / samples

# ====== PID CONTROLLED TURN FUNCTION ======
def turn_to_angle(target_angle, current_angle):
    """
    Turn the robot to a specific heading using PID-based gyro control.
    Here, target_angle is a relative target (can be negative or >360).
    """
    bias_z = gyro_cal()
    prev_time = time.time()
    init_angle = 0
    Kp = 0.4
    Ki = 0.01
    Kd = 0.2
    integral = 0
    last_error = 0
    filter_size = deque(maxlen=7)
    
    while True:
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        angular_vel = mpu.get_gyro_data()['z'] - bias_z
        filter_size.append(angular_vel)
        avg_angular_vel = sum(filter_size) / len(filter_size)
        init_angle += avg_angular_vel * dt

        # Relative error: desired relative turn minus the measured change
        error = (target_angle - current_angle) - init_angle
        integral += error * dt
        derivative = (error - last_error) / dt
        last_error = error
        output = Kp * error + Ki * integral + Kd * derivative

        speed = abs(output) * 0.4
        speed = min(max(speed, 0.4), 0.6)

        if abs(error) < 2:
            break

        if error > 0:
            print(f"Turning right: Offset {init_angle:.2f}°, Relative Target {target_angle:.2f}°")
            robot.right(speed)
        else:
            print(f"Turning left: Offset {init_angle:.2f}°, Relative Target {target_angle:.2f}°")
            robot.left(speed)

        time.sleep(0.001)

    robot.stop()
    return current_angle + init_angle

# ====== MOVEMENT FUNCTIONS ======
def get_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def get_angle(x1, y1, x2, y2):
    return math.degrees(math.atan2(y2 - y1, x2 - x1))

def move_to_target(target_x, target_y, current_x, current_y, current_angle):
    """
    Move to the specified waypoint while checking for keyboard obstacles.
    If a keyboard is detected, perform a 45° avoidance turn and then
    travel forward the distance recorded between the object and the rover.
    New coordinates are computed and printed once the avoidance maneuver stops.
    """
    global countl, countr, avoidance_flag
    target_distance = get_distance(current_x, current_y, target_x, target_y)
    target_angle = get_angle(current_x, current_y, target_x, target_y)
    target_angle = (target_angle + 360) % 360
    current_angle = (current_angle + 360) % 360

    # Face the waypoint using the shortest turning direction
    angle_diff = (target_angle - current_angle + 180) % 360 - 180
    shortest_target_angle = current_angle + angle_diff

    print(f"Navigating from ({current_x:.2f}, {current_y:.2f}) to ({target_x:.2f}, {target_y:.2f})")
    print(f"Target heading: {target_angle:.2f}°")
    current_angle = turn_to_angle(shortest_target_angle, current_angle)

    # Reset encoders and compute target ticks
    countl, countr = 0, 0
    target_ticks = int(target_distance / DISTANCE_PER_TICK)
    robot.forward(0.4)
    print(f"Target ticks: {target_ticks:.2f}")

    while (countl + countr) / 2 < target_ticks:
        progress = (countl + countr) / 2
        print(f"Encoder ticks (Right): {countr:.2f}")

        # ----- OBSTACLE CHECK VIA VISION (Keyboard Only) -----
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if color_frame and depth_frame:
            frame = np.asanyarray(color_frame.get_data())
            objects = detector.get_objects(frame, threshold=0.3)
            keyboard_objects = [obj for obj in objects if labels[obj.id].lower() == "keyboard"]
            if keyboard_objects:
                main_keyboard = max(keyboard_objects, key=lambda obj: obj.bbox.width * obj.bbox.height)
                x_center = main_keyboard.bbox.xmin + (main_keyboard.bbox.width // 2)
                y_center = main_keyboard.bbox.ymin + (main_keyboard.bbox.height // 2)
                obstacle_distance = depth_frame.get_distance(x_center, y_center)
                print(f"Keyboard detected at {obstacle_distance:.2f} m, Position: ({current_x:.2f}, {current_y:.2f})")
                
                # Determine avoidance turn direction: if keyboard is on left, turn right (+45); else, turn left (-45)
                if x_center < FRAME_WIDTH / 2:
                    avoidance_turn = 45
                    avoidance_flag = "RIGHT"
                    print("Keyboard on LEFT side. Turning RIGHT by 45°.")
                else:
                    avoidance_turn = -45
                    avoidance_flag = "LEFT"
                    print("Keyboard on RIGHT side. Turning LEFT by 45°.")
                
                # Stop and perform the 45° turn
                robot.stop()
                new_target_angle = current_angle + avoidance_turn  # Relative turn
                current_angle = turn_to_angle(new_target_angle, current_angle)
                print(f"Avoidance flag set to: {avoidance_flag}")
                robot.stop()
                
                # Now, travel forward the recorded obstacle distance
                avoid_ticks = int(obstacle_distance / DISTANCE_PER_TICK)
                print(f"Traveling forward {obstacle_distance:.2f} m (target ticks: {avoid_ticks}) for avoidance maneuver.")
                countl, countr = 0, 0
                robot.forward(0.4)
                while (countl + countr) / 2 < avoid_ticks:
                    time.sleep(0.01)
                robot.stop()
                print("Avoidance forward movement completed.")
                # Compute new coordinates based on obstacle_distance and current_angle
                new_x = current_x + obstacle_distance * math.cos(math.radians(current_angle))
                new_y = current_y + obstacle_distance * math.sin(math.radians(current_angle))
                print(f"New coordinates after avoidance maneuver: ({new_x:.2f}, {new_y:.2f})")
                # Update current position for subsequent navigation
                return new_x, new_y, current_angle
        # ----- END OBSTACLE CHECK -----

        if progress >= 0.6 * target_ticks:
            speed = 0.4 * (1 - (progress / target_ticks))
            speed = max(speed, 0.3)
            robot.forward(speed)
        cv2.imshow("Navigation View", frame)
        cv2.waitKey(1)
        time.sleep(0.01)

    robot.stop()
    time.sleep(1)
    print(f"Arrived at ({target_x:.2f}, {target_y:.2f})")
    print(f"Final encoder counts - Left: {countl}, Right: {countr}")
    print(f"Estimated Distance Traveled: {(countl + countr) / 2 * DISTANCE_PER_TICK:.3f} m")
    return target_x, target_y, current_angle

# ====== WAYPOINTS (Hardcoded) ======
# Two waypoints: starting at (0, 0) and then moving to (1, 0)
waypoints = [(0, 0), (1, 0)]
print("Loaded waypoints:", waypoints)

# ====== NAVIGATION LOOP ======
current_x, current_y = 0, 0   # Starting position
current_angle = 0             # Assume initial heading is 0°

try:
    for target_x, target_y in waypoints:
        current_x, current_y, current_angle = move_to_target(
            target_x, target_y, current_x, current_y, current_angle
        )
        print(f"Current Position: ({current_x:.2f}, {current_y:.2f})")
    print("Final destination reached. Stopping robot.")
    robot.stop()
    
    # Display final camera frame for 3 seconds before closing
    for _ in range(90):
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if color_frame:
            final_frame = np.asanyarray(color_frame.get_data())
            cv2.imshow("Navigation View", final_frame)
            cv2.waitKey(33)
except KeyboardInterrupt:
    print("Program interrupted.")
finally:
    pipeline.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()
    try:
        robot.stop()
    except Exception as e:
        print(f"Error during robot stop: {e}")
    print("GPIO cleaned up.")
