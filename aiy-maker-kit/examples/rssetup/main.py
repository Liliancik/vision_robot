import threading
import time
import math
import RPi.GPIO as GPIO
from collections import deque
import pyrealsense2 as rs
import numpy as np
import cv2
from CRobot import CRobot
import mpu6050
from aiymakerkit import vision, utils
import models

# ====== ENCODER SETUP ======
el = 4   # Left encoder pin
er = 26  # Right encoder pin

# ====== ROBOT METRICS ======
WHEEL_DIAMETER = 0.065  # in meters
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
ENCODER_TICKS_PER_REV = 20
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
LMPins = (8, 11)
RMPins = (10, 18)
PWMPins = (7, 9)
robot = CRobot(LMPins, RMPins, PWMPins)
mpu = mpu6050.mpu6050(0x68)  # MPU6050 Gyroscope

# ====== GLOBAL POSITION TRACKING ======
global_position = [0.0, 0.0]  # [x, y]
current_angle = 0.0         # in degrees

# ====== GYROSCOPE CALIBRATION & TURNING ======
def gyro_cal(samples=150):
    """Calibrates the gyroscope by averaging samples."""
    gyro_z = 0
    for _ in range(samples):
        gyro_z += mpu.get_gyro_data()['z']
        time.sleep(0.002)
    return gyro_z / samples

def turn_to_angle(target_angle, current_angle, speed_factor=0.4):
    """Turns the robot to a specific heading using PID-based gyro control.
       'speed_factor' scales the turning speed.
    """
    bias_z = gyro_cal()  # Recalibrate before turning
    prev_time = time.time()
    init_angle = 0

    # PID Coefficients
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
        error = (target_angle - current_angle) - init_angle

        integral += error * dt
        derivative = (error - last_error) / dt
        last_error = error
        output = Kp * error + Ki * integral + Kd * derivative

        speed = abs(output) * speed_factor
        if speed_factor == 0.4:
            speed = min(max(speed, 0.4), 0.6)
        else:
            speed = min(max(speed, 0.3), 0.4)

        if abs(error) < 2:
            break

        if error > 0:
            print(f"Turning right: turned {init_angle:.2f}°, target {target_angle:.2f}°")
            robot.right(speed)
        else:
            print(f"Turning left: turned {init_angle:.2f}°, target {target_angle:.2f}°")
            robot.left(speed)
        time.sleep(0.001)
    robot.stop()
    return current_angle + init_angle

def controlled_turn(target_heading, speed_factor=0.4):
    """Wrapper to perform a turn and update the global current_angle."""
    global current_angle
    print(f"[Turn] Current heading: {current_angle:.2f}°, turning to: {target_heading:.2f}°")
    current_angle = turn_to_angle(target_heading, current_angle, speed_factor)
    robot.stop()
    print(f"[Turn] New heading: {current_angle:.2f}°")
    return True

# ====== VISION THREAD FOR OBSTACLE DETECTION ======
class VisionThread(threading.Thread):
    def __init__(self, pipeline, align):
        super().__init__()
        self.pipeline = pipeline
        self.align = align
        self.detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
        self.labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)
        self.frame = None
        self.depth_frame = None
        self.objects = []
        self.running = True
        cv2.namedWindow("Robot Vision", cv2.WINDOW_NORMAL)
    
    def run(self):
        while self.running:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=100)
            except RuntimeError:
                continue
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame:
                continue
            self.frame = np.asanyarray(color_frame.get_data())
            if depth_frame:
                self.depth_frame = depth_frame
            self.objects = self.detector.get_objects(self.frame, threshold=0.4)
            vision.draw_objects(self.frame, self.objects, self.labels)
            cv2.imshow("Robot Vision", self.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
        cv2.destroyAllWindows()
    
    def stop(self):
        self.running = False

# ====== UTILITY FUNCTIONS ======
def get_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def get_angle(x1, y1, x2, y2):
    return math.degrees(math.atan2(y2 - y1, x2 - x1))

# ====== OBSTACLE AVOIDANCE ROUTINE ======
def avoid_obstacle(original_target_heading, current_x, current_y):
    """
    Performs an avoidance maneuver:
      - Turns 45° away from the obstacle at a reduced speed.
      - Drives forward a fixed distance (0.3 m) and updates the position.
      - Then turns back toward the original target heading at reduced speed.
    Returns the new (x, y) position.
    """
    global current_angle, countl, countr

    if not vision_thread.objects:
        return current_x, current_y

    # Decide avoidance turn direction based on the first detected object.
    obj = vision_thread.objects[0]
    cx = int(obj.bbox.xmin + obj.bbox.width / 2)
    if cx < FRAME_WIDTH / 2:
        avoidance_heading = (current_angle + 45) % 360
    else:
        avoidance_heading = (current_angle - 45) % 360

    print(f"[Avoidance] Turning to avoidance heading: {avoidance_heading:.2f}° (reduced speed)")
    controlled_turn(avoidance_heading, speed_factor=0.2)

    # Drive forward a fixed distance for avoidance.
    avoidance_distance = 0.3
    countl, countr = 0, 0
    target_ticks_avoid = int(avoidance_distance / DISTANCE_PER_TICK)
    robot.forward(0.6)
    while (countl + countr) / 2 < target_ticks_avoid:
        time.sleep(0.01)
    robot.stop()
    traveled = (countl + countr) / 2 * DISTANCE_PER_TICK
    # Use the avoidance heading (the heading while driving forward) to update position.
    dx = traveled * math.cos(math.radians(current_angle))
    dy = traveled * math.sin(math.radians(current_angle))
    new_x = current_x + dx
    new_y = current_y + dy
    print(f"[Avoidance] Drove {traveled:.2f} m; new position: ({new_x:.2f}, {new_y:.2f})")

    print(f"[Avoidance] Reorienting to original target heading: {original_target_heading:.2f}° (reduced speed)")
    controlled_turn(original_target_heading, speed_factor=0.2)
    return new_x, new_y

# ====== MOVE TO TARGET WITH OBSTACLE AVOIDANCE ======
def move_to_target(target_x, target_y, current_x, current_y):
    """
    Drives toward a waypoint using encoders.
    If an obstacle is detected (depth < 0.4 m), an avoidance maneuver is triggered.
    The robot updates its position based on encoder counts using the saved travel heading.
    After avoidance, move_to_target is recursively called to reattempt reaching the same target.
    """
    global countl, countr, current_angle, global_position

    target_distance = get_distance(current_x, current_y, target_x, target_y)
    target_heading = get_angle(current_x, current_y, target_x, target_y)
    target_heading = (target_heading + 360) % 360
    current_angle = (current_angle + 360) % 360
    angle_diff = (target_heading - current_angle + 180) % 360 - 180
    shortest_target = current_angle + angle_diff

    # Save travel_heading for odometry during the straight drive.
    travel_heading = shortest_target

    print(f"Navigating from ({current_x:.2f}, {current_y:.2f}) to ({target_x:.2f}, {target_y:.2f})")
    print(f"Target heading: {target_heading:.2f}°, using travel heading: {travel_heading:.2f}°")

    controlled_turn(shortest_target)

    countl, countr = 0, 0
    target_ticks = int(target_distance / DISTANCE_PER_TICK)
    print(f"Target distance: {target_distance:.2f} m, Target ticks: {target_ticks}")

    robot.forward(0.6)
    while (countl + countr) / 2 < target_ticks:
        if vision_thread.objects and vision_thread.depth_frame is not None:
            for obj in vision_thread.objects:
                cx = int(obj.bbox.xmin + obj.bbox.width / 2)
                # Get depth at center of bounding box
                detected_distance = vision_thread.depth_frame.get_distance(cx, int(obj.bbox.ymin + obj.bbox.height / 2))
                if detected_distance < 0.4:
                    print(f"Obstacle detected at {detected_distance:.2f} m. Initiating avoidance maneuver.")
                    robot.stop()
                    progress_ticks = (countl + countr) / 2
                    traveled = progress_ticks * DISTANCE_PER_TICK
                    # Use the saved travel_heading for odometry
                    dx = traveled * math.cos(math.radians(travel_heading))
                    dy = traveled * math.sin(math.radians(travel_heading))
                    new_x = current_x + dx
                    new_y = current_y + dy
                    print(f"Stopped after {traveled:.2f} m; estimated position: ({new_x:.2f}, {new_y:.2f})")
                    # Execute avoidance maneuver and then reattempt the target.
                    updated_x, updated_y = avoid_obstacle(shortest_target, new_x, new_y)
                    return move_to_target(target_x, target_y, updated_x, updated_y)
        progress = (countl + countr) / 2
        if progress >= 0.6 * target_ticks:
            speed = 1 - (0.8 * (progress / target_ticks))
            speed = max(speed, 0.4)
            robot.forward(speed)
        time.sleep(0.01)
    robot.stop()
    time.sleep(1)
    # Update final position based on encoder counts from this segment.
    progress_ticks = (countl + countr) / 2
    traveled = progress_ticks * DISTANCE_PER_TICK
    dx = traveled * math.cos(math.radians(travel_heading))
    dy = traveled * math.sin(math.radians(travel_heading))
    new_x = current_x + dx
    new_y = current_y + dy
    global_position = [new_x, new_y]
    print(f"Reached waypoint: ({target_x:.2f}, {target_y:.2f}); odometry: ({new_x:.2f}, {new_y:.2f})")
    return new_x, new_y

# ====== REALSENSE & VISION SETUP ======
pipeline = rs.pipeline()
config = rs.config()
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, 30)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

vision_thread = VisionThread(pipeline, align)
vision_thread.daemon = True
vision_thread.start()

# ====== WAYPOINTS ======
waypoints = [(0.0, 0.0), (1.0, 0.0)]
print("Waypoints:", waypoints)

# ====== NAVIGATION LOOP ======
def main():
    global global_position
    current_x, current_y = waypoints[0]
    for target in waypoints[1:]:
        target_x, target_y = target
        current_x, current_y = move_to_target(target_x, target_y, current_x, current_y)
    print("Navigation completed! Final position: ({:.2f}, {:.2f})".format(global_position[0], global_position[1]))
    robot.stop()
    vision_thread.stop()
    pipeline.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted by user.")
        robot.stop()
        vision_thread.stop()
        pipeline.stop()
        GPIO.cleanup()
        cv2.destroyAllWindows()
