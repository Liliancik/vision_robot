import pyrealsense2 as rs
import numpy as np
import cv2
from aiymakerkit import vision, utils
import models 
import time
import RPi.GPIO as GPIO
import math
import signal 
import sys
from CRobot import CRobot
from collections import deque
from mpu6050 import mpu6050  # Added for MPU6050

# -------------------------------
# Global State Management
# -------------------------------
exit_requested = False
Cleared = False  # Global flag for return-to-path clearance
current_heading = 0.0  # Global heading in degrees (assume starting at 0°)

# -------------------------------
# Hardware Setup
# -------------------------------
ENCODER_LEFT = 4
ENCODER_RIGHT = 26
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(ENCODER_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

encoder_ticks_left = 0
encoder_ticks_right = 0

def encoder_left_callback(channel):
    global encoder_ticks_left
    encoder_ticks_left += 1

def encoder_right_callback(channel):
    global encoder_ticks_right
    encoder_ticks_right += 1

GPIO.add_event_detect(ENCODER_LEFT, GPIO.RISING, callback=encoder_left_callback)
GPIO.add_event_detect(ENCODER_RIGHT, GPIO.RISING, callback=encoder_right_callback)

# Movement Parameters
WHEEL_DIAMETER = 0.065  # meters
ENCODER_TICKS_PER_REV = 20
DISTANCE_PER_TICK = math.pi * WHEEL_DIAMETER / ENCODER_TICKS_PER_REV

# Position Tracking
global_position = [0.0, 0.0]
prev_left_ticks = 0
prev_right_ticks = 0

def update_position(axis):
    global global_position, prev_left_ticks, prev_right_ticks
    delta_left = encoder_ticks_left - prev_left_ticks
    delta_right = encoder_ticks_right - prev_right_ticks
    avg_delta = (delta_left + delta_right) / 2.0
    distance = avg_delta * DISTANCE_PER_TICK
    if axis == 'x':
        global_position[0] += distance
    elif axis == 'y':
        global_position[1] += distance
    prev_left_ticks = encoder_ticks_left
    prev_right_ticks = encoder_ticks_right

def update_position_generic(distance, heading=None, grid=False, grid_direction=None):
    """
    Updates global_position.
    If grid==True and grid_direction is provided:
      - "left": subtract distance from X and add distance to Y.
      - "right": add distance to both X and Y.
    Otherwise, if heading is near 0°, 45°, or 90° (±5°), discrete updates are used.
    """
    tol = 5  # degrees tolerance
    if grid and grid_direction is not None:
        if grid_direction == "left":
            global_position[0] -= distance
            global_position[1] += distance
            print(f"Grid Update (Left): New Position: X = {global_position[0]:.2f}, Y = {global_position[1]:.2f}")
        elif grid_direction == "right":
            global_position[0] += distance
            global_position[1] += distance
            print(f"Grid Update (Right): New Position: X = {global_position[0]:.2f}, Y = {global_position[1]:.2f}")
        return
    if heading is None:
        heading = current_heading  # Use global current_heading now
    if abs(heading - 0) < tol:
        global_position[0] += distance
        print(f"Position (Heading ~0°): X = {global_position[0]:.2f}, Y = {global_position[1]:.2f}")
    elif abs(heading - 45) < tol:
        global_position[0] += distance
        global_position[1] += distance
        print(f"Position (Heading ~45°): X = {global_position[0]:.2f}, Y = {global_position[1]:.2f}")
    elif abs(heading - 90) < tol:
        global_position[1] += distance
        print(f"Position (Heading ~90°): X = {global_position[0]:.2f}, Y = {global_position[1]:.2f}")
    else:
        rad = math.radians(heading)
        global_position[0] += distance * math.cos(rad)
        global_position[1] += distance * math.sin(rad)
        print(f"Position (Heading {heading:.2f}°): X = {global_position[0]:.2f}, Y = {global_position[1]:.2f}")

def update_position_x(distance):
    """Updates only the X coordinate by the given distance."""
    global global_position
    global_position[0] += distance
    print(f"Position Update (X Only): X = {global_position[0]:.2f}, Y = {global_position[1]:.2f}")

def update_position_y(distance):
    """Updates only the Y coordinate by the given distance."""
    global global_position
    global_position[1] += distance
    print(f"Position Update (Y Only): X = {global_position[0]:.2f}, Y = {global_position[1]:.2f}")

# -------------------------------
# Vision System Setup
# -------------------------------
pipeline = rs.pipeline()
config = rs.config()
width, height = 640, 480
config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)

# -------------------------------
# Safety System
# -------------------------------
def emergency_cleanup():
    global exit_requested
    exit_requested = True
    print("\nEMERGENCY STOP INITIATED")
    Robot.stop()
    pipeline.stop()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    sys.exit(0)

def signal_handler(sig, frame):
    emergency_cleanup()

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def check_exit_key():
    key = cv2.waitKey(10) & 0xFF
    if key == ord('q'):
        emergency_cleanup()
    return exit_requested

# -------------------------------
# Robot Control
# -------------------------------
LMPins = (8, 11)
RMPins = (10, 18)
PWMPins = (7, 9)
Robot = CRobot(LMPins, RMPins, PWMPins)
# Removed BNO085 turn module

# -------------------------------
# MPU6050 & Gyro Setup
# -------------------------------
mpu = mpu6050(0x68)  # Initialize MPU6050 (adjust I2C address if needed)

def gyro_cal():
    """Calibrate the MPU6050 gyro (z-axis bias) using 100 samples."""
    samples = [mpu.get_gyro_data()['z'] for _ in range(100)]
    return sum(samples) / len(samples)

def turn_to_angle(target_angle, current_angle):
    """Turns the robot to a specific heading using a PID-based gyro control."""
    bias_z = gyro_cal()  # Recalibrate before turning
    prev_time = time.time()
    init_angle = 0

    # PID Coefficients
    Kp = 0.4  # Proportional gain (affects speed based on error size)
    Ki = 0.01  # Integral gain (corrects small steady-state errors)
    Kd = 0.2  # Derivative gain (smooths rapid changes)
    
    integral = 0
    last_error = 0

    # Moving Average Filter for Smoother Gyro Readings
    filter_size = deque(maxlen=7)

    # Determine Turn Direction (for logging purposes)
    turn_direction = 1 if target_angle <= current_angle else -1

    while True:
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        # Read & Filter Gyro Data
        angular_vel = mpu.get_gyro_data()['z'] - bias_z
        filter_size.append(angular_vel)
        avg_angular_vel = sum(filter_size) / len(filter_size)

        # Integrate Angular Velocity to Get Current Angle Change
        init_angle += avg_angular_vel * dt

        # Calculate Error (How much more we need to turn)
        error = (target_angle - current_angle) - init_angle
        
        # PID Controller Calculation
        integral += error * dt
        derivative = (error - last_error) / dt
        last_error = error

        # Compute PID Output for Speed Adjustment
        output = Kp * error + Ki * integral + Kd * derivative

        # Dynamically adjust speed (ensures smooth turning)
        speed = abs(output) * 0.4  # Scale speed appropriately
        speed = min(max(speed, 0.4), 0.6)  # Ensure speed stays within safe range

        # Stop condition (within 2° error threshold)
        if abs(error) < 2:
            break
        
        # Adjust turning speed dynamically based on PID output
        if error > 0:
            print(f"Current angle: {init_angle:.2f}°, Target angle: {target_angle:.2f}°")
            Robot.right(speed)
        else:
            Robot.left(speed)
            print(f"Current angle: {init_angle:.2f}°, Target angle: {target_angle:.2f}°")

        time.sleep(0.001)

    Robot.stop()
    # Return the new estimated angle (original + integrated change)
    return current_angle + init_angle

# -------------------------------
# Vision Initialization
# -------------------------------
cv2.namedWindow("Robot Vision", cv2.WINDOW_NORMAL)
print("Initializing camera feed...")
start_time = time.time()
while time.time() - start_time < 5 and not exit_requested:
    try:
        frames = pipeline.wait_for_frames(timeout_ms=100)
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        if color_frame:
            frame = np.asanyarray(color_frame.get_data())
            objects = detector.get_objects(frame, threshold=0.4)
            vision.draw_objects(frame, objects, labels)
            cv2.imshow("Robot Vision", frame)
            check_exit_key()
    except RuntimeError:
        continue
print("Starting navigation...")

# -------------------------------
# Navigation Core
# -------------------------------
def avoid_obstacle(keyboard_obj, depth_frame, img_width):
    global current_heading
    Robot.stop()
    orig_heading = current_heading
    print(f"\n[Obstacle] Detected at Position: X = {global_position[0]:.2f}, Y = {global_position[1]:.2f}")
    
    depth_image = np.asanyarray(depth_frame.get_data())
    x_min = max(int(keyboard_obj.bbox.xmin), 0)
    x_max = min(int(keyboard_obj.bbox.xmax), width)
    x_center = (x_min + x_max) / 2
    
    turn_dir = -45 if x_center < width/2 else 45
    avoidance_heading = (orig_heading + turn_dir) % 360

    initial_turn_direction = "left" if turn_dir < 0 else "right"
    print(f"Initial Turn Direction: {initial_turn_direction}")
    
    roi = depth_image[:, x_min:x_max] if x_min != x_max else depth_image
    valid_depths = roi[roi > 0]
    median_depth = np.median(valid_depths) * depth_scale if valid_depths.size > 0 else 0.5
    total_avoid = median_depth
    first_leg = total_avoid / 2

    print(f"Distance to Obstacle: {total_avoid:.2f}m")
    
    print("\n[Phase 1] Turning to Avoidance Heading (45° deviation)")
    current_heading = turn_to_angle(avoidance_heading, current_heading)
    
    print(f"Moving {first_leg:.2f}m at Heading {avoidance_heading:.1f}°")
    start_ticks = (encoder_ticks_left + encoder_ticks_right) / 2
    Robot.forward(0.3)
    while ((encoder_ticks_left + encoder_ticks_right) / 2 - start_ticks) * DISTANCE_PER_TICK < first_leg:
        if exit_requested:
            break
        time.sleep(0.05)
    Robot.stop()
    moved_dist = ((encoder_ticks_left + encoder_ticks_right) / 2 - start_ticks) * DISTANCE_PER_TICK
    update_position_generic(moved_dist, grid=True, grid_direction=initial_turn_direction)
    time.sleep(0.5)
    
    print("\n[Mid-Check] Reorienting to Original Heading for Obstacle Verification")
    current_heading = turn_to_angle(orig_heading, current_heading)
    time.sleep(0.5)
    
    obstacle_still_present = False
    try:
        frames = pipeline.wait_for_frames(timeout_ms=2000)
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        if color_frame:
            frame = np.asanyarray(color_frame.get_data())
            objects = detector.get_objects(frame, threshold=0.4)
            obstacle_still_present = any(labels[obj.id] == "keyboard" for obj in objects)
            vision.draw_objects(frame, objects, labels)
            cv2.imshow("Robot Vision", frame)
            cv2.waitKey(10)
    except RuntimeError:
        pass

    if obstacle_still_present:
        print("\n[Mid-Check] Obstacle Still Detected; Re-turning to Avoidance Heading (45°).")
        current_heading = turn_to_angle(avoidance_heading, current_heading)
        final_heading = avoidance_heading
    else:
        print("\n[Mid-Check] Obstacle Cleared; Proceeding Along Original Heading.")
        final_heading = orig_heading

    final_leg = total_avoid - first_leg
    print(f"Moving {final_leg:.2f}m at Heading {final_heading:.1f}°")
    start_ticks = (encoder_ticks_left + encoder_ticks_right) / 2
    Robot.forward(0.3)
    while ((encoder_ticks_left + encoder_ticks_right) / 2 - start_ticks) * DISTANCE_PER_TICK < final_leg:
        if exit_requested:
            break
        time.sleep(0.05)
    Robot.stop()
    moved_dist = ((encoder_ticks_left + encoder_ticks_right) / 2 - start_ticks) * DISTANCE_PER_TICK
    if abs(final_heading - orig_heading) < 5:
        update_position_x(moved_dist)
    elif abs(final_heading - avoidance_heading) < 5:
        update_position_generic(moved_dist, grid=True, grid_direction=initial_turn_direction)
    else:
        update_position_generic(moved_dist, final_heading)
    time.sleep(0.5)
    
    if abs(final_heading - orig_heading) >= 5:
        print("\n[Final Alignment] Reorienting Back to Original Heading")
        current_heading = turn_to_angle(orig_heading, current_heading)

    print("\n[Extra Buffer] Moving Additional 0.115m Along Original Heading")
    start_ticks = (encoder_ticks_left + encoder_ticks_right) / 2
    Robot.forward(0.3)
    while ((encoder_ticks_left + encoder_ticks_right) / 2 - start_ticks) * DISTANCE_PER_TICK < 0.115:
        if exit_requested:
            break
        time.sleep(0.05)
    Robot.stop()
    moved_dist = ((encoder_ticks_left + encoder_ticks_right) / 2 - start_ticks) * DISTANCE_PER_TICK
    update_position_x(moved_dist)
    time.sleep(0.5)
    
    print(f"\n[Final Position] Robot Stopped At: X = {global_position[0]:.2f}, Y = {global_position[1]:.2f}")
    
    return_to_path(orig_heading, initial_turn_direction)
    
    if Cleared:
        correct_y_offset(orig_heading)
    
    return True

def return_to_path(orig_heading, initial_turn_direction):
    global Cleared, current_heading
    Cleared = False
    print("\n[Return to Path] Initiating Return-to-Path Logic.")
    while not exit_requested:
        print("\n[Return to Path] Moving Forward 0.15m.")
        start_ticks = (encoder_ticks_left + encoder_ticks_right) / 2
        Robot.forward(0.3)
        while (((encoder_ticks_left + encoder_ticks_right) / 2) - start_ticks) * DISTANCE_PER_TICK < 0.15:
            if exit_requested:
                break
            time.sleep(0.05)
        Robot.stop()
        moved = (((encoder_ticks_left + encoder_ticks_right) / 2) - start_ticks) * DISTANCE_PER_TICK
        update_position_x(moved)
        time.sleep(0.5)
        
        if initial_turn_direction == "left":
            check_angle = (orig_heading + 90) % 360
        else:
            check_angle = (orig_heading - 90) % 360
        print(f"[Return to Path] Performing 90° Check at Heading: {check_angle}°")
        
        current_heading = turn_to_angle(check_angle, current_heading)
        time.sleep(0.5)
        
        obstacle_present = False
        try:
            frames = pipeline.wait_for_frames(timeout_ms=100)
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            if color_frame:
                frame = np.asanyarray(color_frame.get_data())
                objects = detector.get_objects(frame, threshold=0.4)
                obstacle_present = any(labels[obj.id] == "keyboard" for obj in objects)
        except RuntimeError:
            obstacle_present = False
        
        if not obstacle_present:
            print("[Return to Path] Path is Clear. Stopping Return-to-Path Logic.")
            Cleared = True
            print(f"[Return to Path] Cleared Coordinates: X = {global_position[0]:.2f}, Y = {global_position[1]:.2f}")
            break
        else:
            print("[Return to Path] Obstacle Detected; Continuing 15cm Increments.")
            current_heading = turn_to_angle(orig_heading, current_heading)
            time.sleep(0.5)
            continue

def correct_y_offset(orig_heading):
    """
    Once the path is clear (Cleared flag asserted), this function drives the rover forward
    by the absolute value of its current Y coordinate, then turns back to the original heading
    and resets the Y coordinate to 0.
    """
    global current_heading
    distance_to_travel = abs(global_position[1])
    print(f"\n[Correct Y Offset] Driving Forward {distance_to_travel:.2f}m to Correct Y Offset.")
    if distance_to_travel < 0.01:
        print("[Correct Y Offset] No Significant Y Offset Correction Needed.")
    else:
        start_ticks = (encoder_ticks_left + encoder_ticks_right) / 2
        Robot.forward(0.3)
        while (((encoder_ticks_left + encoder_ticks_right) / 2) - start_ticks) * DISTANCE_PER_TICK < distance_to_travel:
            if exit_requested:
                break
            time.sleep(0.05)
        Robot.stop()
        moved = (((encoder_ticks_left + encoder_ticks_right) / 2) - start_ticks) * DISTANCE_PER_TICK
        update_position_y(moved)
        print(f"[Correct Y Offset] Correction Complete. Position Before Turn: X = {global_position[0]:.2f}, Y = {global_position[1]:.2f}")
    
    print(f"[Correct Y Offset] Turning Back to Original Heading: {orig_heading}°")
    current_heading = turn_to_angle(orig_heading, current_heading)
    print("[Correct Y Offset] Turn Complete.")
    
    print(f"[Correct Y Offset] Resetting Y Coordinate from {global_position[1]:.2f} to 0.")
    global_position[1] = 0.0
    print(f"[Correct Y Offset] Final Position After Correction: X = {global_position[0]:.2f}, Y = {global_position[1]:.2f}")

def calculate_distance(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def drive_to_waypoint(start, end):
    global prev_left_ticks, prev_right_ticks
    print(f"\nNavigating from {start} to {end}")
    try:
        prev_left_ticks = encoder_ticks_left
        prev_right_ticks = encoder_ticks_right
        planned_distance = calculate_distance(start, end)
        print(f"Planned Distance: {planned_distance:.2f}m")
        Robot.forward(0.3)
        while not exit_requested:
            try:
                frames = pipeline.wait_for_frames(timeout_ms=100)
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
            except RuntimeError:
                continue
            frame = np.zeros((height, width, 3), dtype=np.uint8)
            if color_frame:
                frame = np.asanyarray(color_frame.get_data())
                update_position('x')
                current_dist = calculate_distance(start, global_position)
                print(f"Traveled: {current_dist:.2f}m / {planned_distance:.2f}m")
                objects = detector.get_objects(frame, threshold=0.4)
                vision.draw_objects(frame, objects, labels)
                cv2.imshow("Robot Vision", frame)
                check_exit_key()
                if color_frame and any(labels[obj.id] == "keyboard" for obj in objects):
                    return avoid_obstacle(next(obj for obj in objects if labels[obj.id] == "keyboard"), depth_frame, width)
                if calculate_distance(start, global_position) >= planned_distance:
                    break
            else:
                print("No color frame received.")
    except Exception as e:
        print(f"Navigation error: {str(e)}")
        return False
    finally:
        Robot.stop()
    return not exit_requested

# -------------------------------
# New Final Approach Function
# -------------------------------
def go_to_final(final_coordinate):
    """
    Drives the robot forward along the X axis from its current position
    (which should have been corrected so that Y=0) to the final coordinate.
    """
    final_x = final_coordinate[0]
    current_x = global_position[0]
    distance_to_travel = final_x - current_x
    if distance_to_travel <= 0:
        print(f"[Final Approach] No forward movement needed (Current X = {current_x:.2f} already >= {final_x:.2f}).")
        return
    print(f"\n[Final Approach] Driving forward {distance_to_travel:.2f}m to reach final coordinate {final_coordinate}.")
    start_ticks = (encoder_ticks_left + encoder_ticks_right) / 2
    Robot.forward(0.3)
    while (((encoder_ticks_left + encoder_ticks_right) / 2) - start_ticks) * DISTANCE_PER_TICK < distance_to_travel:
        if exit_requested:
            break
        time.sleep(0.05)
    Robot.stop()
    update_position_x(distance_to_travel)
    print(f"[Final Position] Final Coordinate: X = {global_position[0]:.2f}, Y = {global_position[1]:.2f}")

# -------------------------------
# Main Program
# -------------------------------
waypoints = [(0, 0), (1, 0)]

def main():
    global current_heading
    try:
        current_target = waypoints[0]
        for target in waypoints[1:]:
            print(f"\n=== CURRENT TARGET: {target} ===")
            while True:
                success = drive_to_waypoint(current_target, target)
                if success or exit_requested:
                    break
                print("Retrying waypoint approach...")
            current_target = target
        # Final approach: after corrections, drive to final coordinate.
        if global_position[0] < waypoints[-1][0]:
            go_to_final(waypoints[-1])
    finally:
        emergency_cleanup()

if __name__ == "__main__":
    main()
