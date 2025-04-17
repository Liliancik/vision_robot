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

# --- Calibration constant: scales measured obstacle distance to an effective distance ---
DISTANCE_SCALE = 1.3947  # For example, scales 0.38 m measured to ~0.53 m effective

# Global flag for avoidance direction
avoidance_flag = None

# ====== ENCODER SETUP ======
el = 4   # Left encoder pin
er = 26  # Right encoder pin

# ====== ROBOT METRICS ======
WHEEL_DIAMETER = 0.065  # meters
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
robot = CRobot(LMPins=(8, 11), RMPins=(10, 18), PWMPins=(7, 9))
mpu = mpu6050.mpu6050(0x68)

# ----- CONFIGURE REALSENSE PIPELINE (Depth & Color) -----
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)
FRAME_WIDTH = 640

cv2.namedWindow("Navigation View", cv2.WINDOW_NORMAL)

detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)

def gyro_cal(samples=150):
    gyro_z = 0
    for _ in range(samples):
        gyro_z += mpu.get_gyro_data()['z']
        time.sleep(0.002)
    return gyro_z / samples

def turn_to_angle(target_angle, current_angle):
    """
    Turn the robot to the target_angle (absolute).
    Returns the updated heading.
    """
    bias_z = gyro_cal()
    prev_time = time.time()
    init_angle = 0
    Kp, Ki, Kd = 0.5, 0.05, 0.3  # Tuned PID values
    integral = 0
    last_error = 0
    filter_size = deque(maxlen=15)  # For smoother velocity

    while True:
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time
        angular_vel = mpu.get_gyro_data()['z'] - bias_z
        filter_size.append(angular_vel)
        avg_angular_vel = sum(filter_size) / len(filter_size)
        init_angle += avg_angular_vel * dt

        desired_delta = (target_angle - current_angle + 180) % 360 - 180
        error = desired_delta - init_angle
        derivative = (error - last_error) / dt if dt != 0 else 0

        proposed_integral = integral + error * dt
        output = Kp * error + Ki * proposed_integral + Kd * derivative
        requested_speed = abs(output) * 0.4

        if requested_speed > 0.6 or requested_speed < 0.4:
            pass
        else:
            integral = proposed_integral

        last_error = error
        speed = min(max(abs(output) * 0.4, 0.4), 0.6)

        if abs(error) < 1:
            break

        if error > 0:
            robot.right(speed)
        else:
            robot.left(speed)

        time.sleep(0.001)

    robot.stop()
    new_angle = (current_angle + init_angle) % 360
    return new_angle

def get_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def get_angle(x1, y1, x2, y2):
    return math.degrees(math.atan2(y2 - y1, x2 - x1))

def move_to_target(target_x, target_y, current_x, current_y, current_angle):
    """
    Navigate toward the waypoint. If a keyboard is detected:
      1. Perform a 45° avoidance turn using inverted flag logic.
      2. Drive forward diagonally for distance = effective_distance * √2.
      3. Compute and print intermediate coordinates.
      4. Perform a 90° check turn (–90° if avoidance_flag is "RIGHT", +90° if "LEFT").
      5. If the keyboard is still in view after the 90° check, perform an extra 45° turn,
         then proceed forward half the value of the x-axis difference, update the position,
         and perform a 90° check. If the path is not clear, turn back 90°, move forward the same
         half-x distance, and recheck—repeating until the obstacle is cleared. Once clear,
         drive forward the absolute value of the y coordinate.
      6. Otherwise, continue forward until global y becomes 0.
      7. After reaching y = 0, reorient the robot toward the waypoint and drive the remaining distance.
    """
    global countl, countr, avoidance_flag
    target_distance = get_distance(current_x, current_y, target_x, target_y)
    target_angle = (get_angle(current_x, current_y, target_x, target_y) + 360) % 360
    current_angle = (current_angle + 360) % 360

    print(f"Navigating from ({current_x:.2f}, {current_y:.2f}) to ({target_x:.2f}, {target_y:.2f})")
    print(f"Target heading: {target_angle:.2f}°")
    current_angle = turn_to_angle(target_angle, current_angle)
    
    # Initialize intermediate position in case no obstacle is encountered.
    intermediate_x = current_x
    intermediate_y = current_y

    countl, countr = 0, 0
    target_ticks = int(target_distance / DISTANCE_PER_TICK)
    robot.forward(0.4)
    print(f"Target ticks: {target_ticks:.2f}")
    
    while (countl + countr) / 2 < target_ticks:
        progress = (countl + countr) / 2
        print(f"Encoder ticks (Right): {countr:.2f}")
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if color_frame is not None:
            frame = np.asanyarray(color_frame.get_data())
            cv2.imshow("Navigation View", frame)
            cv2.waitKey(1)
        depth_frame = frames.get_depth_frame()
        if color_frame is not None and depth_frame is not None:
            objects = detector.get_objects(frame, threshold=0.3)
            keyboard_objects = [obj for obj in objects if labels[obj.id].lower() == "keyboard"]
            if keyboard_objects:
                main_keyboard = max(keyboard_objects, key=lambda obj: obj.bbox.width * obj.bbox.height)
                x_center = main_keyboard.bbox.xmin + (main_keyboard.bbox.width // 2)
                y_center = main_keyboard.bbox.ymin + (main_keyboard.bbox.height // 2)
                measured_distance = depth_frame.get_distance(x_center, y_center)
                effective_distance = measured_distance * DISTANCE_SCALE
                print(f"Keyboard detected at measured {measured_distance:.2f} m, effective {effective_distance:.2f} m, Position: ({current_x:.2f}, {current_y:.2f})")
                
                # Inversion logic:
                if x_center >= FRAME_WIDTH / 2:
                    avoidance_turn = 45
                    avoidance_flag = "RIGHT"
                    print("Inverted view: Keyboard on right side. Turning RIGHT by 45°.")
                else:
                    avoidance_turn = -45
                    avoidance_flag = "LEFT"
                    print("Inverted view: Keyboard on left side. Turning LEFT by 45°.")
                
                avoid_heading = current_angle + avoidance_turn
                robot.stop()
                current_angle = turn_to_angle(avoid_heading, current_angle)
                print(f"Avoidance flag: {avoidance_flag}, heading after 45° turn: {current_angle:.2f}°")
                robot.stop()
                
                # Drive forward diagonally: travel distance = effective_distance * √2.
                avoid_travel_distance = effective_distance * math.sqrt(2)
                avoid_ticks = int(avoid_travel_distance / DISTANCE_PER_TICK)
                print(f"Traveling forward {avoid_travel_distance:.2f} m (target ticks: {avoid_ticks}) for avoidance maneuver.")
                countl, countr = 0, 0
                robot.forward(0.4)
                while (countl + countr) / 2 < avoid_ticks:
                    time.sleep(0.01)
                robot.stop()
                print("Avoidance forward movement completed.")
                
                intermediate_x = current_x + avoid_travel_distance * math.cos(math.radians(avoid_heading))
                intermediate_y = current_y + avoid_travel_distance * math.sin(math.radians(avoid_heading))
                print(f"Intermediate coordinates after avoidance: ({intermediate_x:.2f}, {intermediate_y:.2f})")
                
                # Perform a 90° check turn
                if avoidance_flag == "RIGHT":
                    check_turn = -90
                elif avoidance_flag == "LEFT":
                    check_turn = 90
                else:
                    check_turn = 0
                check_heading = current_angle + check_turn
                current_angle = turn_to_angle(check_heading, current_angle)
                print(f"Performed 90° check turn. Final heading after check: {current_angle:.2f}°")
                
                frames_check = pipeline.wait_for_frames()
                color_frame_check = frames_check.get_color_frame()
                if color_frame_check is not None:
                    check_frame = np.asanyarray(color_frame_check.get_data())
                    objects_check = detector.get_objects(check_frame, threshold=0.3)
                    keyboard_check = [obj for obj in objects_check if labels[obj.id].lower() == "keyboard"]
                    if keyboard_check:
                        print("Keyboard is still in view after 90° check.")
                        additional_turn = 45 if check_turn < 0 else -45
                        current_angle = turn_to_angle(current_angle + additional_turn, current_angle)
                        print(f"Path not clear. Additional 45° turn performed. Final heading: {current_angle:.2f}°")
                        robot.stop()
                        
                        # --- NEW CODE: Move forward half the x-axis difference ---
                        dx = target_x - intermediate_x
                        move_distance = abs(dx) / 2.0
                        move_ticks = int(move_distance / DISTANCE_PER_TICK)
                        print(f"Proceeding forward {move_distance:.2f} m (half of x-axis difference) with target ticks: {move_ticks}.")
                        countl, countr = 0, 0
                        robot.forward(0.4)
                        while (countl + countr) / 2 < move_ticks:
                            time.sleep(0.01)
                        robot.stop()
                        new_x = intermediate_x + move_distance * math.cos(math.radians(current_angle))
                        new_y = intermediate_y + move_distance * math.sin(math.radians(current_angle))
                        print(f"Updated position after moving forward: ({new_x:.2f}, {new_y:.2f})")
                        
                        # --- NEW CODE: 90° check with reattempt loop after half x-axis movement ---
                        print("Performing initial 90° check after half x-axis movement.")
                        original_heading = current_angle
                        if avoidance_flag == "RIGHT":
                            check_turn_90 = -90
                        else:
                            check_turn_90 = 90
                        check_heading_90 = (current_angle + check_turn_90) % 360
                        current_angle = turn_to_angle(check_heading_90, current_angle)
                        time.sleep(0.2)  # Allow camera to settle
                        
                        clear_flag = False
                        num_checks = 0
                        while num_checks < 3:
                            frames_90 = pipeline.wait_for_frames()
                            color_frame_90 = frames_90.get_color_frame()
                            if color_frame_90 is not None:
                                check_frame_90 = np.asanyarray(color_frame_90.get_data())
                                objects_90 = detector.get_objects(check_frame_90, threshold=0.3)
                                keyboard_90 = [obj for obj in objects_90 if labels[obj.id].lower() == "keyboard"]
                                if not keyboard_90:
                                    clear_flag = True
                                    break
                            num_checks += 1
                            time.sleep(0.2)
                        
                        # Reattempt loop until path is clear.
                        while not clear_flag:
                            print("Obstacle still detected in 90° check. Reorienting and repeating half x-axis movement.")
                            current_angle = turn_to_angle(original_heading, current_angle)
                            print(f"Reoriented to original heading: {current_angle:.2f}°")
                            print(f"Moving forward half x-axis distance again: {move_distance:.2f} m")
                            countl, countr = 0, 0
                            robot.forward(0.4)
                            while (countl + countr) / 2 < move_ticks:
                                time.sleep(0.01)
                            robot.stop()
                            new_x = intermediate_x + move_distance * math.cos(math.radians(current_angle))
                            new_y = intermediate_y + move_distance * math.sin(math.radians(current_angle))
                            print(f"Updated position after moving forward again: ({new_x:.2f}, {new_y:.2f})")
                            intermediate_x, intermediate_y = new_x, new_y
                            
                            print("Performing 90° recheck after reattempt.")
                            original_heading = current_angle
                            if avoidance_flag == "RIGHT":
                                check_turn_90 = -90
                            else:
                                check_turn_90 = 90
                            check_heading_90 = (current_angle + check_turn_90) % 360
                            current_angle = turn_to_angle(check_heading_90, current_angle)
                            time.sleep(0.2)
                            
                            clear_flag = False
                            num_checks = 0
                            while num_checks < 3:
                                frames_90 = pipeline.wait_for_frames()
                                color_frame_90 = frames_90.get_color_frame()
                                if color_frame_90 is not None:
                                    check_frame_90 = np.asanyarray(color_frame_90.get_data())
                                    objects_90 = detector.get_objects(check_frame_90, threshold=0.3)
                                    keyboard_90 = [obj for obj in objects_90 if labels[obj.id].lower() == "keyboard"]
                                    if not keyboard_90:
                                        clear_flag = True
                                        break
                                num_checks += 1
                                time.sleep(0.2)
                            if clear_flag:
                                print("No obstacle detected after reattempt.")
                                break
                        
                        # Once clear, drive forward the absolute value of the y coordinate.
                        drive_distance = abs(new_y)
                        drive_ticks = int(drive_distance / DISTANCE_PER_TICK)
                        print(f"Clear flag set. Driving forward {drive_distance:.2f} m (drive ticks: {drive_ticks}) based on absolute y coordinate.")
                        countl, countr = 0, 0
                        robot.forward(0.4)
                        while (countl + countr) / 2 < drive_ticks:
                            time.sleep(0.01)
                        robot.stop()
                        new_x = new_x + drive_distance * math.cos(math.radians(current_angle))
                        new_y = new_y + drive_distance * math.sin(math.radians(current_angle))
                        print(f"Updated position after clear forward movement: ({new_x:.2f}, {new_y:.2f})")
                        # --- END NEW CODE: 90° check with reattempt loop ---
                        
                        return new_x, new_y, current_angle
                    else:
                        print("Keyboard is not in view after 90° check.")
        if (countl + countr) / 2 >= 0.6 * target_ticks:
            speed = 0.4 * (1 - (progress / target_ticks))
            speed = max(speed, 0.3)
            robot.forward(speed)
        time.sleep(0.01)
    
    # If no obstacles were detected, or after obstacle avoidance is complete:
    robot.stop()
    time.sleep(1)
    
    # --- Final alignment: drive until global y = 0, then reorient to target and continue ---
    angle_rad = math.radians(current_angle)
    if abs(math.sin(angle_rad)) > 0.001:
        d = -intermediate_y / math.sin(angle_rad)
        if d > 0:
            additional_ticks = int(d / DISTANCE_PER_TICK)
            print(f"Continuing forward {d:.2f} m until global y=0 (target ticks: {additional_ticks}).")
            countl, countr = 0, 0
            robot.forward(0.4)
            while (countl + countr) / 2 < additional_ticks:
                time.sleep(0.01)
            robot.stop()
            final_x = intermediate_x + d * math.cos(angle_rad)
            final_y = intermediate_y + d * math.sin(angle_rad)
            print(f"Final coordinates after aligning y=0: ({final_x:.2f}, {final_y:.2f})")
            
            # --- NEW CODE: Reorient to the waypoint heading and drive the remaining distance ---
            new_target_angle = get_angle(final_x, final_y, target_x, target_y)
            print(f"Reorienting to new target heading: {new_target_angle:.2f}°")
            current_angle = turn_to_angle(new_target_angle, current_angle)
            remaining_distance = get_distance(final_x, final_y, target_x, target_y)
            if remaining_distance > 0.01:
                remaining_ticks = int(remaining_distance / DISTANCE_PER_TICK)
                print(f"Proceeding forward to waypoint for remaining {remaining_distance:.2f} m (target ticks: {remaining_ticks}).")
                countl, countr = 0, 0
                robot.forward(0.4)
                while (countl + countr) / 2 < remaining_ticks:
                    time.sleep(0.01)
                robot.stop()
                final_x = target_x
                final_y = target_y
                print(f"Waypoint reached: ({final_x:.2f}, {final_y:.2f})")
            # --- END NEW CODE ---
            
            return final_x, final_y, current_angle
        else:
            print("No additional forward movement required (d <= 0).")
            return intermediate_x, intermediate_y, current_angle
    else:
        print("Current heading produces no vertical movement; skipping y-alignment.")
        return intermediate_x, intermediate_y, current_angle

# ====== WAYPOINTS (Hardcoded) ======
waypoints = [(0, 0), (1, 0)]
print("Loaded waypoints:", waypoints)

# ====== NAVIGATION LOOP ======
current_x, current_y = 0, 0
current_angle = 0

try:
    for target_x, target_y in waypoints:
        current_x, current_y, current_angle = move_to_target(target_x, target_y, current_x, current_y, current_angle)
        print(f"Current Position: ({current_x:.2f}, {current_y:.2f})")
    print("Final destination reached. Stopping robot.")
    robot.stop()
    for _ in range(90):
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if color_frame is not None:
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
