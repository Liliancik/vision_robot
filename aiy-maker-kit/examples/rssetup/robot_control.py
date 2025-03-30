import RPi.GPIO as GPIO
import math
import time
from collections import deque
import threading

import pyrealsense2 as rs
import numpy as np
import cv2

from CRobot import CRobot
import mpu6050
from aiymakerkit import vision, utils
import models

class VisionThread(threading.Thread):
    """
    A thread that continuously grabs frames from the RealSense pipeline,
    runs object detection, and stores the latest objects.
    """
    def __init__(self, pipeline, align, detector, labels, frame_width):
        super().__init__()
        self.pipeline = pipeline
        self.align = align
        self.detector = detector
        self.labels = labels
        self.frame_width = frame_width
        self.objects = []   # Latest detected objects
        self.running = True
        # Create display window
        cv2.namedWindow("Integrated Vision", cv2.WINDOW_NORMAL)

    def run(self):
        while self.running:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=100)
            except RuntimeError:
                continue
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            if not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data())
            # Run object detection
            self.objects = self.detector.get_objects(frame, threshold=0.4)
            # Draw detections for display
            disp_frame = frame.copy()
            vision.draw_objects(disp_frame, self.objects, self.labels)
            cv2.imshow("Integrated Vision", disp_frame)
            cv2.waitKey(1)
        cv2.destroyAllWindows()

    def stop(self):
        self.running = False

class RobotNavigator:
    def __init__(self, waypoint_file):
        # -------------------------------
        # SETUP: ENCODERS & GPIO
        # -------------------------------
        self.el = 4    # Left encoder pin
        self.er = 26   # Right encoder pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.el, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.er, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.el, GPIO.RISING, callback=self.encoder_callback_left)
        GPIO.add_event_detect(self.er, GPIO.RISING, callback=self.encoder_callback_right)
        self.countl, self.countr = 0, 0

        # -------------------------------
        # ROBOT METRICS
        # -------------------------------
        self.WHEEL_DIAMETER = 0.065          # in meters (65cm wheels)
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER
        self.ENCODER_TICKS_PER_REV = 20      # pulses per rotation
        self.DISTANCE_PER_TICK = self.WHEEL_CIRCUMFERENCE / self.ENCODER_TICKS_PER_REV

        # -------------------------------
        # INITIALIZE ROBOT & MPU6050
        # -------------------------------
        LMPins = (8, 11)
        RMPins = (10, 18)
        PWMPins = (7, 9)
        self.robot = CRobot(LMPins, RMPins, PWMPins)
        self.mpu = mpu6050.mpu6050(0x68)

        # -------------------------------
        # REALSENSE & VISION SETUP
        # -------------------------------
        self.detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
        self.labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.FRAME_WIDTH = 640
        self.FRAME_HEIGHT = 480
        self.config.enable_stream(rs.stream.color, self.FRAME_WIDTH, self.FRAME_HEIGHT, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)
        # Boundaries for obstacle reaction
        self.LEFT_BOUND = self.FRAME_WIDTH // 3
        self.RIGHT_BOUND = 2 * self.LEFT_BOUND

        # -------------------------------
        # START VISION THREAD
        # -------------------------------
        self.vision_thread = VisionThread(self.pipeline, self.align, self.detector, self.labels, self.FRAME_WIDTH)
        self.vision_thread.start()

        # -------------------------------
        # NAVIGATION STATE
        # -------------------------------
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_angle = 0.0  # in degrees

        # -------------------------------
        # LOAD WAYPOINTS
        # -------------------------------
        self.waypoints = self.load_waypoints(waypoint_file)

    # Encoder callbacks
    def encoder_callback_left(self, channel):
        self.countl += 1

    def encoder_callback_right(self, channel):
        self.countr += 1

    def load_waypoints(self, filepath):
        x_vals, y_vals = [], []
        with open(filepath, "r") as f:
            for line in f:
                x, y = map(float, line.strip().split(","))
                # Adjust scaling if needed (here dividing by 2 as an example)
                x_vals.append(x / 2)
                y_vals.append(y / 2)
        waypoints = list(zip(x_vals, y_vals))
        print("[Navigator] Loaded waypoints:", waypoints)
        return waypoints

    # -------------------------------
    # GYROSCOPE CALIBRATION & TURNING
    # -------------------------------
    def gyro_cal(self, samples=150):
        gyro_z = 0
        for _ in range(samples):
            gyro_z += self.mpu.get_gyro_data()['z']
            time.sleep(0.002)
        return gyro_z / samples

    def turn_to_angle(self, target_angle, current_angle):
        bias_z = self.gyro_cal()
        prev_time = time.time()
        init_angle = 0
        Kp = 0.4
        Ki = 0.01
        Kd = 0.2
        integral = 0
        last_error = 0
        filter_size = deque(maxlen=7)
        while True:
            now = time.time()
            dt = now - prev_time
            prev_time = now
            angular_vel = self.mpu.get_gyro_data()['z'] - bias_z
            filter_size.append(angular_vel)
            avg_angular_vel = sum(filter_size) / len(filter_size)
            init_angle += avg_angular_vel * dt
            error = (target_angle - current_angle) - init_angle
            integral += error * dt
            derivative = (error - last_error) / dt if dt > 0 else 0
            last_error = error
            output = Kp * error + Ki * integral + Kd * derivative
            speed = abs(output) * 0.4
            speed = min(max(speed, 0.4), 0.6)
            if abs(error) < 2:
                break
            if error > 0:
                #print(f"[Turning] Integrated: {init_angle:.2f}°, Target: {target_angle:.2f}°")
                self.robot.right(speed)
            else:
                self.robot.left(speed)
                #print(f"[Turning] Integrated: {init_angle:.2f}°, Target: {target_angle:.2f}°")
            time.sleep(0.001)
        self.robot.stop()
        return current_angle + init_angle

    # -------------------------------
    # WAYPOINT NAVIGATION FUNCTIONS
    # -------------------------------
    def get_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def get_angle(self, x1, y1, x2, y2):
        return math.degrees(math.atan2(y2 - y1, x2 - x1))

    def move_to_target(self, target_x, target_y):
        dist = self.get_distance(self.current_x, self.current_y, target_x, target_y)
        ang = self.get_angle(self.current_x, self.current_y, target_x, target_y)
        ang = (ang + 360) % 360
        self.current_angle = (self.current_angle + 360) % 360
        angle_diff = (ang - self.current_angle + 180) % 360 - 180
        turn_target = self.current_angle + angle_diff
        print(f"[Navigation] From ({self.current_x:.2f}, {self.current_y:.2f}) to ({target_x:.2f}, {target_y:.2f}); desired angle: {ang:.2f}°, turning to: {turn_target:.2f}°")
        self.current_angle = self.turn_to_angle(turn_target, self.current_angle)
        # Reset encoder counts
        self.countl, self.countr = 0, 0
        target_ticks = int(dist / self.DISTANCE_PER_TICK)
        #print(f"[Navigation] Target encoder ticks: {target_ticks}")
        self.robot.forward(0.6)
        while (self.countl + self.countr) / 2 < target_ticks:
            progress = (self.countl + self.countr) / 2
            #print(f"[Navigation] Encoder ticks (Right): {self.countr:.2f}")
            if progress >= 0.6 * target_ticks:
                speed = 1 - (0.8 * (progress / target_ticks))
                speed = max(speed, 0.4)
                self.robot.forward(speed)
            time.sleep(0.01)
        self.robot.stop()
        time.sleep(1)
        print(f"[Navigation] Arrived at ({target_x:.2f}, {target_y:.2f})")
        print(f"[Navigation] Encoder counts - Left: {self.countl}, Right: {self.countr}")
        print(f"[Navigation] Estimated Distance: {(self.countl + self.countr) / 2 * self.DISTANCE_PER_TICK:.3f} m")
        self.current_x, self.current_y = target_x, target_y
        return self.current_x, self.current_y, self.current_angle

    # -------------------------------
    # OBSTACLE AVOIDANCE
    # -------------------------------
    def avoid_object(self):
        """
        If objects are detected in the vision thread, this method computes an avoidance turn.
        It uses the largest object's bounding box to compute an avoidance angle offset.
        After performing the avoidance maneuver, it waits until the object is no longer detected.
        """
        # Access the latest objects from the vision thread
        objects = self.vision_thread.objects
        if not objects:
            return
        main_object = max(objects, key=lambda o: o.bbox.width * o.bbox.height)
        # Calculate horizontal center of the object
        x_center = main_object.bbox.xmin + (main_object.bbox.width / 2)
        # Calculate object area ratio relative to frame area
        frame_area = self.FRAME_WIDTH * self.FRAME_HEIGHT
        obj_area = main_object.bbox.width * main_object.bbox.height
        ratio = obj_area / frame_area
        # Compute an avoidance angle offset (e.g., scale ratio to degrees, cap at 90°)
        avoidance_offset = min(ratio * 300, 90)  # For example: ratio 0.1 -> 30°; ratio 0.2 -> 60°
        if x_center < self.LEFT_BOUND:
            # Object on left, so turn right by the offset
            avoidance_heading = (self.current_angle + avoidance_offset) % 360
            direction = "right"
        elif x_center > self.RIGHT_BOUND:
            # Object on right, so turn left by the offset
            avoidance_heading = (self.current_angle - avoidance_offset) % 360
            direction = "left"
        else:
            print("[Avoidance] Object in center; stopping.")
            self.robot.stop()
            return

        print(f"[Avoidance] Detected object with area ratio {ratio:.3f}. Turning {direction} to heading {avoidance_heading:.2f}°")
        self.current_angle = self.turn_to_angle(avoidance_heading, self.current_angle)
        # Drive forward a short distance to clear the obstacle
        self.robot.forward(0.4)
        time.sleep(1)  # Adjust the time as needed based on your robot
        self.robot.stop()
        # Wait until the obstacle is cleared (i.e., no object detected or object area below a threshold)
        start_time = time.time()
        timeout = 5  # seconds
        while time.time() - start_time < timeout:
            if not self.vision_thread.objects:
                break
            main_object = max(self.vision_thread.objects, key=lambda o: o.bbox.width * o.bbox.height)
            obj_area = main_object.bbox.width * main_object.bbox.height
            if (obj_area / frame_area) < 0.01:
                break
            time.sleep(0.1)
        print("[Avoidance] Object cleared; resuming navigation.")

    # -------------------------------
    # MAIN NAVIGATION LOOP
    # -------------------------------
    def run_navigation(self):
        try:
            for (tx, ty) in self.waypoints:
                # If objects are detected, perform avoidance maneuver
                if self.vision_thread.objects:
                    print("[Navigator] Detected objects; initiating avoidance...")
                    self.avoid_object()
                # Proceed to next waypoint
                self.move_to_target(tx, ty)
            print("[Navigator] All waypoints completed. Returning to base...")
            self.robot.stop()
        except KeyboardInterrupt:
            print("[Navigator] Navigation interrupted by user.")
        finally:
            self.cleanup()

    def cleanup(self):
        print("[Navigator] Cleaning up resources...")
        self.vision_thread.stop()
        self.vision_thread.join()
        GPIO.cleanup()
        self.pipeline.stop()
        cv2.destroyAllWindows()
        print("[Navigator] Cleanup complete.")

# -------------------------------
# MAIN ENTRY POINT
# -------------------------------
def main():
    navigator = RobotNavigator("/home/pi/Desktop/robot/aiy-maker-kit/examples/rssetup/waypoint.txt")
    navigator.run_navigation()

if __name__ == "__main__":
    main()