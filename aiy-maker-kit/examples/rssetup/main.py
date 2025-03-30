import threading
import time
import math
import RPi.GPIO as GPIO
from collections import deque

import pyrealsense2 as rs
import numpy as np
import cv2

# Robot & sensor libraries
from CRobot import CRobot
import mpu6050
from aiymakerkit import vision, utils
import models

# -------------------------------
# VisionThread for RealSense Capture & Display
# -------------------------------
class VisionThread(threading.Thread):
    def __init__(self, pipeline, align):
        super().__init__()
        self.pipeline = pipeline
        self.align = align
        
        # Initialize the object detector
        self.detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
        self.labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)

        # Shared data
        self.frame = None      # Latest color frame (numpy array)
        self.objects = []      # Latest detected objects
        self.running = True    # Control flag to stop the thread

        # Create a named window for display
        cv2.namedWindow("VisionThread", cv2.WINDOW_NORMAL)

    def run(self):
        while self.running:
            # Grab frames from RealSense (with a timeout to avoid blocking)
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=100)
            except RuntimeError:
                # Frame timeout, skip iteration
                continue

            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            if not color_frame:
                continue

            # Convert to numpy array
            frame = np.asanyarray(color_frame.get_data())
            self.frame = frame

            # Run object detection
            objects = self.detector.get_objects(frame, threshold=0.4)
            self.objects = objects

            # Draw detections for display
            display_frame = frame.copy()
            vision.draw_objects(display_frame, objects, self.labels)

            # Show the frame
            cv2.imshow("VisionThread", display_frame)
            cv2.waitKey(1)

        cv2.destroyAllWindows()

    def stop(self):
        self.running = False

# -------------------------------
# Example Robot Navigation Class
# -------------------------------
class RobotNavigator:
    def __init__(self, waypoint_file):
        # Encoders
        self.el = 4
        self.er = 26
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.el, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.er, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.el, GPIO.RISING, callback=self.encoder_left_cb)
        GPIO.add_event_detect(self.er, GPIO.RISING, callback=self.encoder_right_cb)
        self.countl = 0
        self.countr = 0

        # Robot metrics
        self.WHEEL_DIAMETER = 0.065
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER
        self.ENCODER_TICKS_PER_REV = 20
        self.DISTANCE_PER_TICK = self.WHEEL_CIRCUMFERENCE / self.ENCODER_TICKS_PER_REV

        # Robot & Gyro
        LMPins = (8, 11)
        RMPins = (10, 18)
        PWMPins = (7, 9)
        self.robot = CRobot(LMPins, RMPins, PWMPins)
        self.mpu = mpu6050.mpu6050(0x68)

        # RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.FRAME_WIDTH = 640
        self.FRAME_HEIGHT = 480
        self.config.enable_stream(rs.stream.color, self.FRAME_WIDTH, self.FRAME_HEIGHT, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)
        self.LEFT_BOUND = self.FRAME_WIDTH // 3
        self.RIGHT_BOUND = 2 * self.LEFT_BOUND

        # Start the vision thread
        self.vision_thread = VisionThread(self.pipeline, self.align)
        self.vision_thread.start()

        # Waypoints
        self.waypoints = self.load_waypoints(waypoint_file)
        self.current_x, self.current_y = 0.0, 0.0
        self.current_angle = 0.0

    # Encoder callbacks
    def encoder_left_cb(self, channel):
        self.countl += 1

    def encoder_right_cb(self, channel):
        self.countr += 1

    def load_waypoints(self, file_path):
        x_vals, y_vals = [], []
        with open(file_path, "r") as f:
            for line in f:
                x, y = map(float, line.strip().split(","))
                # Example: divide by 2 for scaling
                x_vals.append(x)
                y_vals.append(y)
        wpts = list(zip(x_vals, y_vals))
        print("[Navigator] Loaded waypoints:", wpts)
        return wpts

    # -------------------------------
    # Gyro & Turn
    # -------------------------------
    def gyro_cal(self, samples=150):
        bias = 0.0
        for _ in range(samples):
            bias += self.mpu.get_gyro_data()['z']
            time.sleep(0.002)
        return bias / samples

    def turn_to_angle(self, target_angle, current_angle):
        bias_z = self.gyro_cal()
        prev_time = time.time()
        init_angle = 0.0

        # PID Gains
        Kp = 0.4
        Ki = 0.1
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
                self.robot.right(speed)
            else:
                self.robot.left(speed)

            time.sleep(0.001)

        self.robot.stop()
        return current_angle + init_angle

    # -------------------------------
    # Navigation
    # -------------------------------
    def get_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def get_angle(self, x1, y1, x2, y2):
        return math.degrees(math.atan2(y2 - y1, x2 - x1))

    def move_to_target(self, target_x, target_y):
        # Compute distance & angle
        dist = self.get_distance(self.current_x, self.current_y, target_x, target_y)
        ang = self.get_angle(self.current_x, self.current_y, target_x, target_y)
        ang = (ang + 360) % 360
        self.current_angle = (self.current_angle + 360) % 360

        # Shortest path
        angle_diff = (ang - self.current_angle + 180) % 360 - 180
        turn_target = self.current_angle + angle_diff
        print(f"[Navigator] Move from ({self.current_x:.2f},{self.current_y:.2f}) to ({target_x:.2f},{target_y:.2f}), angle={ang:.2f}")

        self.current_angle = self.turn_to_angle(turn_target, self.current_angle)

        # Reset encoders
        self.countl, self.countr = 0, 0
        target_ticks = int(dist / self.DISTANCE_PER_TICK)
        print(f"[Navigator] Target ticks = {target_ticks}")
        self.robot.forward(0.6)

        while (self.countl + self.countr) / 2 < target_ticks:
            progress = (self.countl + self.countr) / 2
            if progress >= 0.6 * target_ticks:
                speed = 1 - (0.8 * (progress / target_ticks))
                speed = max(speed, 0.4)
                self.robot.forward(speed)
            time.sleep(0.01)

        self.robot.stop()
        time.sleep(1)

        self.current_x, self.current_y = target_x, target_y
        return self.current_x, self.current_y, self.current_angle
    
    def avoid_object(self):
        """
        If objects are detected in the vision thread, this method computes an avoidance turn.
        It uses the largest object's bounding box to compute an avoidance angle offset.
        After performing the avoidance maneuver, it waits until the object is no longer detected.
        """
        # Get the latest detected objects from the vision thread
        objects = self.vision_thread.objects
        if not objects:
            return
        
        # Choose the largest detected object (based on bounding box area)
        main_object = max(objects, key=lambda o: o.bbox.width * o.bbox.height)
        
        # Calculate the horizontal center of the object's bounding box
        x_center = main_object.bbox.xmin + (main_object.bbox.width / 2)
        
        # Calculate the area ratio of the object relative to the frame
        frame_area = self.FRAME_WIDTH * self.FRAME_HEIGHT
        obj_area = main_object.bbox.width * main_object.bbox.height
        ratio = obj_area / frame_area
        
        # Compute an avoidance angle offset based on the area ratio (adjust the factor as needed)
        avoidance_offset = min(ratio * 300, 90)  # e.g., if ratio is 0.1, offset becomes 30°; ratio 0.2 becomes 60°
        
        if x_center < self.LEFT_BOUND:
            # Object is on the left side; so turn right to avoid
            avoidance_heading = (self.current_angle + avoidance_offset) % 360
            direction = "right"
        elif x_center > self.RIGHT_BOUND:
            # Object is on the right side; so turn left to avoid
            avoidance_heading = (self.current_angle - avoidance_offset) % 360
            direction = "left"
        else:
            print("[Avoidance] Object in center; stopping.")
            self.robot.stop()
            return
        
        print(f"[Avoidance] Detected object with area ratio {ratio:.3f}. Turning {direction} to heading {avoidance_heading:.2f}°")
        
        # Turn the robot by the computed avoidance offset
        self.current_angle = self.turn_to_angle(avoidance_heading, self.current_angle)
        
        # Drive forward a short distance to clear the obstacle
        self.robot.forward(0.4)
        time.sleep(1)  # Adjust as needed based on your robot's speed and obstacle distance
        self.robot.stop()
        
        # Wait until the obstacle is cleared (i.e. no object detected or object area is small)
        start_time = time.time()
        timeout = 5  # seconds
        while time.time() - start_time < timeout:
            if not self.vision_thread.objects:
                break
            # Check if the detected object's area has reduced below a threshold
            main_object = max(self.vision_thread.objects, key=lambda o: o.bbox.width * o.bbox.height)
            obj_area = main_object.bbox.width * main_object.bbox.height
            if (obj_area / frame_area) < 0.01:
                break
            time.sleep(0.1)
        
        print("[Avoidance] Object cleared; resuming navigation.")
        
    
    # -------------------------------
    # Main Loop
    # -------------------------------
    def run_navigation(self):
        try:
            for (tx, ty) in self.waypoints:
                # Access the vision thread’s objects
                # Example: If you want to react if an object is found
                if self.vision_thread.objects:
                    print("[Navigator] Detected objects, can react if needed.")
                    # For instance, you might do a quick check here or call a function
                    self.avoid_object()

                # Move to the next waypoint
                self.move_to_target(tx, ty)

            print("[Navigator] All waypoints done.")
            self.robot.stop()

        except KeyboardInterrupt:
            print("[Navigator] Interrupted by user.")
        finally:
            self.cleanup()

    def cleanup(self):
        print("[Navigator] Cleaning up resources...")
        # Stop the vision thread
        self.vision_thread.stop()
        self.vision_thread.join()

        GPIO.cleanup()
        self.pipeline.stop()
        cv2.destroyAllWindows()
        print("[Navigator] Cleanup complete.")

# -------------------------------
# MAIN
# -------------------------------
def main():
    nav = RobotNavigator("/home/pi/Desktop/robot/aiy-maker-kit/examples/rssetup/waypoint.txt")
    nav.run_navigation()

if __name__ == "__main__":
    main()