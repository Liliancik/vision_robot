import pyrealsense2 as rs
import numpy as np
import cv2
from aiymakerkit import vision, utils
import models
from CRobot import CRobot

# Initialize object detector
detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)

# Initialize robot motor driver
robot = CRobot(
    LMPins=(8, 11),  # AIN1, AIN2
    RMPins=(10, 12),  # BIN1, BIN2
    PWMPins=(7, 9)  # PWMA, PWMB
)

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Define frame segmentation
FRAME_WIDTH = 640
LEFT_BOUND = FRAME_WIDTH // 3  # Left section
RIGHT_BOUND = 2 * LEFT_BOUND   # Right section

try:
    while True:
        # Get frames from RealSense
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
       
        if not color_frame:
            continue
       
        # Convert to NumPy array
        frame = np.asanyarray(color_frame.get_data())

        # Run object detection
        objects = detector.get_objects(frame, threshold=0.4)

        # Determine object position
        if objects:
            # Select the largest detected object
            main_object = max(objects, key=lambda obj: obj.bbox.width * obj.bbox.height)
            x_center = main_object.bbox.xmin + (main_object.bbox.width // 2)

            # Determine movement based on position
            if x_center < LEFT_BOUND:
                print("Object on LEFT → Turning RIGHT")
                robot.right(0.4)
            elif x_center > RIGHT_BOUND:
                print("Object on RIGHT → Turning LEFT")
                robot.left(0.4)
            else:
                print("Object in CENTER → STOP")
                robot.stop()
        else:
            print("No object detected → Run")
            robot.backward(0.4)

        # Draw detected objects
        vision.draw_objects(frame, objects, labels)
        cv2.imshow("Object Detection - RealSense", frame)

        # Exit on 'q' press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Cleanup
    pipeline.stop()
    cv2.destroyAllWindows()
robot.stop()
