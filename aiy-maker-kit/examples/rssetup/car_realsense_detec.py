import pyrealsense2 as rs
import numpy as np
import cv2
from aiymakerkit import vision, utils
import models
from CRobot import CRobot

# Initialize the object detector
detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)

# Initialize the robot
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

FRAME_WIDTH = 640
LEFT_BOUND = FRAME_WIDTH // 3
RIGHT_BOUND = 2 * LEFT_BOUND

try:
    while True:
        # Wait for a coherent frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
       
        if not color_frame:
            continue
       
        # Convert RealSense frame to NumPy array
        frame = np.asanyarray(color_frame.get_data())

        # Run object detection
        objects = detector.get_objects(frame, threshold=0.4)

        # Find the main detected object
        if objects:
            main_object = max(objects, key=lambda obj: obj.bbox.width * obj.bbox.height)  # Largest object
            x_center = main_object.bbox.xmin + (main_object.bbox.width // 2)

            # Determine quadrant
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
            print("No object detected → Stopping")
            robot.backward(0.4)

        # Draw detected objects on the frame
        vision.draw_objects(frame, objects, labels)
        cv2.imshow("Object Detection - RealSense", frame)

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the RealSense pipeline
    pipeline.stop()
    cv2.destroyAllWindows()
    robot.stop()