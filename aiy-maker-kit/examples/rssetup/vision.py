import pyrealsense2 as rs
import numpy as np
import cv2
from aiymakerkit import vision, utils
import models
from CRobot import CRobot

class VisionController:
    def __init__(self, left_bound_ratio=1/3):
        # Initialize the object detector
        self.detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
        self.labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)
        
        # Initialize the robot
        self.robot = CRobot(
            LMPins=(8, 11),  # AIN1, AIN2
            RMPins=(10, 18), # BIN1, BIN2
            PWMPins=(7, 9)   # PWMA, PWMB
        )
        
        # Configure RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        
        self.FRAME_WIDTH = 640
        self.LEFT_BOUND = int(self.FRAME_WIDTH * left_bound_ratio)
        self.RIGHT_BOUND = 2 * self.LEFT_BOUND
        
        # Create the display window
        cv2.namedWindow("Object Detection - RealSense", cv2.WINDOW_NORMAL)
    
    def run(self):
        try:
            while True:
                # Wait for a coherent frame
                try:
                    frames = self.pipeline.wait_for_frames(timeout_ms=100)
                except RuntimeError:
                    # Frame timeout; skip this iteration.
                    continue

                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                # Convert RealSense frame to a NumPy array
                frame = np.asanyarray(color_frame.get_data())
                
                # Run object detection
                objects = self.detector.get_objects(frame, threshold=0.4)
                
                if objects:
                    # Find the largest detected object
                    main_object = max(objects, key=lambda obj: obj.bbox.width * obj.bbox.height)
                    x_center = main_object.bbox.xmin + (main_object.bbox.width // 2)
                    # Determine the quadrant and issue a command
                    if x_center < self.LEFT_BOUND:
                        print("Object on LEFT → Turning RIGHT")
                        self.robot.left(0.4)
                    elif x_center > self.RIGHT_BOUND:
                        print("Object on RIGHT → Turning LEFT")
                        self.robot.right(0.4)
                    else:
                        print("Object in CENTER → STOP")
                        self.robot.stop()
                else:
                    print("No object detected → Forward")
                    self.robot.forward(0.4)
                
                # Draw detections on the frame for visualization
                vision.draw_objects(frame, objects, self.labels)
                cv2.imshow("Object Detection - RealSense", frame)
                
                # Press 'q' to exit the loop
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.stop()
    
    def stop(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        self.robot.stop()