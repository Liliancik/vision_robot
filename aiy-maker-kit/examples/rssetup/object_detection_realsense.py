import pyrealsense2 as rs
import numpy as np
import cv2
from aiymakerkit import vision
from aiymakerkit import utils
import models

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Color stream, resolution 640x480, 30 FPS
pipeline.start(config)

# Initialize the object detector
detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)

try:
    while True:
        # Wait for a frame of data
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue  # If no frame is available, skip the iteration

        # Convert the frame to a numpy array
        frame = np.asanyarray(color_frame.get_data())

        # Perform object detection
        objects = detector.get_objects(frame, threshold=0.4)

        # Draw the detected objects on the frame
        vision.draw_objects(frame, objects, labels)

        # Display the frame with OpenCV
        cv2.imshow("Object Detection", frame)

        # Exit loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the pipeline and close OpenCV window after processing
    pipeline.stop()
    cv2.destroyAllWindows()