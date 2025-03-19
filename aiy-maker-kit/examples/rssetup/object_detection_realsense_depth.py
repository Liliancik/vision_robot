import pyrealsense2 as rs
import numpy as np
import cv2
from aiymakerkit import vision
from aiymakerkit import utils
import models

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Color stream
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Depth stream
pipeline.start(config)

# Align depth to color
align = rs.align(rs.stream.color)

# Initialize the object detector
detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)

try:
    while True:
        # Wait for a frame of data
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)  # Align depth to color frame
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue  # Skip if frames are not available

        # Convert the color frame to a numpy array
        frame = np.asanyarray(color_frame.get_data())

        # Perform object detection
        objects = detector.get_objects(frame, threshold=0.4)

        for obj in objects:
            # Access bounding box and other attributes
            bbox = obj.bbox  # `bbox` is an instance of `BBox`
            xmin, ymin, xmax, ymax = bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax

            # Calculate the center of the bounding box
            x_center = int((xmin + xmax) / 2)
            y_center = int((ymin + ymax) / 2)

            # Get the depth value at the center of the bounding box
            distance = depth_frame.get_distance(x_center, y_center)

            # Access label using `obj.id` and annotate the frame
            label = f"{labels[obj.id]}: {distance:.2f} m"
            cv2.putText(frame, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Draw the detected objects on the frame
        vision.draw_objects(frame, objects, labels)

        # Display the frame with OpenCV
        cv2.imshow("Object Detection with Depth", frame)

        # Exit loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the pipeline and close OpenCV window after processing
    pipeline.stop()
    cv2.destroyAllWindows()