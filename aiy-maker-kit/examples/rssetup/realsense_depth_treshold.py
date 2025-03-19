import pyrealsense2 as rs
import numpy as np
import cv2
from aiymakerkit import vision, utils
import models

# Initialize the object detector
detector = vision.Detector(models.OBJECT_DETECTION_MODEL)
labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable color and depth streams
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

# Align depth to color
align = rs.align(rs.stream.color)

# Get frame dimensions
frame_width = 640
frame_height = 480
depth_width = 640
depth_height = 480

# Define quadrant boundaries
left_boundary = frame_width // 3
right_boundary = 2 * (frame_width // 3)

try:
    while True:
        # Wait for frames and align them
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        # Get color and depth frames
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Convert RealSense frame to NumPy array
        frame = np.asanyarray(color_frame.get_data())

        # Run object detection
        objects = detector.get_objects(frame, threshold=0.4)

        # Process detected objects
        for obj in objects:
            x, y, width, height = obj.bbox
            cx, cy = int(x + width / 2), int(y + height / 2)  # Center of the bounding box

            # Ensure (cx, cy) is within valid depth frame range
            if 0 <= cx < depth_width and 0 <= cy < depth_height:
                depth_value = depth_frame.get_distance(cx, cy)  # Distance in meters
               
                # Determine quadrant
                if cx < left_boundary:
                    quadrant = "Left"
                elif cx < right_boundary:
                    quadrant = "Center"
                else:
                    quadrant = "Right"

                # Filter out objects beyond 2 meters
                if 0 < depth_value <= 2.0:
                    label = f"{labels.get(obj.id, 'Unknown')} ({depth_value:.2f}m) - {quadrant}"
                    vision.draw_objects(frame, [obj], labels={obj.id: label})

        # Display the result
        cv2.imshow("Object Detection - RealSense", frame)

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the RealSense pipeline
    pipeline.stop()
    cv2.destroyAllWindows()