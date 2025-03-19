import pyrealsense2 as rs
import numpy as np
import cv2

# Initialize pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth)

# Start pipeline
pipeline.start(config)

# Get depth sensor
depth_sensor = pipeline.get_active_profile().get_device().first_depth_sensor()

while True:
    # Wait for a new frame
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()

    # Get depth data (in meters)
    depth_image = np.asanyarray(depth_frame.get_data())

    # Find the depth at the center of the frame
    center_x, center_y = depth_image.shape[1] // 2, depth_image.shape[0] // 2
    distance = depth_image[center_y, center_x] * depth_sensor.get_depth_scale()  # in meters

    # Display the distance on the image
    cv2.putText(depth_image, f"Distance: {distance:.2f} meters", (center_x - 100, center_y - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

    # Show depth image
    cv2.imshow('Depth Image', depth_image)

    # Break on key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop pipeline
pipeline.stop()