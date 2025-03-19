from aiymakerkit import vision
from aiymakerkit import utils
import models

# Initialize the detector with the object detection model
detector = vision.Detector(models.OBJECT_DETECTION_MODEL)

# Read the labels from the model's metadata
labels = utils.read_labels_from_metadata(models.OBJECT_DETECTION_MODEL)

# Loop through each frame from the camera
for frame in vision.get_frames():
    # Detect objects in the frame with a confidence threshold of 0.4
    objects = detector.get_objects(frame, threshold=0.4)
   
    # Draw the detected objects on the frame
    vision.draw_objects(frame, objects, labels)
   
    # Display the frame with detected objects
    #vision.show_frame(frame)vision.waitKey(1)

    # Break the loop if 'q' is pressed
      

# Release resources
#vision.release()