import time
import math
from CRobot import CRobot

file_path = "/home/pi/Downloads/waypoints.txt"

robot = CRobot(
    LMPins=(8, 11),  # AIN1, AIN2
    RMPins=(10, 18),  # BIN1, BIN2
    PWMPins=(7, 9)  # PWMA, PWMB
)

x_vals, y_vals = [], []

# Read waypoints
with open(file_path, "r") as file:
    for line in file:
        x, y = map(int, line.strip().replace(" ", "").split(","))  # Remove spaces and split
        x_vals.append(x/100)
        y_vals.append(y/100)

waypoints = list(zip(x_vals, y_vals))  # Pair x, y coordinates as waypoints
print("Loaded waypoints:", waypoints)

# Function to calculate distance between two points
def get_distance(x1, y1, x2, y2):
    return (math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))

# Function to calculate angle to target
def get_angle(x1, y1, x2, y2):
    return math.degrees(math.atan2(y2 - y1, x2 - x1))

# Function to move towards a target point
def move_to_target(robot, target_x, target_y, current_x, current_y, robot_heading):
    angle_threshold = 4  # Reduce threshold for finer adjustments
    distance_threshold = 4  # Reduce for more precise waypoint following

    while get_distance(current_x, current_y, target_x, target_y) > distance_threshold:
        # Calculate the required angle
        angle_to_target = get_angle(current_x, current_y, target_x, target_y)
        angle_diff = angle_to_target - robot_heading

        # Normalize angle difference to [-180, 180] range
        angle_diff = (angle_diff + 180) % 360 - 180

        if abs(angle_diff) > angle_threshold:
            # Adjust turning speed based on angle difference
            turn_speed = max(0.4, min(0.8, abs(angle_diff) / 90))  # Scale speed with angle
            if angle_diff > 0:
                print("Turning LEFT")
                robot.right(0.8)
            else:
                print("Turning RIGHT")
                robot.left(0.8)
            time.sleep(0.5)  # Small turn duration
        else:
            print("Moving FORWARD")
            robot.backward(0.4)  # Move forward at moderate speed
            time.sleep(0.5)

        # Update estimated position (simple model)
        move_step = 4
        current_x += move_step * math.cos(math.radians(angle_to_target))
        current_y += move_step * math.sin(math.radians(angle_to_target))

        # Update robot heading to last movement direction
        robot_heading = angle_to_target

    robot.stop()
    return current_x, current_y, robot_heading  # Return updated position and heading

# Follow the path
current_x, current_y = 0, 0  # Start position (adjust based on reality)
robot_heading = 1  # Start heading (adjust if known)

for target_x, target_y in waypoints:
    print(f"Moving to waypoint: {target_x}, {target_y}")
    current_x, current_y, robot_heading = move_to_target(robot, target_x, target_y, current_x, current_y, robot_heading)

print("Path completed!")
robot.stop()