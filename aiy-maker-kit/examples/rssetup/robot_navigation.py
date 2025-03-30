# navigation.py
import math
import time
from hardware import DISTANCE_PER_TICK, countl, countr
from turning import turn_to_angle
from robot_control import robot

def get_distance(x1, y1, x2, y2):
    """Calculate Euclidean distance between two points."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def get_angle(x1, y1, x2, y2):
    """Calculate angle (in degrees) from (x1, y1) to (x2, y2)."""
    return math.degrees(math.atan2(y2 - y1, x2 - x1))

def move_to_target(target_x, target_y, current_x, current_y, current_angle):
    """
    Moves the robot toward the next waypoint using encoder feedback and gyro for heading.
    Returns updated (current_x, current_y, current_angle).
    """
    global countl, countr
    # Calculate required turn
    target_distance = get_distance(current_x, current_y, target_x, target_y)
    target_angle = get_angle(current_x, current_y, target_x, target_y)
    target_angle = (target_angle + 360) % 360  # Normalize to [0, 360)
    current_angle = (current_angle + 360) % 360

    # Determine shortest path turn
    angle_diff = (target_angle - current_angle + 180) % 360 - 180
    shortest_target_angle = current_angle + angle_diff

    print(f"Moving from ({current_x}, {current_y}) to ({target_x}, {target_y})")
    print(f"Target angle: {target_angle:.2f}°")

    # Adjust heading before moving forward
    current_angle = turn_to_angle(shortest_target_angle, current_angle)

    # Reset encoder counts
    countl, countr = 0, 0

    # Convert distance to encoder ticks
    target_ticks = int(target_distance / DISTANCE_PER_TICK)
    robot.forward(0.6)
    print(f"Target ticks: {target_ticks:.2f}")
    print(f"Initial ticks (Right): {countr:.2f}")

    while (countl + countr) / 2 < target_ticks:
        progress = (countl + countr) / 2
        print(f"Ticks (Right): {countr:.2f}")
        # Reduce speed gradually for precision
        if progress >= 0.6 * target_ticks:
            speed = 1 - (0.8 * (progress / target_ticks))
            speed = max(speed, 0.4)
            robot.forward(speed)
        time.sleep(0.01)

    robot.stop()
    time.sleep(1)

    print(f"Arrived at ({target_x}, {target_y})")
    print(f"Final encoder counts - Left: {countl}, Right: {countr}")
    print(f"Estimated Distance Traveled: {(countl + countr) / 2 * DISTANCE_PER_TICK:.3f} meters")
    return target_x, target_y, current_angle