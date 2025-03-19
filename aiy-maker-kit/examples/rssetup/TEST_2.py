import time
import math
import board
import adafruit_mpu6050
import RPi.GPIO as GPIO
from CRobot import CRobot

# --- Initialize MPU6050 ---
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)


# --- Initialize Motors ---
robot = CRobot(
    LMPins=(8, 11),  # AIN1, AIN2
    RMPins=(10, 18),  # BIN1, BIN2
    PWMPins=(7, 9)  # PWMA, PWMB
)

# --- Encoder Setup ---
el = 4   # Left encoder pin
er = 26  # Right encoder pin
countl, countr = 0, 0

GPIO.setup(el, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(er, GPIO.IN, pull_up_down=GPIO.PUD_UP)


# --- Encoder Callbacks ---
def encoder_callback_left(channel):
    global countl
    countl += 1

def encoder_callback_right(channel):
    global countr
    countr += 1

GPIO.add_event_detect(el, GPIO.RISING, callback=encoder_callback_left)
GPIO.add_event_detect(er, GPIO.RISING, callback=encoder_callback_right)

# --- Function to Read Gyroscope Data ---
def read_gyro_z():
    """Reads gyroscope Z-axis angular velocity in degrees/sec"""
    return mpu.gyro[2] * (180 / 3.14159)  # Convert rad/s to deg/s

# --- Function to Turn Exactly to a Target Heading ---
def turn_to_angle(target_angle):
    """Turns the robot to a specific heading using the gyroscope."""
    current_angle = 0.0
    last_time = time.time()

    #its being called again here:
    #global countl, countr
    #countl, countr = 0, 0  # Reset encoder counts
    
    #implement:
    # target ang/4.5deg = tic count
    #left is target > 0
    #right is target < 0
    #use encoder to calculate distance
    
    
    robot.right(0.6)  # Start turning right

    while countl < 19 and countr < 20 and abs(current_angle) < abs(target_angle):  
        now = time.time()
        dt = now - last_time
        last_time = now

        gyro_z = read_gyro_z()
        current_angle += gyro_z * dt  # Integrate gyro data to estimate angle

        time.sleep(0.001)  # Small delay for stability

    robot.stop()
    print(f"Turn completed! Final angle: {current_angle:.2f}°")
    print(f"Final encoder counts - Left: {countl}, Right: {countr}")

# --- Function to Calculate Distance Between Points ---
def get_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# --- Function to Calculate Angle to Target ---
def get_angle(x1, y1, x2, y2):   
    return math.degrees(math.atan2(y2 - y1, x2 - x1))

# --- Function to Move Towards a Target Using Encoders & Gyro ---
def move_to_target(robot, target_x, target_y, current_x, current_y, robot_heading):
    angle_threshold = 4  # Allow small angle error
    distance_threshold = 4  # Stop when close to the target

    target_distance = get_distance(current_x, current_y, target_x, target_y)
    angle_to_target = get_angle(current_x, current_y, target_x, target_y)
    
    print(angle_to_target)

    print(f"Moving from ({current_x}, {current_y}) to ({target_x}, {target_y})")
    print(f"Target angle: {angle_to_target:.2f}°")

    # Adjust heading before moving forward
    turn_to_angle(angle_to_target)
    robot_heading = angle_to_target  # Update heading

    # Move forward while checking encoders
    countl, countr = 0, 0  # Reset encoder counts
    robot.backward(0.7)  # Move forward

    while (countl + countr) / 2 < target_distance * 10:  # Adjust scale factor if needed
        time.sleep(0.01)  # Small delay for stability

    robot.stop()
    print(f"Arrived at ({target_x}, {target_y})")

    return target_x, target_y, robot_heading  # Update position and heading

num = 0
# --- Load Waypoints from File ---
file_path = "/home/pi/Downloads/waypoints.txt"
x_vals, y_vals = [], []

with open(file_path, "r") as file:
    for line in file:
        x, y = map(int, line.strip().replace(" ", "").split(","))  # Remove spaces
        if (num == 0):
            x_vals.append(0)
            y_vals.append(0)

        else:
            x_vals.append(x / 100)
            y_vals.append(y / 100)
        
        num+=1

waypoints = list(zip(x_vals, y_vals))  # Pair x, y coordinates
print("Loaded waypoints:", waypoints)

# --- Follow the Path ---
current_x, current_y = 0, 0  # Start position
robot_heading = 0  # Assume initial heading is 0°

try:
    for target_x, target_y in waypoints:
        current_x, current_y, robot_heading = move_to_target(
            robot, target_x, target_y, current_x, current_y, robot_heading
        )

    print("Path completed!")
    robot.stop()

except KeyboardInterrupt:
    print("Program interrupted.")

finally:
    GPIO.cleanup()
    print("GPIO cleaned up.")

