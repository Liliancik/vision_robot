import RPi.GPIO as GPIO
import math
import time
from CRobot import CRobot
from collections import deque
import mpu6050

# ====== ENCODER SETUP ======
el = 4  # Left encoder pin
er = 26  # Right encoder pin

# ====== ROBOT METRICS ======
WHEEL_DIAMETER = 0.065  # 65mm wheels
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER  # Wheel circumference in meters
ENCODER_TICKS_PER_REV = 20  # Encoder pulses per full wheel rotation
DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REV  # Distance per tick

# ====== ENCODER COUNTERS ======
countl, countr = 0, 0

# ====== ENCODER CALLBACKS ======
def encoder_callback_left(channel):
    global countl
    countl += 1

def encoder_callback_right(channel):
    global countr
    countr += 1

# ====== SETUP GPIO ======
GPIO.setmode(GPIO.BCM)
GPIO.setup(el, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(er, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(el, GPIO.RISING, callback=encoder_callback_left)
GPIO.add_event_detect(er, GPIO.RISING, callback=encoder_callback_right)

# ====== INITIALIZE ROBOT & SENSORS ======
LMPins = (8, 11)
RMPins = (10, 18)
PWMPins = (7, 9)
robot = CRobot(LMPins, RMPins, PWMPins)
mpu = mpu6050.mpu6050(0x68)  # MPU6050 Gyroscope Initialization

# ====== GYROSCOPE CALIBRATION ======
def gyro_cal(samples=150):
    """Calibrates the gyroscope by averaging samples."""
    gyro_z = 0
    for _ in range(samples):
        gyro_z += mpu.get_gyro_data()['z']
        time.sleep(0.002)  # Faster sampling, reduced delay
    return gyro_z / samples  # Return bias_z

# ====== PID CONTROLLED TURN FUNCTION ======
def turn_to_angle(target_angle, current_angle):
    """Turns the robot to a specific heading using a PID-based gyro control."""
    bias_z = gyro_cal()  # Recalibrate before turning
    prev_time = time.time()
    init_angle = 0

    # PID Coefficients
    Kp = 0.6  # Proportional gain (affects speed based on error size)
    Ki = 0.01  # Integral gain (corrects small steady-state errors)
    Kd = 0.4  # Derivative gain (smooths rapid changes)
    
    integral = 0
    last_error = 0

    # Moving Average Filter for Smoother Gyro Readings
    filter_size = deque(maxlen=7)

    # Determine Turn Direction
    turn_direction = 1 if target_angle > current_angle else -1

    while True:
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        # Read & Filter Gyro Data
        angular_vel = mpu.get_gyro_data()['z'] - bias_z
        filter_size.append(angular_vel)
        avg_angular_vel = sum(filter_size) / len(filter_size)

        # Integrate Angular Velocity to Get Current Angle
        init_angle += avg_angular_vel * dt

        # Calculate Error (How much more we need to turn)
        error = (target_angle - current_angle) - init_angle
        
        # **PID Controller Calculation**
        integral += error * dt
        derivative = (error - last_error) / dt
        last_error = error

        # Compute PID Output for Speed Adjustment
        output = Kp * error + Ki * integral + Kd * derivative

        # Dynamically adjust speed (ensures smooth turning)
        speed = abs(output) * 0.02  # Scale speed appropriately
        speed = min(max(speed, 0.15), 0.4)  # Ensure speed stays within safe range

        # Stop condition (within 2° error threshold)
        if abs(error) < 2:
            break
        
        # Adjust turning speed dynamically based on PID output
        if error > 0:
            robot.right(speed)
        else:
            robot.left(speed)

        time.sleep(0.01)

    robot.stop()
    return current_angle + init_angle  # Return the new estimated angle

# ====== FUNCTION TO MOVE TOWARDS A TARGET ======
def get_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def get_angle(x1, y1, x2, y2):
    return math.degrees(math.atan2(y2 - y1, x2 - x1))

def move_to_target(target_x, target_y, current_x, current_y, current_angle):
    """Move forward exactly to the next waypoint using encoders and gyro."""
    global countl, countr

    # Calculate required turn
    target_distance = get_distance(current_x, current_y, target_x, target_y)
    target_angle = get_angle(current_x, current_y, target_x, target_y)

    print(f"Moving from ({current_x}, {current_y}) to ({target_x}, {target_y})")
    print(f"Target angle: {target_angle:.2f}°")

    # Adjust heading before moving forward
    current_angle = turn_to_angle(target_angle, current_angle)

    # Reset encoder counts
    countl, countr = 0, 0

    # Convert distance to encoder ticks
    target_ticks = int(target_distance / DISTANCE_PER_TICK)

    # Start moving forward
    robot.forward(0.4)

    while (countl + countr) / 2 < target_ticks:
        progress = (countl + countr) / 2

        # Reduce speed gradually for precision stopping
        if progress >= 0.6 * target_ticks:
            speed = 0.4 - (0.2 * (progress / target_ticks))
            speed = max(speed, 0.2)
            robot.forward(speed)

        time.sleep(0.01)

    robot.stop()
    time.sleep(1)

    print(f"Arrived at ({target_x}, {target_y})")
    print(f"Final encoder counts - Left: {countl}, Right: {countr}")
    print(f"Estimated Distance Traveled: {(countl + countr) / 2 * DISTANCE_PER_TICK:.3f} meters")

    return target_x, target_y, current_angle

# ====== LOAD WAYPOINTS FROM FILE ======
file_path = "/home/pi/Downloads/waypoints.txt"
x_vals, y_vals = [], []

with open(file_path, "r") as file:
    for line in file:
        x, y = map(float, line.strip().split(","))
        x_vals.append(x)
        y_vals.append(y)

waypoints = list(zip(x_vals, y_vals))
print("Loaded waypoints:", waypoints)

# ====== NAVIGATION LOOP ======
current_x, current_y = 0, 0  # Start position
current_angle = 0  # Assume initial heading is 0°

try:
    for target_x, target_y in waypoints:
        current_x, current_y, current_angle = move_to_target(
            target_x, target_y, current_x, current_y, current_angle
        )

    print("Path completed! Returning to base...")
    robot.stop()

except KeyboardInterrupt:
    print("Program interrupted.")

finally:
    GPIO.cleanup()
    print("GPIO cleaned up.")
