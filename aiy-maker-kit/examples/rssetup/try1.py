import time
import board
import adafruit_mpu6050
import RPi.GPIO as GPIO
from CRobot import CRobot

# Initialize MPU6050
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)

# Initialize Motor Driver
LMPins = (8, 11)
RMPins = (10, 18)
PWMPins = (7, 9)
Robot = CRobot(LMPins, RMPins, PWMPins)

def turn_90_degrees():
    """Turns the robot 90 degrees using the gyroscope Z-axis."""
    target_angle = -87.5
    current_angle = 0.0
    last_time = time.time()

    Robot.right(0.6)  # Start turning right

    while current_angle > target_angle:
        now = time.time()
        dt = now - last_time  # Time difference
        last_time = now
        
        print(current_angle)

        gyro_z = mpu.gyro[2]  # Read Z-axis angular velocity (rad/s)
        current_angle += (gyro_z * dt) * (180 / 3.14159)  # Convert to degrees

        time.sleep(0.01)  # Small delay for stability

    Robot.stop()  # Stop once 90° is reached
    print(f"Turn completed! Final angle: {current_angle:.2f}°")

# --- Main Execution ---
try:
    turn_90_degrees()

except KeyboardInterrupt:
    print("Program interrupted by user.")

except Exception as e:
    print(f"Error: {e}")

finally:
    GPIO.cleanup()
    print("GPIO cleaned up.")