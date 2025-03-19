import time
import board
import adafruit_mpu6050
import RPi.GPIO as GPIO
from CRobot import CRobot

# Initialize MPU6050
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)

# Initialize Motors
LMPins = (8, 11)
RMPins = (10, 18)
PWMPins = (7, 9)
Robot = CRobot(LMPins, RMPins, PWMPins)

# Encoder Setup
el = 4   # Left encoder pin
er = 26  # Right encoder pin
countl = 0
countr = 0

GPIO.setup(el, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(er, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Encoder Callbacks
def encoder_callback_left(channel):
    global countl
    countl += 1

def encoder_callback_right(channel):
    global countr
    countr += 1

GPIO.add_event_detect(el, GPIO.RISING, callback=encoder_callback_left)
GPIO.add_event_detect(er, GPIO.RISING, callback=encoder_callback_right)

# Function to Read Gyroscope Data
def read_gyro_z():
    """Reads gyroscope Z-axis angular velocity in degrees/sec"""
    return mpu.gyro[2] * (180 / 3.14159)  # Convert rad/s to deg/s

# Function to Turn Exactly 90 Degrees
def turn_90_degrees():
    """Uses both gyroscope and encoder counts for precise 90-degree rotation"""
    target_angle = -85.5
    current_angle = 0.0
    last_time = time.time()
    
    # Reset Encoder Counts
    global countl, countr
    countl, countr = 0, 0

    Robot.right(0.7)  # Start turning right

    while(countl < 19 and countr < 20):  # Dual check
        
        
        
        now = time.time()
        dt = now - last_time
        last_time = now

        gyro_z = read_gyro_z()
        if(current_angle < target_angle):
            break
        else:
            current_angle += gyro_z * dt  # Integrate gyro data to estimate angle

        time.sleep(0.001)  # Small delay for stability

    Robot.stop()  # Stop once 90° is reached
    print(f"Turn completed! Final angle: {current_angle:.2f}°")
    print(f"Final encoder counts - Left: {countl}, Right: {countr}")

# --- Main Execution ---
try:
    # Move backward a bit before turning
    countl, countr = 0, 0
    Robot.backward(0.5)
    Robot.stop()
    time.sleep(2)

    # Perform the 90-degree turn
    turn_90_degrees()
    time.sleep(2)

    # Repeat pattern multiple times
    for _ in range(3):
        countl, countr = 0, 0
        Robot.backward(0.5)
        Robot.stop()
        time.sleep(2)

        turn_90_degrees()
        time.sleep(2)

except KeyboardInterrupt:
    print("Program interrupted.")

finally:
    GPIO.cleanup()
    print("GPIO cleaned up.")