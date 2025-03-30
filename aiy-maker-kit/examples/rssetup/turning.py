import time
from collections import deque
from robot_control import robot, mpu

def gyro_cal(samples=150):
    """Calibrates the MPU6050 by averaging samples."""
    gyro_z = 0
    for _ in range(samples):
        gyro_z += mpu.get_gyro_data()['z']
        time.sleep(0.002)  # Reduced delay for faster sampling
    return gyro_z / samples

def turn_to_angle(target_angle, current_angle):
    """
    Turns the robot to a specific heading using a PID-based gyro control.
    Returns the updated estimated heading.
    """
    bias_z = gyro_cal()  # Recalibrate before turning
    prev_time = time.time()
    init_angle = 0
    # PID coefficients (tune as needed)
    Kp = 0.4
    Ki = 0.01
    Kd = 0.2
    integral = 0
    last_error = 0
    filter_size = deque(maxlen=7)
    # Determine turning direction (not used directly in control here)
    turn_direction = 1 if target_angle <= current_angle else -1

    while True:
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        # Read and filter gyro data
        angular_vel = mpu.get_gyro_data()['z'] - bias_z
        filter_size.append(angular_vel)
        avg_angular_vel = sum(filter_size) / len(filter_size)
        init_angle += avg_angular_vel * dt

        # Calculate error between desired turn and integrated angle
        error = (target_angle - current_angle) - init_angle

        # PID Controller calculations
        integral += error * dt
        derivative = (error - last_error) / dt
        last_error = error

        output = Kp * error + Ki * integral + Kd * derivative
        speed = abs(output) * 0.4
        speed = min(max(speed, 0.4), 0.6)  # Clamp speed within [0.4, 0.6]

        # Stop if within 2° error
        if abs(error) < 2:
            break

        # Adjust turning direction
        if error > 0:
            print(f"Current angle: {init_angle:.2f}°, Target angle: {target_angle:.2f}°")
            robot.right(speed)
        else:
            robot.left(speed)
            print(f"Current angle: {init_angle:.2f}°, Target angle: {target_angle:.2f}°")
        time.sleep(0.001)

    robot.stop()
    return current_angle + init_angle