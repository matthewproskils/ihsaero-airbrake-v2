# bno055_test.py Simple test program for MicroPython bno055 driver

# Copyright (c) Peter Hinch 2019
# Released under the MIT licence.

from machine import Pin ,I2C
import time
from bno055 import *
import time
import math
# Tested configurations
# Pyboard hardware I2C
# i2c = machine.I2C(1)

# Pico: hard I2C doesn't work without this patch
# https://github.com/micropython/micropython/issues/8167#issuecomment-1013696765
i2c = I2C(0, scl=Pin(9), sda=Pin(8)) #EIO error almost immediately
# YOU MUST USE RP2040
print("I2C scan:", i2c.scan())

imu = BNO055(i2c, 0x28)
calibrated = False

# sensor fusion
i2c.writeto_mem(0x28, 0x3D, b'\x0C')  # Set to NDOF (Sensor Fusion)
time.sleep(0.1)  # Allow time for mode change


with open('calibration.bin', 'rb') as file:
    # Read the previous calibration data
    data = file.read()
    
imu.set_offsets(data)
# Initialize variables for filtering

# Initialize Kalman filter variables
velocity = [0.0, 0.0, 0.0]  # Estimated velocity
P = [1.0, 1.0, 1.0]  # Error covariance (initial uncertainty for each axis)
Q = 0.1  # Process noise (acceleration uncertainty)
R = 0.5  # Measurement noise (accelerometer uncertainty)
K = [0.0, 0.0, 0.0]  # Kalman Gain

# Initialize previous time
previous_time = time.time()

# Initialize the buffer for the high-pass filter
window_size = 30  # Number of samples for moving average
accel_buffer = [[0.0, 0.0, 0.0]] * window_size  # Buffer to hold previous acceleration readings
buffer_index = 0  # Index for circular buffer

# Initialize Kalman filter variables
velocity = [0.0, 0.0, 0.0]  # Estimated velocity
P = [1.0, 1.0, 1.0]  # Error covariance (initial uncertainty for each axis)
Q = 0.1  # Process noise (acceleration uncertainty)
R = 0.5  # Measurement noise (accelerometer uncertainty)
K = [0.0, 0.0, 0.0]  # Kalman Gain

# Initialize previous time
previous_time = time.time()

# Initialize the buffer for the high-pass filter
window_size = 30  # Number of samples for moving average
accel_buffer = [[0.0, 0.0, 0.0]] * window_size  # Buffer to hold previous acceleration readings
buffer_index = 0  # Index for circular buffer

# Gyro orientation variables (for adjusting accelerometer data to world frame)
gyro_orientation = [0.0, 0.0, 0.0]  # Roll, pitch, yaw

# Initialize Kalman filter variables
velocity = [0.0, 0.0, 0.0]  # Estimated velocity
P = [1.0, 1.0, 1.0]  # Error covariance (initial uncertainty for each axis)
Q = 0.1  # Process noise (acceleration uncertainty)
R = 0.5  # Measurement noise (accelerometer uncertainty)
K = [0.0, 0.0, 0.0]  # Kalman Gain

# Initialize previous time
previous_time = time.time()

# Initialize the buffer for the high-pass filter
window_size = 30  # Number of samples for moving average
accel_buffer = [[0.0, 0.0, 0.0]] * window_size  # Buffer to hold previous acceleration readings
buffer_index = 0  # Index for circular buffer

# Gyro orientation variables (for adjusting accelerometer data to world frame)
gyro_orientation = [0.0, 0.0, 0.0]  # Roll, pitch, yaw

# Gravity vector (m/s^2)
gravity = [0.0, 0.0, 9.81]

# Initialize Kalman filter variables
velocity = [0.0, 0.0, 0.0]  # Estimated velocity
P = [1.0, 1.0, 1.0]  # Error covariance (initial uncertainty for each axis)
Q = 0.1  # Process noise (acceleration uncertainty)
R = 0.5  # Measurement noise (accelerometer uncertainty)
K = [0.0, 0.0, 0.0]  # Kalman Gain

# Initialize previous time
previous_time = time.time()

# Initialize the buffer for the high-pass filter
window_size = 30  # Number of samples for moving average
accel_buffer = [[0.0, 0.0, 0.0]] * window_size  # Buffer to hold previous acceleration readings
buffer_index = 0  # Index for circular buffer

# Gyro orientation variables (for adjusting accelerometer data to world frame)
gyro_orientation = [0.0, 0.0, 0.0]  # Roll, pitch, yaw

# Gravity vector (m/s^2)
gravity = [0.0, 0.0, 9.81]

while True:
    # Get the current time and the time difference
    current_time = time.time()
    delta_time = current_time - previous_time
    previous_time = current_time

    # Get the current accelerometer and gyroscope readings
    accel = imu.accel()  # Accelerometer data (in m/s^2)
    gyro = imu.gyro()  # Gyroscope data (in rad/s)

    # Update the buffer with the new acceleration reading
    accel_buffer[buffer_index] = accel
    buffer_index = (buffer_index + 1) % window_size  # Circular buffer

    # Compute the moving average (low-pass) manually for the acceleration
    sum_accel = [0.0, 0.0, 0.0]
    for i in range(window_size):
        sum_accel[0] += accel_buffer[i][0]
        sum_accel[1] += accel_buffer[i][1]
        sum_accel[2] += accel_buffer[i][2]
    
    # Compute the mean acceleration (low-pass)
    mean_accel = [sum_accel[0] / window_size, sum_accel[1] / window_size, sum_accel[2] / window_size]

    # Apply high-pass filtering by subtracting the mean acceleration from current acceleration
    accel_filtered = [accel[0] - mean_accel[0], accel[1] - mean_accel[1], accel[2] - mean_accel[2]]

    # Update gyro orientation based on gyroscope readings
    gyro_orientation[0] += gyro[0] * delta_time  # Roll
    gyro_orientation[1] += gyro[1] * delta_time  # Pitch
    gyro_orientation[2] += gyro[2] * delta_time  # Yaw

    # Assuming gyro orientation is relatively small (integrated angular velocity), convert accel data to world frame
    roll = gyro_orientation[0]
    pitch = gyro_orientation[1]

    # Apply basic rotation correction to accelerometer data using roll and pitch
    cos_roll = math.cos(roll)
    sin_roll = math.sin(roll)
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)

    accel_world_frame = [
        accel_filtered[0] * cos_pitch + accel_filtered[2] * sin_pitch,  # x-axis adjusted
        accel_filtered[1] - accel_filtered[0] * sin_roll,  # y-axis adjusted by roll
        accel_filtered[2] * cos_roll - accel_filtered[0] * sin_pitch * cos_roll  # z-axis adjusted
    ]

    # Gravity compensation (assuming gravity is aligned with the z-axis)
    accel_no_gravity = [accel_world_frame[0], accel_world_frame[1], accel_world_frame[2] - gravity[2]]

    # Low-pass filter to smooth the accelerometer data (especially in z-axis)
    alpha = 0.1  # Smoothing factor (adjust based on your needs)
    accel_filtered_smooth = [
        alpha * accel_filtered[0] + (1 - alpha) * accel_buffer[buffer_index][0],
        alpha * accel_filtered[1] + (1 - alpha) * accel_buffer[buffer_index][1],
        alpha * accel_filtered[2] + (1 - alpha) * accel_buffer[buffer_index][2]
    ]

    # Kalman filter update for velocity (combined accelerometer and gyro data)
    for i in range(3):
        # Prediction step (we predict the velocity based on previous velocity)
        velocity_pred = velocity[i] + accel_no_gravity[i] * delta_time

        # Prediction error covariance
        P_pred = P[i] + Q

        # Compute the Kalman gain
        K[i] = P_pred / (P_pred + R)

        # Update step (adjust estimate based on the new measurement)
        velocity[i] = velocity_pred + K[i] * (accel_no_gravity[i] - velocity_pred)

        # Update the error covariance
        P[i] = (1 - K[i]) * P_pred

    # If the device is stationary (on the table), reset z-velocity to 0
    if abs(accel_filtered_smooth[2]) < 0.1:  # Check if the z-axis velocity is near zero
        velocity[2] = 0.0

    # Print the filtered velocity
    print('Velocity  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*velocity))

    # Add a small delay to avoid overwhelming the output
    time.sleep(0.1)
