# bno055_test.py Simple test program for MicroPython bno055 driver

# Copyright (c) Peter Hinch 2019
# Released under the MIT licence.

from machine import Pin ,I2C
import time
from bno055 import *
import time
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

"""
while True:
    sys, gyro, accel, mag = imu.cal_status()
    print('sys: {}, gyro: {}, accel: {}, mag: {}'.format(sys, gyro, accel, mag))
    if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
        print("Calibration complete!")
        with open("calibration.bin", "wb") as file:
            file.write(imu.sensor_offsets())
        break
    time.sleep(1)
"""

with open('calibration.bin', 'rb') as file:
    # Read the previous calibration data
    data = file.read()
    
imu.set_offsets(data)
# Initialize variables for filtering
window_size = 30  # Number of samples for moving average
accel_buffer = [[0.0, 0.0, 0.0]] * window_size  # Buffer to hold previous acceleration readings
buffer_index = 0  # Index for circular buffer

# Initialize velocity
velocity = [0.0, 0.0, 0.0]
previous_time = time.time()

# Modify the loop to apply high-pass filtering
while True:
    # Get the current time and the time difference
    current_time = time.time()
    delta_time = current_time - previous_time
    previous_time = current_time

    # Get the current acceleration (in m/s^2)
    accel = imu.accel()

    # Update the buffer with the new acceleration reading
    accel_buffer[buffer_index] = accel
    buffer_index = (buffer_index + 1) % window_size  # Circular buffer

    # Compute the moving average (low-pass) manually
    sum_accel = [0.0, 0.0, 0.0]
    for i in range(window_size):
        sum_accel[0] += accel_buffer[i][0]
        sum_accel[1] += accel_buffer[i][1]
        sum_accel[2] += accel_buffer[i][2]
    
    # Compute the mean acceleration (low-pass)
    mean_accel = [sum_accel[0] / window_size, sum_accel[1] / window_size, sum_accel[2] / window_size]

    # Apply high-pass filtering by subtracting the mean acceleration from current acceleration
    accel_filtered = [accel[0] - mean_accel[0], accel[1] - mean_accel[1], accel[2] - mean_accel[2]]

    # Integrate the filtered acceleration to compute velocity
    velocity[0] += accel_filtered[0] * delta_time
    velocity[1] += accel_filtered[1] * delta_time
    velocity[2] += accel_filtered[2] * delta_time

    # Print the filtered acceleration and velocity
    print('Accel     x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*accel))
    print('Filtered  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*accel_filtered))
    print('Velocity  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*velocity))
    
    """
    print('Temperature {}°C'.format(imu.temperature()))
    print('Mag       x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.mag()))
    print('Gyro      x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.gyro()))
    print('Accel     x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.accel()))
    print('Lin acc.  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.lin_acc()))
    print('Gravity   x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.gravity()))
    print('Heading     {:4.0f} roll {:4.0f} pitch {:4.0f}'.format(*imu.euler()))
    """

