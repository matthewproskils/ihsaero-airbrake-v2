import sdcard
import uos
from machine import SPI, Pin, I2C
import time
from bno055 import *
import math

# Setup SPI and CS pin for the SD card
spi = SPI(1, baudrate=4000000, polarity=0, phase=0)
cs = Pin(15, Pin.OUT)

if "/sd" not in uos.listdir("/"):
    import sdcard

    spi = machine.SPI(0,
                      sck=machine.Pin(2),
                      mosi=machine.Pin(3),
                      miso=machine.Pin(4))
    cs = machine.Pin(5, machine.Pin.OUT)
    sd = sdcard.SDCard(spi, cs)
    vfs = uos.VfsFat(sd)
    uos.mount(vfs, "/sd")
    print("SD card mounted.")
else:
    print("SD already mounted.")

# Initialize I2C for BNO055
i2c = I2C(0, scl=Pin(9), sda=Pin(8))
print("I2C scan:", i2c.scan())

# Create BNO055 object
imu = BNO055(i2c, 0x28)

# Set to NDOF (Sensor Fusion)
i2c.writeto_mem(0x28, 0x3D, b'\x0C')
time.sleep(0.1)

# Load calibration data if available
with open('calibration.bin', 'rb') as file:
    data = file.read()
imu.set_offsets(data)

# Initialize Kalman filter variables
velocity = [0.0, 0.0, 0.0]  # Estimated velocity
P = [1.0, 1.0, 1.0]  # Error covariance (initial uncertainty for each axis)
Q = 0.1  # Process noise (acceleration uncertainty)
R = 0.5  # Measurement noise (accelerometer uncertainty)
K = [0.0, 0.0, 0.0]  # Kalman Gain

# Complementary filter for position
position = [0.0, 0.0, 0.0]
alpha = 0.98  # Blending factor

# Initialize previous time for delta_time calculation
previous_time = time.ticks_ms()

# Initialize the buffer for the high-pass filter
window_size = 10  # Number of samples for moving average
accel_buffer = [[0.0, 0.0, 0.0]] * window_size  # Buffer to hold previous acceleration readings
buffer_index = 0  # Index for circular buffer

# Buffer to store data for batch writing to SD
data_buffer = []
write_interval = 10  # Write to SD every 10 samples

while True:
    current_time = time.ticks_ms()
    delta_time = (current_time - previous_time) / 1000.0  # Time difference in seconds
    previous_time = current_time

    # Get accelerometer, gyroscope, magnetometer, and other sensor data
    accel = imu.accel()
    gyro = imu.gyro()
    mag = imu.mag()
    euler = imu.euler()
    lin_acc = imu.lin_acc()
    gravity = imu.gravity()
    temperature = imu.temperature()

    # High-pass filter (removes low-frequency noise from accelerometer data)
    accel_buffer[buffer_index] = accel
    buffer_index = (buffer_index + 1) % window_size
    sum_accel = [0.0, 0.0, 0.0]
    for i in range(window_size):
        sum_accel[0] += accel_buffer[i][0]
        sum_accel[1] += accel_buffer[i][1]
        sum_accel[2] += accel_buffer[i][2]
    mean_accel = [sum_accel[0] / window_size, sum_accel[1] / window_size, sum_accel[2] / window_size]
    accel_filtered = [accel[0] - mean_accel[0], accel[1] - mean_accel[1], accel[2] - mean_accel[2]]

    # Kalman filter for velocity
    for i in range(3):
        velocity_pred = velocity[i] + accel_filtered[i] * delta_time
        P_pred = P[i] + Q
        K[i] = P_pred / (P_pred + R)
        velocity[i] = velocity_pred + K[i] * (accel_filtered[i] - velocity_pred)
        P[i] = (1 - K[i]) * P_pred

        # Complementary filter for position
        position[i] = alpha * (position[i] + velocity[i] * delta_time) + (1 - alpha) * velocity[i] * delta_time

    # Print the filtered velocity and position
    print('Velocity  x {:5.2f} y {:5.2f} z {:5.2f} | Position x {:5.2f} y {:5.2f} z {:5.2f}'.format(
        *velocity, *position))

    # Format the data line for writing
    log_line = (
        "{:.2f},".format(current_time) +
        "{:.3f},{:.3f},{:.3f},".format(*velocity) +
        "{:.3f},{:.3f},{:.3f},".format(*position) +
        "{:.3f},{:.3f},{:.3f},".format(*accel) +
        "{:.3f},{:.3f},{:.3f},".format(*gyro) +
        "{:.3f},{:.3f},{:.3f},".format(*mag) +
        "{:.3f},{:.3f},{:.3f},".format(*euler) +
        "{:.3f},{:.3f},{:.3f},".format(*lin_acc) +
        "{:.3f},{:.3f},{:.3f},".format(*gravity) +
        "{:d}\n".format(temperature)
    )
    data_buffer.append(log_line)

    # Write to SD every specified interval (e.g., 10 samples)
    if len(data_buffer) >= write_interval:
        with open("/sd/data.txt", "a") as f:
            for line in data_buffer:
                f.write(line)
        data_buffer = []  # Clear buffer after writing
