
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

    # Kalman filter update for velocity
    for i in range(3):
        # Prediction step (we predict the velocity based on previous velocity)
        velocity_pred = velocity[i] + accel_filtered[i] * delta_time

        # Prediction error covariance
        P_pred = P[i] + Q

        # Compute the Kalman gain
        K[i] = P_pred / (P_pred + R)

        # Update step (adjust estimate based on the new measurement)
        velocity[i] = velocity_pred + K[i] * (accel_filtered[i] - velocity_pred)

        # Update the error covariance
        P[i] = (1 - K[i]) * P_pred

    # Print the filtered velocity
    print('Velocity  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*velocity))

    # Add a small delay to avoid overwhelming the output
    time.sleep(0.1)