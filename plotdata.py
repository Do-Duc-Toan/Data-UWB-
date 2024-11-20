import serial
import time
import csv
import numpy as np
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter

# Open the serial port (replace with the correct port and baud rate)
try:
    serialCom = serial.Serial('/dev/ttyUSB0', 115200)
    print(f"Connected to: {serialCom.port}")
except serial.SerialException as e:
    print(f"Error connecting to serial port: {e}")
    exit()

# Reset the ESP32
serialCom.setRTS(True)  # Set RTS to True to reset
time.sleep(0.1)         # Small delay
serialCom.setRTS(False) # Set RTS to False to release the reset
serialCom.flushInput()  # Flush input

# Prepare CSV file for writing
file_name = "plotdata.csv"
filecsv = open(file_name, mode='w', encoding='UTF-8', newline='')
csv_writer = csv.writer(filecsv)
csv_writer.writerow(["Time", "Anchor1", "Anchor2", "Anchor3", "Anchor4"])  # CSV header

# Set up real-time plotting with Matplotlib
fig, ax = plt.subplots()

# Initialize data lists for each anchor
times = []
anchor1_data = []
anchor2_data = []
anchor3_data = []
anchor4_data = []

# Plot placeholders for real-time data
line1, = ax.plot([], [], label='Anchor 1')
line2, = ax.plot([], [], label='Anchor 2')
line3, = ax.plot([], [], label='Anchor 3')
line4, = ax.plot([], [], label='Anchor 4')

# Setup the plot with labels
ax.set_xlim(0, 20)  # Set the x-axis limit to show last 20 readings
ax.set_ylim(0, 5)  # Adjust according to the expected range of values for anchors
ax.set_xlabel('Time (seconds)')
ax.set_ylabel('Distance (meters)')
ax.set_title('Real-time UWB Distance Readings')
ax.legend()

# Initialize Kalman Filters for each anchor
def initialize_kalman_filter():
    kf = KalmanFilter(dim_x=1, dim_z=1)
    kf.x = np.array([[0.]])       # Initial estimate
    kf.F = np.array([[1.]])       # State transition matrix
    kf.H = np.array([[1.]])       # Measurement function
    kf.P *= 1000.                 # Covariance matrix
    kf.R = 5                      # Measurement uncertainty
    kf.Q = 0.1                    # Process uncertainty
    return kf

kf1 = initialize_kalman_filter()
kf2 = initialize_kalman_filter()
kf3 = initialize_kalman_filter()
kf4 = initialize_kalman_filter()

# Function to update the plot with new data
def update_plot():
    # Set the data for each line to the current lists of anchor readings
    line1.set_data(times, anchor1_data)
    line2.set_data(times, anchor2_data)
    line3.set_data(times, anchor3_data)
    line4.set_data(times, anchor4_data)
    
    # Dynamically adjust the x-axis to show recent data
    ax.set_xlim(max(0, times[-1] - 20), times[-1])
    
    # Redraw the plot
    fig.canvas.draw()
    fig.canvas.flush_events()

# Real-time data collection and plotting
try:
    start_time = time.time()  # Record the starting time

    while True:
        if serialCom.in_waiting > 0:
            # Read a line from serial, decode it, and split by commas
            line = serialCom.readline().decode('utf-8').strip()
            print(line)  # Debugging purposes
            data = line.split(',')

            # Ensure correct number of values (4 anchors)
            if len(data) == 4:
                # Get the current time (since start) for x-axis
                current_time = time.time() - start_time
                times.append(current_time)
                
                # Parse the anchor distances
                anchor1 = float(data[0])
                anchor2 = float(data[1])
                anchor3 = float(data[2])
                anchor4 = float(data[3])

                # Apply Kalman filter to each anchor data
                kf1.predict()
                kf1.update(anchor1)
                anchor1_filtered = kf1.x[0]
                anchor1_data.append(anchor1_filtered)

                kf2.predict()
                kf2.update(anchor2)
                anchor2_filtered = kf2.x[0]
                anchor2_data.append(anchor2_filtered)

                kf3.predict()
                kf3.update(anchor3)
                anchor3_filtered = kf3.x[0]
                anchor3_data.append(anchor3_filtered)

                kf4.predict()
                kf4.update(anchor4)
                anchor4_filtered = kf4.x[0]
                anchor4_data.append(anchor4_filtered)
                
                # Write the filtered data to CSV
                current_time_str = time.strftime('%H:%M:%S', time.localtime())
                csv_writer.writerow([current_time_str, anchor1_filtered, anchor2_filtered, anchor3_filtered, anchor4_filtered])
                
                # Update the plot
                update_plot()
                
                # Pause for a short time to allow real-time plotting
                plt.pause(0.01)
                
except KeyboardInterrupt:
    print("Serial reading interrupted.")

finally:
    # Close the CSV file and serial port
    filecsv.close()
    serialCom.close()
    plt.show()  # Keep the plot open after the loop ends
