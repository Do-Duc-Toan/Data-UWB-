import serial
import time
import csv
import numpy as np
import matplotlib.pyplot as plt

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
file_name = "nokalman.csv"
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

                # Append raw anchor data to lists
                anchor1_data.append(anchor1)
                anchor2_data.append(anchor2)
                anchor3_data.append(anchor3)
                anchor4_data.append(anchor4)
                
                # Write the raw data to CSV
                current_time_str = time.strftime('%H:%M:%S', time.localtime())
                csv_writer.writerow([current_time_str, anchor1, anchor2, anchor3, anchor4])
                
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