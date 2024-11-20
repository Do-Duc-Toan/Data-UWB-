import serial
import time
import csv
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter
import numpy as np

# Function to plot only the filtered position on the same figure
def plot_position(filtered_x, filtered_y):
    plt.clf()  # Clear the previous plot
    
    # Anchors at fixed points
    anchors = [(0, 0), (3, 0), (3, 3), (0, 3)]
    
    # Plot anchors
    for i, (anchor_x, anchor_y) in enumerate(anchors):
        plt.scatter(anchor_x, anchor_y, c='green', marker='s', label=f'Anchor {i+1}' if i == 0 else "")
        plt.text(anchor_x, anchor_y, f"A{i+1}", fontsize=12, color="green")
    
    # Add lines connecting the anchors to form a rectangle
    rect_x = [anchors[0][0], anchors[1][0], anchors[2][0], anchors[3][0], anchors[0][0]]
    rect_y = [anchors[0][1], anchors[1][1], anchors[2][1], anchors[3][1], anchors[0][1]]
    plt.plot(rect_x, rect_y, 'g--')  # Dashed green line connecting the anchors

    # Plot filtered position
    plt.scatter(filtered_x, filtered_y, c='blue', label='Filtered Position (Kalman)')
    plt.text(filtered_x, filtered_y, f"Filtered ({filtered_x:.2f}, {filtered_y:.2f})", fontsize=12, color="blue")

    # Set limits and labels
    plt.xlim(-0.5, 3.5)
    plt.ylim(-0.5, 3.5)
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Real-time Filtered Tag Position")
    plt.grid(True)
    plt.legend(loc='upper left')
    plt.pause(0.01)  # Pause to update the plot

# Initialize the Kalman filter for X and Y positions
def initialize_kalman_filter():
    kf = KalmanFilter(dim_x=2, dim_z=2)  # 2D position (x, y)
    kf.x = np.array([[0.], [0.]])  # Initial state estimate (x, y)
    kf.F = np.array([[1., 0.], [0., 1.]])  # State transition matrix
    kf.H = np.array([[1., 0.], [0., 1.]])  # Measurement function
    kf.P *= 1000.  # Covariance matrix
    kf.R = np.array([[5., 0.], [0., 5.]])  # Measurement noise (uncertainty)
    kf.Q = np.array([[0.1, 0.], [0., 0.1]])  # Process noise (uncertainty)
    return kf

# Initialize the Kalman filter
kf = initialize_kalman_filter()

# Open the serial port
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

# Open the CSV file to write data
file_name = "not_algo.csv"
with open(file_name, mode='w', encoding='UTF-8', newline='') as filecsv:
    csv_writer = csv.writer(filecsv)
    
    # Write a header row with a timestamp and the position columns
    csv_writer.writerow(["Time", "Anchor1", "Anchor2", "Anchor3", "Anchor4", "Filtered_X", "Filtered_Y"])

    # Initialize the plot
    plt.ion()  # Turn interactive mode on for real-time plotting
    plt.figure()  # Initialize the figure

    try:
        while True:
            if serialCom.in_waiting > 0:
                # Read the line from serial and decode it
                line = serialCom.readline().decode('utf-8').strip()
                print(line)  # Print for debug purposes
                
                # Split the data assuming comma-separated values
                data = line.split(',')
                
                if len(data) == 6:  # Ensure we have 6 values: 4 distances + x, y
                    try:
                        # Extract distances and x, y
                        d_A, d_B, d_C, d_D, x, y = [float(value) for value in data]

                        # Get the current time (hours, minutes, and seconds)
                        current_time = time.strftime('%H:%M:%S', time.localtime())

                        # Apply Kalman filter to smooth the x and y positions
                        z = np.array([[x], [y]])  # Current measurement
                        kf.predict()
                        kf.update(z)
                        filtered_x, filtered_y = kf.x[0, 0], kf.x[1, 0]

                        # Write the time, distances, and filtered positions to CSV
                        csv_writer.writerow([current_time, d_A, d_B, d_C, d_D, f"{filtered_x:.2f}", f"{filtered_y:.2f}"])

                        # Print out the filtered positions
                        print(f"Filtered Position: X={filtered_x:.2f}, Y={filtered_y:.2f}")

                        # Update the plot with the new filtered positions
                        plot_position(filtered_x, filtered_y)

                    except Exception as e:
                        print(f"Error in data processing: {e}")
                else:
                    print("Error: Line does not contain 6 values, skipping.")
        
    except KeyboardInterrupt:
        print("Serial reading interrupted.")

# Close the serial port when done
serialCom.close()
