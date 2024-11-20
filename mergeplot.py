import serial
import time
import csv
import sympy as sp
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter
import numpy as np

# Trilateration function
def trilateration(d_A, d_B, d_C, d_D):
    # Define the variables
    x, y, z = sp.symbols('x y z')

    # Define the equations for the distances to each anchor
    eq1 = x**2 + y**2 + z**2 - d_A**2
    eq2 = (x - 3)**2 + y**2 + z**2 - d_B**2
    eq3 = (x - 3)**2 + (y - 3)**2 + z**2 - d_C**2
    eq4 = x**2 + (y - 3)**2 + z**2 - d_D**2

    # Solve for x using eq1 and eq2
    eq_x = sp.simplify(eq2 - eq1)
    sol_x = sp.solve(eq_x, x)[0]

    # Solve for y using eq1 and eq4
    eq_y = sp.simplify(eq4 - eq1)
    sol_y = sp.solve(eq_y, y)[0]

    return float(sol_x), float(sol_y)

# Function to plot the anchors and tag position
def plot_position(x, y, ax2):
    ax2.clear()
    # Anchors at fixed points
    anchors = [(0, 0), (3, 0), (3, 3), (0, 3)]
    
    # Plot anchors
    for i, (anchor_x, anchor_y) in enumerate(anchors):
        ax2.scatter(anchor_x, anchor_y, c='green', marker='s', label=f'Anchor {i+1}' if i == 0 else "")
        ax2.text(anchor_x, anchor_y, f"A{i+1}", fontsize=12, color="green")

    # Add lines connecting the anchors to form a rectangle
    # Connect A1 -> A2 -> A3 -> A4 -> A1
    rect_x = [anchors[0][0], anchors[1][0], anchors[2][0], anchors[3][0], anchors[0][0]]
    rect_y = [anchors[0][1], anchors[1][1], anchors[2][1], anchors[3][1], anchors[0][1]]
    ax2.plot(rect_x, rect_y, 'g--')  # Dashed green line connecting the anchors
    
    # Plot the real-time tag position
    ax2.scatter(x, y, c='red', label='Tag')
    ax2.text(x, y, f"Tag\n({x:.2f}, {y:.2f})", fontsize=12, color="red")

    # Set limits and labels
    ax2.set_xlim(-0.5, 3.5)
    ax2.set_ylim(-0.5, 3.5)
    ax2.set_xlabel("x (m)")
    ax2.set_ylabel("y (m)")
    ax2.grid(True)

# Initialize Kalman filter for each anchor and position
def initialize_kalman_filter(dim_x=1, dim_z=1):
    kf = KalmanFilter(dim_x=dim_x, dim_z=dim_z)
    kf.x = np.zeros((dim_x, 1))  # Initial state estimate
    kf.F = np.eye(dim_x)  # State transition matrix
    kf.H = np.eye(dim_z)  # Measurement function
    kf.P *= 1000.        # Covariance matrix
    kf.R = np.eye(dim_z) * 5  # Measurement uncertainty
    kf.Q = np.eye(dim_x) * 0.1  # Process uncertainty
    return kf

# Initialize Kalman filters for each anchor and position
kf1 = initialize_kalman_filter()  # For anchor 1
kf2 = initialize_kalman_filter()  # For anchor 2
kf3 = initialize_kalman_filter()  # For anchor 3
kf4 = initialize_kalman_filter()  # For anchor 4

# Kalman filter for estimated position (x, y)
kf_position = initialize_kalman_filter(dim_x=2, dim_z=2)  # 2D Kalman filter for position

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

# Prepare CSV file for writing
file_name = "merge_plot.csv"
filecsv = open(file_name, mode='w', encoding='UTF-8', newline='')
csv_writer = csv.writer(filecsv)
csv_writer.writerow(["Time", "Anchor1", "Anchor2", "Anchor3", "Anchor4", "Est_X", "Est_Y"])  # CSV header

# Initialize data lists for distance readings
times = []
anchor1_data = []
anchor2_data = []
anchor3_data = []
anchor4_data = []

# Set up real-time plotting with Matplotlib
plt.ion()  # Interactive mode on
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))  # Two subplots: ax1 for distances, ax2 for positions

# Plot placeholders for real-time UWB distance data
line1, = ax1.plot([], [], label='Anchor 1')
line2, = ax1.plot([], [], label='Anchor 2')
line3, = ax1.plot([], [], label='Anchor 3')
line4, = ax1.plot([], [], label='Anchor 4')

# Setup the distance plot with labels
ax1.set_xlim(0, 20)  # Set the x-axis limit to show last 20 readings
ax1.set_ylim(0, 5)  # Adjust according to the expected range of values for anchors
ax1.set_xlabel('Time (seconds)')
ax1.set_ylabel('Distance (meters)')
ax1.set_title('Real-time UWB Distance Readings')
ax1.legend()

# Function to update the plot with new data
def update_plot():
    # Set the data for each line to the current lists of anchor readings
    line1.set_data(times, anchor1_data)
    line2.set_data(times, anchor2_data)
    line3.set_data(times, anchor3_data)
    line4.set_data(times, anchor4_data)
    
    # Dynamically adjust the x-axis to show recent data
    ax1.set_xlim(max(0, times[-1] - 20), times[-1])
    
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
            print(f"Raw data: {line}")  # Print raw data for debugging purposes
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

                # Apply Kalman filters to each anchor
                kf1.predict()
                kf1.update(anchor1)
                anchor1_filtered = kf1.x[0, 0]
                anchor1_data.append(anchor1_filtered)

                kf2.predict()
                kf2.update(anchor2)
                anchor2_filtered = kf2.x[0, 0]
                anchor2_data.append(anchor2_filtered)

                kf3.predict()
                kf3.update(anchor3)
                anchor3_filtered = kf3.x[0, 0]
                anchor3_data.append(anchor3_filtered)

                kf4.predict()
                kf4.update(anchor4)
                anchor4_filtered = kf4.x[0, 0]
                anchor4_data.append(anchor4_filtered)

                # Use trilateration to calculate x and y positions using filtered distances
                x, y = trilateration(anchor1_filtered, anchor2_filtered, anchor3_filtered, anchor4_filtered)

                # Apply Kalman filter to smooth the x and y positions
                z = np.array([[x], [y]])  # Current measurement
                kf_position.predict()
                kf_position.update(z)
                x_filtered, y_filtered = kf_position.x[0, 0], kf_position.x[1, 0]

                # Write the filtered data to CSV
                current_time_str = time.strftime('%H:%M:%S', time.localtime())
                csv_writer.writerow([current_time_str, 
                     f"{anchor1_filtered:.2f}", 
                     f"{anchor2_filtered:.2f}", 
                     f"{anchor3_filtered:.2f}", 
                     f"{anchor4_filtered:.2f}", 
                     f"{x_filtered:.2f}", 
                     f"{y_filtered:.2f}"])

                # Print filtered distances and positions to the terminal
#                print(f"Time: {current_time_str}")
#                print(f"Filtered Distances - Anchor1: {anchor1_filtered:.2f} m, Anchor2: {anchor2_filtered:.2f} m, Anchor3: {anchor3_filtered:.2f} m, Anchor4: {anchor4_filtered:.2f} m")
                print(f"Estimated Position - X: {x_filtered:.2f} m, Y: {y_filtered:.2f} m")
#                print("-" * 60)

                # Update the position plot
                plot_position(x_filtered, y_filtered, ax2)

                # Update the UWB distance plot
                update_plot()
                
                # # Pause for a short time to allow real-time plotting
                plt.pause(0.01)
            else:
                print("Received data does not have 4 values. Skipping this line.")
                
except KeyboardInterrupt:
    print("Serial reading interrupted.")

finally:
    # Close the CSV file and serial port
    filecsv.close()
    serialCom.close()
    plt.show()  # Keep the plot open after the loop ends
