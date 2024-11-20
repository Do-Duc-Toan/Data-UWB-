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
    plt.clf()
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
    ax2.text(x, y, f"Target\n({x:.2f}, {y:.2f})", fontsize=12, color="red")

    # Set limits and labels
    ax2.xlim(-0.5, 3.5)
    ax2.ylim(-0.5, 3.5)
    ax2.xlabel("x (m)")
    ax2.ylabel("y (m)")
    ax2.grid(True)
    ax2.pause(0.01)  # Pause to update the plot

# Initialize the Kalman filter for X and Y positions
def initialize_kalman_filter_distance_position():
    kf = KalmanFilter(dim_x=2, dim_z=2)  # 2D position (x, y)
    kf.x = np.array([[0.], [0.]])  # Initial state estimate (x, y)
    kf.F = np.array([[1., 0.], [0., 1.]])  # State transition matrix
    kf.H = np.array([[1., 0.], [0., 1.]])  # Measurement function
    kf.P *= 1000.  # Covariance matrix
    kf.R = np.array([[5., 0.], [0., 5.]])  # Measurement noise (uncertainty)
    kf.Q = np.array([[0.1, 0.], [0., 0.1]])  # Process noise (uncertainty)
    return kf

# Initialize the Kalman filter
kf = initialize_kalman_filter_distance_position()

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
file_name = "plot_algo.csv"
filecsv = open(file_name, mode='w', encoding='UTF-8', newline='')
csv_writer = csv.writer(filecsv)
csv_writer.writerow(["Time", "Anchor1", "Anchor2", "Anchor3", "Anchor4", "Est_X", "Est_Y"])  # CSV header

# Set up real-time plotting with Matplotlib
plt.ion()  # Interactive mode on
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))  # Two subplots: ax1 for distances, ax2 for positions

# Initialize data lists for each anchor
times = []
anchor1_data = []
anchor2_data = []
anchor3_data = []
anchor4_data = []

# Plot placeholders for real-time data
line1, = ax2.plot([], [], label='Anchor 1')
line2, = ax2.plot([], [], label='Anchor 2')
line3, = ax2.plot([], [], label='Anchor 3')
line4, = ax2.plot([], [], label='Anchor 4')

# Setup the plot with labels
ax2.set_xlim(0, 20)  # Set the x-ax2is limit to show last 20 readings
ax2.set_ylim(0, 5)  # Adjust according to the expected range of values for anchors
ax2.set_xlabel('Time (seconds)')
ax2.set_ylabel('Distance (meters)')
ax2.set_title('Real-time UWB Distance Readings')
ax2.legend()

# Initialize Kalman Filters for each anchor
def initialize_kalman_filter_distance():
    kf = KalmanFilter(dim_x=1, dim_z=1)
    kf.x = np.array([[0.]])       # Initial estimate
    kf.F = np.array([[1.]])       # State transition matrix
    kf.H = np.array([[1.]])       # Measurement function
    kf.P *= 1000.                 # Covariance matrix
    kf.R = 5                      # Measurement uncertainty
    kf.Q = 0.1                    # Process uncertainty
    return kf

kf1 = initialize_kalman_filter_distance()
kf2 = initialize_kalman_filter_distance()
kf3 = initialize_kalman_filter_distance()
kf4 = initialize_kalman_filter_distance()

# Function to update the plot with new data
def update_plot():
    # Set the data for each line to the current lists of anchor readings
    line1.set_data(times, anchor1_data)
    line2.set_data(times, anchor2_data)
    line3.set_data(times, anchor3_data)
    line4.set_data(times, anchor4_data)
    
    # Dynamically adjust the x-ax2is to show recent data
    ax2.set_xlim(max(0, times[-1] - 20), times[-1])
    
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
                
                # Parse the anchor distances and filter them using Kalman filters
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
                kf.predict()
                kf.update(z)
                x_filtered, y_filtered = kf.x[0, 0], kf.x[1, 0]

                # Write the filtered data to CSV
                current_time_str = time.strftime('%H:%M:%S', time.localtime())
                csv_writer.writerow([current_time_str, anchor1_filtered, anchor2_filtered, anchor3_filtered, anchor4_filtered, f"{x_filtered:.2f}", f"{y_filtered:.2f}"])

                # Update the position plot
                plot_position(x_filtered, y_filtered, ax2)

                # Update the UWB distance plot
                update_plot()
                
                # Pause for a short time to allow real-time plotting
                plt.pause(0.005)
                
except KeyboardInterrupt:
    print("Serial reading interrupted.")

finally:
    # Close the CSV file and serial port
    filecsv.close()
    serialCom.close()
    plt.show()  # Keep the plot open after the loop ends
