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
    eq2 = (x - 2.5)**2 + y**2 + z**2 - d_B**2
    eq3 = (x - 2.5)**2 + (y - 2.5)**2 + z**2 - d_C**2
    eq4 = x**2 + (y - 2.5)**2 + z**2 - d_D**2

    # Solve for x using eq1 and eq2
    eq_x = sp.simplify(eq2 - eq1)
    sol_x = sp.solve(eq_x, x)[0]

    # Solve for y using eq1 and eq4
    eq_y = sp.simplify(eq4 - eq1)
    sol_y = sp.solve(eq_y, y)[0]

    return float(sol_x), float(sol_y)

# Function to plot the anchors and tag position
def plot_position(x, y):
    plt.clf()
    # Anchors at fixed points
    anchors = [(0, 0), (2.5, 0), (2.5, 2.5), (0, 2.5)]
    
    # Plot anchors
    for i, (anchor_x, anchor_y) in enumerate(anchors):
        plt.scatter(anchor_x, anchor_y, c='green', marker='s', label=f'Anchor {i+1}' if i == 0 else "")
        plt.text(anchor_x, anchor_y, f"A{i+1}", fontsize=12, color="green")
    
    # Add lines connecting the anchors to form a rectangle
    # Connect A1 -> A2 -> A3 -> A4 -> A1
    rect_x = [anchors[0][0], anchors[1][0], anchors[2][0], anchors[3][0], anchors[0][0]]
    rect_y = [anchors[0][1], anchors[1][1], anchors[2][1], anchors[3][1], anchors[0][1]]
    plt.plot(rect_x, rect_y, 'g--')  # Dashed green line connecting the anchors
    
    # Set aspect ratio to be equal, so the rectangle looks proportional
    plt.gca().set_aspect('equal')

    # Plot the real-time tag position
    plt.scatter(x, y, c='red', label='Tag')
    plt.text(x, y, f"Target\n({x:.2f}, {y:.2f})", fontsize=12, color="red")

    # Set limits and labels
    plt.xlim(-0.5, 3)
    plt.ylim(-0.5, 3)
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.grid(True)
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
file_name = "plot_algo.csv"
with open(file_name, mode='w', encoding='UTF-8', newline='') as filecsv:
    csv_writer = csv.writer(filecsv)
    
    # Write a header row with a timestamp and the position columns
    csv_writer.writerow(["Time", "Anchor1", "Anchor2", "Anchor3", "Anchor4", "Est_X", "Est_Y"])

    # Initialize the plot
    plt.ion()
    plt.figure()

    try:
        while True:
            if serialCom.in_waiting > 0:
                # Read the line from serial and decode it
                line = serialCom.readline().decode('utf-8').strip()
                print(line)  # Print for debug purposes
                
                # Split the data assuming comma-separated values
                data = line.split(',')
                
                if len(data) == 4:  # Ensure we have four distance values
                    try:
                        # Convert the distances to float values
                        d_A, d_B, d_C, d_D = [float(dist) for dist in data]

                        # Get the current time (hours, minutes, and seconds)
                        current_time = time.strftime('%H:%M:%S', time.localtime())

                        # Use the trilateration method to compute the position
                        x, y = trilateration(d_A, d_B, d_C, d_D)

                        # Apply Kalman filter to smooth the x and y positions
                        z = np.array([[x], [y]])  # Current measurement
                        kf.predict()
                        kf.update(z)
                        x_filtered, y_filtered = kf.x[0, 0], kf.x[1, 0]

                        # Write the time, distances, and estimated position to CSV
                        csv_writer.writerow([current_time, d_A, d_B, d_C, d_D, f"{x_filtered:.2f}", f"{y_filtered:.2f}"])

                        # Print out the estimated position
                        print(f"Estimated Position: X={x_filtered:.2f}, Y={y_filtered:.2f}")

                        # Update the plot with the new position
                        plot_position(x_filtered, y_filtered)

                    except Exception as e:
                        print(f"Error in trilateration calculation: {e}")
                else:
                    print("Error: Line does not contain 4 values, skipping.")
        
    except KeyboardInterrupt:
        print("Serial reading interrupted.")

# Close the serial port when done
serialCom.close()
