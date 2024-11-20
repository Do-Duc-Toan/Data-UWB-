import pandas as pd
import matplotlib.pyplot as plt
import time

# Load CSV file
csv_file = 'csv_algo.csv'  # Replace with your actual CSV file path

# Initialize plot
plt.ion()  # Turn on interactive mode for live plotting
fig, ax = plt.subplots(figsize=(15, 10))

# Plot green frame (rectangle) connecting the four anchors
anchors = {
    'A1': (0, 0),
    'A2': (2.5, 0),
    'A3': (2.5, 2.5),
    'A4': (0, 2.5)
}

# Plot the anchor positions
for anchor, (x, y) in anchors.items():
    ax.plot(x, y, 'go')  # Green circle for anchors
    ax.text(x, y, anchor, fontsize=12, ha='right')

# Connect the anchors with green dashed lines
frame_x = [anchors['A1'][0], anchors['A2'][0], anchors['A3'][0], anchors['A4'][0], anchors['A1'][0]]
frame_y = [anchors['A1'][1], anchors['A2'][1], anchors['A3'][1], anchors['A4'][1], anchors['A1'][1]]
ax.plot(frame_x, frame_y, 'g--')

# Add blue diagonal line from A1 to A3
ax.plot([anchors['A1'][0], anchors['A3'][0]], [anchors['A1'][1], anchors['A3'][1]], 'b-', label="Diagonal A1 to A3")

# Set aspect ratio to be equal, so the rectangle looks proportional
ax.set_aspect('equal')

# Set limits to match the display
ax.set_xlim(left=-0.5, right=3)
ax.set_ylim(bottom=-0.5, top=3)
ax.set_xlabel('x (cm)')
ax.set_ylabel('y (cm)')
ax.set_title('Real-time Position Estimation')

# Continuously read the CSV file and plot the data point by point
with open(csv_file) as file:
    reader = pd.read_csv(file)
    
    for i, row in enumerate(reader.iterrows()):
        if i != 0:  # Only plot every 3rd row
            # Get the estimated x and y coordinates
            est_x = row[1]['Est_X']
            est_y = row[1]['Est_Y']
            
            # Plot each point as it's read
            ax.plot(est_x, est_y, 'ro')  # Red circle for estimated points
            plt.draw()
            plt.pause(0.1)  # Pause for half a second to simulate real-time plotting

plt.ioff()  # Turn off interactive mode
plt.show()
