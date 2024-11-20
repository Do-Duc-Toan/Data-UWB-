import matplotlib.pyplot as plt

# Initialize plot with a larger figure size
fig, ax = plt.subplots(figsize=(12, 8))  # Increase figure size for larger plot

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


# Plot the fixed point at (2.5, 2.3)
center_x = 2.5
center_y = 2.3
ax.plot(center_x, center_y, 'bo', markersize=10, label='Fixed Center Point')
ax.text(center_x, center_y, f"({center_x:.2f}, {center_y:.2f})", fontsize=12, color='blue')

# Set aspect ratio to be equal, so the rectangle looks proportional
ax.set_aspect('equal')

# Set limits to match the display
ax.set_xlim(xmin=-0.5, xmax=3)
ax.set_ylim(bottom=-0.5, top=3)
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_title('Frame with Fixed Center Point')

plt.legend()
plt.show()
