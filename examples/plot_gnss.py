import re
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load and parse log file
with open("log_gnss.txt", "r") as f:
    lines = f.readlines()

# Extract coordinates
antenna_points = []
pattern = re.compile(r"Antenna\d: \[\[([-\d.]+), ([-\d.]+)\]\]")
for line in lines:
    match = pattern.search(line)
    if match:
        x, y = float(match.group(1)), float(match.group(2))
        antenna_points.append((x, y))

# Convert to numpy array
data = np.array(antenna_points)

# Define grid bins
bins = 100  # increase for more detail
heatmap, xedges, yedges = np.histogram2d(data[:, 0], data[:, 1], bins=bins)

# Make meshgrid for plotting
x_centers = 0.5 * (xedges[:-1] + xedges[1:])
y_centers = 0.5 * (yedges[:-1] + yedges[1:])
X, Y = np.meshgrid(x_centers, y_centers)
Z = heatmap.T  # transpose for correct orientation

# Plot 3D heatmap
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection="3d")
ax.plot_surface(X, Y, Z, cmap="viridis")
ax.set_xlim(-15, 15)  # optional
ax.set_ylim(-5, 5)    # zoom Y
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Density")
ax.set_title("GNSS Antenna Position Distribution")
plt.tight_layout()
plt.show()
