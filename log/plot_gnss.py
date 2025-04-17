# Libraries for data manipulation
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import gaussian_kde
import toml

# Custom Libraries
from utils.load_data import load_data



# === Load config ===
config_path = os.path.join("config.toml")
config = toml.load(config_path)



# === Load latest logged data ===
gnss_data = load_data(
    file_name="gnss.csv",
    state_names=[
        "timestamp",
        "x1", "y1", "z1", # Antenna 1
        "x2", "y2", "z2", # Antenna 2
        "vx", "vy", "vz", # Velocity
    ],
)

x_data = load_data(
    file_name="x.csv",
    state_names=[
        "timestamp",
        "vx", "vy", "vz",
        "wx", "wy", "wz",
        "px", "py", "pz",
        "roll", "pitch", "yaw"
    ],
)



# === Ground Truth Values from latest x.csv ===
# General data get
antenna1_offset = config["sensor"]["gnss_antenna1_placement"]
antenna2_offset = config["sensor"]["gnss_antenna2_placement"]
com_x = x_data["px"].iloc[-1]
com_y = x_data["py"].iloc[-1]
com_z = x_data["pz"].iloc[-1]

# Find XY truths
truth_ant1_x = com_x + antenna1_offset[0]
truth_ant1_y = com_y + antenna1_offset[1]
truth_ant2_x = com_x + antenna2_offset[0]
truth_ant2_y = com_y + antenna2_offset[1]

# Find Z truths
truth_ant1_z = com_z + antenna1_offset[2]
truth_ant2_z = com_z + antenna2_offset[2]

# Find velocity truths
truth_vx = x_data["vx"].iloc[-1]
truth_vy = x_data["vy"].iloc[-1]
truth_vz = x_data["vz"].iloc[-1]



# === Plot GNSS XY Position plane  ===
# Split into antenna data for reuse
ant1 = gnss_data[["x1", "y1"]].rename(columns={"x1": "x", "y1": "y"})
ant2 = gnss_data[["x2", "y2"]].rename(columns={"x2": "x", "y2": "y"})

# Axis limits
x_lim_1 = [8.0, 12.0]
y_lim_1 = [-2.0, 2.0]
x_lim_2 = [-12.0, -8.0]
y_lim_2 = [-2.0, 2.0]

def plot_kde_surface(ax, data, title, cmap, x_lim, y_lim):
    x = data["x"].to_numpy()
    y = data["y"].to_numpy()
    xy = np.vstack([x, y])
    kde = gaussian_kde(xy)

    x_grid, y_grid = np.mgrid[x_lim[0]:x_lim[1]:100j, y_lim[0]:y_lim[1]:100j]
    pos = np.vstack([x_grid.ravel(), y_grid.ravel()])
    z = kde(pos).reshape(x_grid.shape)

    ax.plot_surface(x_grid, y_grid, z, cmap=cmap, linewidth=0, antialiased=True)
    ax.set_title(title)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Density")
    ax.set_xlim(x_lim)
    ax.set_ylim(y_lim)

def plot_kde_heatmap(ax, data, title, cmap, x_lim, y_lim):
    x = data["x"].to_numpy()
    y = data["y"].to_numpy()
    xy = np.vstack([x, y])
    kde = gaussian_kde(xy)

    x_grid, y_grid = np.mgrid[x_lim[0]:x_lim[1]:300j, y_lim[0]:y_lim[1]:300j]
    pos = np.vstack([x_grid.ravel(), y_grid.ravel()])
    z = kde(pos).reshape(x_grid.shape)

    ax.contourf(x_grid, y_grid, z, levels=50, cmap=cmap)
    ax.set_title(title)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_xlim(x_lim)
    ax.set_ylim(y_lim)

# Plot all 4 graphs
fig = plt.figure(figsize=(14, 10))

ax1 = fig.add_subplot(221, projection="3d")
plot_kde_surface(ax1, ant1, "GNSS Antenna 1 - 3D KDE", cmap="viridis", x_lim=x_lim_1, y_lim=y_lim_1)

ax2 = fig.add_subplot(222, projection="3d")
plot_kde_surface(ax2, ant2, "GNSS Antenna 2 - 3D KDE", cmap="plasma", x_lim=x_lim_2, y_lim=y_lim_2)

ax3 = fig.add_subplot(223)
plot_kde_heatmap(ax3, ant1, "GNSS Antenna 1 - 2D KDE", cmap="viridis", x_lim=x_lim_1, y_lim=y_lim_1)
ax3.plot(truth_ant1_x, truth_ant1_y, 'ro', label="Antenna 1 Truth", markersize=8)
ax3.legend()

ax4 = fig.add_subplot(224)
plot_kde_heatmap(ax4, ant2, "GNSS Antenna 2 - 2D KDE", cmap="plasma", x_lim=x_lim_2, y_lim=y_lim_2)
ax4.plot(truth_ant2_x, truth_ant2_y, 'ro', label="Antenna 2 Truth", markersize=8)
ax4.legend()

plt.tight_layout()
plt.show()



# === Antenna Z Distributions ===
# Extend loader to handle Zs first
z1 = gnss_data["z1"].dropna().to_numpy()
z2 = gnss_data["z2"].dropna().to_numpy()
kde_z1 = gaussian_kde(z1)
kde_z2 = gaussian_kde(z2)

z1_grid = np.linspace(z1.min() - 0.5, z1.max() + 0.5, 300)
z2_grid = np.linspace(z2.min() - 0.5, z2.max() + 0.5, 300)

plt.figure(figsize=(10, 4))
plt.plot(z1_grid, kde_z1(z1_grid), label="Z1 (Antenna 1)", color="green")
plt.plot(z2_grid, kde_z2(z2_grid), label="Z2 (Antenna 2)", color="red")

# Ground truth markers
plt.axvline(truth_ant1_z, color="green", linestyle="--", label="Truth Z1")
plt.axvline(truth_ant2_z, color="red", linestyle="--", label="Truth Z2")

plt.title("GNSS Antenna Altitude Distribution")
plt.xlabel("Z [m]")
plt.ylabel("Density")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()



# === Vx, Vy, Vz Distribution Combined ===
# Compute KDEs
vx = gnss_data["vx"].dropna().to_numpy()
vy = gnss_data["vy"].dropna().to_numpy()
vz = gnss_data["vz"].dropna().to_numpy()
kde_vx = gaussian_kde(vx)
kde_vy = gaussian_kde(vy)
kde_vz = gaussian_kde(vz)

# Define grid
vx_grid = np.linspace(vx.min() - 0.5, vx.max() + 0.5, 300)
vy_grid = np.linspace(vy.min() - 0.5, vy.max() + 0.5, 300)
vz_grid = np.linspace(vz.min() - 0.5, vz.max() + 0.5, 300)

# Plot KDEs
plt.figure(figsize=(10, 4))
plt.plot(vx_grid, kde_vx(vx_grid), label="Vx", color="blue")
plt.plot(vy_grid, kde_vy(vy_grid), label="Vy", color="orange")
plt.plot(vz_grid, kde_vz(vz_grid), label="Vz", color="purple")

# Ground truth lines
plt.axvline(truth_vx, color="blue", linestyle="--", label="Truth Vx")
plt.axvline(truth_vy, color="orange", linestyle="--", label="Truth Vy")
plt.axvline(truth_vz, color="purple", linestyle="--", label="Truth Vz")

plt.title("GNSS Velocity Distribution")
plt.xlabel("Velocity [m/s]")
plt.ylabel("Density")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()