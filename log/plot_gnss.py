import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import gaussian_kde
from mpl_toolkits.mplot3d import Axes3D

# === Load latest GNSS logs ===
log_base = "log/data"
subdirs = [d for d in os.listdir(log_base) if os.path.isdir(os.path.join(log_base, d))]
latest_log_dir = sorted(subdirs)[-1]
full_path = os.path.join(log_base, latest_log_dir)

def load_gnss(filepath):
    df = pd.read_csv(filepath, header=None, names=["timestamp", "x", "y"], sep=",|\s+", engine="python")
    return df[["x", "y"]].dropna()

ant1 = load_gnss(os.path.join(full_path, "gnss_antenna1.csv"))
ant2 = load_gnss(os.path.join(full_path, "gnss_antenna2.csv"))

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

# === Plot all 4 graphs ===
fig = plt.figure(figsize=(14, 10))

ax1 = fig.add_subplot(221, projection="3d")
plot_kde_surface(ax1, ant1, "GNSS Antenna 1 - 3D KDE", cmap="viridis", x_lim=x_lim_1, y_lim=y_lim_1)

ax2 = fig.add_subplot(222, projection="3d")
plot_kde_surface(ax2, ant2, "GNSS Antenna 2 - 3D KDE", cmap="plasma", x_lim=x_lim_2, y_lim=y_lim_2)

ax3 = fig.add_subplot(223)
plot_kde_heatmap(ax3, ant1, "GNSS Antenna 1 - 2D KDE", cmap="viridis", x_lim=x_lim_1, y_lim=y_lim_1)

ax4 = fig.add_subplot(224)
plot_kde_heatmap(ax4, ant2, "GNSS Antenna 2 - 2D KDE", cmap="plasma", x_lim=x_lim_2, y_lim=y_lim_2)

plt.tight_layout()
plt.show()
