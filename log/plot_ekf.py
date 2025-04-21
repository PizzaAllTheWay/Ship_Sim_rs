import os
import pandas as pd
import matplotlib.pyplot as plt

# === Paths ===
log_base = "log/data"
latest_log_dir = sorted([d for d in os.listdir(log_base) if os.path.isdir(os.path.join(log_base, d))])[-1]
ekf_path = os.path.join(log_base, latest_log_dir, "ekf.csv")
truth_path = os.path.join(log_base, latest_log_dir, "x.csv")

# === Load EKF data ===
df_ekf = pd.read_csv(
    ekf_path,
    header=None,
    names=["timestamp", "vx", "vy", "vz", "wx", "wy", "wz", "px", "py", "pz", "roll", "pitch", "yaw"],
    sep=",|\s+",
    engine="python"
)
df_ekf["timestamp"] = pd.to_datetime(df_ekf["timestamp"])
df_ekf["time_sec"] = (df_ekf["timestamp"] - df_ekf["timestamp"].iloc[0]).dt.total_seconds()

# === Load Ground Truth ===
df_truth = pd.read_csv(
    truth_path,
    header=None,
    names=["timestamp", "vx", "vy", "vz", "wx", "wy", "wz", "px", "py", "pz", "roll", "pitch", "yaw"],
    sep=",|\s+",
    engine="python"
)
df_truth["timestamp"] = pd.to_datetime(df_truth["timestamp"])
df_truth["time_sec"] = (df_truth["timestamp"] - df_truth["timestamp"].iloc[0]).dt.total_seconds()

# === Fixed color scheme ===
axis_colors = {
    "x": "red",
    "y": "blue",
    "z": "green"
}

# === Plotting Function ===
def plot_compare(ax, axis: str, label: str, ylabel: str):
    color = axis_colors[axis]
    # EKF (dashed, slightly transparent)
    ax.plot(df_ekf["time_sec"], df_ekf[label], label=f"EKF {label.upper()}", linestyle="-", alpha=0.3, color=color)
    # GT (solid, on top)
    ax.plot(df_truth["time_sec"], df_truth[label], label=f"GT {label.upper()}", linestyle="-", alpha=1.0, color=color)
    ax.set_ylabel(ylabel)
    ax.grid(True)
    ax.legend(fontsize=8)

# === 2x2 Layout ===
fig, axs = plt.subplots(2, 2, figsize=(12, 8), sharex=True)

# Top-left: Linear Velocity
plot_compare(axs[0, 0], "x", "vx", "Lin Vel [m/s]")
plot_compare(axs[0, 0], "y", "vy", "")
plot_compare(axs[0, 0], "z", "vz", "")
axs[0, 0].set_title("Linear Velocity")

# Top-right: Angular Velocity
plot_compare(axs[0, 1], "x", "wx", "Ang Vel [rad/s]")
plot_compare(axs[0, 1], "y", "wy", "")
plot_compare(axs[0, 1], "z", "wz", "")
axs[0, 1].set_title("Angular Velocity")

# Bottom-left: Position
plot_compare(axs[1, 0], "x", "px", "Pos [m]")
plot_compare(axs[1, 0], "y", "py", "")
plot_compare(axs[1, 0], "z", "pz", "")
axs[1, 0].set_title("Position")

# Bottom-right: Orientation
plot_compare(axs[1, 1], "x", "roll", "Euler [rad]")
plot_compare(axs[1, 1], "y", "pitch", "")
plot_compare(axs[1, 1], "z", "yaw", "")
axs[1, 1].set_title("Orientation")

# Final layout
for ax in axs[1]:
    ax.set_xlabel("Time [s]")

plt.suptitle("EKF vs Ground Truth", fontsize=14)
plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.show()