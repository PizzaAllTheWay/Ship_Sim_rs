import os
import pandas as pd
import matplotlib.pyplot as plt

# === Locate latest IMU log ===
log_base = "log/data"
latest_log_dir = sorted([d for d in os.listdir(log_base) if os.path.isdir(os.path.join(log_base, d))])[-1]
imu_path = os.path.join(log_base, latest_log_dir, "imu.csv")

# === Load IMU data ===
df = pd.read_csv(
    imu_path,
    header=None,
    names=["timestamp", "ax", "ay", "az", "gx", "gy", "gz", "yaw"],
    sep=",|\s+",
    engine="python"
)

df["timestamp"] = pd.to_datetime(df["timestamp"])
df["time_sec"] = (df["timestamp"] - df["timestamp"].iloc[0]).dt.total_seconds()

# === Plotting ===
fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True,
                        gridspec_kw={"height_ratios": [2, 2, 1]})

# Acceleration
axs[0].plot(df["time_sec"], df["ax"], label="Ax")
axs[0].plot(df["time_sec"], df["ay"], label="Ay")
axs[0].plot(df["time_sec"], df["az"], label="Az")
axs[0].set_ylabel("Acceleration [m/s²]")
axs[0].set_title("IMU Acceleration")
axs[0].legend()
axs[0].grid(True)

# Angular velocity
axs[1].plot(df["time_sec"], df["gx"], label="Gx")
axs[1].plot(df["time_sec"], df["gy"], label="Gy")
axs[1].plot(df["time_sec"], df["gz"], label="Gz")
axs[1].set_ylabel("Gyro [rad/s]")
axs[1].set_title("IMU Angular Velocity")
axs[1].legend()
axs[1].grid(True)

# Yaw angle
axs[2].plot(df["time_sec"], df["yaw"], label="Yaw", color="purple")
axs[2].set_ylabel("Yaw [rad]")
axs[2].set_title("Yaw Angle")
axs[2].set_xlabel("Time [s]")
axs[2].grid(True)

plt.tight_layout()
plt.show()
