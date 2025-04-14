import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

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
dt = np.gradient(df["time_sec"])

# === Integration ===
vx, vy, vz = np.cumsum(df["ax"] * dt), np.cumsum(df["ay"] * dt), np.cumsum(df["az"] * dt)
px, py, pz = np.cumsum(vx * dt), np.cumsum(vy * dt), np.cumsum(vz * dt)
gx_angle = np.cumsum(df["gx"] * dt)
gy_angle = np.cumsum(df["gy"] * dt)
gz_angle = np.cumsum(df["gz"] * dt)

# === Plot Window 1: Density Distributions Only ===
fig0, axs0 = plt.subplots(3, 1, figsize=(10, 8))
scale = 1e3

axs0[0].hist(df["ax"], bins=10, density=True, alpha=0.5, label="Ax")
axs0[0].hist(df["ay"], bins=10, density=True, alpha=0.5, label="Ay")
#axs0[0].hist(df["az"] * scale, bins=10, density=True, alpha=0.5, label="Az")
axs0[0].set_title("Acceleration Density")
axs0[0].set_ylabel("Probability Density")
axs0[0].legend()
axs0[0].grid(True)

axs0[1].hist(df["gx"], bins=10, density=True, alpha=0.5, label="Gx")
axs0[1].hist(df["gy"], bins=10, density=True, alpha=0.5, label="Gy")
axs0[1].hist(df["gz"], bins=10, density=True, alpha=0.5, label="Gz")
axs0[1].set_title("Gyro Density")
axs0[1].set_ylabel("Probability Density")
axs0[1].legend()
axs0[1].grid(True)

axs0[2].hist(df["yaw"], bins=10, density=True, alpha=0.7, label="Yaw", color="purple")
axs0[2].set_title("Yaw Density")
axs0[2].set_xlabel("Sensor Value")
axs0[2].set_ylabel("Probability Density")
axs0[2].legend()
axs0[2].grid(True)

plt.tight_layout()
plt.show()

# === Plot Window 2: Accel + Velocity + Position ===
fig1, axs1 = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
axs1[0].plot(df["time_sec"], df["ax"], label="Ax")
axs1[0].plot(df["time_sec"], df["ay"], label="Ay")
axs1[0].plot(df["time_sec"], df["az"], label="Az")
axs1[0].set_title("Linear Acceleration")
axs1[0].legend()
axs1[0].grid(True)

axs1[1].plot(df["time_sec"], vx, label="Vx")
axs1[1].plot(df["time_sec"], vy, label="Vy")
axs1[1].plot(df["time_sec"], vz, label="Vz")
axs1[1].set_title("Linear Velocity")
axs1[1].legend()
axs1[1].grid(True)

axs1[2].plot(df["time_sec"], px, label="Px")
axs1[2].plot(df["time_sec"], py, label="Py")
axs1[2].plot(df["time_sec"], pz, label="Pz")
axs1[2].set_title("Position Estimate")
axs1[2].set_xlabel("Time [s]")
axs1[2].legend()
axs1[2].grid(True)

plt.tight_layout()
plt.show()

# === Plot Window 3: Gyro + Integrated Angles ===
fig2, axs2 = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
axs2[0].plot(df["time_sec"], df["gx"], label="Gx")
axs2[0].plot(df["time_sec"], df["gy"], label="Gy")
axs2[0].plot(df["time_sec"], df["gz"], label="Gz")
axs2[0].set_title("Angular Velocity")
axs2[0].legend()
axs2[0].grid(True)

axs2[1].plot(df["time_sec"], gx_angle, label="Roll (∫Gx)")
axs2[1].plot(df["time_sec"], gy_angle, label="Pitch (∫Gy)")
axs2[1].plot(df["time_sec"], gz_angle, label="Yaw (∫Gz)")
axs2[1].set_title("Integrated Angles")
axs2[1].set_xlabel("Time [s]")
axs2[1].legend()
axs2[1].grid(True)

plt.tight_layout()
plt.show()

# === Plot Window 4: Yaw only ===
fig3, ax3 = plt.subplots(figsize=(10, 3))
ax3.plot(df["time_sec"], df["yaw"], label="Yaw", color="purple")
ax3.set_title("Yaw Angle (Magnetometer)")
ax3.set_xlabel("Time [s]")
ax3.set_ylabel("Yaw [rad]")
ax3.grid(True)
ax3.legend()
plt.tight_layout()
plt.show()
