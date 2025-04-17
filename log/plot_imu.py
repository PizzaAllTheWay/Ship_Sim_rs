# Libraries for data manipulation
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import gaussian_kde
import toml

# Custom Libraries
from utils.load_data import load_data



# === Load config ===
config_path = os.path.join("config.toml")
config = toml.load(config_path)



# === Load latest logged data ===
imu_data = load_data(
    file_name="imu.csv",
    state_names=[
        "time",
        "ax", "ay", "az", # Accelerometer
        "gx", "gy", "gz", # Gyroscope
        "yaw",            # Magnetometer
    ],
)

x_data = load_data(
    file_name="x.csv",
    state_names=[
        "time",
        "vx", "vy", "vz",
        "wx", "wy", "wz",
        "px", "py", "pz",
        "roll", "pitch", "yaw"
    ],
)

dx_data = load_data(
    file_name="dx.csv",
    state_names=[
        "time",
        "ax", "ay", "az",
        "alphax", "alphay", "alphaz",
        "vx", "vy", "vz",
        "wx", "wy", "wz",
    ],
)



# === Ground Truth Values from dx and x ===
# Linear acceleration truth (body frame)
truth_ax = dx_data["ax"].iloc[-1]
truth_ay = dx_data["ay"].iloc[-1]
truth_az = dx_data["az"].iloc[-1] + 9.81 # Add extra because accelerometer is tuned for with "g" in mind

# Angular velocity truth (body frame)
truth_wx = x_data["wx"].iloc[-1]
truth_wy = x_data["wy"].iloc[-1]
truth_wz = x_data["wz"].iloc[-1]

# Yaw angle truth (from orientation)
truth_yaw = x_data["yaw"].iloc[-1]



# === Integration ===
dt = np.gradient(imu_data["time"].to_numpy())
vx = np.cumsum(imu_data["ax"].to_numpy() * dt)
vy = np.cumsum(imu_data["ay"].to_numpy() * dt)
vz = np.cumsum(imu_data["az"].to_numpy() * dt)
px = np.cumsum(vx * dt)
py = np.cumsum(vy * dt)
pz = np.cumsum(vz * dt)
gx_angle = np.cumsum(imu_data["gx"].to_numpy() * dt)
gy_angle = np.cumsum(imu_data["gy"].to_numpy() * dt)
gz_angle = np.cumsum(imu_data["gz"].to_numpy() * dt)



# === Plot Window 1: Smoothed 1D Gaussian KDEs with Ground Truths ===
fig0, axs0 = plt.subplots(4, 1, figsize=(10, 10))

# Acceleration KDEs (Ax & Ay)
for axis, color, label, truth in zip(
    ["ax", "ay"], ["blue", "orange"], ["Ax", "Ay"], [truth_ax, truth_ay]
):
    data = imu_data[axis].dropna()
    kde = gaussian_kde(data)
    x_grid = np.linspace(data.min(), data.max(), 300)
    axs0[0].plot(x_grid, kde(x_grid), label=label, color=color)
    axs0[0].axvline(truth, color=color, linestyle="--", label=f"{label} Truth")

axs0[0].set_title("Acceleration KDE (Ax & Ay)")
axs0[0].set_ylabel("Density")
axs0[0].legend()
axs0[0].grid(True)

# Acceleration KDE (Az)
az_data = imu_data["az"].dropna()
kde_az = gaussian_kde(az_data)
x_grid_az = np.linspace(az_data.min(), az_data.max(), 300)
axs0[1].plot(x_grid_az, kde_az(x_grid_az), label="Az", color="green")
axs0[1].axvline(truth_az, color="green", linestyle="--", label="Az Truth")

axs0[1].set_title("Acceleration KDE (Az)")
axs0[1].set_ylabel("Density")
axs0[1].legend()
axs0[1].grid(True)

# Gyro KDEs
for axis, color, label, truth in zip(
    ["gx", "gy", "gz"], ["red", "brown", "gray"], ["Gx", "Gy", "Gz"], [truth_wx, truth_wy, truth_wz]
):
    data = imu_data[axis].dropna()
    kde = gaussian_kde(data)
    x_grid = np.linspace(data.min(), data.max(), 300)
    axs0[2].plot(x_grid, kde(x_grid), label=label, color=color)
    axs0[2].axvline(truth, color=color, linestyle="--", label=f"{label} Truth")

axs0[2].set_title("Gyro KDE")
axs0[2].set_ylabel("Density")
axs0[2].legend()
axs0[2].grid(True)

# Yaw KDE
yaw_data = imu_data["yaw"].dropna()
kde_yaw = gaussian_kde(yaw_data)
x_grid_yaw = np.linspace(yaw_data.min(), yaw_data.max(), 300)
axs0[3].plot(x_grid_yaw, kde_yaw(x_grid_yaw), label="Yaw", color="purple")
axs0[3].axvline(truth_yaw, color="purple", linestyle="--", label="Yaw Truth")

axs0[3].set_title("Yaw KDE")
axs0[3].set_xlabel("Sensor Value")
axs0[3].set_ylabel("Density")
axs0[3].legend()
axs0[3].grid(True)

plt.tight_layout()
plt.show()



# === Plot Window 2: Accel + Velocity + Position ===
fig1, axs1 = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
axs1[0].plot(imu_data["time"], imu_data["ax"], label="Ax")
axs1[0].plot(imu_data["time"], imu_data["ay"], label="Ay")
axs1[0].plot(imu_data["time"], imu_data["az"], label="Az")
axs1[0].set_title("Linear Acceleration")
axs1[0].legend()
axs1[0].grid(True)

axs1[1].plot(imu_data["time"], vx, label="Vx")
axs1[1].plot(imu_data["time"], vy, label="Vy")
axs1[1].plot(imu_data["time"], vz, label="Vz")
axs1[1].set_title("Linear Velocity")
axs1[1].legend()
axs1[1].grid(True)

axs1[2].plot(imu_data["time"], px, label="Px")
axs1[2].plot(imu_data["time"], py, label="Py")
axs1[2].plot(imu_data["time"], pz, label="Pz")
axs1[2].set_title("Position Estimate")
axs1[2].set_xlabel("Time [s]")
axs1[2].legend()
axs1[2].grid(True)

plt.tight_layout()
plt.show()



# === Plot Window 3: Gyro + Integrated Angles ===
fig2, axs2 = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
axs2[0].plot(imu_data["time"], imu_data["gx"], label="Gx")
axs2[0].plot(imu_data["time"], imu_data["gy"], label="Gy")
axs2[0].plot(imu_data["time"], imu_data["gz"], label="Gz")
axs2[0].set_title("Angular Velocity")
axs2[0].legend()
axs2[0].grid(True)

axs2[1].plot(imu_data["time"], gx_angle, label="Roll (∫Gx)")
axs2[1].plot(imu_data["time"], gy_angle, label="Pitch (∫Gy)")
axs2[1].plot(imu_data["time"], gz_angle, label="Yaw (∫Gz)")
axs2[1].set_title("Integrated Angles")
axs2[1].set_xlabel("Time [s]")
axs2[1].legend()
axs2[1].grid(True)

plt.tight_layout()
plt.show()



# === Plot Window 4: Yaw only ===
fig3, ax3 = plt.subplots(figsize=(10, 3))
ax3.plot(imu_data["time"], imu_data["yaw"], label="Yaw", color="purple")
ax3.set_title("Yaw Angle (Magnetometer)")
ax3.set_xlabel("Time [s]")
ax3.set_ylabel("Yaw [rad]")
ax3.grid(True)
ax3.legend()
plt.tight_layout()
plt.show()
