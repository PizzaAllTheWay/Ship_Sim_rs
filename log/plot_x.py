import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# === Locate latest X log ===
log_base = "log/data"
latest_log_dir = sorted([d for d in os.listdir(log_base) if os.path.isdir(os.path.join(log_base, d))])[-1]
imu_path = os.path.join(log_base, latest_log_dir, "x.csv")

# === Load x state data ===
x_path = os.path.join(log_base, latest_log_dir, "x.csv")
df_x = pd.read_csv(
    x_path,
    header=None,
    names=[
        "timestamp",
        "vx", "vy", "vz",
        "wx", "wy", "wz",
        "px", "py", "pz",
        "roll", "pitch", "yaw"
    ],
    sep=",|\s+",
    engine="python"
)
df_x["timestamp"] = pd.to_datetime(df_x["timestamp"])
df_x["time_sec"] = (df_x["timestamp"] - df_x["timestamp"].iloc[0]).dt.total_seconds()

# === Plot Window 5: Full State ===
fig4, axs4 = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

# Linear velocity
axs4[0].plot(df_x["time_sec"], df_x["vx"], label="Vx")
axs4[0].plot(df_x["time_sec"], df_x["vy"], label="Vy")
axs4[0].plot(df_x["time_sec"], df_x["vz"], label="Vz")
axs4[0].set_title("Linear Velocity")
axs4[0].legend()
axs4[0].grid(True)

# Angular velocity
axs4[1].plot(df_x["time_sec"], df_x["wx"], label="ωx")
axs4[1].plot(df_x["time_sec"], df_x["wy"], label="ωy")
axs4[1].plot(df_x["time_sec"], df_x["wz"], label="ωz")
axs4[1].set_title("Angular Velocity")
axs4[1].legend()
axs4[1].grid(True)

# Position
axs4[2].plot(df_x["time_sec"], df_x["px"], label="Px")
axs4[2].plot(df_x["time_sec"], df_x["py"], label="Py")
axs4[2].plot(df_x["time_sec"], df_x["pz"], label="Pz")
axs4[2].set_title("Position")
axs4[2].legend()
axs4[2].grid(True)

# Orientation (Euler angles)
axs4[3].plot(df_x["time_sec"], df_x["roll"], label="Roll")
axs4[3].plot(df_x["time_sec"], df_x["pitch"], label="Pitch")
axs4[3].plot(df_x["time_sec"], df_x["yaw"], label="Yaw")
axs4[3].set_title("Orientation (Euler Angles)")
axs4[3].set_xlabel("Time [s]")
axs4[3].legend()
axs4[3].grid(True)

plt.tight_layout()
plt.show()
