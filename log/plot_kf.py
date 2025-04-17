# Libraries for data manipulation
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import gaussian_kde
import toml

# Custom Libraries
from utils.load_data import load_data

kf_data = load_data(
    file_name="kf.csv",
    state_names=[
        "time",
        "vx", "vy", "vz",
        "wx", "wy", "wz",
        "px", "py", "pz",
        "roll", "pitch", "yaw"
    ],
)

fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

axs[0].plot(kf_data["time"], kf_data["vx"], label="Vx")
axs[0].plot(kf_data["time"], kf_data["vy"], label="Vy")
axs[0].plot(kf_data["time"], kf_data["vz"], label="Vz")
axs[0].set_title("KF: Linear Velocity")
axs[0].legend(); axs[0].grid(True)

axs[1].plot(kf_data["time"], kf_data["px"], label="Px")
axs[1].plot(kf_data["time"], kf_data["py"], label="Py")
axs[1].plot(kf_data["time"], kf_data["pz"], label="Pz")
axs[1].set_title("KF: Position")
axs[1].legend(); axs[1].grid(True)

axs[2].plot(kf_data["time"], kf_data["roll"], label="Roll")
axs[2].plot(kf_data["time"], kf_data["pitch"], label="Pitch")
axs[2].plot(kf_data["time"], kf_data["yaw"], label="Yaw")
axs[2].set_title("KF: Angles")
axs[2].set_xlabel("Time [s]")
axs[2].legend(); axs[2].grid(True)

plt.tight_layout()
plt.show()
