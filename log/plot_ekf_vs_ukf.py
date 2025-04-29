import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

UKF_LAG_SECONDS = 7.5
EKF_LAG_SECONDS = 7.5

# === Paths ===
log_base = "log/data"
latest_log_dir = sorted([d for d in os.listdir(log_base) if os.path.isdir(os.path.join(log_base, d))])[-1]
truth_path = os.path.join(log_base, latest_log_dir, "x.csv")
ukf_path = os.path.join(log_base, latest_log_dir, "ukf.csv")
ekf_path = os.path.join(log_base, latest_log_dir, "ekf.csv")

# === Load CSVs ===
def load_df(path, lag=0.0):
    df = pd.read_csv(
        path,
        header=None,
        names=["timestamp", "vx", "vy", "vz", "wx", "wy", "wz", "px", "py", "pz", "roll", "pitch", "yaw"],
        sep=",|\s+",
        engine="python"
    )
    df["timestamp"] = pd.to_datetime(df["timestamp"])
    df["time_sec"] = (df["timestamp"] - df["timestamp"].iloc[0]).dt.total_seconds() + lag
    return df

df_truth = load_df(truth_path)
df_ukf = load_df(ukf_path, UKF_LAG_SECONDS)
df_ekf = load_df(ekf_path, EKF_LAG_SECONDS)

# === Interpolate to common timestamps ===
common_time = np.intersect1d(df_truth["time_sec"], df_ukf["time_sec"])
common_time = np.intersect1d(common_time, df_ekf["time_sec"])

def interp(df, time_base):
    return df.set_index("time_sec").reindex(time_base).interpolate().reset_index()

df_truth = interp(df_truth, common_time)
df_ukf = interp(df_ukf, common_time)
df_ekf = interp(df_ekf, common_time)

# === Absolute error ===
def abs_err(df_est, df_gt, labels):
    return {label: np.abs(df_est[label] - df_gt[label]) for label in labels}

vel_labels = ["vx", "vy", "vz"]
pos_labels = ["px", "py", "pz"]
yaw_labels = ["wz", "yaw"]

err_ukf = abs_err(df_ukf, df_truth, vel_labels + pos_labels + yaw_labels)
err_ekf = abs_err(df_ekf, df_truth, vel_labels + pos_labels + yaw_labels)

# === Plotting helper ===
def plot_row_errors(fig_title, labels, yunits):
    fig, axs = plt.subplots(len(labels), 1, figsize=(10, 3.5 * len(labels)), sharex=True)
    if len(labels) == 1:
        axs = [axs]
    for i, label in enumerate(labels):
        axs[i].plot(common_time, err_ukf[label], label="UKF", alpha=0.6)
        axs[i].plot(common_time, err_ekf[label], label="EKF", alpha=0.6)
        axs[i].set_ylabel(f"{label.upper()} Error [{yunits[i]}]")
        axs[i].legend(fontsize=8)
        axs[i].grid(True)
    axs[-1].set_xlabel("Time [s]")
    fig.suptitle(fig_title, fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()

# === Plotting ===
plot_row_errors("Velocity Absolute Error", vel_labels, ["m/s"] * 3)
plot_row_errors("Position Absolute Error", pos_labels, ["m"] * 3)
plot_row_errors("Yaw Velocity & Angle Absolute Error", ["wz", "yaw"], ["rad/s", "rad"])
