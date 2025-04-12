import re
import matplotlib.pyplot as plt

# Load raw data from log file
with open("log_external_forces.txt", "r") as file:
    lines = file.readlines()

# Prepare data
wind_speed = []
wind_angle = []
current_speed = []
current_angle = []

for line in lines:
    wind_match = re.search(r"Wind:\s*\(([^,]+),\s*([^)]+)\)", line)
    current_match = re.search(r"Current:\s*\(([^,]+),\s*([^)]+)\)", line)

    if wind_match:
        wind_speed.append(float(wind_match.group(1)))
        wind_angle.append(float(wind_match.group(2)))

    if current_match:
        current_speed.append(float(current_match.group(1)))
        current_angle.append(float(current_match.group(2)))

# Plotting
fig, axs = plt.subplots(2, 2, figsize=(12, 8))
axs[0, 0].plot(wind_speed, label="Wind Speed")
axs[0, 0].set_title("Wind Speed")
axs[0, 0].grid()

axs[0, 1].plot(wind_angle, label="Wind Angle", color="orange")
axs[0, 1].set_title("Wind Angle")
axs[0, 1].grid()

axs[1, 0].plot(current_speed, label="Current Speed", color="green")
axs[1, 0].set_title("Current Speed")
axs[1, 0].grid()

axs[1, 1].plot(current_angle, label="Current Angle", color="red")
axs[1, 1].set_title("Current Angle")
axs[1, 1].grid()

plt.tight_layout()
plt.show()
