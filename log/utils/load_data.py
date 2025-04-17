import os
import pandas as pd

def load_data(file_name, state_names):
    log_base = "log/data"
    latest_log_dir = sorted([d for d in os.listdir(log_base) if os.path.isdir(os.path.join(log_base, d))])[-1]
    file_path = os.path.join(log_base, latest_log_dir, file_name)

    # Read full line as string to parse first part manually
    raw_lines = []
    with open(file_path, "r") as f:
        for line in f:
            parts = line.strip().split(",", 1)  # Split once at the first comma
            if len(parts) != 2:
                continue
            timestamp = parts[0]
            rest = parts[1]
            raw_lines.append([timestamp] + rest.split())

    # Convert to DataFrame
    data = pd.DataFrame(raw_lines)

    # Parse first column from date and time to total seconds
    date_and_time = pd.to_datetime(data[0], format="%Y-%m-%d %H:%M:%S.%f", errors="coerce");
    date_and_time_diff = date_and_time - date_and_time[0]
    seconds = date_and_time_diff.dt.total_seconds()
    data[0] = seconds

    # Rename each column to appropriate state name
    data.columns = state_names

    # Typecast each column to floats
    data = data.astype(float).fillna(0.0)

    return data