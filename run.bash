#!/bin/bash

# Handle Ctrl+C (SIGINT)
trap 'echo "Stopping..."; kill 0; exit' INT

# Check if logging is enabled
LOG_ENABLED=false
for arg in "$@"; do
    if [[ "$arg" == "log=true" ]]; then
        LOG_ENABLED=true
    fi
done

# List of binary names (conditionally include logger)
BINS=("ship_sim" "external_force_sim" "sensor_sim" "sim_interface")
if $LOG_ENABLED; then
    echo "Running logger..."
    cargo run --bin "logger" &

    echo "Waiting for logger to initialize..."
    sleep 5
fi

# Run each binary with cargo in background
for BIN in "${BINS[@]}"; do
    echo "Running $BIN..."
    cargo run --bin "$BIN" &
done

# Kill all background rust binaries if they are still running
wait
echo "All binaries finished."
