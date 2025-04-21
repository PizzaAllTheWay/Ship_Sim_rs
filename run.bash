#!/bin/bash

# Handle Ctrl+C (SIGINT)
trap 'echo "Stopping..."; kill 0; exit' INT

# Optional: delay config per binary [in seconds]
DELAY_ship_sim=0
DELAY_external_forces_sim=0
DELAY_sensors_sim=0
DELAY_sim_interface=0
DELAY_state_estimators=5


# Check if logging is enabled
LOG_ENABLED=false
for arg in "$@"; do
    if [[ "$arg" == "log=true" ]]; then
        LOG_ENABLED=true
    fi
done

# Conditionally run logger
if $LOG_ENABLED; then
    echo "Running logger..."
    cargo run --bin "logger" &
    sleep 5
fi

# List of binaries to launch (excluding logger)
BINS=("ship_sim" "external_forces_sim" "sensors_sim" "sim_interface" "state_estimators")

# Run each with delay
for BIN in "${BINS[@]}"; do
    DELAY_VAR="DELAY_${BIN}"
    DELAY=${!DELAY_VAR:-0} # Default to 0 if undefined
    echo "Waiting ${DELAY}s before starting $BIN..."
    sleep "$DELAY"
    echo "Running $BIN..."
    cargo run --bin "$BIN" &
done

# Wait for all background jobs to finish
wait
echo "All binaries finished."
