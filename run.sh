#!/bin/bash

# List of binary names (without .rs extension)
BINS=("ship_sim" "external_force_sim" "sim_interface")

# Handle Ctrl+C (SIGINT)
trap 'echo "Stopping..."; kill 0; exit' INT

# Run each binary with cargo in background
for BIN in "${BINS[@]}"
do
    echo "Running $BIN..."
    cargo run --bin "$BIN" &
done

# Forcefully kill all the rust binaries
pkill -f target/debug/

# Wait for any other processes to be done
wait

echo "All binaries finished."
