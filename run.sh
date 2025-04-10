#!/bin/bash

# List of binary names (without .rs extension)
BINS=("ship_sim" "sim_interface")

# Handle Ctrl+C (SIGINT)
trap 'echo "Stopping..."; kill 0; exit' INT

# Run each binary with cargo in background
for BIN in "${BINS[@]}"
do
    echo "Running $BIN..."
    cargo run --bin "$BIN" &
done

# Wait for all background jobs to finish (or be killed)
wait

echo "All binaries finished."
