#!/bin/bash

# WaveShare Robotaxi Startup Script
# This script starts the robotaxi system with proper initialization

set -e

# Configuration
PROJECT_DIR="/home/pi/WaveShare"
VENV_DIR="$PROJECT_DIR/robotaxi_env"
LOG_DIR="$PROJECT_DIR/logs"
MAIN_SCRIPT="$PROJECT_DIR/robotaxi_simple.py"

# Create log directory if it doesn't exist
mkdir -p "$LOG_DIR"

# Log file with timestamp
LOG_FILE="$LOG_DIR/robotaxi_$(date +%Y%m%d_%H%M%S).log"

# Function to log messages
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

# Start logging
log "=== WaveShare Robotaxi Starting ==="
log "Project directory: $PROJECT_DIR"
log "Virtual environment: $VENV_DIR"
log "Main script: $MAIN_SCRIPT"

# Check if virtual environment exists
if [ ! -d "$VENV_DIR" ]; then
    log "ERROR: Virtual environment not found at $VENV_DIR"
    log "Please run the setup script first"
    exit 1
fi

# Check if main script exists
if [ ! -f "$MAIN_SCRIPT" ]; then
    log "ERROR: Main script not found at $MAIN_SCRIPT"
    exit 1
fi

# Activate virtual environment
log "Activating virtual environment..."
source "$VENV_DIR/bin/activate"

# Set Python path for ugv_rpi
export PYTHONPATH="$PROJECT_DIR/ugv_rpi:$PYTHONPATH"
log "PYTHONPATH set to: $PYTHONPATH"

# Check hardware connection (optional)
log "Checking hardware connection..."
if python3 -c "import serial; serial.Serial('/dev/serial0', 115200, timeout=1).close()" 2>/dev/null; then
    log "Hardware connection available"
else
    log "WARNING: Hardware connection not available (this is OK if hardware not connected)"
fi

# Start the main robotaxi system
log "Starting robotaxi system..."
cd "$PROJECT_DIR"

# Run the main script with proper error handling
exec python3 "$MAIN_SCRIPT" 2>&1 | tee -a "$LOG_FILE" 