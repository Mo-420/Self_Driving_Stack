#!/bin/bash

# WaveShare Robotaxi Web App Startup Script

set -e

echo "=== Starting WaveShare Robotaxi Web App ==="

# Check if Node.js is installed
if ! command -v node &> /dev/null; then
    echo "Node.js is not installed. Please install Node.js first."
    exit 1
fi

# Check if npm is installed
if ! command -v npm &> /dev/null; then
    echo "npm is not installed. Please install npm first."
    exit 1
fi

# Install React dependencies
echo "Installing React dependencies..."
npm install

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install -r requirements.txt

# Start the Flask backend server in the background
echo "Starting Flask backend server..."
python3 server.py &
BACKEND_PID=$!

# Wait a moment for the backend to start
sleep 3

# Start the React development server
echo "Starting React development server..."
npm start &
FRONTEND_PID=$!

echo "=== Web App Started Successfully ==="
echo "Backend server (Flask): http://localhost:5000"
echo "Frontend app (React): http://localhost:3000"
echo ""
echo "Press Ctrl+C to stop both servers"

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Stopping servers..."
    kill $BACKEND_PID 2>/dev/null || true
    kill $FRONTEND_PID 2>/dev/null || true
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Wait for both processes
wait 