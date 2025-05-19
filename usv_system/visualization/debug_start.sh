#!/bin/bash

# Simplified script to test the simulation start functionality
echo "Starting Debug Environment for USV Visualization"

# Kill existing processes
echo "Stopping any existing Python processes..."
kill $(ps aux | grep "python.*visualization" | grep -v grep | awk '{print $2}') 2>/dev/null

# Wait for processes to terminate
sleep 2

# Remove old log files
echo "Removing old log files..."
rm -f debug.log server.log

# Start the debug app in the foreground for better error visibility
echo "Starting debug server on port 9000..."
echo "Press Ctrl+C to stop the server"
python debug_app.py --port 9000 