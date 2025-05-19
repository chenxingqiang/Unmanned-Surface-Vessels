#!/bin/bash

# Script to kill existing USV visualization processes and start the debug server

echo "Stopping any existing USV visualization processes..."

# Find PIDs of Python processes that contain "run_server" or "app.py"
pids=$(ps aux | grep python | grep -E 'run_server|app.py' | grep -v grep | awk '{print $2}')

if [ -z "$pids" ]; then
  echo "No existing USV visualization processes found."
else
  # Kill each process
  for pid in $pids; do
    echo "Killing process $pid"
    kill -9 $pid 2>/dev/null
  done
  echo "Waiting for processes to terminate..."
  sleep 2
fi

# Check if port 9000 is already in use
if nc -z localhost 9000 2>/dev/null; then
  echo "Port 9000 is already in use. Attempting to kill the process..."
  pid=$(lsof -i:9000 -t)
  if [ ! -z "$pid" ]; then
    kill -9 $pid
    sleep 1
  fi
fi

# Start debug server
echo "Starting debug server on port 9000..."
python debug_app.py --port 9000 &

# Wait for server to start
echo "Waiting for server to start..."
sleep 3

# Open browser
echo "Opening browser..."
open http://localhost:9000

echo "Server should now be running. Check the debug.log file for diagnostic information."
echo "Press Ctrl+C to stop the script"

# Keep script running to show log output
tail -f debug.log 