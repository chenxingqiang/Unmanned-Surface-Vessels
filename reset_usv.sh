#!/bin/bash
# Script to reset the USV simulation environment by stopping any running processes

echo "Resetting USV simulation environment..."

# Kill processes on common USV visualization ports
for port in 8080 8081 8082
do
    # Check if any process is using this port
    if command -v lsof &> /dev/null; then
        # macOS and some Linux distributions
        pid=$(lsof -i TCP:$port -t)
        if [ ! -z "$pid" ]; then
            echo "Killing process on port $port (PID: $pid)"
            kill -9 $pid 2>/dev/null
        fi
    elif command -v fuser &> /dev/null; then
        # Linux
        fuser -k ${port}/tcp 2>/dev/null
        if [ $? -eq 0 ]; then
            echo "Killed process on port $port"
        fi
    fi
done

# Kill any Python processes containing run_server in their command line
echo "Stopping USV server processes..."
pkill -f "python.*run_server" 2>/dev/null
pkill -f "run_usv_visualization.py" 2>/dev/null

# Wait a moment for processes to terminate
sleep 1

# Check if processes were actually terminated
if pgrep -f "python.*run_server" > /dev/null || pgrep -f "run_usv_visualization.py" > /dev/null; then
    echo "Warning: Some USV processes could not be terminated automatically."
    echo "You may need to manually kill these processes."
    
    # List the processes that are still running
    ps aux | grep -E "python.*run_server|run_usv_visualization.py" | grep -v grep
else
    echo "All USV processes have been successfully terminated."
fi

echo "USV environment reset complete."