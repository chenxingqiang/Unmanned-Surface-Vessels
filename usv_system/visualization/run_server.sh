#!/bin/bash
#
# Script to run the USV Visualization Server
#

# Define default port
DEFAULT_PORT=5540

# Function to display help
show_help() {
    echo "USV Visualization Server Runner"
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  -p, --port PORT      Specify the port number (default: $DEFAULT_PORT)"
    echo "  -d, --debug          Run in debug mode"
    echo "  -h, --help           Display this help message and exit"
    echo ""
    echo "Example:"
    echo "  $0 --port 8080       Run the server on port 8080"
    echo ""
}

# Parse command line arguments
DEBUG_MODE=false
PORT=$DEFAULT_PORT

while [[ $# -gt 0 ]]; do
    case $1 in
        -p|--port)
            PORT="$2"
            shift 2
            ;;
        -d|--debug)
            DEBUG_MODE=true
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Validate port number
if ! [[ "$PORT" =~ ^[0-9]+$ ]]; then
    echo "Error: Port must be a number"
    exit 1
fi

# Set environment variables
export USV_VISUALIZATION_PORT=$PORT

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Check if Python is available
if ! command -v python &> /dev/null; then
    if command -v python3 &> /dev/null; then
        PYTHON_CMD="python3"
    else
        echo "Error: Python not found. Please install Python 3."
        exit 1
    fi
else
    PYTHON_CMD="python"
fi

# Print startup message
echo "Starting USV Visualization Server..."
echo "Using Python: $($PYTHON_CMD --version)"
echo "Server port: $PORT"
if [ "$DEBUG_MODE" = true ]; then
    echo "Debug mode: enabled"
else
    echo "Debug mode: disabled"
fi
echo ""

# Run the server
cd "$SCRIPT_DIR"
if [ "$DEBUG_MODE" = true ]; then
    $PYTHON_CMD app.py
else
    # Run in background and capture output
    $PYTHON_CMD app.py > server.log 2>&1 &
    SERVER_PID=$!
    echo "Server started with PID: $SERVER_PID"
    echo "Log file: $SCRIPT_DIR/server.log"
    
    # Wait briefly to check if server started successfully
    sleep 2
    if ! ps -p $SERVER_PID > /dev/null; then
        echo "Error: Server failed to start. Check log file for details."
        exit 1
    fi
    
    # Check if port.txt exists to get actual port
    if [ -f "$SCRIPT_DIR/port.txt" ]; then
        ACTUAL_PORT=$(cat "$SCRIPT_DIR/port.txt")
        if [ "$ACTUAL_PORT" != "$PORT" ]; then
            echo "Note: Server is using port $ACTUAL_PORT instead of $PORT"
            echo "Visit: http://localhost:$ACTUAL_PORT"
        else
            echo "Visit: http://localhost:$PORT"
        fi
    else
        echo "Visit: http://localhost:$PORT"
    fi
fi 