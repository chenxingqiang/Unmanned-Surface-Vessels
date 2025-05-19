#!/bin/bash
# Script to run the USV visualization server with obstacle avoidance enabled

# Set the base directory to the script location
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
cd "$SCRIPT_DIR"

# Default values
PORT=9000
ENABLE_BROWSER=false
DEBUG_MODE=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        -p|--port)
            PORT="$2"
            shift
            shift
            ;;
        -b|--browser)
            ENABLE_BROWSER=true
            shift
            ;;
        -d|--debug)
            DEBUG_MODE=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  -p, --port PORT      Specify port (default: 9000)"
            echo "  -b, --browser        Open browser automatically"
            echo "  -d, --debug          Enable debug mode"
            echo "  -h, --help           Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for available options"
            exit 1
            ;;
    esac
done

# Check if Python is available
if ! command -v python3 &> /dev/null; then
    if command -v python &> /dev/null; then
        PYTHON_CMD="python"
    else
        echo "Error: Python is not installed or not in PATH"
        exit 1
    fi
else
    PYTHON_CMD="python3"
fi

echo "Starting USV Visualization Server with Obstacle Avoidance..."
echo "Using Python: $($PYTHON_CMD --version)"
echo "Port: $PORT"

# Construct command
CMD="$PYTHON_CMD run_server_with_obstacles.py --port $PORT --obstacles --scenario obstacle"

# Add optional flags
if $ENABLE_BROWSER; then
    CMD="$CMD --browser"
fi

if $DEBUG_MODE; then
    CMD="$CMD --debug"
fi

# Execute the command
echo "Executing: $CMD"
eval "$CMD" 