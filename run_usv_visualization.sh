#!/bin/bash
#
# Script to run the USV Visualization Server from the root directory
#

# Define default values
DEFAULT_PORT=8080
DEBUG_MODE=false
ENABLE_OBSTACLES=true
OPEN_BROWSER=true

# Set the path to the visualization directory
VIZ_DIR="usv_system/visualization"

# Function to display help
show_help() {
    echo "USV Visualization Server Runner"
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  -p, --port PORT      Specify the port number (default: $DEFAULT_PORT)"
    echo "  -d, --debug          Run in debug mode"
    echo "  -o, --no-obstacles   Disable obstacle avoidance"
    echo "  -b, --no-browser     Don't open browser automatically"
    echo "  -h, --help           Display this help message and exit"
    echo ""
    echo "Example:"
    echo "  $0 --port 9000       Run the server on port 9000"
    echo ""
}

# Parse command line arguments
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
        -o|--no-obstacles)
            ENABLE_OBSTACLES=false
            shift
            ;;
        -b|--no-browser)
            OPEN_BROWSER=false
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

# Check if Python is available
if ! command -v python3 &> /dev/null; then
    if command -v python &> /dev/null; then
        PYTHON_CMD="python"
    else
        echo "Error: Python not found. Please install Python 3."
        exit 1
    fi
else
    PYTHON_CMD="python3"
fi

# Print startup message
echo "Starting USV Visualization Server..."
echo "Using Python: $($PYTHON_CMD --version)"
echo "Server port: $PORT"
echo "Debug mode: $([ "$DEBUG_MODE" = true ] && echo "enabled" || echo "disabled")"
echo "Obstacles: $([ "$ENABLE_OBSTACLES" = true ] && echo "enabled" || echo "disabled")"
echo "Opening browser: $([ "$OPEN_BROWSER" = true ] && echo "yes" || echo "no")"
echo ""

# Construct command
if [ "$ENABLE_OBSTACLES" = true ]; then
    SCRIPT="$VIZ_DIR/run_server_with_obstacles.py"
    CMD="$PYTHON_CMD $SCRIPT --port $PORT --scenario obstacle --obstacles"
else
    SCRIPT="$VIZ_DIR/run_server.py"
    CMD="$PYTHON_CMD $SCRIPT --port $PORT"
fi

# Add browser flag if needed
if [ "$OPEN_BROWSER" = true ]; then
    CMD="$CMD --browser"
fi

# Add debug flag if needed
if [ "$DEBUG_MODE" = true ]; then
    CMD="$CMD --debug"
fi

# Execute the command
echo "Executing: $CMD"
eval "$CMD" 