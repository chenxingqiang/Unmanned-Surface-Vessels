#!/bin/bash
# Simple script to run the USV visualization system

# Get the directory where this script is located
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
cd "$SCRIPT_DIR"

# Display help message
function show_help {
    echo "USV Visualization System Runner"
    echo "------------------------------"
    echo "Usage: ./run_usv.sh [options]"
    echo ""
    echo "Options:"
    echo "  -m, --mode MODE     Server mode: standard or obstacles (default: obstacles)"
    echo "  -p, --port PORT     Port to run the server on (default: 9000)"
    echo "  -d, --debug         Run in debug mode"
    echo "  -b, --browser       Open web browser automatically"
    echo "  -h, --help          Show this help message"
    echo ""
    exit 0
}

# Default values
MODE="obstacles"
PORT=9000
DEBUG=false
BROWSER=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        -m|--mode)
            MODE="$2"
            shift
            shift
            ;;
        -p|--port)
            PORT="$2"
            shift
            shift
            ;;
        -d|--debug)
            DEBUG=true
            shift
            ;;
        -b|--browser)
            BROWSER=true
            shift
            ;;
        -h|--help)
            show_help
            ;;
        *)
            # Unknown option
            echo "Unknown option: $1"
            show_help
            ;;
    esac
done

# Validate mode
if [[ "$MODE" != "standard" && "$MODE" != "obstacles" ]]; then
    echo "Error: Invalid mode '$MODE'. Mode must be 'standard' or 'obstacles'."
    exit 1
fi

# Build command
CMD="./run_usv.py --mode $MODE --port $PORT"

if [[ "$DEBUG" == true ]]; then
    CMD="$CMD --debug"
fi

if [[ "$BROWSER" == true ]]; then
    CMD="$CMD --browser"
fi

# Run the command
echo "Starting USV visualization in $MODE mode on port $PORT"
echo "Running: $CMD"
$CMD 