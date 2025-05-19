#!/bin/bash
# Run the USV simulator with visualization and auto-start simulation

# Get the directory where this script is located
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
cd "$SCRIPT_DIR"

# Default port
PORT=9000

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        -p|--port)
            PORT="$2"
            shift
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  -p, --port PORT      Specify port (default: 9000)"
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

echo "Starting USV Simulator with visualization on port $PORT"
echo "Press Ctrl+C to stop the simulation"

# Run the Python script with the browser option enabled
python3 start_simulation.py --port $PORT --browser 