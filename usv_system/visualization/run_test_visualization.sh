#!/bin/bash
# Script to run both visualization server and test socket server

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

# Default port settings
VIZ_PORT=8085
TEST_PORT=8090

# Check command line arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --viz-port)
            VIZ_PORT="$2"
            shift
            shift
            ;;
        --test-port)
            TEST_PORT="$2"
            shift
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--viz-port PORT] [--test-port PORT]"
            exit 1
            ;;
    esac
done

# Function to check if port is available
function is_port_available() {
    local port=$1
    if nc -z localhost $port >/dev/null 2>&1; then
        return 1
    else
        return 0
    fi
}

# Find available ports
while ! is_port_available $VIZ_PORT; do
    echo "Port $VIZ_PORT is in use, trying next port..."
    VIZ_PORT=$((VIZ_PORT + 1))
done

while ! is_port_available $TEST_PORT; do
    echo "Port $TEST_PORT is in use, trying next port..."
    TEST_PORT=$((TEST_PORT + 1))
done

echo "=== USV Visualization Test Runner ==="
echo "Starting visualization server on port $VIZ_PORT"
echo "Starting test socket server on port $TEST_PORT"
echo ""

# Check if we're using macOS or Linux
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS terminal
    osascript <<EOF
    tell application "Terminal"
        do script "cd '$SCRIPT_DIR' && python debug_app.py --port $VIZ_PORT --debug"
    end tell
    tell application "Terminal"
        do script "cd '$SCRIPT_DIR' && python test_socket_events.py --port $TEST_PORT"
    end tell
EOF
elif command -v gnome-terminal >/dev/null 2>&1; then
    # Linux with Gnome
    gnome-terminal -- bash -c "cd '$SCRIPT_DIR' && python debug_app.py --port $VIZ_PORT --debug; bash"
    gnome-terminal -- bash -c "cd '$SCRIPT_DIR' && python test_socket_events.py --port $TEST_PORT; bash"
elif command -v xterm >/dev/null 2>&1; then
    # Linux with xterm
    xterm -e "cd '$SCRIPT_DIR' && python debug_app.py --port $VIZ_PORT --debug; bash" &
    xterm -e "cd '$SCRIPT_DIR' && python test_socket_events.py --port $TEST_PORT; bash" &
else
    # Fallback: run both in background with output to files
    echo "Could not open new terminal windows. Running in background instead."
    echo "Output will be logged to files."
    echo ""
    
    python debug_app.py --port $VIZ_PORT --debug > viz_server.log 2>&1 &
    VIZ_PID=$!
    echo "Visualization server running with PID $VIZ_PID"
    echo "Log file: $SCRIPT_DIR/viz_server.log"
    
    python test_socket_events.py --port $TEST_PORT > test_server.log 2>&1 &
    TEST_PID=$!
    echo "Test socket server running with PID $TEST_PID"
    echo "Log file: $SCRIPT_DIR/test_server.log"
    
    echo ""
    echo "Visit the visualization at: http://localhost:$VIZ_PORT"
    echo "Test Socket.IO server at: http://localhost:$TEST_PORT"
    echo ""
    echo "Press Ctrl+C to stop both servers"
    
    # Wait for Ctrl+C and then kill both servers
    trap "kill $VIZ_PID $TEST_PID; echo 'Stopped servers'; exit 0" INT
    wait
fi

echo "Started both servers in separate terminals"
echo "Visualization server: http://localhost:$VIZ_PORT"
echo "Test socket server: http://localhost:$TEST_PORT"
echo ""
echo "NOTE: To run the test, open the visualization in your browser, then press the Start button."
echo "      This will cause the test socket server to generate mock data for display." 