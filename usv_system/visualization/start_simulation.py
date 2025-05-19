#!/usr/bin/env python3
"""
Script to start the USV visualization with obstacle avoidance and ensure simulation is running.
This script handles port specification and initializes the simulation automatically.
"""

import os
import sys
import argparse
import subprocess
import time
import socket
import signal
import webbrowser
import threading
import requests
import socketio

# Add the parent directory to the system path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Default port settings
DEFAULT_PORT = 9000
PORT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'port.txt')

def is_port_available(port):
    """Check if the given port is available for use."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind(('0.0.0.0', port))
        sock.close()
        return True
    except OSError:
        return False

def find_available_port(start_port, max_attempts=100):
    """Find an available port starting from the given port."""
    for port in range(start_port, start_port + max_attempts):
        if is_port_available(port):
            return port
    raise RuntimeError(f"Could not find an available port after {max_attempts} attempts")

def start_visualization_server(port):
    """Start the visualization server on the specified port."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    app_path = os.path.join(script_dir, 'app_with_obstacles.py')
    
    # Start server process
    server_process = subprocess.Popen(
        [sys.executable, app_path, '--port', str(port)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True
    )
    
    return server_process

def wait_for_server(port, timeout=10):
    """Wait for the server to start and be responsive."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            response = requests.get(f"http://localhost:{port}", timeout=1)
            if response.status_code == 200:
                return True
        except requests.exceptions.RequestException:
            time.sleep(0.5)
    return False

def start_simulation(port):
    """Connect to the server and start the simulation."""
    # Create a Socket.IO client
    sio = socketio.Client()
    
    try:
        # Connect to the server
        sio.connect(f'http://localhost:{port}')
        
        # Start the simulation
        sio.emit('start_simulation', {
            'scenario': 'obstacle',
            'controller': 'pid',
            'use_obstacle_avoidance': True
        })
        
        print("Simulation started successfully")
        sio.disconnect()
        return True
    except Exception as e:
        print(f"Error starting simulation: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Start USV Visualization with Obstacles")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"Port to run the server on (default: {DEFAULT_PORT})")
    parser.add_argument("--browser", action="store_true", help="Open browser automatically")
    parser.add_argument("--no-simulation", action="store_true", help="Don't start simulation automatically")
    args = parser.parse_args()
    
    # Ensure port is available or find an alternative
    try:
        port = args.port
        if not is_port_available(port):
            print(f"Port {port} is already in use")
            port = find_available_port(port + 1)
            print(f"Using alternative port: {port}")
    except Exception as e:
        print(f"Error setting up port: {e}")
        return 1
    
    # Start the visualization server
    print(f"Starting visualization server on port {port}...")
    server_process = start_visualization_server(port)
    
    # Wait for the server to start
    if not wait_for_server(port):
        print("Server failed to start in the expected time")
        server_process.terminate()
        return 1
    
    # Open browser if requested
    url = f"http://localhost:{port}"
    print(f"Server is running at: {url}")
    if args.browser:
        print("Opening browser...")
        webbrowser.open(url)
    
    # Start the simulation automatically if not disabled
    if not args.no_simulation:
        # Wait a moment for the page to load
        time.sleep(2)
        if not start_simulation(port):
            print("Warning: Failed to auto-start the simulation")
            print("Please start it manually through the web interface")
    
    print("Press Ctrl+C to stop the server")
    try:
        # Keep the script running
        server_process.wait()
    except KeyboardInterrupt:
        print("\nStopping server...")
        server_process.terminate()
        try:
            server_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server_process.kill()
        print("Server stopped")
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 