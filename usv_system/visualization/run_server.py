#!/usr/bin/env python3
"""
Script to run the USV Visualization Server
This script is a cross-platform alternative to run_server.sh
"""

import os
import sys
import argparse
import subprocess
import time
import signal
import webbrowser

DEFAULT_PORT = 5540

def show_help():
    """Display help message"""
    print("USV Visualization Server Runner")
    print("")
    print("Usage: python run_server.py [options]")
    print("")
    print("Options:")
    print(f"  -p, --port PORT      Specify the port number (default: {DEFAULT_PORT})")
    print("  -d, --debug          Run in debug mode")
    print("  -b, --browser        Automatically open browser")
    print("  -h, --help           Display this help message and exit")
    print("")
    print("Example:")
    print("  python run_server.py --port 8080 --browser  Run the server on port 8080 and open browser")
    print("")

def read_port_file(file_path):
    """Read port from port.txt file"""
    try:
        if not os.path.exists(file_path):
            return None
            
        with open(file_path, 'r') as f:
            content = f.read().strip()
            return int(content)
    except (ValueError, OSError) as e:
        print(f"Error reading port file: {e}")
        return None

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="USV Visualization Server Runner")
    parser.add_argument("-p", "--port", type=int, default=DEFAULT_PORT, help=f"Port to run the server on (default: {DEFAULT_PORT})")
    parser.add_argument("-d", "--debug", action="store_true", help="Run in debug mode")
    parser.add_argument("-b", "--browser", action="store_true", help="Automatically open browser")
    args = parser.parse_args()

    # Validate port
    if args.port < 1 or args.port > 65535:
        print("Error: Port must be between 1 and 65535")
        return 1

    # Set environment variables
    os.environ["USV_VISUALIZATION_PORT"] = str(args.port)

    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    app_path = os.path.join(script_dir, 'app.py')
    port_file = os.path.join(script_dir, 'port.txt')

    # Print startup message
    print("Starting USV Visualization Server...")
    print(f"Using Python: {sys.version.split()[0]}")
    print(f"Server port: {args.port}")
    print(f"Debug mode: {'enabled' if args.debug else 'disabled'}")
    print("")

    # Change to script directory
    os.chdir(script_dir)

    # Remove port.txt if it exists
    if os.path.exists(port_file):
        os.remove(port_file)

    # Run the server
    if args.debug:
        # Run in foreground with debug output
        try:
            subprocess.run([sys.executable, app_path])
        except KeyboardInterrupt:
            print("\nServer stopped by user")
    else:
        # Run in background and capture output
        log_file = os.path.join(script_dir, 'server.log')
        
        with open(log_file, 'w') as log:
            proc = subprocess.Popen(
                [sys.executable, app_path],
                stdout=log,
                stderr=log,
                universal_newlines=True
            )
            
            # Store the PID
            print(f"Server started with PID: {proc.pid}")
            print(f"Log file: {log_file}")
            
            # Wait briefly for server to start
            time.sleep(2)
            
            # Check if process is still running
            if proc.poll() is not None:
                print("Error: Server failed to start. Check log file for details.")
                return 1
            
            # Check if port.txt exists to get actual port
            port = args.port
            if os.path.exists(port_file):
                actual_port = read_port_file(port_file)
                if actual_port is not None and actual_port != args.port:
                    port = actual_port
                    print(f"Note: Server is using port {port} instead of {args.port}")
            
            url = f"http://localhost:{port}"
            print(f"Visit: {url}")
            
            # Open browser if requested
            if args.browser:
                print("Opening browser...")
                webbrowser.open(url)
            
            print("\nPress Ctrl+C to stop the server")
            
            try:
                # Keep the script running to allow Ctrl+C to stop the server
                while True:
                    time.sleep(1)
                    # Check if process is still running
                    if proc.poll() is not None:
                        print("Server has stopped unexpectedly. Check log file for details.")
                        break
            except KeyboardInterrupt:
                print("\nStopping server...")
                # On Windows, use taskkill to kill the process tree
                if sys.platform == 'win32':
                    subprocess.run(['taskkill', '/F', '/T', '/PID', str(proc.pid)])
                else:
                    # On Unix, use process group kill
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=5)
                print("Server stopped")

    return 0

if __name__ == "__main__":
    sys.exit(main()) 