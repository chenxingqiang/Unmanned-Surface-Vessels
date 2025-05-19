#!/usr/bin/env python3
"""
Cross-platform script to run the USV Visualization Server from the root directory
"""

import os
import sys
import argparse
import subprocess
import time
import signal
import webbrowser
import platform

# Default settings
DEFAULT_PORT = 8080

def kill_process_on_port(port):
    """Kill any process currently using the specified port."""
    print(f"Checking for processes using port {port}...")
    
    try:
        # Different commands for different operating systems
        if platform.system() == "Darwin":  # macOS
            # Get the PID of the process using the port
            cmd = f"lsof -i tcp:{port} | grep LISTEN | awk '{{print $2}}'"
            process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
            pid = process.stdout.read().decode().strip()
            
            if pid:
                print(f"Found process (PID: {pid}) using port {port}, terminating...")
                # Kill the process
                os.kill(int(pid), signal.SIGTERM)
                # Wait for the process to terminate
                time.sleep(1)
                return True
                
        elif platform.system() == "Linux":
            # Get the PID of the process using the port
            cmd = f"netstat -tuln | grep {port} > /dev/null && fuser -k {port}/tcp"
            subprocess.run(cmd, shell=True)
            time.sleep(1)
            return True
            
        else:  # Windows or other
            print("Automatic port clearing not supported on this platform.")
    except Exception as e:
        print(f"Error attempting to kill process on port {port}: {e}")
        
    return False

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="USV Visualization Server Runner")
    parser.add_argument("-p", "--port", type=int, default=DEFAULT_PORT, help=f"Port to run the server on (default: {DEFAULT_PORT})")
    parser.add_argument("-d", "--debug", action="store_true", help="Run in debug mode")
    parser.add_argument("-o", "--no-obstacles", action="store_false", dest="obstacles", help="Disable obstacle avoidance")
    parser.add_argument("-b", "--no-browser", action="store_false", dest="browser", help="Don't open browser automatically")
    parser.add_argument("-s", "--scenario", type=str, default="obstacle", help="Scenario to initialize")
    
    # Set defaults for optional flags
    parser.set_defaults(obstacles=True, browser=True)
    
    args = parser.parse_args()

    # Validate port
    if args.port < 1 or args.port > 65535:
        print("Error: Port must be between 1 and 65535")
        return 1

    # Kill any process using the target port
    if args.port:
        kill_process_on_port(args.port)
        # Also check port+1 since the server sometimes uses next port
        kill_process_on_port(args.port + 1)

    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    viz_dir = os.path.join(script_dir, "usv_system", "visualization")
    
    # Build command arguments
    cmd = [sys.executable, os.path.join(viz_dir, "run_server_with_obstacles.py")]
    
    if args.port:
        cmd.extend(["--port", str(args.port)])
    
    if args.debug:
        cmd.append("--debug")
    
    if args.obstacles:
        cmd.extend(["--obstacles", "--scenario", args.scenario])
    
    if args.browser:
        cmd.append("--browser")
    
    # Print startup information
    print("Starting USV Visualization Server...")
    print(f"Using Python: {sys.version.split()[0]}")
    print(f"Server port: {args.port}")
    print(f"Debug mode: {'enabled' if args.debug else 'disabled'}")
    print(f"Obstacles: {'enabled' if args.obstacles else 'disabled'}")
    print(f"Opening browser: {'yes' if args.browser else 'no'}")
    print("")
    print(f"Executing: {' '.join(cmd)}")
    
    # Run the server
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\nServer stopped by user")
    except Exception as e:
        print(f"Error running server: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 