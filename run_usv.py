#!/usr/bin/env python3
"""
Runner script for USV Visualization System
This script allows easy launching of the USV visualization from the project root directory
"""

import os
import sys
import argparse
import subprocess

def main():
    parser = argparse.ArgumentParser(description='Run USV Visualization Server')
    parser.add_argument('--mode', '-m', choices=['standard', 'obstacles'], default='obstacles',
                        help='Server mode: standard or obstacles (with obstacle avoidance)')
    parser.add_argument('--port', '-p', type=int, default=9000,
                        help='Port to run the server on')
    parser.add_argument('--debug', '-d', action='store_true',
                        help='Run in debug mode')
    parser.add_argument('--browser', '-b', action='store_true',
                        help='Open web browser automatically')
    args = parser.parse_args()

    # Change directory to the visualization folder
    vis_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'usv_system', 'visualization')
    
    # Kill any running Python processes related to the visualization
    try:
        subprocess.call(f"pkill -f 'python.*visualization'", shell=True)
        print("Stopped any running visualization servers.")
    except Exception as e:
        print(f"Note: Could not stop previous servers: {e}")
    
    # Wait for processes to terminate
    import time
    time.sleep(1)
    
    # Update the port configuration file
    port_file_path = os.path.join(vis_dir, "port.txt")
    with open(port_file_path, "w") as f:
        f.write(str(args.port))
    print(f"Set server port to {args.port}")
    
    if args.mode == 'standard':
        if args.debug:
            cmd = f"cd {vis_dir} && python debug_app.py --port {args.port}"
        else:
            cmd = f"cd {vis_dir} && python app.py --port {args.port}"
    else:  # obstacles mode
        if args.debug:
            cmd = f"cd {vis_dir} && python debug_app.py --port {args.port}"
        else:
            cmd = f"cd {vis_dir} && python app_with_obstacles.py --port {args.port}"
    
    # Add browser option if requested
    if args.browser:
        cmd += " --browser"
    
    print(f"Starting USV visualization in {args.mode} mode on port {args.port}")
    print(f"Running command: {cmd}")
    
    # Execute the command
    os.system(cmd)

if __name__ == "__main__":
    main() 