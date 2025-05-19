#!/usr/bin/env python3
"""
Enhanced runner script for the USV Visualization Server with obstacle avoidance integration
"""

import os
import sys
import argparse
import subprocess
import time
import signal
import webbrowser

# Add the parent directory to the system path
sys.path.append(os.path.abspath(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))

from usv_system.simulation.simulator import USVSimulator
from usv_system.obstacle_avoidance.avoidance_strategy import COLREGAvoidanceStrategy, APFAvoidanceStrategy, ObstacleAvoidanceSystem
from usv_system.obstacle_avoidance.detector import LidarDetector, RadarDetector

DEFAULT_PORT = 5540

def show_help():
    """Display help message"""
    print("USV Visualization Server Runner with Obstacle Avoidance")
    print("")
    print("Usage: python run_server_with_obstacles.py [options]")
    print("")
    print("Options:")
    print(f"  -p, --port PORT      Specify the port number (default: {DEFAULT_PORT})")
    print("  -d, --debug          Run in debug mode")
    print("  -b, --browser        Automatically open browser")
    print("  -o, --obstacles      Enable obstacle avoidance (default: False)")
    print("  -s, --scenario NAME  Specify scenario (default: 'obstacle')")
    print("  -h, --help           Display this help message and exit")
    print("")
    print("Example:")
    print("  python run_server_with_obstacles.py --port 8080 --browser --obstacles  Run with obstacles enabled")
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

def setup_obstacle_avoidance_environment():
    """Setup environment for obstacle avoidance simulation"""
    # This function sets environment variables or creates temporary config files
    # that will be read by the app.py server
    
    # Create a modified config file with obstacles enabled
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.abspath(os.path.join(script_dir, '../../config/default_config.yaml'))
    temp_config_path = os.path.join(script_dir, 'temp_config.yaml')
    
    try:
        import yaml
        
        # Read original config
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Enable obstacles
        config['environment']['obstacles']['enabled'] = True
        
        # Add more obstacles for testing
        config['scenarios']['obstacle']['environment']['obstacles']['static'] = [
            {"position": [25, 25], "radius": 5.0},
            {"position": [75, 25], "radius": 5.0},
            {"position": [40, 40], "radius": 4.0},
            {"position": [60, 60], "radius": 3.0}
        ]
        
        config['scenarios']['obstacle']['environment']['obstacles']['dynamic'] = [
            {"initial_position": [50, 75], "velocity": [-0.5, -0.5], "radius": 3.0, 
             "heading": 3.9, "speed": 1.0},
            {"initial_position": [20, 60], "velocity": [0.3, -0.2], "radius": 2.5,
             "heading": 5.2, "speed": 0.8}
        ]
        
        # Write modified config
        with open(temp_config_path, 'w') as f:
            yaml.dump(config, f)
            
        # Set environment variable to point to this config
        os.environ["USV_CONFIG_PATH"] = temp_config_path
        print(f"Created custom config with obstacles at: {temp_config_path}")
        
        return True
    except Exception as e:
        print(f"Error setting up obstacle avoidance: {e}")
        return False

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="USV Visualization Server Runner with Obstacle Avoidance")
    parser.add_argument("-p", "--port", type=int, default=DEFAULT_PORT, help=f"Port to run the server on (default: {DEFAULT_PORT})")
    parser.add_argument("-d", "--debug", action="store_true", help="Run in debug mode")
    parser.add_argument("-b", "--browser", action="store_true", help="Automatically open browser")
    parser.add_argument("-o", "--obstacles", action="store_true", help="Enable obstacle avoidance")
    parser.add_argument("-s", "--scenario", type=str, default="obstacle", help="Specify scenario")
    args = parser.parse_args()

    # Show help if --help flag is passed
    if '--help' in sys.argv or '-h' in sys.argv:
        show_help()
        return 0

    # Validate port
    if args.port < 1 or args.port > 65535:
        print("Error: Port must be between 1 and 65535")
        return 1

    # Set environment variables
    os.environ["USV_VISUALIZATION_PORT"] = str(args.port)
    os.environ["USV_DEFAULT_SCENARIO"] = args.scenario
    
    # Setup obstacle avoidance if enabled
    if args.obstacles:
        print("Setting up obstacle avoidance simulation...")
        if not setup_obstacle_avoidance_environment():
            print("Warning: Failed to setup obstacle avoidance environment")
    
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Use app_with_obstacles.py instead of app.py
    app_path = os.path.join(script_dir, 'app_with_obstacles.py')
    port_file = os.path.join(script_dir, 'port.txt')

    # Print startup message
    print("Starting USV Visualization Server...")
    print(f"Using Python: {sys.version.split()[0]}")
    print(f"Server port: {args.port}")
    print(f"Debug mode: {'enabled' if args.debug else 'disabled'}")
    print(f"Scenario: {args.scenario}")
    print(f"Obstacle avoidance: {'enabled' if args.obstacles else 'disabled'}")
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
                
                # Clean up temp config if it exists
                temp_config = os.path.join(script_dir, 'temp_config.yaml')
                if os.path.exists(temp_config):
                    os.remove(temp_config)

    return 0

if __name__ == "__main__":
    sys.exit(main()) 