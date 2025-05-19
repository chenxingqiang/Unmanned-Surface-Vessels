#!/usr/bin/env python3
"""
Test script for running the USV visualization with simulated data
This script starts both the visualization server and a test socket server
that generates mock simulation data
"""

import os
import sys
import time
import argparse
import webbrowser
import subprocess
import threading
import signal
import socket

def is_port_available(port):
    """Check if a port is available to use"""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(('localhost', port))
        s.close()
        return True
    except OSError:
        return False

def find_available_port(start_port, max_attempts=100):
    """Find an available port starting from start_port"""
    for port in range(start_port, start_port + max_attempts):
        if is_port_available(port):
            return port
    
    # If we get here, we couldn't find an available port
    raise RuntimeError(f"Could not find an available port after {max_attempts} attempts")

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Test USV Visualization with Simulated Data')
    parser.add_argument('--viz-port', type=int, default=8085, help='Port for visualization server')
    parser.add_argument('--test-port', type=int, default=9090, help='Port for test socket server')
    parser.add_argument('--no-browser', action='store_true', help="Don't open a browser automatically")
    parser.add_argument('--debug', action='store_true', help="Run in debug mode")
    args = parser.parse_args()
    
    # Find available ports
    try:
        viz_port = find_available_port(args.viz_port)
        if viz_port != args.viz_port:
            print(f"Port {args.viz_port} is in use, using port {viz_port} instead")
        
        test_port = find_available_port(args.test_port)
        if test_port != args.test_port:
            print(f"Port {args.test_port} is in use, using port {test_port} instead")
    except RuntimeError as e:
        print(f"Error finding available ports: {e}")
        return 1
    
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    viz_dir = os.path.join(script_dir, 'usv_system', 'visualization')
    
    # Commands to run
    debug_app_cmd = [
        sys.executable,
        os.path.join(script_dir, 'usv_system', 'visualization', 'debug_app.py'),
        '--port', str(viz_port),
        '--debug' if args.debug else '',
        '--obstacles'  # Always enable obstacles for testing
    ]
    
    test_socket_cmd = [
        sys.executable,
        os.path.join(script_dir, 'usv_system', 'visualization', 'test_socket_events.py'),
        '--port', str(test_port)
    ]
    
    # Remove empty arguments
    debug_app_cmd = [arg for arg in debug_app_cmd if arg]
    
    # Start servers
    print(f"\nStarting USV Visualization Test Environment")
    print("-" * 40)
    print(f"Visualization server port: {viz_port}")
    print(f"Test socket server port: {test_port}")
    print(f"Debug mode: {'enabled' if args.debug else 'disabled'}")
    print("-" * 40 + "\n")
    
    try:
        # Start visualization server
        print("Starting visualization server...")
        viz_server = subprocess.Popen(
            debug_app_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True
        )
        
        # Wait briefly for visualization server to start
        time.sleep(2)
        if viz_server.poll() is not None:
            print("Error: Visualization server failed to start")
            return 1
        
        # Start test socket server
        print("Starting test socket server...")
        test_server = subprocess.Popen(
            test_socket_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True
        )
        
        # Wait briefly for test server to start
        time.sleep(2)
        if test_server.poll() is not None:
            print("Error: Test socket server failed to start")
            viz_server.terminate()
            return 1
        
        # Open browser if requested
        if not args.no_browser:
            viz_url = f"http://localhost:{viz_port}"
            print(f"\nOpening visualization in browser: {viz_url}")
            webbrowser.open(viz_url)
        
        # Print instructions
        print("\nTest Environment Ready!")
        print("-" * 40)
        print("1. Open the visualization URL in your browser if not automatically opened")
        print("2. Click the 'Start Simulation' button to begin the test")
        print("3. Use the controls to interact with the simulation")
        print("4. Press Ctrl+C to stop the servers and exit")
        print("-" * 40)
        
        # Define output monitor threads
        def monitor_output(process, prefix):
            try:
                for line in iter(process.stdout.readline, ''):
                    print(f"{prefix}: {line.strip()}")
            except Exception as e:
                print(f"Error monitoring {prefix} output: {e}")
        
        # Start monitoring threads
        viz_thread = threading.Thread(target=monitor_output, args=(viz_server, "VIZ"), daemon=True)
        test_thread = threading.Thread(target=monitor_output, args=(test_server, "TEST"), daemon=True)
        viz_thread.start()
        test_thread.start()
        
        # Handle Ctrl+C gracefully
        def signal_handler(sig, frame):
            print("\nShutting down servers...")
            viz_server.terminate()
            test_server.terminate()
            try:
                viz_server.wait(timeout=5)
                test_server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("Warning: Some processes did not terminate gracefully")
            print("Servers stopped.")
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        
        # Keep the script running until Ctrl+C
        while True:
            time.sleep(1)
            
            # Check if processes are still running
            if viz_server.poll() is not None:
                print(f"Error: Visualization server exited unexpectedly with code {viz_server.returncode}")
                test_server.terminate()
                break
            
            if test_server.poll() is not None:
                print(f"Error: Test socket server exited unexpectedly with code {test_server.returncode}")
                viz_server.terminate()
                break
        
        return 1
    
    except Exception as e:
        print(f"Error running test environment: {e}")
        # Ensure servers are stopped
        try:
            viz_server.terminate()
            test_server.terminate()
        except:
            pass
        return 1

if __name__ == "__main__":
    main() 