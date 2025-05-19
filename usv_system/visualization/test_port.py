#!/usr/bin/env python3
"""
Test script for the automatic port selection feature.
This script verifies that the USV visualization server can automatically
find an available port when the default port is in use.
"""

import subprocess
import time
import os
import sys
import signal
import socket
import random

# Test configuration
TIMEOUT = 10  # seconds to wait for server startup
DEFAULT_PORT = 5540  # default port the server tries to use

def occupy_port(port):
    """
    Occupy a specific port to force the server to use a different port.
    
    Args:
        port (int): The port to occupy
        
    Returns:
        socket.socket or None: The socket occupying the port, or None if occupation failed
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', port))
        sock.listen(1)
        print(f"Successfully occupied port {port}")
        return sock
    except OSError as e:
        print(f"Failed to occupy port {port}: {e}")
        return None

def read_port_file(filename='port.txt'):
    """
    Read the port number from the port file.
    
    Args:
        filename (str): The file to read the port from
        
    Returns:
        int or None: The port number, or None if the file could not be read
    """
    try:
        if not os.path.exists(filename):
            return None
            
        with open(filename, 'r') as f:
            content = f.read().strip()
            return int(content)
    except (ValueError, OSError) as e:
        print(f"Error reading port file: {e}")
        return None

def run_test_with_occupied_port():
    """
    Test that the server can find an available port when the default port is occupied.
    
    Returns:
        bool: True if the test passed, False otherwise
    """
    # Get the path to app.py
    current_dir = os.path.dirname(os.path.abspath(__file__))
    app_path = os.path.join(current_dir, 'app.py')
    port_file = os.path.join(current_dir, 'port.txt')
    
    # Remove port.txt if it exists
    if os.path.exists(port_file):
        os.remove(port_file)
    
    # Occupy the default port
    sock = occupy_port(DEFAULT_PORT)
    if not sock:
        print(f"Could not occupy port {DEFAULT_PORT} for testing, test skipped")
        return False
        
    try:
        # Run app.py in a subprocess
        print(f"Running {app_path}...")
        proc = subprocess.Popen([sys.executable, app_path], 
                               stdout=subprocess.PIPE, 
                               stderr=subprocess.PIPE,
                               universal_newlines=True)
        
        # Wait for the port file to be created
        port = None
        for _ in range(TIMEOUT):
            port = read_port_file(port_file)
            if port is not None:
                break
            time.sleep(1)
            
        # Check if server started on a different port
        if port is not None and port != DEFAULT_PORT:
            print(f"Success: Server started on port {port} instead of default {DEFAULT_PORT}")
            success = True
        else:
            print(f"Error: Server should have used a different port than {DEFAULT_PORT}")
            # Print server output for debugging
            out, err = proc.communicate(timeout=1)
            print(f"Server stdout: {out}")
            print(f"Server stderr: {err}")
            success = False
            
        return success
    except Exception as e:
        print(f"Test failed with exception: {e}")
        return False
    finally:
        # Clean up
        print("Cleaning up...")
        
        # Close the socket
        if sock:
            sock.close()
            print(f"Released port {DEFAULT_PORT}")
            
        # Terminate the server process
        if 'proc' in locals() and proc:
            print("Terminating server process...")
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                print("Process did not terminate gracefully, sending SIGKILL...")
                os.kill(proc.pid, signal.SIGKILL)
            
        # Remove port file
        if os.path.exists(port_file):
            os.remove(port_file)
            print("Removed port.txt file")

def run_test_with_env_variable():
    """
    Test that the server uses the port specified in the environment variable.
    
    Returns:
        bool: True if the test passed, False otherwise
    """
    # Get the path to app.py
    current_dir = os.path.dirname(os.path.abspath(__file__))
    app_path = os.path.join(current_dir, 'app.py')
    port_file = os.path.join(current_dir, 'port.txt')
    
    # Remove port.txt if it exists
    if os.path.exists(port_file):
        os.remove(port_file)
    
    # Choose a random port that's not the default
    test_port = random.randint(8000, 9000)
    while test_port == DEFAULT_PORT:
        test_port = random.randint(8000, 9000)
    
    # Check if the port is available
    sock = occupy_port(test_port)
    if not sock:
        print(f"Could not occupy test port {test_port} for testing, skipping")
        return False
    
    # Release the port for the server to use
    sock.close()
        
    try:
        # Set environment variable for port
        env = os.environ.copy()
        env['USV_VISUALIZATION_PORT'] = str(test_port)
        
        # Run app.py in a subprocess with the environment variable
        print(f"Running {app_path} with USV_VISUALIZATION_PORT={test_port}...")
        proc = subprocess.Popen([sys.executable, app_path], 
                               stdout=subprocess.PIPE, 
                               stderr=subprocess.PIPE,
                               universal_newlines=True,
                               env=env)
        
        # Wait for the port file to be created
        port = None
        for _ in range(TIMEOUT):
            port = read_port_file(port_file)
            if port is not None:
                break
            time.sleep(1)
            
        # Check if server started on the specified port
        if port is not None and port == test_port:
            print(f"Success: Server started on specified port {port} from environment variable")
            success = True
        else:
            print(f"Error: Server should have used specified port {test_port} but used {port}")
            # Print server output for debugging
            out, err = proc.communicate(timeout=1)
            print(f"Server stdout: {out}")
            print(f"Server stderr: {err}")
            success = False
            
        return success
    except Exception as e:
        print(f"Test failed with exception: {e}")
        return False
    finally:
        # Clean up
        print("Cleaning up...")
            
        # Terminate the server process
        if 'proc' in locals() and proc:
            print("Terminating server process...")
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                print("Process did not terminate gracefully, sending SIGKILL...")
                os.kill(proc.pid, signal.SIGKILL)
            
        # Remove port file
        if os.path.exists(port_file):
            os.remove(port_file)
            print("Removed port.txt file")

def main():
    """
    Run all tests and report results.
    """
    print("=== USV Visualization Port Selection Tests ===")
    
    # Run tests
    print("\n--- Test 1: Server finds alternative port when default is occupied ---")
    test1_result = run_test_with_occupied_port()
    
    print("\n--- Test 2: Server uses port from environment variable ---")
    test2_result = run_test_with_env_variable()
    
    # Print summary
    print("\n=== Test Results ===")
    print(f"Test 1 (Alternative Port): {'PASSED' if test1_result else 'FAILED'}")
    print(f"Test 2 (Environment Variable): {'PASSED' if test2_result else 'FAILED'}")
    
    # Return overall result
    if test1_result and test2_result:
        print("\nALL TESTS PASSED!")
        return 0
    else:
        print("\nSOME TESTS FAILED!")
        return 1

if __name__ == "__main__":
    sys.exit(main()) 