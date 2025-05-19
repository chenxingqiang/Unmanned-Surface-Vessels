#!/usr/bin/env python3
"""
Diagnostic script to check if the USV simulation is sending data properly.
This script connects to the server and monitors data transfer.
"""

import sys
import socketio
import time
import json

def check_simulation(port=9000):
    """Connect to the server and monitor simulation data."""
    # Create a Socket.IO client
    sio = socketio.Client()
    
    # Counters for diagnostics
    data_count = 0
    last_data_time = None
    connection_issues = False
    
    @sio.event
    def connect():
        print("Connected to server!")
    
    @sio.event
    def connect_error(data):
        nonlocal connection_issues
        connection_issues = True
        print(f"Connection error: {data}")
    
    @sio.event
    def disconnect():
        print("Disconnected from server")
    
    @sio.on('simulation_status')
    def on_simulation_status(data):
        print(f"Simulation status: {data['status']}")
    
    @sio.on('simulation_data')
    def on_simulation_data(data):
        nonlocal data_count, last_data_time
        data_count += 1
        current_time = time.time()
        
        if last_data_time:
            time_diff = current_time - last_data_time
            if data_count % 10 == 0:  # Only print every 10th message to avoid flooding
                print(f"[{data_count}] Received data at simulation time {data['time']:.2f}s (interval: {time_diff:.3f}s)")
                
                # Print position data
                if 'state' in data:
                    pos = data['state']
                    print(f"  Position: x={pos['x']:.2f}, y={pos['y']:.2f}, heading={pos['heading']:.2f}")
                
                # Print control data
                if 'control' in data:
                    ctrl = data['control']
                    print(f"  Control: thrust={ctrl['thrust']:.2f}, moment={ctrl['moment']:.2f}")
                
                # Print obstacle data if available
                if 'obstacles' in data:
                    static_count = len(data['obstacles'].get('static', []))
                    dynamic_count = len(data['obstacles'].get('dynamic', []))
                    print(f"  Obstacles: {static_count} static, {dynamic_count} dynamic")
        
        last_data_time = current_time
    
    @sio.on('simulation_state')
    def on_simulation_state(data):
        print(f"Simulation state: {json.dumps(data, indent=2)}")
    
    try:
        # Connect to the server
        print(f"Connecting to server at http://localhost:{port}...")
        sio.connect(f'http://localhost:{port}')
        
        if connection_issues:
            print("Warning: Encountered connection issues")
        
        # Get current simulation state
        print("Getting current simulation state...")
        sio.emit('get_simulation_state')
        time.sleep(1)
        
        # Monitor data for a while
        print("\nMonitoring simulation data for 30 seconds...")
        print("(Press Ctrl+C to exit earlier)")
        
        deadline = time.time() + 30
        try:
            while time.time() < deadline:
                time.sleep(0.1)
                
                if data_count > 0 and time.time() - last_data_time > 2:
                    print("WARNING: No data received in the last 2 seconds!")
        except KeyboardInterrupt:
            print("\nMonitoring stopped by user")
        
        # Print summary
        print("\nDiagnostic Summary:")
        print(f"- Total data packets received: {data_count}")
        if data_count == 0:
            print("  ERROR: No simulation data was received!")
            print("  Possible issues:")
            print("  - Simulation not started")
            print("  - Server not sending data")
            print("  - Network/firewall blocking WebSocket connections")
        else:
            print(f"- Average data rate: {data_count / min(30, time.time() - (deadline - 30)):.2f} packets/second")
        
        return data_count > 0
    except Exception as e:
        print(f"Error during diagnostics: {e}")
        return False
    finally:
        try:
            sio.disconnect()
        except:
            pass

if __name__ == "__main__":
    port = 9000
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            print(f"Invalid port number: {sys.argv[1]}")
            sys.exit(1)
    
    success = check_simulation(port)
    sys.exit(0 if success else 1) 