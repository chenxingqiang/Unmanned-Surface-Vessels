#!/usr/bin/env python3
"""
Simple script to test socket connection and simulation data flow.
This directly tests starting the simulation and verifies data is being sent.
"""

import os
import sys
import socketio
import time
import json

def test_simulation(port=8080):
    """Test socket connection and simulation data flow."""
    print(f"Testing socket connection to http://localhost:{port}")
    
    # Create Socket.IO client
    sio = socketio.Client()
    
    # Track connection and data
    connected = False
    data_received = False
    simulation_started = False
    
    # Event handlers
    @sio.event
    def connect():
        nonlocal connected
        connected = True
        print("✓ Connected to server")
    
    @sio.event
    def disconnect():
        print("× Disconnected from server")
    
    @sio.event
    def connect_error(error):
        print(f"× Connection error: {error}")
    
    @sio.on('simulation_data')
    def on_data(data):
        nonlocal data_received
        data_received = True
        print(f"✓ Received simulation data: time={data.get('time', 'unknown')}")
        print(f"  Position: x={data.get('state', {}).get('x', 'unknown')}, y={data.get('state', {}).get('y', 'unknown')}")
    
    @sio.on('simulation_status')
    def on_status(data):
        nonlocal simulation_started
        status = data.get('status', 'unknown')
        print(f"✓ Simulation status: {status}")
        if status == 'started':
            simulation_started = True
    
    try:
        # Connect to server
        sio.connect(f'http://localhost:{port}')
        time.sleep(1)
        
        if not connected:
            print("× Failed to connect to server")
            return False
        
        # Stop any existing simulation
        print("\nStopping any existing simulation...")
        sio.emit('stop_simulation')
        time.sleep(1)
        
        # Start simulation
        print("\nStarting new simulation...")
        sio.emit('start_simulation', {
            'scenario': 'obstacle',
            'controller': 'pid',
            'use_obstacle_avoidance': True
        })
        
        # Wait for simulation to start
        print("Waiting for simulation to start...")
        for _ in range(10):
            if simulation_started:
                break
            time.sleep(0.5)
        
        if not simulation_started:
            print("× Simulation failed to start properly")
        else:
            print("✓ Simulation started successfully")
        
        # Wait for data
        print("\nWaiting for simulation data...")
        for _ in range(10):
            if data_received:
                break
            time.sleep(0.5)
        
        if not data_received:
            print("× No simulation data received - CRITICAL ISSUE")
            print("\nDEBUG INFORMATION:")
            print("1. Requesting simulation state for debugging...")
            sio.emit('get_simulation_state')
            time.sleep(2)
            
            print("\n2. Checking server version...")
            
            print("\n3. Trying to toggle obstacle avoidance...")
            sio.emit('toggle_obstacle_avoidance', {'enabled': False})
            time.sleep(0.5)
            sio.emit('toggle_obstacle_avoidance', {'enabled': True})
            
            print("\n4. Waiting for data after toggling...")
            for _ in range(5):
                if data_received:
                    break
                time.sleep(1)
                
            if not data_received:
                print("\n× PROBLEM IDENTIFIED: Simulation is not generating data")
                print("  This indicates an issue with the simulation worker thread")
                print("  Check if the simulation_worker function is running correctly")
                print("  Also check for any errors in the server console output")
                return False
        else:
            print("✓ Simulation data is flowing correctly")
        
        return data_received and simulation_started
    except Exception as e:
        print(f"× Error: {e}")
        return False
    finally:
        try:
            if connected:
                sio.disconnect()
        except:
            pass

if __name__ == "__main__":
    port = 8080
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            print(f"Invalid port number: {sys.argv[1]}")
            sys.exit(1)
    
    success = test_simulation(port)
    
    if success:
        print("\n✓ TEST PASSED: Socket connection and simulation work correctly")
        print("  If the web interface still doesn't work, the issue is in the browser JavaScript")
    else:
        print("\n× TEST FAILED: Socket connection or simulation has issues")
        print("  Check the server logs for more information")
    
    sys.exit(0 if success else 1) 