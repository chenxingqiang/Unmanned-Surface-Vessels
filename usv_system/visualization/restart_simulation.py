#!/usr/bin/env python3
"""
Script to restart the USV simulation if it's not updating.
This script will:
1. Stop any running simulation
2. Clear any existing states
3. Start a new simulation with obstacle avoidance
"""

import sys
import socketio
import time

def restart_simulation(port=9000):
    """Connect to the server and restart the simulation."""
    # Create a Socket.IO client
    sio = socketio.Client()
    
    connected = False
    simulation_started = False
    
    @sio.event
    def connect():
        nonlocal connected
        connected = True
        print("Connected to server!")
    
    @sio.event
    def disconnect():
        print("Disconnected from server")
    
    @sio.on('simulation_status')
    def on_simulation_status(data):
        nonlocal simulation_started
        print(f"Simulation status: {data['status']}")
        if data['status'] == 'started':
            simulation_started = True
    
    try:
        # Connect to the server
        print(f"Connecting to server at http://localhost:{port}...")
        sio.connect(f'http://localhost:{port}')
        
        # Wait for connection
        timeout = 5
        start_time = time.time()
        while not connected and time.time() - start_time < timeout:
            time.sleep(0.1)
        
        if not connected:
            print("Failed to connect to server")
            return False
        
        # Stop any existing simulation
        print("Stopping any running simulation...")
        sio.emit('stop_simulation')
        time.sleep(2)
        
        # Start a new simulation with obstacle avoidance
        print("Starting new simulation...")
        sio.emit('start_simulation', {
            'scenario': 'obstacle',
            'controller': 'pid',
            'use_obstacle_avoidance': True
        })
        
        # Wait for simulation to start
        print("Waiting for simulation to initialize...")
        timeout = 5
        start_time = time.time()
        while not simulation_started and time.time() - start_time < timeout:
            time.sleep(0.1)
        
        if not simulation_started:
            print("Warning: Simulation may not have started properly")
        else:
            print("Simulation restarted successfully!")
        
        # Toggle obstacle avoidance off and on again to ensure it's active
        print("Toggling obstacle avoidance...")
        sio.emit('toggle_obstacle_avoidance', {'enabled': False})
        time.sleep(0.5)
        sio.emit('toggle_obstacle_avoidance', {'enabled': True})
        
        return True
    except Exception as e:
        print(f"Error restarting simulation: {e}")
        return False
    finally:
        if connected:
            sio.disconnect()

if __name__ == "__main__":
    port = 9000
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            print(f"Invalid port number: {sys.argv[1]}")
            sys.exit(1)
    
    success = restart_simulation(port)
    sys.exit(0 if success else 1) 