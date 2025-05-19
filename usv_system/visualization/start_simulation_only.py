#!/usr/bin/env python3
"""
Script to start the USV simulation on an already running server.
"""

import sys
import socketio
import time

def start_simulation(port=9000):
    """Connect to the server and start the simulation."""
    # Create a Socket.IO client
    sio = socketio.Client()
    
    connected = False
    
    @sio.event
    def connect():
        nonlocal connected
        connected = True
        print("Connected to server!")
    
    @sio.event
    def connect_error(data):
        print(f"Connection error: {data}")
    
    @sio.event
    def disconnect():
        print("Disconnected from server")
    
    @sio.on('simulation_status')
    def on_simulation_status(data):
        print(f"Simulation status: {data['status']}")
    
    @sio.on('simulation_data')
    def on_simulation_data(data):
        print(f"Receiving simulation data at time: {data['time']}")
    
    @sio.on('simulation_state')
    def on_simulation_state(data):
        print(f"Simulation state: {data}")
    
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
        
        # Get current simulation state
        print("Getting current simulation state...")
        sio.emit('get_simulation_state')
        time.sleep(1)
        
        # Start the simulation with obstacle avoidance
        print("Starting simulation...")
        sio.emit('start_simulation', {
            'scenario': 'obstacle',
            'controller': 'pid',
            'use_obstacle_avoidance': True
        })
        
        # Wait for simulation to start
        print("Waiting for simulation to initialize...")
        time.sleep(3)
        
        # Keep the script running to receive updates
        print("Simulation started! Press Ctrl+C to exit.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Stopping simulation...")
            sio.emit('stop_simulation')
            time.sleep(1)
            
        return True
    except Exception as e:
        print(f"Error starting simulation: {e}")
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
    
    success = start_simulation(port)
    sys.exit(0 if success else 1) 