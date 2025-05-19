#!/usr/bin/env python3
"""
Script to fix visualization issues in the USV simulation by applying
various fixes and testing different approaches.
"""

import sys
import socketio
import time
import json
import argparse

def fix_visualization(port=9000, fix_type='all'):
    """
    Apply fixes to the visualization system.
    
    Args:
        port (int): Server port number
        fix_type (str): Type of fix to apply ('all', 'restart', 'toggle', 'reconnect')
    """
    print(f"Applying fix '{fix_type}' to visualization system on port {port}...")
    
    # Create a Socket.IO client
    sio = socketio.Client()
    
    # Track connection status
    connected = False
    simulation_status = None
    
    @sio.event
    def connect():
        nonlocal connected
        connected = True
        print("✓ Connected to server")
    
    @sio.event
    def connect_error(data):
        print(f"✗ Connection error: {data}")
    
    @sio.event
    def disconnect():
        nonlocal connected
        connected = False
        print("Disconnected from server")
    
    @sio.on('simulation_status')
    def on_simulation_status(data):
        nonlocal simulation_status
        simulation_status = data['status']
        print(f"✓ Simulation status: {data['status']}")
    
    try:
        # Connect to the server
        print("Connecting to server...")
        sio.connect(f'http://localhost:{port}')
        
        if not connected:
            print("✗ Failed to connect to server")
            return False
        
        # Get current state
        print("Checking current simulation state...")
        sio.emit('get_simulation_state')
        time.sleep(1)
        
        # Apply fixes based on fix_type
        if fix_type in ['all', 'restart']:
            print("\n[FIX] Stopping and restarting simulation...")
            sio.emit('stop_simulation')
            time.sleep(2)
            
            print("Starting new simulation...")
            sio.emit('start_simulation', {
                'scenario': 'obstacle',
                'controller': 'pid',
                'use_obstacle_avoidance': True
            })
            time.sleep(3)
        
        if fix_type in ['all', 'toggle']:
            print("\n[FIX] Toggling obstacle avoidance system...")
            sio.emit('toggle_obstacle_avoidance', {'enabled': False})
            time.sleep(1)
            sio.emit('toggle_obstacle_avoidance', {'enabled': True})
            time.sleep(1)
        
        if fix_type in ['all', 'reconnect']:
            print("\n[FIX] Forcing client reconnection...")
            # Disconnect and reconnect
            sio.disconnect()
            time.sleep(2)
            sio.connect(f'http://localhost:{port}')
            time.sleep(1)
            
            if not connected:
                print("✗ Failed to reconnect to server")
                return False
        
        # Check if simulation is running after fixes
        print("\nVerifying simulation state after fixes...")
        sio.emit('get_simulation_state')
        time.sleep(1)
        
        if not simulation_status or simulation_status not in ['started', 'running']:
            print("✗ Simulation not running. Starting simulation...")
            sio.emit('start_simulation', {
                'scenario': 'obstacle',
                'controller': 'pid',
                'use_obstacle_avoidance': True
            })
            time.sleep(3)
        
        print("\nAll fixes have been applied.")
        print("Please refresh your browser window if you're viewing the simulation.")
        print("If the visualization still doesn't work, try different fix types:")
        print("  python fix_visualization.py --fix restart")
        print("  python fix_visualization.py --fix toggle")
        print("  python fix_visualization.py --fix reconnect")
        
        return True
    except Exception as e:
        print(f"✗ Error during fix procedure: {e}")
        return False
    finally:
        try:
            sio.disconnect()
        except:
            pass

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Fix USV visualization issues")
    parser.add_argument("--port", type=int, default=9000, help="Server port number")
    parser.add_argument("--fix", type=str, default="all", 
                        choices=["all", "restart", "toggle", "reconnect"],
                        help="Type of fix to apply")
    args = parser.parse_args()
    
    success = fix_visualization(args.port, args.fix)
    sys.exit(0 if success else 1) 