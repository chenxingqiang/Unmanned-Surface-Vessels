#!/usr/bin/env python3
"""
Test script to send mock simulation data to the visualization frontend
"""

import sys
import os
import time
import json
import argparse
import threading
import numpy as np
from flask import Flask, render_template
from flask_socketio import SocketIO, emit

# Create a minimal Flask app for testing
app = Flask(__name__)
app.config['SECRET_KEY'] = 'usv_test_key'

# Improved Socket.IO configuration
socketio = SocketIO(
    app, 
    cors_allowed_origins="*",
    logger=True,
    engineio_logger=True,
    ping_timeout=60,
    ping_interval=25,
    async_mode='threading'
)

def generate_test_data(step, total_steps=100):
    """Generate test data for a simulation step"""
    t = step / 10.0  # time in seconds
    
    # Simple circular trajectory
    radius = 50.0
    x = radius * np.cos(t * 0.1)
    y = radius * np.sin(t * 0.1)
    heading = np.arctan2(-np.sin(t * 0.1), -np.cos(t * 0.1))
    
    # Create a data packet similar to the real simulation
    data = {
        'time': t,
        'state': {
            'x': float(x),
            'y': float(y),
            'heading': float(heading),
            'surge': 10.0,  # constant speed
            'sway': 0.0,
            'yaw_rate': 0.1
        },
        'control': {
            'thrust': 50.0,
            'moment': 5.0 * np.sin(t * 0.2)
        },
        'reference': {
            'x': float(x + 5 * np.cos(t * 0.3)),
            'y': float(y + 5 * np.sin(t * 0.3)),
            'heading': float(heading),
            'speed': 10.0
        },
        'metrics': {
            'path_error': float(5.0 * np.sin(t * 0.3)),
            'heading_error': float(0.1 * np.sin(t * 0.5))
        },
        'progress': {
            'step': step,
            'total_steps': total_steps,
            'percentage': (step / total_steps) * 100
        }
    }
    
    # Add static and dynamic obstacles
    data['obstacles'] = {
        'static': [
            {'x': 30.0, 'y': 30.0, 'radius': 5.0},
            {'x': -30.0, 'y': -30.0, 'radius': 8.0},
            {'x': -20.0, 'y': 40.0, 'radius': 4.0},
            {'x': 40.0, 'y': -20.0, 'radius': 6.0}
        ],
        'dynamic': [
            {
                'x': 20.0 + 10.0 * np.sin(t * 0.1),
                'y': 10.0 + 10.0 * np.cos(t * 0.1),
                'radius': 3.0,
                'heading': t * 0.1,
                'speed': 2.0
            },
            {
                'x': -10.0 + 15.0 * np.cos(t * 0.05),
                'y': 30.0 + 15.0 * np.sin(t * 0.05),
                'radius': 4.0,
                'heading': t * 0.05 + np.pi,
                'speed': 1.5
            }
        ]
    }
    
    return data

def test_data_thread(total_steps=100, interval=0.1):
    """Thread function to emit simulated data"""
    for step in range(1, total_steps + 1):
        # Check if we need to stop
        if getattr(test_data_thread, "stop", False):
            print("Test data thread stopped")
            break
            
        data = generate_test_data(step, total_steps)
        socketio.emit('simulation_data', data)
        print(f"Emitted step {step}/{total_steps} - Position: ({data['state']['x']:.1f}, {data['state']['y']:.1f})")
        
        # If step is 1, emit a simulation status "started" event
        if step == 1:
            socketio.emit('simulation_status', {'status': 'started'})
            print("Simulation started event emitted")
        
        # If we're at the last step, emit a "completed" event
        if step == total_steps:
            socketio.emit('simulation_status', {'status': 'completed'})
            print("Simulation completed event emitted")
        
        # Sleep between emits
        time.sleep(interval)

# Global variable to track test thread
active_test_thread = None

@app.route('/')
def index():
    """Render a test page"""
    return render_template('test_socket.html', port=socketio.server.eio.port)

@app.route('/test_socket.html')
def test_socket_html():
    """Provide a simple test socket page"""
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Socket.IO Test Server</title>
        <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
        <style>
            body { font-family: Arial, sans-serif; margin: 20px; }
            .log { height: 300px; overflow-y: scroll; border: 1px solid #ccc; padding: 10px; margin-bottom: 20px; }
            button { padding: 10px; margin-right: 10px; }
        </style>
    </head>
    <body>
        <h1>Test Socket.IO Server</h1>
        <div class="log" id="log"></div>
        <button id="send-test-data">Send Test Data</button>
        <button id="connect-btn">Connect</button>
        <button id="disconnect-btn">Disconnect</button>
        
        <script>
            const logEl = document.getElementById('log');
            let socket;
            
            function log(msg) {
                const entry = document.createElement('div');
                entry.textContent = new Date().toLocaleTimeString() + ': ' + msg;
                logEl.appendChild(entry);
                logEl.scrollTop = logEl.scrollHeight;
            }
            
            function connectSocket() {
                log('Connecting...');
                socket = io({
                    transports: ['websocket', 'polling'],
                    reconnectionAttempts: 5,
                    reconnectionDelay: 1000
                });
                
                socket.on('connect', () => {
                    log('Connected! Socket ID: ' + socket.id);
                });
                
                socket.on('disconnect', () => {
                    log('Disconnected');
                });
                
                socket.on('simulation_data', (data) => {
                    log('Received simulation data for time: ' + data.time);
                });
                
                socket.on('simulation_status', (data) => {
                    log('Simulation status: ' + data.status);
                });
            }
            
            document.getElementById('connect-btn').addEventListener('click', connectSocket);
            document.getElementById('disconnect-btn').addEventListener('click', () => {
                if (socket) {
                    socket.disconnect();
                    log('Manually disconnected');
                }
            });
            
            document.getElementById('send-test-data').addEventListener('click', () => {
                if (socket && socket.connected) {
                    socket.emit('start_simulation', { scenario: 'test', controller: 'test' });
                    log('Sent test data request');
                } else {
                    log('Not connected! Cannot send data.');
                }
            });
            
            // Connect automatically
            connectSocket();
        </script>
    </body>
    </html>
    """

@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print(f'Client connected: {request.sid}')
    emit('connection_response', {'status': 'connected'})

@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection"""
    print(f'Client disconnected: {request.sid}')

@socketio.on('start_simulation')
def handle_start_simulation(data):
    """Handle start simulation request"""
    global active_test_thread
    
    print(f"Received start simulation request: {data}")
    
    # Stop any existing test thread
    if active_test_thread and active_test_thread.is_alive():
        setattr(active_test_thread, "stop", True)
        active_test_thread.join()
    
    # Reset stop flag
    setattr(test_data_thread, "stop", False)
    
    # Configure simulation parameters
    total_steps = 1000  # Increased for longer simulation
    interval = 0.1  # 10 Hz update rate
    
    # Start new test thread
    active_test_thread = threading.Thread(
        target=test_data_thread,
        args=(total_steps, interval),
        daemon=True
    )
    active_test_thread.start()
    
    # Emit initial simulation state
    socketio.emit('simulation_state', {
        'status': 'initialized',
        'scenario': data.get('scenario', 'test'),
        'controller': data.get('controller', 'test'),
        'obstacle_avoidance': data.get('use_obstacle_avoidance', True),
        'running': True,
        'paused': False
    })
    
    print("Test data thread started")

@socketio.on('get_simulation_state')
def handle_get_simulation_state():
    """Handle simulation state request"""
    emit('simulation_state', {
        'status': 'initialized',
        'scenario': 'test',
        'controller': 'test',
        'obstacle_avoidance': True
    })

@socketio.on('stop_simulation')
def handle_stop_simulation():
    """Handle stop simulation request"""
    global active_test_thread
    
    print("Received stop simulation request")
    
    # Stop test thread if running
    if active_test_thread and active_test_thread.is_alive():
        setattr(active_test_thread, "stop", True)
        active_test_thread.join(timeout=1.0)
        active_test_thread = None
    
    # Respond with stopped status
    emit('simulation_status', {'status': 'stopped'})

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test socket events for USV visualization')
    parser.add_argument('-p', '--port', type=int, default=9090, help='Port to run the test server on')
    parser.add_argument('--debug', action='store_true', help='Run in debug mode')
    args = parser.parse_args()
    
    print(f"Starting test socket server on port {args.port}")
    print("Connect to the visualization app in another window")
    print("Or visit http://localhost:{args.port}/ for a simple test interface")
    print("Press Ctrl+C to stop")
    
    socketio.run(app, host='0.0.0.0', port=args.port, debug=args.debug) 