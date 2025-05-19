#!/usr/bin/env python3
"""
Flask application that serves as the backend for the USV visualization system.
"""

from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import os
import sys
import yaml
import json
import numpy as np
import threading
import time
import socket

# Add the parent directory to the system path
sys.path.append(os.path.abspath(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))

from usv_system.simulation.simulator import USVSimulator
from usv_system.models.environment import Environment

# Default port and port range configuration
DEFAULT_PORT = 5540
MAX_PORT_ATTEMPTS = 100
PORT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'port.txt')

# Initialize Flask and SocketIO
app = Flask(__name__)
app.config['SECRET_KEY'] = 'usv_visualization_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global simulator and state
simulation_lock = threading.Lock()
simulator = None
simulation_thread = None
simulation_running = False
simulation_paused = False
current_scenario = 'waypoint'
current_controller = 'pid'

def load_config(config_path='../../config/default_config.yaml'):
    """Load configuration from YAML file."""
    try:
        with open(os.path.abspath(os.path.join(os.path.dirname(__file__), config_path)), 'r') as f:
            config = yaml.safe_load(f)
        return config
    except Exception as e:
        print(f"Error loading configuration: {e}")
        return None

def initialize_simulator(scenario='waypoint', controller_type='pid'):
    """Initialize the USV simulator with specified scenario and controller."""
    global simulator, current_scenario, current_controller
    
    config = load_config()
    if not config:
        return False
    
    # Map frontend scenario names to config scenario names if needed
    scenario_mapping = {
        'highspeed': 'waypoint',  # Map highspeed to waypoint
        'path': 'waypoint',       # Map path to waypoint
        'disturbance': 'waypoint' # Map disturbance to waypoint
    }
    
    # Use the mapped scenario name if it exists in the mapping
    actual_scenario = scenario_mapping.get(scenario, scenario)
    
    with simulation_lock:
        # Create simulator
        simulator = USVSimulator(
            dt=config['simulation']['dt'],
            simulation_time=config['simulation']['simulation_time'],
            x0=np.array(config['simulation']['initial_state']),
            vessel_params=config['vessel'],
            env_params=config['environment'],
            controller_type=controller_type,
            controller_params=config['controllers'][controller_type]
        )
        
        # Set reference trajectory
        simulator.set_reference_trajectory(config['scenarios'][actual_scenario]['reference_trajectory'])
        
        current_scenario = scenario
        current_controller = controller_type
        
    return True

def simulation_worker():
    """Worker function to run the simulation in a separate thread."""
    global simulator, simulation_running, simulation_paused
    
    if not simulator:
        print("ERROR: Simulator not initialized")
        socketio.emit('simulation_status', {'status': 'error', 'message': 'Simulator not initialized'})
        simulation_running = False
        return
    
    print(f"Starting simulation with scenario: {current_scenario}, controller: {current_controller}")
    
    # Number of steps
    dt = simulator.dt
    total_steps = int(simulator.simulation_time / dt)
    step = 0
    
    # Real-time factor adjustment
    real_time_factor = 1.0  # can be adjusted later
    
    simulation_running = True
    last_time = time.time()
    
    while simulation_running and step < total_steps:
        # Check if simulation is paused
        if simulation_paused:
            time.sleep(0.1)
            continue
        
        # Run simulation step
        with simulation_lock:
            simulator.simulate_step()
            step += 1
        
        # Extract current state and send to frontend
        state = simulator.usv_model.state.copy()
        control = simulator.control_history[-1].copy() if simulator.control_history else np.zeros(2)
        
        # Get reference data
        pos_ref, heading_ref, speed_ref = simulator.get_current_reference()
        
        # Get performance metrics
        path_error = simulator.path_error_history[-1] if simulator.path_error_history else 0
        heading_error = simulator.heading_error_history[-1] if simulator.heading_error_history else 0
        
        # Format data for frontend
        data = {
            'time': simulator.time,
            'state': {
                'x': float(state[0]),
                'y': float(state[1]),
                'heading': float(state[2]),
                'surge': float(state[3]),
                'sway': float(state[4]),
                'yaw_rate': float(state[5])
            },
            'control': {
                'thrust': float(control[0]),
                'moment': float(control[1])
            },
            'reference': {
                'x': float(pos_ref[0]),
                'y': float(pos_ref[1]),
                'heading': float(heading_ref),
                'speed': float(speed_ref)
            },
            'metrics': {
                'path_error': float(path_error),
                'heading_error': float(heading_error)
            },
            'progress': {
                'step': step,
                'total_steps': total_steps,
                'percentage': (step / total_steps) * 100
            }
        }
        
        # Print debug info every 10 steps
        if step % 10 == 0:
            print(f"Simulation step {step}/{total_steps} - Position: ({data['state']['x']:.2f}, {data['state']['y']:.2f})")
        
        # Send data to frontend
        socketio.emit('simulation_data', data)
        
        # Control simulation speed
        current_time = time.time()
        elapsed = current_time - last_time
        sleep_time = max(0, (dt / real_time_factor) - elapsed)
        time.sleep(sleep_time)
        last_time = time.time()
    
    # Simulation completed
    simulation_running = False
    socketio.emit('simulation_status', {'status': 'completed'})

@app.route('/')
def index():
    """Render the main visualization page."""
    return render_template('index.html')

@app.route('/api/scenarios')
def get_scenarios():
    """Get available scenarios."""
    config = load_config()
    if not config:
        return jsonify({'error': 'Failed to load configuration'}), 500
    
    scenarios = list(config['scenarios'].keys())
    return jsonify({'scenarios': scenarios})

@app.route('/api/controllers')
def get_controllers():
    """Get available controllers."""
    return jsonify({'controllers': ['pid', 'lqr', 'mpc']})

@app.route('/api/config', methods=['GET'])
def get_config():
    """Get current configuration."""
    config = load_config()
    if not config:
        return jsonify({'error': 'Failed to load configuration'}), 500
    
    return jsonify({
        'simulation': config['simulation'],
        'vessel': config['vessel'],
        'environment': config['environment'],
        'controllers': config['controllers'],
        'scenarios': config['scenarios']
    })

@socketio.on('connect')
def handle_connect():
    """Handle client connection."""
    print('Client connected')
    emit('connection_response', {'status': 'connected'})

@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection."""
    print('Client disconnected')

@socketio.on('start_simulation')
def handle_start_simulation(data):
    """Start the simulation with the specified parameters."""
    global simulation_thread, simulation_running, simulation_paused
    
    # Stop any existing simulation
    if simulation_running:
        simulation_running = False
        if simulation_thread:
            simulation_thread.join()
    
    # Extract parameters
    scenario = data.get('scenario', 'waypoint')
    controller = data.get('controller', 'pid')
    
    print(f"Handling start simulation request: scenario={scenario}, controller={controller}")
    
    # Initialize simulator
    success = initialize_simulator(scenario, controller)
    if not success:
        print(f"Failed to initialize simulator for scenario: {scenario}, controller: {controller}")
        emit('simulation_status', {'status': 'error', 'message': 'Failed to initialize simulator'})
        return
    
    print(f"Simulator successfully initialized for scenario: {scenario}, controller: {controller}")
    
    # Start simulation thread
    simulation_paused = False
    simulation_thread = threading.Thread(target=simulation_worker)
    simulation_thread.daemon = True
    simulation_thread.start()
    
    print(f"Simulation thread started for scenario: {scenario}, controller: {controller}")
    emit('simulation_status', {'status': 'started'})

@socketio.on('pause_simulation')
def handle_pause_simulation():
    """Pause the simulation."""
    global simulation_paused
    
    simulation_paused = True
    emit('simulation_status', {'status': 'paused'})

@socketio.on('resume_simulation')
def handle_resume_simulation():
    """Resume the simulation."""
    global simulation_paused
    
    simulation_paused = False
    emit('simulation_status', {'status': 'resumed'})

@socketio.on('stop_simulation')
def handle_stop_simulation():
    """Stop the simulation."""
    global simulation_running, simulation_thread
    
    simulation_running = False
    if simulation_thread:
        simulation_thread.join()
        simulation_thread = None
    
    emit('simulation_status', {'status': 'stopped'})

@socketio.on('get_simulation_state')
def handle_get_simulation_state():
    """Get the current simulation state."""
    if not simulator:
        emit('simulation_state', {'status': 'not_initialized'})
        return
    
    with simulation_lock:
        state = simulator.usv_model.state.copy()
    
    data = {
        'status': 'initialized',
        'running': simulation_running,
        'paused': simulation_paused,
        'state': {
            'x': float(state[0]),
            'y': float(state[1]),
            'heading': float(state[2]),
            'surge': float(state[3]),
            'sway': float(state[4]),
            'yaw_rate': float(state[5])
        },
        'scenario': current_scenario,
        'controller': current_controller
    }
    
    emit('simulation_state', data)

def find_available_port(start_port, max_attempts=MAX_PORT_ATTEMPTS):
    """
    Find an available port starting from start_port.
    
    Args:
        start_port (int): The port to start searching from
        max_attempts (int): Maximum number of ports to try
        
    Returns:
        int: An available port number
        
    Raises:
        RuntimeError: If no available port is found after max_attempts
    """
    for port in range(start_port, start_port + max_attempts):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.bind(('0.0.0.0', port))
            sock.close()
            return port
        except OSError:
            print(f"Port {port} is already in use, trying next port...")
            continue
    
    # If we get here, we couldn't find an available port
    raise RuntimeError(f"Could not find an available port after {max_attempts} attempts")

def save_port_to_file(port, filename=PORT_FILE):
    """
    Save the port number to a file for external reference.
    
    Args:
        port (int): The port number to save
        filename (str): The file to save the port number to
    """
    try:
        with open(filename, 'w') as f:
            f.write(str(port))
        return True
    except Exception as e:
        print(f"Warning: Could not save port to file: {e}")
        return False

def get_port_from_env_or_config():
    """
    Get port from environment variable, config, or use default.
    
    Returns:
        int: The port number to use
    """
    # Try to get port from environment variable
    try:
        env_port = os.environ.get('USV_VISUALIZATION_PORT')
        if env_port is not None:
            return int(env_port)
    except (ValueError, TypeError):
        print("Warning: Invalid port in environment variable, using default")
    
    # Try to get port from config file
    config = load_config()
    if config and 'visualization' in config and 'port' in config['visualization']:
        try:
            return int(config['visualization']['port'])
        except (ValueError, TypeError):
            print("Warning: Invalid port in config file, using default")
    
    # Return default port
    return DEFAULT_PORT

if __name__ == '__main__':
    try:
        # Get starting port from environment or config
        default_port = get_port_from_env_or_config()
        
        # Find available port
        port = find_available_port(default_port)
        
        # Save port to file for external applications
        save_port_to_file(port)
        
        # Log information
        if port != default_port:
            print(f"Default port {default_port} was in use, using port {port} instead")
        else:
            print(f"Starting server on port {port}")
        
        # Start server
        socketio.run(app, debug=True, host='0.0.0.0', port=port)
    except Exception as e:
        print(f"Error starting server: {e}")
        sys.exit(1) 