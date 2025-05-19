#!/usr/bin/env python3
"""
Debug version of the Flask application for USV visualization system with enhanced logging
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
import logging
import argparse
import traceback

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('debug.log')
    ]
)
logger = logging.getLogger('usv_visualization')

# Add the parent directory to the system path
sys.path.append(os.path.abspath(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))

try:
    from usv_system.simulation.simulator import USVSimulator
    from usv_system.models.environment import Environment
    from usv_system.obstacle_avoidance.avoidance_strategy import APFAvoidanceStrategy, COLREGAvoidanceStrategy, ObstacleAvoidanceSystem
    from usv_system.obstacle_avoidance.detector import LidarDetector, RadarDetector
    logger.info("Successfully imported simulator modules")
except Exception as e:
    logger.error(f"Failed to import simulator modules: {e}")

# Default port and port range configuration
DEFAULT_PORT = 5540
MAX_PORT_ATTEMPTS = 100
PORT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'port.txt')

# Initialize Flask and SocketIO
app = Flask(__name__)
app.config['SECRET_KEY'] = 'usv_visualization_secret_key'

# Improved Socket.IO configuration with CORS handling
socketio = SocketIO(
    app, 
    cors_allowed_origins="*",  # Allow connections from any origin
    logger=True, 
    engineio_logger=True,
    ping_timeout=60,
    ping_interval=25,
    async_mode='threading',
    always_connect=True,
    path='/socket.io'  # Explicit path
)

# Global simulator and state
simulation_lock = threading.Lock()
simulator = None
obstacle_avoidance_system = None
simulation_thread = None
simulation_running = False
simulation_paused = False
current_scenario = 'waypoint'
current_controller = 'pid'
use_obstacle_avoidance = True
simulation_step = 0

def load_config(config_path='../../config/default_config.yaml'):
    """Load configuration from YAML file."""
    try:
        with open(os.path.abspath(os.path.join(os.path.dirname(__file__), config_path)), 'r') as f:
            config = yaml.safe_load(f)
        logger.info(f"Successfully loaded configuration from {config_path}")
        return config
    except Exception as e:
        logger.error(f"Error loading configuration: {e}")
        return None

def initialize_simulator(scenario='waypoint', controller_type='pid', use_avoidance=True):
    """Initialize the USV simulator with specified scenario, controller, and obstacle avoidance."""
    global simulator, obstacle_avoidance_system, current_scenario, current_controller, use_obstacle_avoidance
    
    logger.info(f"Initializing simulator with scenario={scenario}, controller={controller_type}, obstacle_avoidance={use_avoidance}")
    
    config = load_config()
    if not config:
        logger.error("Failed to load configuration")
        return False
    
    with simulation_lock:
        try:
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
            simulator.set_reference_trajectory(config['scenarios'][scenario]['reference_trajectory'])
            
            # Set up obstacle avoidance if enabled
            use_obstacle_avoidance = use_avoidance
            if use_obstacle_avoidance:
                logger.debug("Setting up obstacle avoidance")
                # Create obstacle avoidance components
                apf_strategy = APFAvoidanceStrategy(
                    safety_distance=8.0,
                    attractive_gain=1.0,
                    repulsive_gain=150.0,
                    influence_distance=25.0
                )
                
                colreg_strategy = COLREGAvoidanceStrategy(
                    safety_distance=12.0,
                    attractive_gain=1.0,
                    repulsive_gain=120.0,
                    influence_distance=30.0
                )
                
                # Initialize obstacle avoidance system
                from usv_system.navigation.local_planner import DynamicWindowPlanner
                
                # Create DWA planner with default parameters
                dwa_planner = DynamicWindowPlanner(
                    model_params={
                        'max_speed': 5.0,
                        'min_speed': 0.0,
                        'max_yawrate': 1.0,  # rad/s
                        'max_accel': 2.0,    # m/s^2
                        'max_dyawrate': 1.0, # rad/s^2
                        'robot_radius': 3.0  # m
                    },
                    cost_params={
                        'heading': 1.0,
                        'dist': 1.0,
                        'velocity': 1.0
                    },
                    window_params={
                        'v_reso': 0.1,       # m/s
                        'yawrate_reso': 0.05  # rad/s
                    },
                    dt=simulator.dt,
                    predict_time=3.0
                )
                
                # Create hybrid strategy
                from usv_system.obstacle_avoidance.avoidance_strategy import HybridAvoidanceStrategy
                hybrid_strategy = HybridAvoidanceStrategy(
                    dwa_planner=dwa_planner,
                    apf_strategy=apf_strategy,
                    colreg_strategy=colreg_strategy,
                    usv_model=simulator.usv_model
                )
                
                # Set up optimal weights for scenario
                hybrid_strategy.set_strategy_weights({
                    'dwa': 0.3,
                    'apf': 0.3,
                    'colreg': 0.4   # Emphasize COLREG compliance
                })
                
                # Create avoidance system
                obstacle_avoidance_system = ObstacleAvoidanceSystem(
                    avoidance_strategy=hybrid_strategy,
                    detection_range=100.0,
                    visualization_enabled=True
                )
                
                # Extract obstacles from environment config
                obstacles_config = config['scenarios'][scenario].get('environment', {}).get('obstacles', {})
                static_obstacles = []
                dynamic_obstacles = []
                
                if obstacles_config and obstacles_config.get('enabled', False):
                    # Process static obstacles
                    for obs in obstacles_config.get('static', []):
                        if isinstance(obs, dict) and 'position' in obs and 'radius' in obs:
                            pos = obs['position']
                            radius = obs['radius']
                            static_obstacles.append((pos[0], pos[1], radius))
                    
                    # Process dynamic obstacles
                    for obs in obstacles_config.get('dynamic', []):
                        if isinstance(obs, dict) and 'initial_position' in obs:
                            dyn_obs = {
                                'position': tuple(obs['initial_position']),
                                'heading': obs.get('heading', 0.0),
                                'speed': obs.get('speed', 1.0),
                                'radius': obs.get('radius', 3.0)
                            }
                            dynamic_obstacles.append(dyn_obs)
                
                # Update obstacle avoidance system with obstacles
                obstacle_avoidance_system.update_obstacles(static_obstacles, dynamic_obstacles)
                
                logger.info(f"Initialized obstacle avoidance with {len(static_obstacles)} static and {len(dynamic_obstacles)} dynamic obstacles")
            
            current_scenario = scenario
            current_controller = controller_type
            
            logger.info("Simulator initialization successful")
            return True
        except Exception as e:
            logger.error(f"Error initializing simulator: {e}")
            return False

def simulation_worker():
    """Worker thread for running the simulation."""
    global simulation_running, simulation_paused, simulation_step, simulation_thread
    
    try:
        # Initialize simulation metrics
        metrics = {'path_error': 0.0, 'heading_error': 0.0}
        
        # Send simulation started status
        logger.info(f"Starting simulation with scenario '{current_scenario}' and controller '{current_controller}'")
        socketio.emit('simulation_status', {'status': 'started'})
        
        # Main simulation loop
        while simulation_running:
            # Check if paused
            if simulation_paused:
                time.sleep(0.1)
                continue
            
            # Run simulation step
            with simulation_lock:
                try:
                    # Apply obstacle avoidance if enabled
                    if use_obstacle_avoidance and obstacle_avoidance_system:
                        # Get current reference
                        pos_ref, heading_ref, speed_ref = simulator.get_current_reference()
                        
                        # Call the correct method to compute avoidance controls
                        thrust, moment = obstacle_avoidance_system.compute_avoidance_controls(
                            simulator.usv_model.state,
                            heading_ref,
                            speed_ref
                        )
                        
                        # Create obstacle data dictionary
                        obstacle_data = {
                            'avoidance_control': {
                                'thrust': thrust,
                                'moment': moment
                            }
                        }
                        
                        # Add visualization data if available
                        if hasattr(obstacle_avoidance_system, 'get_visualization_data'):
                            obstacle_data['visualization'] = obstacle_avoidance_system.get_visualization_data()
                    else:
                        obstacle_data = None
                    
                    # Run simulation step
                    result = simulator.simulate_step()
                    
                    # Create result dictionary with simulation data
                    result = {
                        'completed': False,
                        'data': {
                            'time': simulator.time,
                            'state': {
                                'x': float(simulator.usv_model.state[0]),
                                'y': float(simulator.usv_model.state[1]),
                                'heading': float(simulator.usv_model.state[2]),
                                'surge': float(simulator.usv_model.state[3]),
                                'sway': float(simulator.usv_model.state[4]),
                                'yaw_rate': float(simulator.usv_model.state[5])
                            },
                            'control': {
                                'thrust': float(simulator.control_history[-1][0]) if simulator.control_history else 0.0,
                                'moment': float(simulator.control_history[-1][1]) if simulator.control_history else 0.0
                            }
                        },
                        'obstacles': obstacle_data if obstacle_data else {},
                        'step_start_time': time.time()
                    }
                    
                    # Get reference data
                    pos_ref, heading_ref, speed_ref = simulator.get_current_reference()
                    result['data']['reference'] = {
                        'x': float(pos_ref[0]),
                        'y': float(pos_ref[1]),
                        'heading': float(heading_ref),
                        'speed': float(speed_ref)
                    }
                    
                    # Check if simulation is complete
                    if result.get('completed', False):
                        logger.info("Simulation completed")
                        socketio.emit('simulation_status', {'status': 'completed'})
                        simulation_running = False
                        break
                    
                    # Extract data from simulation result
                    simulator_data = result.get('data', {})
                    obstacles = result.get('obstacles', {})
                    completion = result.get('completion', 0.0)
                    
                    # Calculate path error metric
                    if 'state' in simulator_data and 'reference' in simulator_data:
                        state = simulator_data['state']
                        reference = simulator_data['reference']
                        
                        if all(k in state for k in ['x', 'y']) and all(k in reference for k in ['x', 'y']):
                            dx = state['x'] - reference['x']
                            dy = state['y'] - reference['y']
                            metrics['path_error'] = np.sqrt(dx**2 + dy**2)
                        
                        if all(k in state for k in ['heading']) and all(k in reference for k in ['heading']):
                            # Calculate heading error and normalize to [-pi, pi]
                            heading_error = state['heading'] - reference['heading']
                            metrics['heading_error'] = ((heading_error + np.pi) % (2 * np.pi)) - np.pi
                    
                    # Create packet with simulation data
                    packet = {
                        'time': simulator_data.get('time', 0.0),
                        'state': simulator_data.get('state', {}),
                        'reference': simulator_data.get('reference', {}),
                        'control': simulator_data.get('control', {}),
                        'obstacles': obstacles,
                        'metrics': metrics,
                        'progress': {
                            'step': simulation_step,
                            'percentage': completion * 100.0
                        }
                    }
                    
                    # Emit simulation data to clients
                    socketio.emit('simulation_data', packet)
                    
                    # Increment step counter
                    simulation_step += 1
                    
                    # Adaptive sleep to maintain desired update rate
                    time.sleep(max(0.01, 0.1 - (time.time() - result.get('step_start_time', time.time()))))
                    
                except Exception as e:
                    logger.error(f"Error in simulation step: {str(e)}")
                    # Send error to clients
                    socketio.emit('simulation_status', {
                        'status': 'error',
                        'message': str(e),
                        'details': traceback.format_exc()
                    })
                    # Continue the simulation if possible
                    time.sleep(0.5)  # Slow down if there are errors
            
    except Exception as e:
        logger.error(f"Simulation worker error: {str(e)}")
        logger.error(traceback.format_exc())
        socketio.emit('simulation_status', {
            'status': 'error',
            'message': f"Simulation terminated due to error: {str(e)}",
            'details': traceback.format_exc()
        })
    finally:
        # Ensure cleanup happens
        logger.info(f"Simulation completed after {simulation_step} steps")
        socketio.emit('simulation_status', {'status': 'completed'})
        simulation_running = False
        simulation_paused = False

@app.route('/')
def index():
    """Render the main visualization page."""
    logger.info("Serving debug index page")
    return render_template('debug_index.html')

@app.route('/favicon.ico')
def favicon():
    """Serve the favicon."""
    return app.send_static_file('favicon.ico')

@app.route('/api/scenarios')
def get_scenarios():
    """Get available scenarios."""
    logger.info("API request: get scenarios")
    config = load_config()
    if not config:
        logger.error("Failed to load configuration for scenarios")
        return jsonify({'error': 'Failed to load configuration'}), 500
    
    scenarios = list(config['scenarios'].keys())
    logger.info(f"Available scenarios: {scenarios}")
    return jsonify({'scenarios': scenarios})

@app.route('/api/controllers')
def get_controllers():
    """Get available controllers."""
    logger.info("API request: get controllers")
    return jsonify({'controllers': ['pid', 'lqr', 'mpc']})

@app.route('/api/config', methods=['GET'])
def get_config():
    """Get current configuration."""
    logger.info("API request: get config")
    config = load_config()
    if not config:
        logger.error("Failed to load configuration for API request")
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
    logger.info(f"Client connected: {request.sid}")
    emit('connection_response', {'status': 'connected'})

@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection."""
    logger.info(f"Client disconnected: {request.sid}")

@socketio.on('start_simulation')
def handle_start_simulation(data):
    """Start the simulation with the specified parameters."""
    global simulation_thread, simulation_running, simulation_paused, simulation_step
    global current_scenario, current_controller, use_obstacle_avoidance
    
    logger.info(f"Received start_simulation request: {data}")
    
    # Stop any existing simulation
    if simulation_running:
        logger.info("Stopping existing simulation before starting new one")
        simulation_running = False
        if simulation_thread and simulation_thread.is_alive():
            simulation_thread.join(timeout=2.0)
    
    # Extract parameters
    scenario = data.get('scenario', 'waypoint')
    # Map 'path' to 'waypoint' for backward compatibility
    if scenario == 'path':
        scenario = 'waypoint'
        logger.info("Mapped 'path' scenario to 'waypoint' for compatibility")
    
    controller = data.get('controller', 'pid')
    use_obstacle_avoidance = data.get('use_obstacle_avoidance', True)
    
    # Update current settings
    current_scenario = scenario
    current_controller = controller
    
    # Initialize simulator
    if initialize_simulator(scenario, controller):
        # Reset simulation step counter
        simulation_step = 0
        
        # Start simulation thread
        simulation_running = True
        simulation_paused = False
        simulation_thread = threading.Thread(target=simulation_worker)
        simulation_thread.daemon = True
        simulation_thread.start()
        
        return {'status': 'started'}
    else:
        return {'status': 'error', 'message': 'Failed to initialize simulator'}

@socketio.on('pause_simulation')
def handle_pause_simulation():
    """Pause the simulation."""
    global simulation_paused
    
    logger.info("Received pause_simulation request")
    simulation_paused = True
    emit('simulation_status', {'status': 'paused'})

@socketio.on('resume_simulation')
def handle_resume_simulation():
    """Resume the simulation."""
    global simulation_paused
    
    logger.info("Received resume_simulation request")
    simulation_paused = False
    emit('simulation_status', {'status': 'resumed'})

@socketio.on('stop_simulation')
def handle_stop_simulation():
    """Stop the simulation."""
    global simulation_running, simulation_thread
    
    logger.info("Received stop_simulation request")
    simulation_running = False
    if simulation_thread and simulation_thread.is_alive():
        simulation_thread.join(timeout=2.0)
        simulation_thread = None
    
    emit('simulation_status', {'status': 'stopped'})

@socketio.on('get_simulation_state')
def handle_get_simulation_state():
    """Get the current simulation state."""
    logger.info("Received get_simulation_state request")
    
    if not simulator:
        logger.info("No simulator initialized yet")
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
        'controller': current_controller,
        'obstacle_avoidance': use_obstacle_avoidance
    }
    
    logger.debug(f"Sending simulation state: {data}")
    emit('simulation_state', data)

@socketio.on('toggle_obstacle_avoidance')
def handle_toggle_obstacle_avoidance(data):
    """Toggle obstacle avoidance on/off."""
    global use_obstacle_avoidance
    use_obstacle_avoidance = data.get('enabled', True)
    logger.info(f"Toggling obstacle avoidance: {use_obstacle_avoidance}")
    emit('obstacle_avoidance_status', {'enabled': use_obstacle_avoidance})

@socketio.on_error()
def handle_error(e):
    """Handle socket.io errors."""
    logger.error(f"SocketIO error: {str(e)}")
    socketio.emit('error', {'message': f'Server error: {str(e)}'})

@socketio.on_error_default
def handle_default_error(e):
    """Handle socket.io errors on namespaces."""
    logger.error(f"SocketIO namespace error: {str(e)}")
    socketio.emit('error', {'message': f'Server namespace error: {str(e)}'})

# Testing route
@app.route('/test_socket')
def test_socket():
    """Test page for Socket.IO connection."""
    return """
    <html>
    <head>
        <title>Socket.IO Test</title>
        <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
        <script>
            document.addEventListener('DOMContentLoaded', function() {
                const socket = io({
                    transports: ['websocket', 'polling'],
                    reconnectionAttempts: 5,
                    reconnectionDelay: 1000
                });
                
                socket.on('connect', function() {
                    document.getElementById('status').innerText = 'Connected';
                    document.getElementById('status').style.color = 'green';
                });
                
                socket.on('disconnect', function() {
                    document.getElementById('status').innerText = 'Disconnected';
                    document.getElementById('status').style.color = 'red';
                });
                
                socket.on('connection_response', function(data) {
                    document.getElementById('events').innerHTML += '<p>Received connection response: ' + JSON.stringify(data) + '</p>';
                });
                
                document.getElementById('test-btn').addEventListener('click', function() {
                    socket.emit('get_simulation_state');
                    document.getElementById('events').innerHTML += '<p>Sent get_simulation_state event</p>';
                });
            });
        </script>
    </head>
    <body>
        <h1>Socket.IO Test</h1>
        <p>Status: <span id="status">Connecting...</span></p>
        <button id="test-btn">Test Connection</button>
        <div id="events"></div>
    </body>
    </html>
    """

def find_available_port(start_port, max_attempts=MAX_PORT_ATTEMPTS):
    """Find an available port starting from start_port."""
    for port in range(start_port, start_port + max_attempts):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.bind(('0.0.0.0', port))
            sock.close()
            logger.info(f"Found available port: {port}")
            return port
        except OSError:
            logger.debug(f"Port {port} is already in use, trying next port...")
            continue
    
    # If we get here, we couldn't find an available port
    logger.error(f"Could not find an available port after {max_attempts} attempts")
    raise RuntimeError(f"Could not find an available port after {max_attempts} attempts")

def save_port_to_file(port, filename=PORT_FILE):
    """Save the port number to a file for external reference."""
    try:
        with open(filename, 'w') as f:
            f.write(str(port))
        logger.info(f"Saved port {port} to file {filename}")
        return True
    except Exception as e:
        logger.error(f"Could not save port to file: {e}")
        return False

def get_port_from_env_or_config():
    """Get port from environment variable, config, or use default."""
    # Try to get port from environment variable
    try:
        env_port = os.environ.get('USV_VISUALIZATION_PORT')
        if env_port is not None:
            port = int(env_port)
            logger.info(f"Using port {port} from environment variable")
            return port
    except (ValueError, TypeError):
        logger.warning("Invalid port in environment variable, using default")
    
    # Try to get port from config file
    config = load_config()
    if config and 'visualization' in config and 'port' in config['visualization']:
        try:
            port = int(config['visualization']['port'])
            logger.info(f"Using port {port} from config file")
            return port
        except (ValueError, TypeError):
            logger.warning("Invalid port in config file, using default")
    
    # Return default port
    logger.info(f"Using default port {DEFAULT_PORT}")
    return DEFAULT_PORT

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='USV Visualization Server (Debug Version)')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT, help=f'Default port to run the server on (default: {DEFAULT_PORT})')
    parser.add_argument('--debug', action='store_true', help='Run in debug mode')
    parser.add_argument('--obstacles', action='store_true', help='Enable obstacle avoidance by default')
    args = parser.parse_args()
    
    # Find available port
    available_port = find_available_port(args.port)
    if available_port != args.port:
        logger.warning(f"Default port {args.port} was in use, using port {available_port} instead")
    
    # Save port to file for client access
    save_port_to_file(available_port)
    
    logger.info(f"Starting USV Visualization Server (Debug Version) on port {available_port}")
    socketio.run(app, debug=args.debug, host='0.0.0.0', port=available_port) 