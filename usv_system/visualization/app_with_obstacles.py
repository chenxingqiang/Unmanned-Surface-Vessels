#!/usr/bin/env python3
"""
Enhanced Flask application that serves as the backend for the USV visualization system
with obstacle avoidance integration.
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
import argparse

# Add the parent directory to the system path
sys.path.append(os.path.abspath(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))

from usv_system.simulation.simulator import USVSimulator
from usv_system.models.environment import Environment
from usv_system.obstacle_avoidance.avoidance_strategy import APFAvoidanceStrategy, COLREGAvoidanceStrategy, ObstacleAvoidanceSystem
from usv_system.obstacle_avoidance.detector import LidarDetector, RadarDetector
from usv_system.obstacle_avoidance.avoidance_strategy import HybridAvoidanceStrategy
from usv_system.navigation.local_planner import DynamicWindowPlanner

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
obstacle_avoidance_system = None
simulation_thread = None
simulation_running = False
simulation_paused = False
current_scenario = 'obstacle'  # Default to obstacle scenario
current_controller = 'pid'
use_obstacle_avoidance = True

def load_config(config_path=None):
    """Load configuration from YAML file."""
    # First try from environment variable
    if config_path is None:
        config_path = os.environ.get('USV_CONFIG_PATH', '../../config/default_config.yaml')
    
    try:
        with open(os.path.abspath(os.path.join(os.path.dirname(__file__), config_path)), 'r') as f:
            config = yaml.safe_load(f)
        return config
    except Exception as e:
        print(f"Error loading configuration: {e}")
        return None

def initialize_simulator(scenario='obstacle', controller_type='pid', use_avoidance=True):
    """Initialize the USV simulator with specified scenario, controller, and obstacle avoidance."""
    global simulator, obstacle_avoidance_system, current_scenario, current_controller, use_obstacle_avoidance
    
    config = load_config()
    if not config:
        return False
    
    # Map frontend scenario names to config scenario names if needed
    scenario_mapping = {
        'highspeed': 'waypoint',  # Map highspeed to waypoint as a fallback
        'path': 'waypoint',       # Map path to waypoint as a fallback
        'disturbance': 'waypoint' # Map disturbance to waypoint as a fallback
    }
    
    # Use the mapped scenario name if it exists in the mapping
    original_scenario = scenario
    config_scenario = scenario_mapping.get(scenario, scenario)
    
    # Make sure we have a valid scenario in the config
    if config_scenario not in config['scenarios']:
        print(f"Warning: Scenario '{config_scenario}' not found in config, using default")
        config_scenario = next(iter(config['scenarios'].keys()))
        
    # Initialize simulator with the specified scenario and controller
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
    simulator.set_reference_trajectory(config['scenarios'][config_scenario]['reference_trajectory'])
    
    # Save for UI state
    use_obstacle_avoidance = use_avoidance
    
    # Initialize obstacles for obstacle avoidance if needed
    if use_avoidance:
        # Get scenario config
        scenario_config = config['scenarios'][config_scenario]
        
        # Load obstacle configuration
        obstacles_config = scenario_config.get('obstacles', {})
        static_obstacles = []
        dynamic_obstacles = []
        
        # Create obstacle avoidance system if not exists
        if obstacle_avoidance_system is None:
            try:
                # We'll initialize our obstacle avoidance system with the basic strategies
                # In a real application, these would be more sophisticated
                
                # Get the USV model for dynamics
                usv_model = simulator.usv_model
                
                # Create basic strategies
                apf_strategy = APFAvoidanceStrategy(
                    safety_distance=10.0,
                    attractive_gain=1.0,
                    repulsive_gain=100.0,
                    influence_distance=30.0
                )
                
                colreg_strategy = COLREGAvoidanceStrategy(
                    safety_distance=12.0,
                    attractive_gain=1.0,
                    repulsive_gain=120.0,
                    influence_distance=40.0
                )
                
                # Create dynamic window planner if needed
                try:
                    from usv_system.navigation.local_planner import DynamicWindowPlanner
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
                        dt=0.1,
                        predict_time=3.0
                    )
                except ImportError:
                    dwa_planner = None
                
                # Create hybrid strategy with proper arguments
                hybrid_strategy = HybridAvoidanceStrategy(
                    dwa_planner=dwa_planner,
                    apf_strategy=apf_strategy,
                    colreg_strategy=colreg_strategy,
                    usv_model=None  # We don't have access to the model here
                )
                
                # Create the obstacle avoidance system with visualization enabled
                obstacle_avoidance_system = ObstacleAvoidanceSystem(
                    avoidance_strategy=hybrid_strategy,
                    detection_range=100.0,
                    visualization_enabled=True
                )
                
                print("Created obstacle avoidance system successfully")
            except Exception as e:
                print(f"Error creating obstacle avoidance system: {e}")
                import traceback
                traceback.print_exc()
                return False
        
        # Process static obstacles
        if isinstance(obstacles_config.get('static', []), list):
            for obs in obstacles_config.get('static', []):
                if isinstance(obs, dict) and 'position' in obs:
                    pos = obs['position']
                    radius = obs.get('radius', 3.0)
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
        
        print(f"Initialized obstacle avoidance with {len(static_obstacles)} static and {len(dynamic_obstacles)} dynamic obstacles")
    
    # Store the original scenario name for frontend display
    current_scenario = original_scenario
    current_controller = controller_type
    
    return True

def simulation_worker():
    """Worker function to run the simulation in a separate thread."""
    global simulator, simulation_running, simulation_paused, obstacle_avoidance_system
    
    try:
        if not simulator:
            error_msg = "Simulator not initialized"
            print(f"ERROR: {error_msg}")
            socketio.emit('simulation_status', {'status': 'error', 'message': error_msg})
            simulation_running = False
            return
        
        # Number of steps
        dt = simulator.dt
        total_steps = int(simulator.simulation_time / dt)
        step = 0
        
        # Real-time factor adjustment - can be controlled by the frontend
        real_time_factor = 1.0
        
        simulation_running = True
        print(f"Simulation worker starting... total_steps={total_steps}, dt={dt}")
        socketio.emit('simulation_status', {'status': 'started'})
        last_time = time.time()
        
        # For testing, create initial state with dummy data if no simulation
        # This ensures charts and visualization have data even at startup
        if simulator.time_history is None or len(simulator.time_history) == 0:
            simulator.time_history = [0.0]
            simulator.state_history = [simulator.usv_model.state.copy()]
            simulator.control_history = [np.zeros(2)]
            simulator.path_error_history = [0.0]
            simulator.heading_error_history = [0.0]
        
        while simulation_running and step < total_steps:
            # Check if simulation is paused
            if simulation_paused:
                # Even when paused, we should emit the current state data to keep UI alive
                # This ensures the frontend receives updates even when paused
                emit_current_state()
                time.sleep(0.5)  # Slower rate when paused
                continue
            
            try:
                # Run simulation step with or without obstacle avoidance
                with simulation_lock:
                    if use_obstacle_avoidance and obstacle_avoidance_system:
                        # Get current state
                        current_state = simulator.usv_model.state.copy()
                        
                        # Get reference data for this step
                        pos_ref, heading_ref, speed_ref = simulator.get_current_reference()
                        
                        # Compute avoidance controls using obstacle avoidance system
                        thrust, moment = obstacle_avoidance_system.compute_avoidance_controls(
                            current_state, 
                            heading_ref, 
                            speed_ref
                        )
                        
                        # Skip regular controller and directly apply avoidance controls
                        # Note: This bypasses the default controller in simulate_step
                        simulator.usv_model.update([thrust, moment])
                        
                        # Update time and store state/control data
                        simulator.time += simulator.dt
                        simulator.time_history.append(simulator.time)
                        simulator.state_history.append(simulator.usv_model.state.copy())
                        simulator.control_history.append(np.array([thrust, moment]))
                        
                        # Calculate and store performance metrics
                        simulator.path_error_history.append(
                            np.linalg.norm([current_state[0] - pos_ref[0], current_state[1] - pos_ref[1]])
                        )
                        
                        heading_error = heading_ref - current_state[2]
                        heading_error = ((heading_error + np.pi) % (2 * np.pi)) - np.pi  # Normalize to [-pi, pi]
                        simulator.heading_error_history.append(heading_error)
                    else:
                        # Use standard simulation step with built-in controller
                        simulator.simulate_step()
                    
                    step += 1
                
                # Emit the state data to clients
                emit_current_state()
                
                # Control simulation speed
                current_time = time.time()
                elapsed = current_time - last_time
                sleep_time = max(0, (dt / real_time_factor) - elapsed)
                time.sleep(sleep_time)
                last_time = time.time()
                
            except Exception as e:
                print(f"Error in simulation step {step}: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.5)  # Prevent fast error loops
        
        # Simulation completed
        simulation_running = False
        socketio.emit('simulation_status', {'status': 'completed'})
        print("Simulation completed successfully")
        
    except Exception as e:
        error_msg = f"Error in simulation thread: {str(e)}"
        print(f"ERROR: {error_msg}")
        import traceback
        traceback.print_exc()
        simulation_running = False
        socketio.emit('simulation_status', {'status': 'error', 'message': error_msg})

def emit_current_state():
    """Extract current state and send it to all clients."""
    global simulator, obstacle_avoidance_system, use_obstacle_avoidance, simulation_running
    
    if not simulator:
        print("ERROR: Simulator not initialized in emit_current_state")
        return
        
    # Extract current state
    try:
        state = simulator.usv_model.state.copy()
        control = simulator.control_history[-1].copy() if simulator.control_history else np.zeros(2)
        
        # Get reference data
        pos_ref, heading_ref, speed_ref = simulator.get_current_reference()
        
        # Get performance metrics
        path_error = simulator.path_error_history[-1] if simulator.path_error_history else 0
        heading_error = simulator.heading_error_history[-1] if simulator.heading_error_history else 0
        
        # Initialize obstacle information
        obstacles_info = {
            'static': [],
            'dynamic': []
        }
        
        # Get obstacle information if available
        if use_obstacle_avoidance and obstacle_avoidance_system:
            # Update dynamic obstacles positions
            update_dynamic_obstacles()
            
            # Get visualization data from the obstacle avoidance system
            vis_data = obstacle_avoidance_system.get_visualization_data()
            if vis_data and isinstance(vis_data, dict):
                obstacles_info = vis_data
            else:
                # Fallback to directly formatting obstacles
                # Format static obstacles for frontend
                for obs in obstacle_avoidance_system.detected_obstacles:
                    if isinstance(obs, (tuple, list)) and len(obs) >= 3:
                        obstacles_info['static'].append({
                            'x': float(obs[0]),
                            'y': float(obs[1]),
                            'radius': float(obs[2])
                        })
                
                # Format dynamic obstacles for frontend
                for obs in obstacle_avoidance_system.detected_dynamic_obstacles:
                    if isinstance(obs, dict) and 'position' in obs:
                        obstacles_info['dynamic'].append({
                            'x': float(obs['position'][0]),
                            'y': float(obs['position'][1]),
                            'radius': float(obs.get('radius', 3.0)),
                            'heading': float(obs.get('heading', 0.0)),
                            'speed': float(obs.get('speed', 0.0))
                        })
        
        # Determine step and total_steps for progress reporting
        # These values may not be available in this context
        step = 0
        total_steps = 100
        if hasattr(simulator, 'time') and hasattr(simulator, 'dt') and hasattr(simulator, 'simulation_time'):
            step = int(simulator.time / simulator.dt)
            total_steps = int(simulator.simulation_time / simulator.dt)
        
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
                'percentage': (step / total_steps) * 100 if total_steps > 0 else 0
            },
            'obstacles': obstacles_info
        }
        
        # Send data to frontend
        socketio.emit('simulation_data', data)
        
        # Print debug info occasionally
        if step % 10 == 0:
            print(f"Simulation step {step}/{total_steps} - Time: {simulator.time:.1f}s - Position: ({state[0]:.1f}, {state[1]:.1f})")
            print(f"Sent obstacle data: {len(obstacles_info['static'])} static, {len(obstacles_info['dynamic'])} dynamic")
    except Exception as e:
        print(f"Error in emit_current_state: {e}")
        import traceback
        traceback.print_exc()

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
    global simulation_thread, simulation_running, simulation_paused, use_obstacle_avoidance, obstacle_avoidance_system
    
    # Stop any existing simulation
    if simulation_running:
        simulation_running = False
        if simulation_thread:
            try:
                simulation_thread.join(timeout=2.0)
            except Exception as e:
                print(f"Error stopping existing simulation: {e}")
    
    # Extract parameters
    scenario = data.get('scenario', os.environ.get('USV_DEFAULT_SCENARIO', 'obstacle'))
    controller = data.get('controller', 'pid')
    use_avoidance = data.get('use_obstacle_avoidance', True)
    
    print(f"Starting simulation with scenario: {scenario}, controller: {controller}, obstacle avoidance: {use_avoidance}")
    
    try:
        # Initialize simulator
        success = initialize_simulator(scenario, controller, use_avoidance)
        if not success:
            error_msg = "Failed to initialize simulator - Config file may be missing or invalid"
            print(f"ERROR: {error_msg}")
            emit('simulation_status', {'status': 'error', 'message': error_msg})
            return
        
        # Force initialization of obstacle avoidance system if not already done
        if use_avoidance and obstacle_avoidance_system is None:
            print("No existing obstacle avoidance system, creating one with default settings...")
            # Create a basic avoidance system with dummy objects for visualization
            try:
                from usv_system.obstacle_avoidance.avoidance_strategy import HybridAvoidanceStrategy, APFAvoidanceStrategy, COLREGAvoidanceStrategy, ObstacleAvoidanceSystem
                
                # Create basic strategies
                apf_strategy = APFAvoidanceStrategy(
                    safety_distance=10.0,
                    attractive_gain=1.0,
                    repulsive_gain=100.0,
                    influence_distance=30.0
                )
                
                colreg_strategy = COLREGAvoidanceStrategy(
                    safety_distance=12.0,
                    attractive_gain=1.0,
                    repulsive_gain=120.0,
                    influence_distance=40.0
                )
                
                # Create dynamic window planner if needed
                try:
                    from usv_system.navigation.local_planner import DynamicWindowPlanner
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
                        dt=0.1,
                        predict_time=3.0
                    )
                except ImportError:
                    dwa_planner = None
                
                # Create hybrid strategy with proper arguments
                hybrid_strategy = HybridAvoidanceStrategy(
                    dwa_planner=dwa_planner,
                    apf_strategy=apf_strategy,
                    colreg_strategy=colreg_strategy,
                    usv_model=None  # We don't have access to the model here
                )
                
                # Create the obstacle avoidance system with visualization enabled
                obstacle_avoidance_system = ObstacleAvoidanceSystem(
                    avoidance_strategy=hybrid_strategy,
                    detection_range=100.0,
                    visualization_enabled=True
                )
                
                print("Created obstacle avoidance system successfully")
            except Exception as e:
                print(f"Error creating obstacle avoidance system: {e}")
                emit('simulation_status', {'status': 'error', 'message': f"Failed to initialize obstacle avoidance: {str(e)}"})
                return
        
        # Reset current simulation state 
        simulation_paused = False
        
        # Start simulation thread
        simulation_thread = threading.Thread(target=simulation_worker)
        simulation_thread.daemon = True
        simulation_thread.start()
        
        # Send status update
        emit('simulation_status', {'status': 'started'})
        print(f"Simulation thread started successfully: {simulation_thread.ident}")
        
        # Also emit initial state to ensure UI updates immediately
        emit_current_state()
    except Exception as e:
        error_msg = f"Error starting simulation: {str(e)}"
        print(f"ERROR: {error_msg}")
        import traceback
        traceback.print_exc()
        emit('simulation_status', {'status': 'error', 'message': error_msg})

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
        'controller': current_controller,
        'obstacle_avoidance': use_obstacle_avoidance
    }
    
    emit('simulation_state', data)
    
    # Also send current simulation data to update charts/visualization
    if simulation_running:
        emit_current_state()

@socketio.on('toggle_obstacle_avoidance')
def handle_toggle_obstacle_avoidance(data):
    """Toggle obstacle avoidance on or off."""
    global use_obstacle_avoidance
    
    use_obstacle_avoidance = data.get('enabled', True)
    emit('obstacle_avoidance_status', {'enabled': use_obstacle_avoidance})

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

def update_dynamic_obstacles():
    """Update positions of dynamic obstacles based on their velocity."""
    global simulator, obstacle_avoidance_system, use_obstacle_avoidance
    
    try:
        if not simulator or not obstacle_avoidance_system or not use_obstacle_avoidance:
            return
        
        # Get dynamic obstacles
        if not hasattr(obstacle_avoidance_system, 'detected_dynamic_obstacles'):
            print("No detected_dynamic_obstacles attribute in obstacle_avoidance_system")
            return
            
        dynamic_obstacles = obstacle_avoidance_system.detected_dynamic_obstacles
        if not dynamic_obstacles:
            return
        
        dt = simulator.dt if simulator else 0.1
        
        # Update each obstacle's position based on heading and speed
        for obs in dynamic_obstacles:
            # Current position
            x, y = obs['position']
            heading = obs.get('heading', 0.0)
            speed = obs.get('speed', 0.0)
            
            # Calculate new position
            dx = speed * dt * np.cos(heading)
            dy = speed * dt * np.sin(heading)
            new_x = x + dx
            new_y = y + dy
            
            # Update position
            obs['position'] = (new_x, new_y)
        
        # Update the obstacle avoidance system with the new dynamic obstacles
        if not hasattr(obstacle_avoidance_system, 'detected_obstacles'):
            print("No detected_obstacles attribute in obstacle_avoidance_system")
            return
            
        if hasattr(obstacle_avoidance_system, 'update_obstacles'):
            obstacle_avoidance_system.update_obstacles(
                obstacle_avoidance_system.detected_obstacles, 
                dynamic_obstacles
            )
        else:
            print("No update_obstacles method in obstacle_avoidance_system")
    except Exception as e:
        print(f"Error in update_dynamic_obstacles: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    try:
        # Parse command line arguments
        parser = argparse.ArgumentParser(description="USV Visualization Server with Obstacle Avoidance")
        parser.add_argument("--port", type=int, default=None, help="Port to run the server on")
        args = parser.parse_args()
        
        # Set the port from command line if provided
        if args.port is not None:
            os.environ["USV_VISUALIZATION_PORT"] = str(args.port)
        
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