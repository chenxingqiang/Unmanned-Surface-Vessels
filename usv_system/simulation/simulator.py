"""
USV simulator module.

This module provides classes for simulating USV dynamics with various controllers
and environmental conditions.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List, Tuple, Optional, Callable, Union, Any
import time

from usv_system.models.usv_model import USVDynamicModel
from usv_system.models.environment import Environment
from usv_system.control.pid_controller import USVPIDController
from usv_system.control.lqr_controller import USVLQRController
from usv_system.control.mpc_controller import USVMPCController


class USVSimulator:
    """
    Simulator for USV dynamics with controller and environmental effects.
    """

    def __init__(self,
                 dt: float = 0.1,
                 simulation_time: float = 100.0,
                 x0: np.ndarray = None,
                 vessel_params: Dict = None,
                 env_params: Dict = None,
                 controller_type: str = 'pid',
                 controller_params: Dict = None):
        """
        Initialize the USV simulator.
        
        Args:
            dt: Time step for simulation [s]
            simulation_time: Total simulation time [s]
            x0: Initial state vector [x, y, psi, u, v, r]
            vessel_params: Dictionary of vessel parameters
            env_params: Dictionary of environment parameters
            controller_type: Type of controller ('pid', 'lqr', or 'mpc')
            controller_params: Dictionary of controller parameters
        """
        self.dt = dt
        self.simulation_time = simulation_time
        self.time = 0.0
        
        # Default initial state
        if x0 is None:
            x0 = np.zeros(6)  # [x, y, psi, u, v, r]
            
        # Create USV model
        self.usv_model = USVDynamicModel(x0=x0, dt=dt, vessel_params=vessel_params)
        
        # Create environment model
        self.env_model = Environment(params=env_params)
        
        # Create controller
        self.controller_type = controller_type
        self.setup_controller(controller_type, controller_params)
        
        # Initialize simulation data storage
        self.time_history = [0.0]
        self.state_history = [x0.copy()]
        self.control_history = [np.zeros(2)]  # [tau_u, tau_r]
        self.reference_history = []
        
        # Performance metrics
        self.path_error_history = []
        self.heading_error_history = []
        self.speed_error_history = []
        self.control_effort_history = []
        self.collision_occurred = False
        self.mission_completed = False
        
        # Reference trajectory
        self.reference_trajectory = None
        
        # Simulation flags
        self.is_running = False
        self.is_paused = False
        
    def setup_controller(self, controller_type: str, controller_params: Dict = None):
        """
        Set up the controller for the simulation.
        
        Args:
            controller_type: Type of controller ('pid', 'lqr', or 'mpc')
            controller_params: Dictionary of controller parameters
        """
        if controller_params is None:
            controller_params = {}
            
        if controller_type.lower() == 'pid':
            # Convert from config format to PID controller format
            Kp_heading = 5.0
            Ki_heading = 0.1
            Kd_heading = 1.0
            Kp_speed = 10.0
            Ki_speed = 0.5
            Kd_speed = 1.0
            Kp_position = 1.0
            max_yaw_rate = 0.5
            max_thrust = 100.0
            max_speed = 5.0
            
            # Extract parameters from config if provided
            if 'heading_gains' in controller_params:
                Kp_heading = controller_params['heading_gains'].get('kp', Kp_heading)
                Ki_heading = controller_params['heading_gains'].get('ki', Ki_heading)
                Kd_heading = controller_params['heading_gains'].get('kd', Kd_heading)
                
            if 'speed_gains' in controller_params:
                Kp_speed = controller_params['speed_gains'].get('kp', Kp_speed)
                Ki_speed = controller_params['speed_gains'].get('ki', Ki_speed)
                Kd_speed = controller_params['speed_gains'].get('kd', Kd_speed)
                
            Kp_position = controller_params.get('position_gain', Kp_position)
            max_yaw_rate = controller_params.get('max_yaw_rate', max_yaw_rate)
            max_thrust = controller_params.get('max_thrust', max_thrust)
            max_speed = controller_params.get('max_speed', max_speed)
            
            # Create the PID controller with extracted parameters
            self.controller = USVPIDController(
                Kp_heading=Kp_heading,
                Ki_heading=Ki_heading,
                Kd_heading=Kd_heading,
                Kp_speed=Kp_speed,
                Ki_speed=Ki_speed,
                Kd_speed=Kd_speed,
                Kp_position=Kp_position,
                max_yaw_rate=max_yaw_rate,
                max_thrust=max_thrust,
                max_speed=max_speed,
                dt=self.dt
            )
        elif controller_type.lower() == 'lqr':
            # Convert from config format to LQR controller format
            Q_param = {}
            R_param = {}
            max_thrust = 100.0
            max_moment = 50.0
            
            # Extract parameters from config if provided
            if 'Q' in controller_params:
                if isinstance(controller_params['Q'], list):
                    # Convert list format to dict format
                    state_names = ['x', 'y', 'psi', 'u', 'v', 'r']
                    Q_list = controller_params['Q']
                    for i, name in enumerate(state_names):
                        if i < len(Q_list):
                            Q_param[name] = Q_list[i]
                        else:
                            Q_param[name] = 1.0  # Default value
                else:
                    Q_param = controller_params['Q']
                    
            if 'R' in controller_params:
                if isinstance(controller_params['R'], list):
                    # Convert list format to dict format
                    control_names = ['tau_u', 'tau_r']
                    R_list = controller_params['R']
                    for i, name in enumerate(control_names):
                        if i < len(R_list):
                            R_param[name] = R_list[i]
                        else:
                            R_param[name] = 1.0  # Default value
                else:
                    R_param = controller_params['R']
                    
            max_thrust = controller_params.get('max_thrust', max_thrust)
            max_moment = controller_params.get('max_yaw_rate', max_moment)  # Use max_yaw_rate as max_moment
            
            # Create LQR controller with extracted parameters
            self.controller = USVLQRController(
                Q=Q_param,
                R=R_param,
                vessel_params=self.usv_model.params,
                max_thrust=max_thrust,
                max_moment=max_moment,
                dt=self.dt
            )
        elif controller_type.lower() == 'mpc':
            # Convert from config format to MPC controller format
            prediction_horizon = 20
            Q_param = {}
            R_param = {}
            max_thrust = 100.0
            max_moment = 50.0
            max_speed = 5.0
            
            # Extract parameters from config if provided
            prediction_horizon = controller_params.get('prediction_horizon', prediction_horizon)
            
            if 'Q' in controller_params:
                if isinstance(controller_params['Q'], list):
                    # Convert list format to dict format
                    state_names = ['x', 'y', 'psi', 'u', 'v', 'r']
                    Q_list = controller_params['Q']
                    for i, name in enumerate(state_names):
                        if i < len(Q_list):
                            Q_param[name] = Q_list[i]
                        else:
                            Q_param[name] = 1.0  # Default value
                else:
                    Q_param = controller_params['Q']
                    
            if 'R' in controller_params:
                if isinstance(controller_params['R'], list):
                    # Convert list format to dict format
                    control_names = ['tau_u', 'tau_r']
                    R_list = controller_params['R']
                    for i, name in enumerate(control_names):
                        if i < len(R_list):
                            R_param[name] = R_list[i]
                        else:
                            R_param[name] = 1.0  # Default value
                else:
                    R_param = controller_params['R']
                    
            max_thrust = controller_params.get('max_thrust', max_thrust)
            max_moment = controller_params.get('max_yaw_rate', max_moment)  # Use max_yaw_rate as max_moment
            max_speed = controller_params.get('max_speed', max_speed)
            
            # Create MPC controller with extracted parameters
            self.controller = USVMPCController(
                prediction_horizon=prediction_horizon,
                Q=Q_param,
                R=R_param,
                vessel_params=self.usv_model.params,
                max_thrust=max_thrust,
                max_moment=max_moment,
                max_speed=max_speed,
                dt=self.dt
            )
        else:
            raise ValueError(f"Unknown controller type: {controller_type}")
        
    def set_reference_trajectory(self, reference_config: Dict):
        """
        Set the reference trajectory for the simulation based on configuration.
        
        Args:
            reference_config: Dictionary containing reference trajectory configuration:
                - 'type': Type of trajectory ('waypoints', 'station', 'line', 'circular')
                - Type-specific parameters
        """
        trajectory_type = reference_config.get('type', 'waypoints')
        
        if trajectory_type == 'waypoints':
            waypoints = reference_config.get('waypoints', [[0, 0]])
            desired_speed = reference_config.get('desired_speed', 1.0)
            
            # Convert waypoints to reference trajectory format
            self.reference_trajectory = []
            
            for wp in waypoints:
                # If already at the last waypoint position, stay there
                if self.reference_trajectory and wp[0] == self.reference_trajectory[-1]['position'][0] and wp[1] == self.reference_trajectory[-1]['position'][1]:
                    continue
                    
                # Calculate heading to next waypoint (if not first waypoint)
                if not self.reference_trajectory:
                    # First waypoint - use 0 heading if no previous heading to use
                    heading = 0.0
                else:
                    prev_pos = self.reference_trajectory[-1]['position']
                    dx = wp[0] - prev_pos[0]
                    dy = wp[1] - prev_pos[1]
                    heading = np.arctan2(dy, dx)
                
                # Add waypoint to trajectory
                self.reference_trajectory.append({
                    'position': (wp[0], wp[1]),
                    'heading': heading,
                    'speed': desired_speed
                })
                
        elif trajectory_type == 'station':
            # Station keeping at a fixed point
            position = reference_config.get('position', [0, 0])
            heading = reference_config.get('heading', 0.0)
            
            self.reference_trajectory = [{
                'position': (position[0], position[1]),
                'heading': heading,
                'speed': 0.0  # Zero speed for station keeping
            }]
            
        elif trajectory_type == 'line':
            # Straight line path
            start = reference_config.get('start', [0, 0])
            end = reference_config.get('end', [10, 0])
            desired_speed = reference_config.get('desired_speed', 1.0)
            
            # Calculate heading
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            heading = np.arctan2(dy, dx)
            
            self.reference_trajectory = [
                {
                    'position': (start[0], start[1]),
                    'heading': heading,
                    'speed': desired_speed
                },
                {
                    'position': (end[0], end[1]),
                    'heading': heading,
                    'speed': desired_speed
                }
            ]
            
        elif trajectory_type == 'circular':
            # Circular path
            center = reference_config.get('center', [0, 0])
            radius = reference_config.get('radius', 10.0)
            angular_speed = reference_config.get('angular_speed', 0.1)  # rad/s
            desired_speed = reference_config.get('desired_speed', 1.0)
            
            # Generate points along the circle
            num_points = 20  # Number of points to define the circle
            self.reference_trajectory = []
            
            for i in range(num_points + 1):
                angle = 2 * np.pi * i / num_points
                x = center[0] + radius * np.cos(angle)
                y = center[1] + radius * np.sin(angle)
                
                # Tangent heading (perpendicular to radius vector)
                heading = angle + np.pi/2
                
                self.reference_trajectory.append({
                    'position': (x, y),
                    'heading': heading,
                    'speed': desired_speed
                })
        
        else:
            raise ValueError(f"Unknown trajectory type: {trajectory_type}")
        
    def get_current_reference(self) -> Tuple[Tuple[float, float], float, float]:
        """
        Get the current reference from the trajectory based on simulation time.
        
        Returns:
            Tuple of (position, heading, speed)
        """
        if self.reference_trajectory is None or len(self.reference_trajectory) == 0:
            # Default reference if none is provided
            return (0.0, 0.0), 0.0, 0.0
        
        # If no time is specified for waypoints, use the first one
        if 'time' not in self.reference_trajectory[0]:
            return (
                self.reference_trajectory[0]['position'],
                self.reference_trajectory[0]['heading'],
                self.reference_trajectory[0]['speed']
            )
            
        # Find the appropriate waypoint based on time
        current_waypoint = self.reference_trajectory[0]
        next_waypoint_idx = 0
        
        for i, waypoint in enumerate(self.reference_trajectory):
            if i > 0 and self.time < waypoint['time']:
                next_waypoint_idx = i
                current_waypoint = self.reference_trajectory[i-1]
                break
                
        # If past the last waypoint, use the last one
        if next_waypoint_idx == 0 and self.time >= self.reference_trajectory[-1]['time']:
            current_waypoint = self.reference_trajectory[-1]
            
        # If between waypoints and interpolation is needed
        if (next_waypoint_idx > 0 and 
            self.time > current_waypoint['time'] and 
            self.time < self.reference_trajectory[next_waypoint_idx]['time']):
            
            next_waypoint = self.reference_trajectory[next_waypoint_idx]
            
            # Calculate interpolation factor
            t_diff = next_waypoint['time'] - current_waypoint['time']
            alpha = (self.time - current_waypoint['time']) / t_diff if t_diff > 0 else 0
            
            # Interpolate position
            x = current_waypoint['position'][0] + alpha * (next_waypoint['position'][0] - current_waypoint['position'][0])
            y = current_waypoint['position'][1] + alpha * (next_waypoint['position'][1] - current_waypoint['position'][1])
            
            # Interpolate heading (accounting for angle wrap)
            heading_diff = next_waypoint['heading'] - current_waypoint['heading']
            heading_diff = ((heading_diff + np.pi) % (2 * np.pi)) - np.pi
            heading = current_waypoint['heading'] + alpha * heading_diff
            
            # Interpolate speed
            speed = current_waypoint['speed'] + alpha * (next_waypoint['speed'] - current_waypoint['speed'])
            
            return (x, y), heading, speed
            
        return (
            current_waypoint['position'],
            current_waypoint['heading'],
            current_waypoint['speed']
        )
        
    def simulate_step(self):
        """
        Simulate a single time step.
        """
        # Get current USV state
        current_state = self.usv_model.state
        
        # Get current reference
        position_ref, heading_ref, speed_ref = self.get_current_reference()
        
        # Store reference
        self.reference_history.append((position_ref, heading_ref, speed_ref))
        
        # Compute control inputs
        tau_u, tau_r = self.controller.compute_control_inputs(
            position_ref, heading_ref, speed_ref, current_state
        )
        
        # Calculate environmental disturbances
        env_forces = self.env_model.calculate_environmental_forces(current_state)
        
        # Update USV model
        self.usv_model.update([tau_u, tau_r], env_forces)
        
        # Update environment model
        self.env_model.update(self.dt)
        
        # Update time
        self.time += self.dt
        
        # Store simulation data
        self.time_history.append(self.time)
        self.state_history.append(current_state.copy())
        self.control_history.append(np.array([tau_u, tau_r]))
        
        # Calculate and store performance metrics
        self._update_performance_metrics(position_ref, heading_ref, speed_ref, current_state, np.array([tau_u, tau_r]))
        
    def _update_performance_metrics(self, position_ref, heading_ref, speed_ref, current_state, control_input):
        """
        Calculate and update performance metrics.
        
        Args:
            position_ref: Reference position (x, y)
            heading_ref: Reference heading [rad]
            speed_ref: Reference speed [m/s]
            current_state: Current USV state [x, y, psi, u, v, r]
            control_input: Control input [tau_u, tau_r]
        """
        # Calculate path error (Euclidean distance to reference position)
        x, y = current_state[0:2]
        path_error = np.sqrt((x - position_ref[0])**2 + (y - position_ref[1])**2)
        self.path_error_history.append(path_error)
        
        # Calculate heading error (normalized to [-pi, pi])
        heading_error = heading_ref - current_state[2]
        heading_error = ((heading_error + np.pi) % (2 * np.pi)) - np.pi
        self.heading_error_history.append(heading_error)
        
        # Calculate speed error
        speed_error = speed_ref - current_state[3]  # Compare to surge velocity
        self.speed_error_history.append(speed_error)
        
        # Calculate control effort (sum of squared control inputs)
        control_effort = np.sum(control_input**2)
        self.control_effort_history.append(control_effort)
        
        # Check for mission completion (within 1m of final waypoint)
        if self.reference_trajectory and len(self.time_history) > 1:
            final_waypoint = self.reference_trajectory[-1]['position']
            if path_error < 1.0 and abs(speed_error) < 0.1:
                self.mission_completed = True
        
        # Check for collisions (if obstacles are present)
        self.collision_occurred = self._check_collisions(current_state)
    
    def _check_collisions(self, state):
        """
        Check if the USV collides with any obstacles.
        
        Args:
            state: Current USV state
            
        Returns:
            True if collision occurred, False otherwise
        """
        # If environment model has obstacle information
        if hasattr(self.env_model, 'obstacles') and self.env_model.obstacles:
            x, y = state[0:2]
            vessel_radius = 1.0  # Approximate vessel radius
            
            # Check static obstacles
            for obstacle in self.env_model.obstacles.get('static', []):
                obs_x, obs_y = obstacle['position']
                obs_radius = obstacle.get('radius', 1.0)
                distance = np.sqrt((x - obs_x)**2 + (y - obs_y)**2)
                if distance < (vessel_radius + obs_radius):
                    return True
            
            # Check dynamic obstacles
            for obstacle in self.env_model.obstacles.get('dynamic', []):
                obs_x, obs_y = obstacle.get('current_position', obstacle['initial_position'])
                obs_radius = obstacle.get('radius', 1.0)
                distance = np.sqrt((x - obs_x)**2 + (y - obs_y)**2)
                if distance < (vessel_radius + obs_radius):
                    return True
                    
        return False
        
    def run_simulation(self, verbose: bool = False, visualize: bool = False):
        """
        Run the full simulation.
        
        Args:
            verbose: Whether to print simulation progress
            visualize: Whether to visualize the simulation results
        """
        # Reset simulation
        self.time = 0.0
        self.time_history = [0.0]
        self.state_history = [self.usv_model.state.copy()]
        self.control_history = [np.zeros(2)]
        self.reference_history = []
        
        # Number of steps
        num_steps = int(self.simulation_time / self.dt)
        
        # Simulation main loop
        self.is_running = True
        start_time = time.time()
        
        for i in range(num_steps):
            if verbose and i % int(num_steps / 10) == 0:
                progress = i / num_steps * 100
                print(f"Simulation progress: {progress:.1f}%")
                
            self.simulate_step()
            
        end_time = time.time()
        self.is_running = False
        
        if verbose:
            sim_time = end_time - start_time
            print(f"Simulation completed in {sim_time:.2f} seconds")
            
        if visualize:
            self.visualize_results()
            
    def visualize_results(self, save_path: Optional[str] = None):
        """
        Visualize the simulation results.
        
        Args:
            save_path: Path to save the figure (if None, display instead)
        """
        # Convert histories to numpy arrays for easier indexing
        time_array = np.array(self.time_history)
        state_array = np.array(self.state_history)
        control_array = np.array(self.control_history)
        
        # Extract reference trajectory if available
        if self.reference_history:
            ref_positions = np.array([ref[0] for ref in self.reference_history])
            ref_headings = np.array([ref[1] for ref in self.reference_history])
            ref_speeds = np.array([ref[2] for ref in self.reference_history])
        
        # Create figure with subplots
        fig = plt.figure(figsize=(15, 10))
        
        # Trajectory plot
        ax1 = fig.add_subplot(221)
        ax1.plot(state_array[:, 0], state_array[:, 1], 'b-', label='USV Trajectory')
        
        if self.reference_history:
            ax1.plot(ref_positions[:, 0], ref_positions[:, 1], 'r--', label='Reference Trajectory')
            
        # Plot initial and final positions
        ax1.plot(state_array[0, 0], state_array[0, 1], 'go', label='Start')
        ax1.plot(state_array[-1, 0], state_array[-1, 1], 'ro', label='End')
        
        # Add orientation markers along trajectory
        traj_length = len(time_array)
        marker_indices = np.linspace(0, traj_length - 1, 20, dtype=int)
        
        for idx in marker_indices:
            x, y, psi = state_array[idx, 0:3]
            dx = 0.5 * np.cos(psi)
            dy = 0.5 * np.sin(psi)
            ax1.arrow(x, y, dx, dy, head_width=0.2, head_length=0.3, fc='blue', ec='blue', alpha=0.5)
        
        ax1.set_xlabel('X Position [m]')
        ax1.set_ylabel('Y Position [m]')
        ax1.set_title('USV Trajectory')
        ax1.grid(True)
        ax1.legend()
        ax1.axis('equal')
        
        # Heading and speed vs time
        ax2 = fig.add_subplot(222)
        
        # Plot heading
        ax2.plot(time_array, np.rad2deg(state_array[:, 2]), 'b-', label='Heading')
        
        if self.reference_history:
            ax2.plot(time_array[1:], np.rad2deg(ref_headings), 'b--', label='Reference Heading')
        
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Heading [deg]', color='b')
        ax2.tick_params(axis='y', labelcolor='b')
        ax2.grid(True)
        
        # Twin axis for speed
        ax2b = ax2.twinx()
        ax2b.plot(time_array, state_array[:, 3], 'g-', label='Surge Speed')
        
        if self.reference_history:
            ax2b.plot(time_array[1:], ref_speeds, 'g--', label='Reference Speed')
            
        ax2b.set_ylabel('Speed [m/s]', color='g')
        ax2b.tick_params(axis='y', labelcolor='g')
        
        # Control inputs vs time
        ax3 = fig.add_subplot(223)
        ax3.plot(time_array, control_array[:, 0], 'r-', label='Thrust Force')
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Thrust Force [N]', color='r')
        ax3.tick_params(axis='y', labelcolor='r')
        ax3.grid(True)
        
        # Twin axis for yaw moment
        ax3b = ax3.twinx()
        ax3b.plot(time_array, control_array[:, 1], 'm-', label='Yaw Moment')
        ax3b.set_ylabel('Yaw Moment [N.m]', color='m')
        ax3b.tick_params(axis='y', labelcolor='m')
        
        # Velocities vs time
        ax4 = fig.add_subplot(224)
        ax4.plot(time_array, state_array[:, 3], 'r-', label='Surge (u)')
        ax4.plot(time_array, state_array[:, 4], 'g-', label='Sway (v)')
        ax4.plot(time_array, state_array[:, 5], 'b-', label='Yaw Rate (r)')
        ax4.set_xlabel('Time [s]')
        ax4.set_ylabel('Velocities')
        ax4.grid(True)
        ax4.legend()
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300)
            plt.close(fig)
        else:
            plt.show()
            
    def compare_controllers(self, 
                           controllers: List[Dict],
                           reference_trajectory: Union[List[Dict], Dict],
                           save_path: Optional[str] = None):
        """
        Compare different controllers on the same reference trajectory.
        
        Args:
            controllers: List of controller configurations, each a dict with:
                - 'type': Controller type ('pid', 'lqr', or 'mpc')
                - 'params': Controller parameters
                - 'name': Display name for the controller
            reference_trajectory: Reference trajectory to follow, either:
                - List of reference dictionaries (old format)
                - Reference configuration dictionary (new format)
            save_path: Path to save the figure (if None, display instead)
        """
        # Store the original controller and state
        original_controller_type = self.controller_type
        original_controller = self.controller
        original_state = self.usv_model.state.copy()
        
        # Results storage
        all_states = []
        all_controls = []
        all_times = []
        controller_names = []
        
        # Set reference trajectory
        # Check if reference_trajectory is in the new or old format
        if isinstance(reference_trajectory, list):
            # Old format - list of dictionaries
            self.reference_trajectory = reference_trajectory
        else:
            # New format - config dictionary
            self.set_reference_trajectory(reference_trajectory)
        
        # Run simulation for each controller
        for controller_config in controllers:
            controller_type = controller_config['type']
            controller_params = controller_config.get('params', {})
            controller_name = controller_config.get('name', controller_type.upper())
            
            # Setup controller
            self.setup_controller(controller_type, controller_params)
            controller_names.append(controller_name)
            
            # Reset USV model to initial state
            self.usv_model.state = original_state.copy()
            
            # Reset performance metrics
            self.path_error_history = []
            self.heading_error_history = []
            self.speed_error_history = []
            self.control_effort_history = []
            self.collision_occurred = False
            self.mission_completed = False
            
            # Run simulation
            self.run_simulation(verbose=False, visualize=False)
            
            # Store results
            all_states.append(np.array(self.state_history))
            all_controls.append(np.array(self.control_history))
            all_times.append(np.array(self.time_history))
        
        # Restore original controller
        self.controller_type = original_controller_type
        self.controller = original_controller
        
        # Visualize comparison results
        self._visualize_controller_comparison(
            all_times, all_states, all_controls, controller_names, save_path
        )
        
    def _visualize_controller_comparison(self,
                                        all_times: List[np.ndarray],
                                        all_states: List[np.ndarray],
                                        all_controls: List[np.ndarray],
                                        controller_names: List[str],
                                        save_path: Optional[str] = None):
        """
        Visualize the comparison between different controllers.
        
        Args:
            all_times: List of time arrays for each controller
            all_states: List of state arrays for each controller
            all_controls: List of control arrays for each controller
            controller_names: List of controller names
            save_path: Path to save the figure (if None, display instead)
        """
        # Colors for different controllers
        colors = ['b', 'r', 'g', 'c', 'm', 'y', 'k']
        
        # Create figure with subplots
        fig = plt.figure(figsize=(15, 10))
        
        # Trajectory plot
        ax1 = fig.add_subplot(221)
        
        # Extract reference trajectory if available
        if self.reference_history:
            ref_positions = np.array([ref[0] for ref in self.reference_history])
            ax1.plot(ref_positions[:, 0], ref_positions[:, 1], 'k--', label='Reference')
        
        # Plot trajectory for each controller
        for i, states in enumerate(all_states):
            color = colors[i % len(colors)]
            ax1.plot(states[:, 0], states[:, 1], color=color, label=controller_names[i])
            
            # Plot start and end positions
            if i == 0:  # Only mark start once
                ax1.plot(states[0, 0], states[0, 1], 'go', label='Start')
                
            ax1.plot(states[-1, 0], states[-1, 1], 'o', color=color)
        
        ax1.set_xlabel('X Position [m]')
        ax1.set_ylabel('Y Position [m]')
        ax1.set_title('USV Trajectories')
        ax1.grid(True)
        ax1.legend()
        ax1.axis('equal')
        
        # Heading error
        ax2 = fig.add_subplot(222)
        
        # Plot heading error for each controller
        if self.reference_history:
            ref_headings = np.array([ref[1] for ref in self.reference_history])
            
            for i, states in enumerate(all_states):
                color = colors[i % len(colors)]
                times = all_times[i]
                
                # Calculate heading error
                heading_errors = []
                for j, heading in enumerate(states[:, 2]):
                    if j < len(ref_headings):
                        error = heading - ref_headings[j]
                        # Normalize to [-pi, pi]
                        error = ((error + np.pi) % (2 * np.pi)) - np.pi
                        heading_errors.append(error)
                    else:
                        heading_errors.append(0)
                        
                ax2.plot(times, np.rad2deg(heading_errors), color=color, label=controller_names[i])
                
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Heading Error [deg]')
        ax2.set_title('Heading Error')
        ax2.grid(True)
        
        # Control inputs
        ax3 = fig.add_subplot(223)
        
        # Plot thrust for each controller
        for i, controls in enumerate(all_controls):
            color = colors[i % len(colors)]
            times = all_times[i]
            ax3.plot(times, controls[:, 0], color=color, label=controller_names[i])
            
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Thrust Force [N]')
        ax3.set_title('Control Inputs - Thrust')
        ax3.grid(True)
        ax3.legend()
        
        # Cross-track error
        ax4 = fig.add_subplot(224)
        
        # Plot cross-track error for each controller
        if self.reference_history:
            ref_positions = np.array([ref[0] for ref in self.reference_history])
            
            for i, states in enumerate(all_states):
                color = colors[i % len(colors)]
                times = all_times[i]
                
                # Calculate cross-track error (Euclidean distance to reference)
                position_errors = []
                for j, pos in enumerate(states[:, 0:2]):
                    if j < len(ref_positions):
                        error = np.linalg.norm(pos - ref_positions[j])
                        position_errors.append(error)
                    else:
                        position_errors.append(0)
                        
                ax4.plot(times, position_errors, color=color, label=controller_names[i])
                
        ax4.set_xlabel('Time [s]')
        ax4.set_ylabel('Position Error [m]')
        ax4.set_title('Position Error')
        ax4.grid(True)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300)
            plt.close(fig)
        else:
            plt.show()
