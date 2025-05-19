"""
Model Predictive Control (MPC) implementation for USV control.

This module provides an MPC controller class for controlling an Unmanned Surface Vessel.
MPC uses a model to predict future behavior and optimizes control inputs over a prediction
horizon, considering constraints on states and inputs.
"""

import numpy as np
import scipy.optimize
from typing import Tuple, Dict, List, Optional


class MPCController:
    """
    MPC controller implementation for USV control.
    """

    def __init__(self,
                 dt: float = 0.1,
                 prediction_horizon: int = 10,
                 Q: np.ndarray = None,
                 R: np.ndarray = None,
                 vessel_params: Dict = None,
                 state_constraints: Dict = None,
                 input_constraints: Dict = None):
        """
        Initialize the MPC controller.

        Args:
            dt: Time step
            prediction_horizon: Number of steps to predict into the future
            Q: State weight matrix
            R: Control input weight matrix
            vessel_params: Dictionary of vessel parameters
            state_constraints: Dictionary of state constraints
            input_constraints: Dictionary of input constraints
        """
        self.dt = dt
        self.N = prediction_horizon  # Prediction horizon
        
        # Default vessel parameters if not provided
        default_params = {
            'm': 50.0,         # mass (kg)
            'Iz': 20.0,        # moment of inertia (kg.m^2)
            'Xu': -25.0,       # surge linear damping
            'Yv': -40.0,       # sway linear damping
            'Nr': -10.0,       # yaw linear damping
            'Xuu': -30.0,      # surge quadratic damping
            'Yvv': -50.0,      # sway quadratic damping
            'Nrr': -5.0,       # yaw quadratic damping
        }
        
        self.vessel_params = default_params
        if vessel_params:
            self.vessel_params.update(vessel_params)
            
        # Default state constraints if not provided
        default_state_constraints = {
            'u_min': -5.0,     # Minimum surge velocity (m/s)
            'u_max': 5.0,      # Maximum surge velocity (m/s)
            'v_min': -2.0,     # Minimum sway velocity (m/s)
            'v_max': 2.0,      # Maximum sway velocity (m/s)
            'r_min': -0.5,     # Minimum yaw rate (rad/s)
            'r_max': 0.5       # Maximum yaw rate (rad/s)
        }
        
        self.state_constraints = default_state_constraints
        if state_constraints:
            self.state_constraints.update(state_constraints)
            
        # Default input constraints if not provided
        default_input_constraints = {
            'tau_u_min': -100.0,   # Minimum surge force (N)
            'tau_u_max': 100.0,    # Maximum surge force (N)
            'tau_r_min': -50.0,    # Minimum yaw moment (N.m)
            'tau_r_max': 50.0      # Maximum yaw moment (N.m)
        }
        
        self.input_constraints = default_input_constraints
        if input_constraints:
            self.input_constraints.update(input_constraints)
            
        # Default weight matrices if not provided
        if Q is None:
            # State weights for [x, y, psi, u, v, r]
            # Higher weights on position and heading errors
            self.Q = np.diag([10.0, 10.0, 20.0, 1.0, 1.0, 5.0])
        else:
            self.Q = Q
            
        if R is None:
            # Control input weights for [tau_u, tau_r]
            # Balance between control effort and state regulation
            self.R = np.diag([1.0, 1.0])
        else:
            self.R = R
            
        # Terminal weights (usually higher than Q)
        self.P = self.Q * 5.0
        
        # Get linearized model matrices
        self.A, self.B = self._linearize_model()
        
        # Discretize the model
        self.Ad = np.eye(6) + self.dt * self.A
        self.Bd = self.dt * self.B
        
        # Initialize reference trajectory
        self.reference_trajectory = [np.zeros(6)] * (self.N + 1)
        
        # Last computed control sequence
        self.last_control_sequence = np.zeros((self.N, 2))
        
    def _linearize_model(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Linearize the USV dynamic model around an equilibrium point.
        
        Returns:
            A: State matrix
            B: Control input matrix
        """
        # Extract vessel parameters
        m = self.vessel_params['m']
        Iz = self.vessel_params['Iz']
        Xu = self.vessel_params['Xu']
        Yv = self.vessel_params['Yv']
        Nr = self.vessel_params['Nr']
        
        # Linearized state matrix (assuming small angle approximation and steady state)
        # State vector: [x, y, psi, u, v, r]
        A = np.zeros((6, 6))
        
        # Position kinematics (linearized around psi=0)
        A[0, 3] = 1.0  # dx/dt = u
        A[1, 4] = 1.0  # dy/dt = v
        A[2, 5] = 1.0  # dpsi/dt = r
        
        # Velocity dynamics (linearized around zero speed)
        A[3, 3] = Xu / m  # du/dt = Xu*u/m + tau_u/m
        A[4, 4] = Yv / m  # dv/dt = Yv*v/m
        A[5, 5] = Nr / Iz  # dr/dt = Nr*r/Iz + tau_r/Iz
        
        # Control input matrix
        # Control input vector: [tau_u, tau_r]
        B = np.zeros((6, 2))
        B[3, 0] = 1.0 / m   # du/dt = tau_u/m
        B[5, 1] = 1.0 / Iz  # dr/dt = tau_r/Iz
        
        return A, B
    
    def _predict_state_trajectory(self, x0: np.ndarray, u_sequence: np.ndarray) -> np.ndarray:
        """
        Predict state trajectory using the linearized model.
        
        Args:
            x0: Initial state
            u_sequence: Control input sequence [N x 2]
            
        Returns:
            Predicted state trajectory [N+1 x 6]
        """
        # Initialize state trajectory with initial state
        x_trajectory = np.zeros((self.N + 1, 6))
        x_trajectory[0] = x0
        
        # Predict future states
        for k in range(self.N):
            x_trajectory[k+1] = self.Ad @ x_trajectory[k] + self.Bd @ u_sequence[k]
            
        return x_trajectory
    
    def _compute_cost(self, u_sequence: np.ndarray, x0: np.ndarray) -> float:
        """
        Compute the cost function for a given control sequence.
        
        Args:
            u_sequence: Control input sequence [N*2]
            x0: Initial state
            
        Returns:
            Cost value
        """
        # Reshape control sequence to [N x 2]
        u_sequence_reshaped = u_sequence.reshape(self.N, 2)
        
        # Predict state trajectory
        x_trajectory = self._predict_state_trajectory(x0, u_sequence_reshaped)
        
        # Initialize cost
        cost = 0.0
        
        # Compute stage costs
        for k in range(self.N):
            # State error (accounting for angle wrapping)
            x_error = x_trajectory[k] - self.reference_trajectory[k]
            x_error[2] = ((x_error[2] + np.pi) % (2 * np.pi)) - np.pi  # Normalize heading error
            
            # State cost
            state_cost = x_error @ self.Q @ x_error
            
            # Control input cost
            control_cost = u_sequence_reshaped[k] @ self.R @ u_sequence_reshaped[k]
            
            # Add to total cost
            cost += state_cost + control_cost
        
        # Terminal cost
        x_error_terminal = x_trajectory[-1] - self.reference_trajectory[-1]
        x_error_terminal[2] = ((x_error_terminal[2] + np.pi) % (2 * np.pi)) - np.pi
        terminal_cost = x_error_terminal @ self.P @ x_error_terminal
        
        cost += terminal_cost
        
        return cost
    
    def _optimize_control(self, x0: np.ndarray) -> np.ndarray:
        """
        Optimize the control sequence to minimize the cost function.
        
        Args:
            x0: Initial state
            
        Returns:
            Optimal control sequence [N x 2]
        """
        # Initial guess (use last control sequence if available)
        if hasattr(self, 'last_control_sequence'):
            # Shift last control sequence one step and append zeros
            initial_guess = np.vstack([
                self.last_control_sequence[1:],
                np.zeros((1, 2))
            ]).flatten()
        else:
            initial_guess = np.zeros(self.N * 2)
        
        # Control input bounds
        bounds = []
        for _ in range(self.N):
            bounds.append((self.input_constraints['tau_u_min'], self.input_constraints['tau_u_max']))
            bounds.append((self.input_constraints['tau_r_min'], self.input_constraints['tau_r_max']))
        
        try:
            # Optimize the control sequence using scipy.optimize
            result = scipy.optimize.minimize(
                fun=self._compute_cost,
                x0=initial_guess,
                args=(x0,),
                method='SLSQP',
                bounds=bounds,
                options={'maxiter': 100, 'disp': False}
            )
            
            # Check if optimization was successful
            if not result.success:
                print(f"Warning: MPC optimization failed: {result.message}")
                # Fallback to a simpler control law if optimization fails
                u_optimal = self._fallback_control(x0)
            else:
                # Reshape the optimal control sequence
                u_optimal = result.x.reshape(self.N, 2)
                
        except Exception as e:
            print(f"Error in MPC optimization: {str(e)}")
            # Use fallback control if exception occurs
            u_optimal = self._fallback_control(x0)
        
        # Store the optimal control sequence for the next iteration
        self.last_control_sequence = u_optimal
        
        return u_optimal
    
    def _fallback_control(self, x0: np.ndarray) -> np.ndarray:
        """
        Fallback control strategy when optimization fails.
        
        Args:
            x0: Initial state
            
        Returns:
            Fallback control sequence [N x 2]
        """
        # Simple proportional control as fallback
        # Create a sequence of control inputs based on current error
        u_sequence = np.zeros((self.N, 2))
        
        for i in range(self.N):
            # Get reference state for this step
            x_ref = self.reference_trajectory[i]
            
            # Calculate position error
            pos_error_x = x_ref[0] - x0[0]
            pos_error_y = x_ref[1] - x0[1]
            
            # Calculate heading error (with angle wrapping)
            heading_error = x_ref[2] - x0[2]
            heading_error = ((heading_error + np.pi) % (2 * np.pi)) - np.pi
            
            # Calculate speed error
            speed_error = x_ref[3] - x0[3]
            
            # Simple proportional control
            # Surge force proportional to speed error and position error projection
            pos_error_proj = pos_error_x * np.cos(x0[2]) + pos_error_y * np.sin(x0[2])
            thrust = 20.0 * speed_error + 5.0 * pos_error_proj
            
            # Moment proportional to heading error
            moment = 10.0 * heading_error
            
            # Apply input constraints
            thrust = np.clip(thrust, self.input_constraints['tau_u_min'], self.input_constraints['tau_u_max'])
            moment = np.clip(moment, self.input_constraints['tau_r_min'], self.input_constraints['tau_r_max'])
            
            u_sequence[i] = [thrust, moment]
        
        return u_sequence
    
    def set_reference_trajectory(self, reference_trajectory: List[np.ndarray]):
        """
        Set the reference trajectory for the MPC controller.
        
        Args:
            reference_trajectory: List of reference states over the prediction horizon
        """
        # Ensure the reference trajectory has the correct length
        if len(reference_trajectory) != self.N + 1:
            raise ValueError(f"Reference trajectory must have length {self.N + 1}")
        
        self.reference_trajectory = reference_trajectory
    
    def compute_control_inputs(self, current_state: np.ndarray) -> np.ndarray:
        """
        Compute control inputs using the MPC controller.
        
        Args:
            current_state: Current state vector [x, y, psi, u, v, r]
            
        Returns:
            Control input vector [tau_u, tau_r]
        """
        # Optimize the control sequence
        u_optimal = self._optimize_control(current_state)
        
        # Return the first control input from the optimal sequence
        return u_optimal[0]


class USVMPCController:
    """
    MPC controller for heading and speed control of a USV.
    """
    
    def __init__(self,
                 prediction_horizon: int = 10,
                 Q: Dict[str, float] = None,
                 R: Dict[str, float] = None,
                 vessel_params: Dict = None,
                 max_thrust: float = 100.0,
                 max_moment: float = 50.0,
                 max_speed: float = 5.0,
                 dt: float = 0.1):
        """
        Initialize the USV MPC controller.
        
        Args:
            prediction_horizon: Number of steps to predict into the future
            Q: Dictionary of state weights
            R: Dictionary of control input weights
            vessel_params: Dictionary of vessel parameters
            max_thrust: Maximum thrust force in Newtons
            max_moment: Maximum yaw moment in Newton-meters
            max_speed: Maximum speed in m/s
            dt: Time step
        """
        # Create weight matrices from dictionaries
        q_dict = {
            'x': 10.0, 'y': 10.0, 'psi': 20.0,
            'u': 1.0, 'v': 1.0, 'r': 5.0
        }
        if Q:
            q_dict.update(Q)
        Q_matrix = np.diag([q_dict['x'], q_dict['y'], q_dict['psi'], 
                             q_dict['u'], q_dict['v'], q_dict['r']])
        
        r_dict = {'tau_u': 1.0, 'tau_r': 1.0}
        if R:
            r_dict.update(R)
        R_matrix = np.diag([r_dict['tau_u'], r_dict['tau_r']])
        
        # Create constraints dictionaries
        state_constraints = {
            'u_min': -max_speed,
            'u_max': max_speed,
            'v_min': -max_speed / 2,  # Typically sway is more restricted
            'v_max': max_speed / 2,
            'r_min': -0.5,  # Typical yaw rate limits
            'r_max': 0.5
        }
        
        input_constraints = {
            'tau_u_min': -max_thrust,
            'tau_u_max': max_thrust,
            'tau_r_min': -max_moment,
            'tau_r_max': max_moment
        }
        
        # Create the MPC controller
        self.controller = MPCController(
            dt=dt,
            prediction_horizon=prediction_horizon,
            Q=Q_matrix,
            R=R_matrix,
            vessel_params=vessel_params,
            state_constraints=state_constraints,
            input_constraints=input_constraints
        )
        
        # Store the prediction horizon
        self.N = prediction_horizon
        
        # Store the last waypoint for generating reference trajectory
        self.last_waypoint = None
        
    def _generate_reference_trajectory(self, 
                                      desired_position: Tuple[float, float],
                                      desired_heading: float,
                                      desired_speed: float,
                                      current_state: np.ndarray) -> List[np.ndarray]:
        """
        Generate a reference trajectory for the MPC controller.
        
        Args:
            desired_position: Desired position (x, y)
            desired_heading: Desired heading in radians
            desired_speed: Desired surge speed in m/s
            current_state: Current state vector [x, y, psi, u, v, r]
            
        Returns:
            Reference trajectory as a list of state vectors
        """
        # Create the target state
        target_state = np.array([
            desired_position[0],
            desired_position[1],
            desired_heading,
            desired_speed,
            0.0,  # Desired sway velocity (typically zero)
            0.0   # Desired yaw rate (typically zero when tracking constant heading)
        ])
        
        # Initialize the reference trajectory
        reference_trajectory = []
        
        # Simple linear interpolation from current state to target state
        for i in range(self.N + 1):
            # Interpolation factor (0 at current state, 1 at target state)
            alpha = min(1.0, i / (self.N / 2))
            
            # Interpolate position and velocity components
            interp_x = current_state[0] + alpha * (target_state[0] - current_state[0])
            interp_y = current_state[1] + alpha * (target_state[1] - current_state[1])
            interp_u = current_state[3] + alpha * (target_state[3] - current_state[3])
            interp_v = current_state[4] + alpha * (target_state[4] - current_state[4])
            interp_r = current_state[5] + alpha * (target_state[5] - current_state[5])
            
            # Special handling for heading angle to avoid discontinuity
            heading_error = target_state[2] - current_state[2]
            heading_error = ((heading_error + np.pi) % (2 * np.pi)) - np.pi
            interp_psi = current_state[2] + alpha * heading_error
            
            # Create the interpolated state
            interp_state = np.array([interp_x, interp_y, interp_psi, interp_u, interp_v, interp_r])
            
            # Add to reference trajectory
            reference_trajectory.append(interp_state)
        
        return reference_trajectory
        
    def compute_control_inputs(self,
                              desired_position: Tuple[float, float],
                              desired_heading: float,
                              desired_speed: float,
                              current_state: np.ndarray) -> Tuple[float, float]:
        """
        Compute control inputs for the USV using MPC.
        
        Args:
            desired_position: Desired position (x, y)
            desired_heading: Desired heading in radians
            desired_speed: Desired surge speed in m/s
            current_state: Current state vector [x, y, psi, u, v, r]
            
        Returns:
            Control input tuple (thrust, moment)
        """
        # Generate reference trajectory
        reference_trajectory = self._generate_reference_trajectory(
            desired_position, desired_heading, desired_speed, current_state
        )
        
        # Set reference trajectory
        self.controller.set_reference_trajectory(reference_trajectory)
        
        # Compute control inputs
        control_inputs = self.controller.compute_control_inputs(current_state)
        
        # Store the last waypoint
        self.last_waypoint = (desired_position, desired_heading, desired_speed)
        
        return control_inputs[0], control_inputs[1]  # thrust, moment
