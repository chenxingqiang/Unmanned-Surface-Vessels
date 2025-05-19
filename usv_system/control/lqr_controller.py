"""
LQR controller implementation for USV control.

This module provides an LQR controller class for controlling an Unmanned Surface Vessel.
The Linear Quadratic Regulator (LQR) is an optimal control method that minimizes a
quadratic cost function of states and control inputs.
"""

import numpy as np
import scipy.linalg
from typing import Tuple, Dict, List, Optional


class LQRController:
    """
    LQR controller implementation for USV control.
    """

    def __init__(self,
                 dt: float = 0.1,
                 Q: np.ndarray = None,
                 R: np.ndarray = None,
                 vessel_params: Dict = None,
                 output_limits: Dict = None):
        """
        Initialize the LQR controller.

        Args:
            dt: Time step
            Q: State weight matrix
            R: Control input weight matrix
            vessel_params: Dictionary of vessel parameters for linearization
            output_limits: Dictionary of output limits
        """
        self.dt = dt
        
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
            
        # Default output limits
        default_limits = {
            'tau_u_max': 100.0,    # Maximum surge force (N)
            'tau_u_min': -100.0,   # Minimum surge force (N)
            'tau_r_max': 50.0,     # Maximum yaw moment (N.m)
            'tau_r_min': -50.0     # Minimum yaw moment (N.m)
        }
        
        self.output_limits = default_limits
        if output_limits:
            self.output_limits.update(output_limits)
            
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
            
        # Calculate the LQR gain matrix
        self.K = self._calculate_lqr_gain()
        
        # Initialize reference and error states
        self.reference_state = np.zeros(6)
        
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
        
    def _calculate_lqr_gain(self) -> np.ndarray:
        """
        Calculate the LQR gain matrix K.
        
        Returns:
            K: LQR gain matrix
        """
        # Linearize the model
        A, B = self._linearize_model()
        
        # Discretize the continuous-time system (zero-order hold)
        Ad = np.eye(6) + self.dt * A
        Bd = self.dt * B
        
        try:
            # Try to solve the discrete-time algebraic Riccati equation
            P = scipy.linalg.solve_discrete_are(Ad, Bd, self.Q, self.R)
            
            # Calculate the LQR gain matrix
            K = np.linalg.inv(self.R + Bd.T @ P @ Bd) @ Bd.T @ P @ Ad
            
        except np.linalg.LinAlgError:
            # If the Riccati solution fails, use a more robust numerical approach
            print("Warning: Failed to solve Riccati equation. Using alternative method.")
            
            # Add a small regularization term to ensure numerical stability
            Q_reg = self.Q + 1e-6 * np.eye(6)
            R_reg = self.R + 1e-6 * np.eye(2)
            
            # Use a simpler approach based on pole placement
            # Select default gains based on the vessel parameters
            k_pos = 0.5   # Position gain
            k_vel = 2.0   # Velocity gain
            k_angle = 1.0 # Angle gain
            
            # Create a diagonal gain matrix
            K = np.zeros((2, 6))
            
            # Surge thrust control affects x position and surge velocity
            K[0, 0] = k_pos  # x position
            K[0, 3] = k_vel  # surge velocity
            
            # Yaw moment control affects heading and yaw rate
            K[1, 2] = k_angle  # heading
            K[1, 5] = k_vel    # yaw rate
            
            # Apply scaling based on Q and R weights
            for i in range(6):
                K[:, i] *= np.sqrt(Q_reg[i, i] / np.mean(np.diag(R_reg)))
        
        return K
        
    def set_reference(self, reference_state: np.ndarray):
        """
        Set the reference state for the controller.
        
        Args:
            reference_state: Reference state vector [x, y, psi, u, v, r]
        """
        self.reference_state = reference_state
        
    def compute_control_inputs(self, current_state: np.ndarray) -> np.ndarray:
        """
        Compute control inputs using the LQR controller.
        
        Args:
            current_state: Current state vector [x, y, psi, u, v, r]
            
        Returns:
            Control input vector [tau_u, tau_r]
        """
        # Calculate state error (accounting for angle wrapping)
        error = current_state - self.reference_state
        error[2] = ((error[2] + np.pi) % (2 * np.pi)) - np.pi  # Normalize heading error
        
        # Compute control inputs using LQR gain
        control_inputs = -self.K @ error
        
        # Apply control limits
        tau_u = np.clip(control_inputs[0], self.output_limits['tau_u_min'], self.output_limits['tau_u_max'])
        tau_r = np.clip(control_inputs[1], self.output_limits['tau_r_min'], self.output_limits['tau_r_max'])
        
        return np.array([tau_u, tau_r])
        
    def update_gains(self, Q: np.ndarray = None, R: np.ndarray = None):
        """
        Update the LQR gain matrix with new weights.
        
        Args:
            Q: New state weight matrix
            R: New control input weight matrix
        """
        if Q is not None:
            self.Q = Q
        if R is not None:
            self.R = R
            
        # Recalculate the LQR gain matrix
        self.K = self._calculate_lqr_gain()


class USVLQRController:
    """
    LQR controller for heading and speed control of a USV.
    """
    
    def __init__(self,
                 Q: Dict[str, float] = None,
                 R: Dict[str, float] = None,
                 vessel_params: Dict = None,
                 max_thrust: float = 100.0,
                 max_moment: float = 50.0,
                 dt: float = 0.1):
        """
        Initialize the USV LQR controller.
        
        Args:
            Q: Dictionary of state weights
            R: Dictionary of control input weights
            vessel_params: Dictionary of vessel parameters
            max_thrust: Maximum thrust force in Newtons
            max_moment: Maximum yaw moment in Newton-meters
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
        
        # Create output limits dictionary
        output_limits = {
            'tau_u_max': max_thrust,
            'tau_u_min': -max_thrust,
            'tau_r_max': max_moment,
            'tau_r_min': -max_moment
        }
        
        # Create the LQR controller
        self.controller = LQRController(
            dt=dt,
            Q=Q_matrix,
            R=R_matrix,
            vessel_params=vessel_params,
            output_limits=output_limits
        )
        
    def compute_control_inputs(self,
                              desired_position: Tuple[float, float],
                              desired_heading: float,
                              desired_speed: float,
                              current_state: np.ndarray) -> Tuple[float, float]:
        """
        Compute control inputs for the USV using LQR.
        
        Args:
            desired_position: Desired position (x, y)
            desired_heading: Desired heading in radians
            desired_speed: Desired surge speed in m/s
            current_state: Current state vector [x, y, psi, u, v, r]
            
        Returns:
            Control input tuple (thrust, moment)
        """
        # Create reference state vector
        reference_state = np.array([
            desired_position[0],
            desired_position[1],
            desired_heading,
            desired_speed,
            0.0,  # Desired sway velocity (typically zero)
            0.0   # Desired yaw rate (typically zero when tracking constant heading)
        ])
        
        # Set reference state
        self.controller.set_reference(reference_state)
        
        # Compute control inputs
        control_inputs = self.controller.compute_control_inputs(current_state)
        
        return control_inputs[0], control_inputs[1]  # thrust, moment
