"""
USV mathematical model implementation.

This module provides classes for both kinematic and dynamic models of an
Unmanned Surface Vessel (USV). It includes methods for state update, model
identification, and simulation of external disturbances.
"""

import numpy as np
from typing import Tuple, Dict, List, Optional


class USVKinematicModel:
    """
    3-DOF kinematic model for Unmanned Surface Vessel.
    State vector: [x, y, psi] - position (x,y) and heading angle (psi)
    Control inputs: [v, r] - surge velocity and yaw rate
    """

    def __init__(self, x0: np.ndarray = None, dt: float = 0.1):
        """
        Initialize the USV kinematic model.

        Args:
            x0: Initial state vector [x, y, psi]
            dt: Time step for integration
        """
        # Default initial state
        if x0 is None:
            x0 = np.zeros(3)  # [x, y, psi]

        self.state = x0
        self.dt = dt

    def update(self, u: np.ndarray) -> np.ndarray:
        """
        Update the USV state based on the kinematic model.

        Args:
            u: Control input vector [v, r] - surge velocity and yaw rate

        Returns:
            Updated state vector [x, y, psi]
        """
        # Extract current state
        x, y, psi = self.state

        # Extract control inputs
        v, r = u

        # Kinematic model equations
        x_dot = v * np.cos(psi)
        y_dot = v * np.sin(psi)
        psi_dot = r

        # Update state using Euler integration
        x_new = x + self.dt * x_dot
        y_new = y + self.dt * y_dot
        psi_new = psi + self.dt * psi_dot

        # Normalize heading angle to [-pi, pi]
        psi_new = ((psi_new + np.pi) % (2 * np.pi)) - np.pi

        # Update state
        self.state = np.array([x_new, y_new, psi_new])

        return self.state


class USVDynamicModel:
    """
    3-DOF dynamic model for Unmanned Surface Vessel based on simplified Fossen model.
    State vector: [x, y, psi, u, v, r] - position, heading, and velocities
    Control inputs: [tau_u, tau_r] - surge force and yaw moment
    """

    def __init__(self,
                 x0: np.ndarray = None,
                 dt: float = 0.1,
                 vessel_params: Dict = None):
        """
        Initialize the USV dynamic model.

        Args:
            x0: Initial state vector [x, y, psi, u, v, r]
            dt: Time step for integration
            vessel_params: Dictionary of vessel parameters
        """
        # Default initial state
        if x0 is None:
            x0 = np.zeros(6)  # [x, y, psi, u, v, r]

        self.state = x0
        self.dt = dt

        # Default vessel parameters
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

        self.params = default_params
        if vessel_params:
            self.params.update(vessel_params)

    def update(self, u: np.ndarray, disturbances: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Update the USV state based on the dynamic model.

        Args:
            u: Control input vector [tau_u, tau_r] - surge force and yaw moment
            disturbances: External disturbances [F_x, F_y, M_z]

        Returns:
            Updated state vector [x, y, psi, u, v, r]
        """
        # Extract current state
        x, y, psi, surge, sway, yaw_rate = self.state

        # Extract control inputs
        tau_u, tau_r = u

        # Rotation matrix (from body to world frame)
        R = np.array([
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi), np.cos(psi), 0],
            [0, 0, 1]
        ])

        # Position and orientation derivatives (kinematic part)
        pos_dot = R @ np.array([surge, sway, yaw_rate])

        # Mass and inertia matrix
        M = np.diag([self.params['m'], self.params['m'], self.params['Iz']])

        # Linear damping matrix
        D_lin = np.diag([-self.params['Xu'], -self.params['Yv'], -self.params['Nr']])

        # Apply velocity bounds to prevent extreme values that could cause overflow
        surge_bounded = np.clip(surge, -10.0, 10.0)  # Reasonable velocity limits
        sway_bounded = np.clip(sway, -5.0, 5.0)
        yaw_rate_bounded = np.clip(yaw_rate, -1.0, 1.0)  # Reasonable yaw rate limits

        # Quadratic damping terms with bounded velocities
        D_quad = np.array([
            self.params['Xuu'] * abs(surge_bounded) * surge_bounded,
            self.params['Yvv'] * abs(sway_bounded) * sway_bounded,
            self.params['Nrr'] * abs(yaw_rate_bounded) * yaw_rate_bounded
        ])

        # Coriolis and centripetal matrix (simplified)
        C = np.array([
            [0, 0, -self.params['m'] * sway_bounded],
            [0, 0, self.params['m'] * surge_bounded],
            [self.params['m'] * sway_bounded, -self.params['m'] * surge_bounded, 0]
        ])

        # Control input vector
        tau = np.array([tau_u, 0, tau_r])  # No direct sway control for underactuated USV

        # Add disturbances if provided
        if disturbances is not None:
            tau += disturbances

        # Velocity derivatives (dynamic part)
        vel = np.array([surge_bounded, sway_bounded, yaw_rate_bounded])
        
        try:
            vel_dot = np.linalg.solve(M, tau - C @ vel - D_lin @ vel - D_quad)
            
            # Apply reasonable acceleration limits to prevent instability
            vel_dot = np.clip(vel_dot, -5.0, 5.0)
        except np.linalg.LinAlgError:
            # Fallback in case of numerical issues
            vel_dot = np.zeros(3)
            print("Warning: Numerical issue in USV model dynamics. Using fallback.")

        # State derivative
        state_dot = np.concatenate((pos_dot, vel_dot))

        # Update state using Euler integration
        new_state = self.state + self.dt * state_dot

        # Normalize heading angle to [-pi, pi]
        new_state[2] = ((new_state[2] + np.pi) % (2 * np.pi)) - np.pi

        # Apply velocity bounds to the new state
        new_state[3] = np.clip(new_state[3], -10.0, 10.0)  # surge
        new_state[4] = np.clip(new_state[4], -5.0, 5.0)    # sway
        new_state[5] = np.clip(new_state[5], -1.0, 1.0)    # yaw rate

        # Update state
        self.state = new_state

        return self.state

    def get_position(self) -> Tuple[float, float]:
        """
        Get the current position of the USV.

        Returns:
            Tuple containing (x, y) position
        """
        return self.state[0], self.state[1]

    def get_heading(self) -> float:
        """
        Get the current heading of the USV.

        Returns:
            Heading angle in radians
        """
        return self.state[2]

    def get_velocities(self) -> Tuple[float, float, float]:
        """
        Get the current velocities of the USV.

        Returns:
            Tuple containing (surge, sway, yaw_rate)
        """
        return self.state[3], self.state[4], self.state[5]


class EnvironmentalDisturbances:
    """
    Class for simulating environmental disturbances on a USV:
    - Wind
    - Waves
    - Current
    """

    def __init__(self,
                 wind_params: Dict = None,
                 wave_params: Dict = None,
                 current_params: Dict = None):
        """
        Initialize environmental disturbance models.

        Args:
            wind_params: Dictionary of wind parameters
            wave_params: Dictionary of wave parameters
            current_params: Dictionary of current parameters
        """
        # Default wind parameters
        default_wind = {
            'speed': 0.0,  # m/s
            'direction': 0.0,  # rad (coming from)
            'gust_intensity': 0.0,  # fraction of speed
            'gust_frequency': 0.1  # Hz
        }
        self.wind_params = default_wind
        if wind_params:
            self.wind_params.update(wind_params)

        # Default wave parameters
        default_wave = {
            'height': 0.0,  # m (significant wave height)
            'direction': 0.0,  # rad (coming from)
            'period': 5.0,  # s
            'spectrum': 'PM'  # Pierson-Moskowitz
        }
        self.wave_params = default_wave
        if wave_params:
            self.wave_params.update(wave_params)

        # Default current parameters
        default_current = {
            'speed': 0.0,  # m/s
            'direction': 0.0,  # rad (going to)
            'variation': 0.0  # fraction of speed
        }
        self.current_params = default_current
        if current_params:
            self.current_params.update(current_params)

        self.time = 0.0

    def update(self, dt: float, usv_state: np.ndarray) -> np.ndarray:
        """
        Calculate environmental forces and moments acting on the USV.

        Args:
            dt: Time step
            usv_state: USV state vector [x, y, psi, u, v, r]

        Returns:
            Disturbance vector [F_x, F_y, M_z]
        """
        self.time += dt

        # Extract USV state
        _, _, psi, surge, sway, _ = usv_state

        # Calculate wind forces and moment
        wind_forces = self._calculate_wind_forces(psi)

        # Calculate wave forces and moment
        wave_forces = self._calculate_wave_forces(psi)

        # Calculate current forces and moment
        current_forces = self._calculate_current_forces(psi, surge, sway)

        # Combine all disturbances
        disturbances = wind_forces + wave_forces + current_forces

        return disturbances

    def _calculate_wind_forces(self, usv_heading: float) -> np.ndarray:
        """
        Calculate wind forces and moment.

        Args:
            usv_heading: USV heading in radians

        Returns:
            Wind disturbance vector [F_x, F_y, M_z]
        """
        if self.wind_params['speed'] == 0.0:
            return np.zeros(3)

        # Calculate effective wind speed with gusting
        gust = self.wind_params['gust_intensity'] * np.sin(2 * np.pi * self.wind_params['gust_frequency'] * self.time)
        wind_speed = self.wind_params['speed'] * (1 + gust)

        # Calculate relative wind angle (wind direction - usv heading)
        rel_angle = self.wind_params['direction'] - usv_heading

        # Calculate forces (simplified model)
        # Coefficients would normally be derived from experimental data
        cx = -0.5 * np.cos(rel_angle)  # drag coefficient
        cy = 1.0 * np.sin(rel_angle)   # lift coefficient
        cn = 0.2 * np.sin(2 * rel_angle)  # moment coefficient

        # Forces and moment
        Fx = cx * wind_speed**2
        Fy = cy * wind_speed**2
        Mz = cn * wind_speed**2

        return np.array([Fx, Fy, Mz])

    def _calculate_wave_forces(self, usv_heading: float) -> np.ndarray:
        """
        Calculate wave forces and moment.

        Args:
            usv_heading: USV heading in radians

        Returns:
            Wave disturbance vector [F_x, F_y, M_z]
        """
        if self.wave_params['height'] == 0.0:
            return np.zeros(3)

        # Calculate relative wave angle
        rel_angle = self.wave_params['direction'] - usv_heading

        # Calculate wave frequency
        omega = 2 * np.pi / self.wave_params['period']

        # Calculate wave amplitude
        amplitude = self.wave_params['height'] / 2

        # Calculate forces (simplified model)
        # In a more complex model, these would be calculated using wave spectrum
        Fx = amplitude * np.cos(omega * self.time) * np.cos(rel_angle)
        Fy = amplitude * np.cos(omega * self.time) * np.sin(rel_angle)
        Mz = 0.1 * amplitude * np.cos(omega * self.time) * np.sin(2 * rel_angle)

        return np.array([Fx, Fy, Mz])

    def _calculate_current_forces(self, usv_heading: float, surge: float, sway: float) -> np.ndarray:
        """
        Calculate current forces and moment.

        Args:
            usv_heading: USV heading in radians
            surge: USV surge velocity
            sway: USV sway velocity

        Returns:
            Current disturbance vector [F_x, F_y, M_z]
        """
        if self.current_params['speed'] == 0.0:
            return np.zeros(3)

        # Calculate relative current angle
        rel_angle = usv_heading - self.current_params['direction']

        # Calculate current velocity components in vessel frame
        u_c = -self.current_params['speed'] * np.cos(rel_angle)
        v_c = -self.current_params['speed'] * np.sin(rel_angle)

        # Calculate relative velocities
        u_r = surge - u_c
        v_r = sway - v_c

        # Calculate forces (simplified model)
        # In reality, these would depend on the vessel's hydrodynamic coefficients
        Fx = -10.0 * u_r * abs(u_r)  # Quadratic drag
        Fy = -15.0 * v_r * abs(v_r)  # Quadratic drag
        Mz = -2.0 * v_r * abs(v_r)    # Moment due to lateral drag

        return np.array([Fx, Fy, Mz])
