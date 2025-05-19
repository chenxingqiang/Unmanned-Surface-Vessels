"""
Environment model for USV simulation.

This module provides classes for modeling environmental factors affecting USV operation,
including wind, waves, and currents.
"""

import numpy as np
from typing import Tuple, Dict, List, Optional


class WindModel:
    """
    Model for simulating wind effects on a USV.
    """

    def __init__(self, params: Dict = None):
        """
        Initialize the wind model.
        
        Args:
            params: Dictionary of wind parameters
        """
        # Default wind parameters
        default_params = {
            'speed': 0.0,  # Mean wind speed [m/s]
            'direction': 0.0,  # Wind direction [rad] (coming from)
            'gust_intensity': 0.2,  # Gust intensity relative to mean speed
            'gust_period': 10.0,  # Typical gust period [s]
            'turbulence_intensity': 0.1,  # Turbulence intensity relative to mean speed
            'turbulence_length_scale': 100.0,  # Turbulence length scale [m]
            'coefficients': {  # Force and moment coefficients
                'X': -0.5,
                'Y': -0.7,
                'N': -0.2
            },
            'areas': {  # Projected areas [m^2]
                'X': 1.0,
                'Y': 2.0
            }
        }
        
        self.params = default_params
        if params:
            # Update default parameters with provided ones
            self._update_nested_dict(self.params, params)
            
        # Air density [kg/m^3]
        self.rho_air = 1.225
        
        # Initialize state
        self.time = 0.0
        self.gust_phase = np.random.uniform(0, 2*np.pi)
        self.turbulence_seed = np.random.randint(0, 1000)
        
    def _update_nested_dict(self, d: Dict, u: Dict):
        """
        Update a nested dictionary with values from another dictionary.
        
        Args:
            d: Dictionary to update
            u: Dictionary with updates
        """
        for k, v in u.items():
            if isinstance(v, dict) and k in d and isinstance(d[k], dict):
                self._update_nested_dict(d[k], v)
            else:
                d[k] = v
        
    def update(self, dt: float) -> None:
        """
        Update the wind model state.
        
        Args:
            dt: Time step [s]
        """
        # Update time
        self.time += dt
        
        # Update gust phase
        if self.params['gust_period'] > 0:
            self.gust_phase += dt * 2 * np.pi / self.params['gust_period']
            self.gust_phase %= 2 * np.pi
            
    def get_wind_velocity(self) -> Tuple[float, float]:
        """
        Get the current wind velocity.
        
        Returns:
            Tuple of (wind_speed, wind_direction)
        """
        # Base wind speed
        wind_speed = self.params['speed']
        
        # Add gust component (sinusoidal)
        gust = self.params['gust_intensity'] * wind_speed * np.sin(self.gust_phase)
        
        # Add turbulence component (random)
        np.random.seed(self.turbulence_seed + int(self.time))
        turbulence = self.params['turbulence_intensity'] * wind_speed * np.random.normal()
        
        # Total wind speed
        total_wind_speed = wind_speed + gust + turbulence
        total_wind_speed = max(0, total_wind_speed)  # Ensure non-negative
        
        # Wind direction (could also include variations)
        wind_direction = self.params['direction']
        
        return total_wind_speed, wind_direction
    
    def calculate_wind_forces(self, usv_state: np.ndarray) -> np.ndarray:
        """
        Calculate wind forces and moment on the USV.
        
        Args:
            usv_state: USV state vector [x, y, psi, u, v, r]
            
        Returns:
            Array of [F_x, F_y, M_z] due to wind
        """
        # Extract USV heading
        usv_heading = usv_state[2]
        
        # Get current wind velocity
        wind_speed, wind_direction = self.get_wind_velocity()
        
        # Calculate relative wind angle
        relative_wind_angle = wind_direction - usv_heading
        
        # Normalize to [-pi, pi]
        relative_wind_angle = ((relative_wind_angle + np.pi) % (2 * np.pi)) - np.pi
        
        # Calculate wind forces using the coefficients
        # Force is 0.5 * rho * V^2 * C * A
        force_x = 0.5 * self.rho_air * wind_speed**2 * self.params['coefficients']['X'] * \
                 self.params['areas']['X'] * np.cos(relative_wind_angle)
        
        force_y = 0.5 * self.rho_air * wind_speed**2 * self.params['coefficients']['Y'] * \
                 self.params['areas']['Y'] * np.sin(relative_wind_angle)
        
        # Wind moment (simplified model)
        moment_z = 0.5 * self.rho_air * wind_speed**2 * self.params['coefficients']['N'] * \
                  self.params['areas']['Y'] * np.sin(2 * relative_wind_angle)
        
        return np.array([force_x, force_y, moment_z])


class WaveModel:
    """
    Model for simulating wave effects on a USV.
    """

    def __init__(self, params: Dict = None):
        """
        Initialize the wave model.
        
        Args:
            params: Dictionary of wave parameters
        """
        # Default wave parameters
        default_params = {
            'significant_height': 0.0,  # Significant wave height [m]
            'peak_period': 5.0,  # Peak period [s]
            'direction': 0.0,  # Wave direction [rad] (coming from)
            'spectrum_type': 'PM',  # Spectrum type ('PM' for Pierson-Moskowitz, 'JONSWAP')
            'gamma': 3.3,  # JONSWAP gamma parameter
            'num_components': 10,  # Number of wave components for simulation
            'coefficients': {  # Force and moment coefficients
                'X': -0.3,
                'Y': -0.4,
                'N': -0.1
            }
        }
        
        self.params = default_params
        if params:
            # Update default parameters with provided ones
            self._update_nested_dict(self.params, params)
            
        # Water density [kg/m^3]
        self.rho_water = 1025.0
        
        # Initialize wave components
        self._initialize_wave_components()
        
        # Initial time
        self.time = 0.0
        
    def _update_nested_dict(self, d: Dict, u: Dict):
        """
        Update a nested dictionary with values from another dictionary.
        
        Args:
            d: Dictionary to update
            u: Dictionary with updates
        """
        for k, v in u.items():
            if isinstance(v, dict) and k in d and isinstance(d[k], dict):
                self._update_nested_dict(d[k], v)
            else:
                d[k] = v
                
    def _initialize_wave_components(self):
        """
        Initialize wave components for simulation using the specified spectrum.
        """
        # Number of wave components
        N = self.params['num_components']
        
        # Frequency range
        omega_min = 0.5 * 2 * np.pi / self.params['peak_period']
        omega_max = 2.0 * 2 * np.pi / self.params['peak_period']
        
        # Generate frequencies and random phases
        self.omega = np.linspace(omega_min, omega_max, N)
        self.phase = np.random.uniform(0, 2*np.pi, N)
        
        # Calculate spectrum values
        self.S = np.zeros(N)
        for i in range(N):
            self.S[i] = self._spectrum_value(self.omega[i])
            
        # Calculate amplitudes
        d_omega = (omega_max - omega_min) / (N - 1)
        self.amplitude = np.sqrt(2 * self.S * d_omega)
        
    def _spectrum_value(self, omega: float) -> float:
        """
        Calculate the wave spectrum value at a given frequency.
        
        Args:
            omega: Angular frequency [rad/s]
            
        Returns:
            Spectrum value [m^2/(rad/s)]
        """
        # Peak frequency
        omega_p = 2 * np.pi / self.params['peak_period']
        
        # Calculate spectrum value based on the type
        if self.params['spectrum_type'] == 'PM':  # Pierson-Moskowitz
            H_s = self.params['significant_height']
            alpha = 0.0081  # Constant
            beta = 0.74  # Constant
            
            return alpha * 9.81**2 / omega**5 * np.exp(-beta * (omega_p / omega)**4)
            
        elif self.params['spectrum_type'] == 'JONSWAP':  # JONSWAP
            H_s = self.params['significant_height']
            gamma = self.params['gamma']
            alpha = 0.0081  # Constant
            
            # Spectral width parameter
            sigma = 0.07 if omega <= omega_p else 0.09
            
            # Base Pierson-Moskowitz spectrum
            S_pm = alpha * 9.81**2 / omega**5 * np.exp(-1.25 * (omega_p / omega)**4)
            
            # JONSWAP enhancement factor
            gamma_exp = np.exp(-0.5 * ((omega - omega_p) / (sigma * omega_p))**2)
            enhancement = gamma**gamma_exp
            
            return S_pm * enhancement
            
        else:
            # Default to Pierson-Moskowitz
            return self._spectrum_value(omega)
        
    def update(self, dt: float) -> None:
        """
        Update the wave model state.
        
        Args:
            dt: Time step [s]
        """
        # Update time
        self.time += dt
        
    def calculate_wave_forces(self, usv_state: np.ndarray) -> np.ndarray:
        """
        Calculate wave forces and moment on the USV.
        
        Args:
            usv_state: USV state vector [x, y, psi, u, v, r]
            
        Returns:
            Array of [F_x, F_y, M_z] due to waves
        """
        # Extract USV position and heading
        x, y, heading = usv_state[0:3]
        
        # Initialize forces and moment
        F_x = 0.0
        F_y = 0.0
        M_z = 0.0
        
        # If significant wave height is negligible, return zero forces
        if self.params['significant_height'] < 0.01:
            return np.array([F_x, F_y, M_z])
        
        # Calculate wave forces as a superposition of component forces
        for i in range(len(self.omega)):
            # Wave number (deep water approximation)
            k = self.omega[i]**2 / 9.81
            
            # Wave angle relative to USV heading
            wave_angle = self.params['direction'] - heading
            
            # Wave elevation
            phase_arg = k * (x * np.cos(self.params['direction']) + 
                            y * np.sin(self.params['direction'])) - \
                        self.omega[i] * self.time + self.phase[i]
            
            elevation = self.amplitude[i] * np.cos(phase_arg)
            
            # Simple force model based on wave elevation and relative angle
            # These are simplified approximations - real wave forces are more complex
            F_x += self.params['coefficients']['X'] * elevation * np.cos(wave_angle)
            F_y += self.params['coefficients']['Y'] * elevation * np.sin(wave_angle)
            M_z += self.params['coefficients']['N'] * elevation * np.sin(2 * wave_angle)
        
        # Scale forces by water density
        F_x *= self.rho_water * 9.81
        F_y *= self.rho_water * 9.81
        M_z *= self.rho_water * 9.81
        
        return np.array([F_x, F_y, M_z])


class CurrentModel:
    """
    Model for simulating water current effects on a USV.
    """

    def __init__(self, params: Dict = None):
        """
        Initialize the current model.
        
        Args:
            params: Dictionary of current parameters
        """
        # Default current parameters
        default_params = {
            'speed': 0.0,  # Current speed [m/s]
            'direction': 0.0,  # Current direction [rad] (flowing to)
            'variation_speed': 0.0,  # Speed variation amplitude [m/s]
            'variation_period': 600.0,  # Speed variation period [s]
            'spatial_variation': False,  # Whether to include spatial variations
            'grid_size': [1000, 1000],  # Grid size for spatial variations [m]
            'grid_resolution': [10, 10]  # Grid resolution for spatial variations
        }
        
        self.params = default_params
        if params:
            self.params.update(params)
            
        # Initialize time
        self.time = 0.0
        
        # Initialize spatial variation grid if enabled
        if self.params['spatial_variation']:
            self._initialize_spatial_grid()
        
    def _initialize_spatial_grid(self):
        """
        Initialize the spatial variation grid for current.
        """
        # Grid dimensions
        nx = self.params['grid_resolution'][0]
        ny = self.params['grid_resolution'][1]
        
        # Generate random variations
        self.speed_grid = np.random.normal(0, 0.2, (nx, ny))
        self.direction_grid = np.random.normal(0, 0.1, (nx, ny))
        
        # Smooth the grids (simple averaging)
        for _ in range(3):
            speed_smoothed = np.zeros_like(self.speed_grid)
            direction_smoothed = np.zeros_like(self.direction_grid)
            
            for i in range(nx):
                for j in range(ny):
                    # Get neighboring cells
                    i_min = max(0, i-1)
                    i_max = min(nx-1, i+1)
                    j_min = max(0, j-1)
                    j_max = min(ny-1, j+1)
                    
                    # Calculate average
                    speed_values = self.speed_grid[i_min:i_max+1, j_min:j_max+1]
                    direction_values = self.direction_grid[i_min:i_max+1, j_min:j_max+1]
                    
                    speed_smoothed[i, j] = np.mean(speed_values)
                    direction_smoothed[i, j] = np.mean(direction_values)
            
            self.speed_grid = speed_smoothed
            self.direction_grid = direction_smoothed
        
    def update(self, dt: float) -> None:
        """
        Update the current model state.
        
        Args:
            dt: Time step [s]
        """
        # Update time
        self.time += dt
        
    def get_current_velocity(self, position: Optional[np.ndarray] = None) -> Tuple[float, float]:
        """
        Get the current velocity at a given position.
        
        Args:
            position: Position [x, y] (if spatial variation is enabled)
            
        Returns:
            Tuple of (current_speed, current_direction)
        """
        # Base current speed
        current_speed = self.params['speed']
        
        # Add time variation
        if self.params['variation_speed'] > 0 and self.params['variation_period'] > 0:
            speed_variation = self.params['variation_speed'] * \
                             np.sin(2 * np.pi * self.time / self.params['variation_period'])
            current_speed += speed_variation
        
        # Base current direction
        current_direction = self.params['direction']
        
        # Add spatial variation if enabled and position provided
        if self.params['spatial_variation'] and position is not None:
            # Map position to grid indices
            grid_x = int((position[0] / self.params['grid_size'][0]) * self.params['grid_resolution'][0])
            grid_y = int((position[1] / self.params['grid_size'][1]) * self.params['grid_resolution'][1])
            
            # Clamp to grid boundaries
            grid_x = max(0, min(grid_x, self.params['grid_resolution'][0] - 1))
            grid_y = max(0, min(grid_y, self.params['grid_resolution'][1] - 1))
            
            # Get variations from grid
            speed_variation = self.speed_grid[grid_x, grid_y] * current_speed
            direction_variation = self.direction_grid[grid_x, grid_y]
            
            current_speed += speed_variation
            current_direction += direction_variation
        
        # Ensure non-negative speed
        current_speed = max(0, current_speed)
        
        # Normalize direction to [0, 2*pi)
        current_direction %= 2 * np.pi
        
        return current_speed, current_direction
    
    def calculate_current_forces(self, usv_state: np.ndarray) -> np.ndarray:
        """
        Calculate current forces on the USV.
        
        Note: Current primarily affects the USV through relative velocity rather than
        direct forces. This method is primarily for compatibility with other models.
        
        Args:
            usv_state: USV state vector [x, y, psi, u, v, r]
            
        Returns:
            Array of [F_x, F_y, M_z] (typically zeros)
        """
        # Current primarily affects USV by changing the relative velocity
        # Direct forces are typically incorporated in the USV dynamics model
        return np.zeros(3)


class Environment:
    """
    Environment model combining wind, wave, and current effects.
    """

    def __init__(self, params: Dict = None):
        """
        Initialize the environment model.
        
        Args:
            params: Dictionary of environment parameters
        """
        # Default parameters
        default_params = {
            'wind': {},
            'wave': {},
            'current': {}
        }
        
        self.params = default_params
        if params:
            # Update default parameters with provided ones
            self._update_nested_dict(self.params, params)
            
        # Create sub-models
        self.wind_model = WindModel(self.params.get('wind'))
        self.wave_model = WaveModel(self.params.get('wave'))
        self.current_model = CurrentModel(self.params.get('current'))
        
    def _update_nested_dict(self, d: Dict, u: Dict):
        """
        Update a nested dictionary with values from another dictionary.
        
        Args:
            d: Dictionary to update
            u: Dictionary with updates
        """
        for k, v in u.items():
            if isinstance(v, dict) and k in d and isinstance(d[k], dict):
                self._update_nested_dict(d[k], v)
            else:
                d[k] = v
        
    def update(self, dt: float) -> None:
        """
        Update the environment model.
        
        Args:
            dt: Time step [s]
        """
        # Update sub-models
        self.wind_model.update(dt)
        self.wave_model.update(dt)
        self.current_model.update(dt)
        
    def get_wind_velocity(self) -> Tuple[float, float]:
        """
        Get the current wind velocity.
        
        Returns:
            Tuple of (wind_speed, wind_direction)
        """
        return self.wind_model.get_wind_velocity()
    
    def get_current_velocity(self, position: Optional[np.ndarray] = None) -> Tuple[float, float]:
        """
        Get the current velocity at a given position.
        
        Args:
            position: Position [x, y]
            
        Returns:
            Tuple of (current_speed, current_direction)
        """
        return self.current_model.get_current_velocity(position)
    
    def calculate_environmental_forces(self, usv_state: np.ndarray) -> np.ndarray:
        """
        Calculate combined environmental forces and moment on the USV.
        
        Args:
            usv_state: USV state vector [x, y, psi, u, v, r]
            
        Returns:
            Array of [F_x, F_y, M_z] due to environmental effects
        """
        # Calculate forces from each model
        wind_forces = self.wind_model.calculate_wind_forces(usv_state)
        wave_forces = self.wave_model.calculate_wave_forces(usv_state)
        current_forces = self.current_model.calculate_current_forces(usv_state)
        
        # Combine forces
        total_forces = wind_forces + wave_forces + current_forces
        
        return total_forces
