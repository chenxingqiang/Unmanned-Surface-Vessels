"""
Run a comprehensive comparison test for USV controllers.

This script runs multiple test scenarios to compare PID, LQR, and MPC controllers
in different conditions and saves the output plots.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import traceback
from typing import Dict, List, Tuple

# Add project root to path to ensure imports work
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from usv_system.simulation.simulator import USVSimulator


def setup_simulator(scenario: str) -> Tuple[USVSimulator, List[Dict], List[Dict]]:
    """
    Create and configure a simulator based on the scenario.
    
    Args:
        scenario: Scenario name ('waypoint', 'station', 'disturbance', 'highspeed', 'obstacle')
        
    Returns:
        Tuple of (simulator, controller_configs, trajectory)
    """
    # Common vessel parameters
    vessel_params = {
        'm': 50.0,    # mass (kg)
        'Iz': 20.0,   # moment of inertia (kg.m^2)
        'Xu': -25.0,  # surge linear damping
        'Yv': -40.0,  # sway linear damping
        'Nr': -10.0,  # yaw linear damping
    }
    
    # Base simulation parameters
    sim_params = {
        'dt': 0.1,
        'x0': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),  # Start at origin
        'vessel_params': vessel_params,
    }
    
    # Base controller configurations
    controller_configs = [
        {
            'type': 'pid',
            'name': 'PID Controller',
            'params': {
                'Kp_heading': 5.0,
                'Ki_heading': 0.01,
                'Kd_heading': 2.0,
                'Kp_speed': 10.0,
                'Ki_speed': 0.1,
                'Kd_speed': 1.0,
                'Kp_position': 2.0
            }
        },
        {
            'type': 'lqr',
            'name': 'LQR Controller',
            'params': {
                'Q': {
                    'x': 10.0, 'y': 10.0, 'psi': 20.0,
                    'u': 1.0, 'v': 1.0, 'r': 5.0
                },
                'R': {
                    'tau_u': 1.0, 'tau_r': 1.0
                }
            }
        },
        {
            'type': 'mpc',
            'name': 'MPC Controller',
            'params': {
                'prediction_horizon': 10,
                'Q': {
                    'x': 10.0, 'y': 10.0, 'psi': 20.0,
                    'u': 1.0, 'v': 1.0, 'r': 5.0
                },
                'R': {
                    'tau_u': 1.0, 'tau_r': 1.0
                }
            }
        }
    ]
    
    # Configure for specific scenario
    if scenario == 'waypoint':
        # Square path waypoint following
        trajectory = [
            {'position': (0.0, 0.0), 'heading': 0.0, 'speed': 0.0, 'time': 0.0},
            {'position': (50.0, 0.0), 'heading': 0.0, 'speed': 2.0, 'time': 40.0},
            {'position': (50.0, 50.0), 'heading': np.pi/2, 'speed': 2.0, 'time': 80.0},
            {'position': (0.0, 50.0), 'heading': np.pi, 'speed': 2.0, 'time': 120.0},
            {'position': (0.0, 0.0), 'heading': -np.pi/2, 'speed': 2.0, 'time': 160.0},
            {'position': (0.0, 0.0), 'heading': 0.0, 'speed': 0.0, 'time': 180.0}
        ]
        
        sim_params.update({
            'simulation_time': 200.0,
            'env_params': {
                'wind': {'speed': 2.0, 'direction': np.pi/4},
                'wave': {'significant_height': 0.3, 'peak_period': 4.0, 'direction': np.pi/4},
                'current': {'speed': 0.2, 'direction': np.pi/2}
            }
        })
        
    elif scenario == 'station':
        # Station keeping with disturbances
        trajectory = [
            {'position': (0.0, 0.0), 'heading': 0.0, 'speed': 0.0}
        ]
        
        sim_params.update({
            'simulation_time': 100.0,
            'x0': np.array([10.0, 10.0, np.pi/4, 0.0, 0.0, 0.0]),  # Start away from target
            'env_params': {
                'wind': {'speed': 5.0, 'direction': np.pi/2, 'gust_intensity': 0.3},
                'wave': {'significant_height': 0.5, 'peak_period': 4.0, 'direction': np.pi/2},
                'current': {'speed': 0.5, 'direction': np.pi/2}
            }
        })
        
        # Adjust controller gains for better station keeping
        for config in controller_configs:
            if config['type'] == 'pid':
                config['params'].update({
                    'Kp_heading': 8.0,
                    'Ki_heading': 0.05,
                    'Kd_heading': 4.0,
                    'Kp_speed': 15.0,
                    'Ki_speed': 0.2,
                    'Kd_speed': 2.0,
                    'Kp_position': 3.0
                })
            elif config['type'] == 'lqr' or config['type'] == 'mpc':
                config['params']['Q'].update({
                    'x': 20.0, 'y': 20.0, 'psi': 30.0
                })
        
    elif scenario == 'disturbance':
        # Path following with varying disturbances
        trajectory = [
            {'position': (0.0, 0.0), 'heading': np.pi/2, 'speed': 0.0, 'time': 0.0},
            {'position': (0.0, 0.0), 'heading': np.pi/2, 'speed': 2.0, 'time': 10.0},
            {'position': (0.0, 100.0), 'heading': np.pi/2, 'speed': 2.0, 'time': 100.0},
            {'position': (0.0, 100.0), 'heading': np.pi/2, 'speed': 0.0, 'time': 110.0}
        ]
        
        sim_params.update({
            'simulation_time': 120.0,
            'env_params': {
                'wind': {
                    'speed': 3.0,
                    'direction': 0.0,
                    'gust_intensity': 0.4,
                    'gust_period': 8.0
                },
                'wave': {
                    'significant_height': 0.4,
                    'peak_period': 5.0,
                    'direction': 0.0
                },
                'current': {
                    'speed': 0.3,
                    'direction': 0.0,
                    'variation_speed': 0.2,
                    'variation_period': 30.0
                }
            }
        })
        
    elif scenario == 'highspeed':
        # High-speed maneuvers
        trajectory = [
            {'position': (0.0, 0.0), 'heading': 0.0, 'speed': 0.0, 'time': 0.0},
            {'position': (0.0, 0.0), 'heading': 0.0, 'speed': 4.0, 'time': 10.0},
            {'position': (50.0, 0.0), 'heading': 0.0, 'speed': 4.0, 'time': 30.0},
            {'position': (50.0, 0.0), 'heading': np.pi/2, 'speed': 4.0, 'time': 35.0},
            {'position': (50.0, 50.0), 'heading': np.pi/2, 'speed': 4.0, 'time': 55.0},
            {'position': (50.0, 50.0), 'heading': np.pi, 'speed': 4.0, 'time': 60.0},
            {'position': (0.0, 50.0), 'heading': np.pi, 'speed': 4.0, 'time': 80.0},
            {'position': (0.0, 50.0), 'heading': -np.pi/2, 'speed': 4.0, 'time': 85.0},
            {'position': (0.0, 0.0), 'heading': -np.pi/2, 'speed': 4.0, 'time': 105.0},
            {'position': (0.0, 0.0), 'heading': 0.0, 'speed': 0.0, 'time': 110.0}
        ]
        
        sim_params.update({
            'simulation_time': 120.0,
            'env_params': {
                'wind': {'speed': 1.0, 'direction': np.pi/4},
                'wave': {'significant_height': 0.2, 'peak_period': 4.0, 'direction': np.pi/4},
                'current': {'speed': 0.1, 'direction': np.pi/4}
            }
        })
        
        # Adjust controller gains for faster response
        for config in controller_configs:
            if config['type'] == 'pid':
                config['params'].update({
                    'Kp_heading': 10.0,
                    'Ki_heading': 0.05,
                    'Kd_heading': 5.0,
                    'Kp_speed': 20.0,
                    'Ki_speed': 0.2,
                    'Kd_speed': 3.0,
                    'Kp_position': 4.0
                })
            elif config['type'] == 'lqr' or config['type'] == 'mpc':
                config['params']['Q'].update({
                    'x': 20.0, 'y': 20.0, 'psi': 40.0,
                    'u': 4.0, 'v': 4.0, 'r': 10.0
                })
                config['params']['R'].update({
                    'tau_u': 0.5, 'tau_r': 0.5
                })
        
    elif scenario == 'obstacle':
        # Obstacle avoidance scenario
        trajectory = [
            {'position': (0.0, 0.0), 'heading': 0.0, 'speed': 0.0, 'time': 0.0},
            {'position': (0.0, 0.0), 'heading': 0.0, 'speed': 2.0, 'time': 10.0},
            {'position': (40.0, 0.0), 'heading': 0.0, 'speed': 2.0, 'time': 50.0},
            {'position': (45.0, 10.0), 'heading': np.pi/4, 'speed': 2.0, 'time': 65.0},
            {'position': (55.0, 10.0), 'heading': 0.0, 'speed': 2.0, 'time': 80.0},
            {'position': (60.0, 0.0), 'heading': -np.pi/4, 'speed': 2.0, 'time': 95.0},
            {'position': (100.0, 0.0), 'heading': 0.0, 'speed': 2.0, 'time': 135.0},
            {'position': (100.0, 0.0), 'heading': 0.0, 'speed': 0.0, 'time': 145.0}
        ]
        
        sim_params.update({
            'simulation_time': 150.0,
            'env_params': {
                'wind': {'speed': 2.0, 'direction': np.pi/2},
                'wave': {'significant_height': 0.3, 'peak_period': 4.0, 'direction': np.pi/2},
                'current': {'speed': 0.2, 'direction': np.pi/2}
            }
        })
        
    else:
        raise ValueError(f"Unknown scenario: {scenario}")
    
    # Create simulator
    simulator = USVSimulator(**sim_params)
    
    return simulator, controller_configs, trajectory


def run_scenario(scenario: str, output_dir: str, controllers=None, config=None, verbose=False):
    """
    Run a comparison test for a specific scenario.
    
    Args:
        scenario: Scenario name
        output_dir: Directory to save output plots
        controllers: List of controller types to compare (default: all)
        config: Configuration dictionary (if None, uses defaults)
        verbose: Whether to print verbose output
    """
    try:
        print(f"Running scenario: {scenario}")
        
        if config is None:
            # Use default parameters if no config provided
            simulator, controller_configs, trajectory = setup_simulator(scenario)
        else:
            # Use configuration from config file
            simulator = USVSimulator(
                dt=config['simulation']['dt'],
                simulation_time=config['simulation']['simulation_time'],
                x0=np.array(config['simulation']['initial_state']),
                vessel_params=config['vessel'],
                env_params=config.get('environment', {})
            )
            
            # Set up controller configurations
            controller_configs = []
            if controllers is None:
                controllers = ['pid', 'lqr', 'mpc']
                
            for controller_type in controllers:
                if controller_type in config['controllers']:
                    controller_configs.append({
                        'type': controller_type,
                        'name': controller_type.upper(),
                        'params': config['controllers'][controller_type]
                    })
            
            # Apply scenario-specific environment overrides if present
            if scenario in config['scenarios'] and 'environment' in config['scenarios'][scenario]:
                scenario_env = config['scenarios'][scenario]['environment']
                # Deep merge environment parameters
                for key, value in scenario_env.items():
                    if key in simulator.env_model.params:
                        if isinstance(value, dict) and isinstance(simulator.env_model.params[key], dict):
                            simulator.env_model.params[key].update(value)
                        else:
                            simulator.env_model.params[key] = value
            
        # Set up the reference trajectory
        if config is not None and scenario in config['scenarios'] and 'reference_trajectory' in config['scenarios'][scenario]:
            # Use reference trajectory from config
            reference_config = config['scenarios'][scenario]['reference_trajectory']
            simulator.set_reference_trajectory(reference_config)
        else:
            # Use default trajectory from setup_simulator
            _, _, trajectory = setup_simulator(scenario)
            simulator.reference_trajectory = trajectory
        
        # Run controller comparison
        plot_path = os.path.join(output_dir, f"{scenario}_comparison.png")
        simulator.compare_controllers(controller_configs, simulator.reference_trajectory, save_path=plot_path)
        
        if verbose:
            print(f"Saved comparison plot to: {plot_path}")
        
    except Exception as e:
        print(f"Error running scenario {scenario}: {str(e)}")
        traceback.print_exc()


def main():
    """
    Main function to run all test scenarios.
    """
    # Create output directory for plots if it doesn't exist
    output_dir = os.path.join(os.path.dirname(__file__), 'outputs')
    os.makedirs(output_dir, exist_ok=True)
    print(f"Output directory: {output_dir}")
    
    # Define scenarios to run
    scenarios = ['waypoint', 'station', 'disturbance', 'highspeed', 'obstacle']
    print(f"Will run these scenarios: {scenarios}")
    
    # Set Matplotlib backend to avoid GUI issues
    plt.switch_backend('agg')
    
    # Run each scenario
    for scenario in scenarios:
        print(f"\n{'='*50}\nRunning scenario: {scenario}\n{'='*50}")
        run_scenario(scenario, output_dir, verbose=True)
    
    print("All scenarios completed.")


if __name__ == "__main__":
    print("Starting USV controller comparison tests...")
    main()
    print("All tests completed.") 