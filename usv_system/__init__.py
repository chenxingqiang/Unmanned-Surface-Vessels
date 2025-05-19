"""
Unmanned Surface Vessel (USV) Navigation and Control System.

This package provides a comprehensive framework for simulating, 
controlling, and testing unmanned surface vessels in various environments.
"""

__version__ = '1.0.0'

# Import main components for easier access
from usv_system.models.usv_model import USVDynamicModel
from usv_system.models.environment import Environment
from usv_system.control.pid_controller import USVPIDController
from usv_system.control.lqr_controller import USVLQRController
from usv_system.control.mpc_controller import USVMPCController
from usv_system.simulation.simulator import USVSimulator

# Define high-level API functions
def create_simulator(config_file=None, **kwargs):
    """
    Create a simulator instance with optional configuration.
    
    Args:
        config_file: Path to YAML configuration file
        **kwargs: Additional configuration parameters
        
    Returns:
        USVSimulator instance
    """
    import yaml
    import numpy as np
    
    if config_file:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
            
        # Extract simulation parameters
        dt = config['simulation'].get('dt', 0.1)
        simulation_time = config['simulation'].get('simulation_time', 100.0)
        x0 = np.array(config['simulation'].get('initial_state', [0, 0, 0, 0, 0, 0]))
        vessel_params = config.get('vessel', None)
        env_params = config.get('environment', None)
        controller_type = kwargs.get('controller_type', 'pid')
        controller_params = config['controllers'].get(controller_type, {})
        
        # Override with kwargs if provided
        dt = kwargs.get('dt', dt)
        simulation_time = kwargs.get('simulation_time', simulation_time)
        x0 = kwargs.get('x0', x0)
        vessel_params = kwargs.get('vessel_params', vessel_params)
        env_params = kwargs.get('env_params', env_params)
        controller_params = kwargs.get('controller_params', controller_params)
        
        # Create simulator
        simulator = USVSimulator(
            dt=dt,
            simulation_time=simulation_time,
            x0=x0,
            vessel_params=vessel_params,
            env_params=env_params,
            controller_type=controller_type,
            controller_params=controller_params
        )
        
        return simulator
    else:
        # Create simulator with provided kwargs only
        return USVSimulator(**kwargs)

