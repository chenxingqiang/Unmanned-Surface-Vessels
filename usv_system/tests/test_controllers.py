"""
Test script for comparing different USV control strategies.

This script provides tests to compare PID, LQR, and MPC controllers
in different scenarios such as waypoint following, station keeping,
and disturbance rejection.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List, Tuple

# Add project root to path to ensure imports work
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from usv_system.simulation.simulator import USVSimulator


def waypoint_following_test():
    """
    Test the controllers in a waypoint following scenario.
    """
    print("Running waypoint following test...")

    # Define common simulation parameters
    sim_params = {
        'dt': 0.1,
        'simulation_time': 200.0,
        'x0': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),  # Start at origin, zero heading
        'vessel_params': {
            'm': 50.0,    # mass (kg)
            'Iz': 20.0,   # moment of inertia (kg.m^2)
            'Xu': -25.0,  # surge linear damping
            'Yv': -40.0,  # sway linear damping
            'Nr': -10.0,  # yaw linear damping
        },
        'env_params': {
            'wind': {
                'speed': 2.0,           # Wind speed (m/s)
                'direction': np.pi/4,   # Wind direction (rad)
            },
            'wave': {
                'significant_height': 0.3,  # Wave height (m)
                'peak_period': 4.0,         # Wave period (s)
                'direction': np.pi/4,        # Wave direction (rad)
            },
            'current': {
                'speed': 0.2,           # Current speed (m/s)
                'direction': np.pi/2,   # Current direction (rad)
            }
        }
    }

    # Create simulator
    simulator = USVSimulator(**sim_params)

    # Define waypoints for the test
    waypoint_trajectory = [
        {
            'position': (0.0, 0.0),
            'heading': 0.0,
            'speed': 0.0,
            'time': 0.0
        },
        {
            'position': (50.0, 0.0),
            'heading': 0.0,
            'speed': 2.0,
            'time': 40.0
        },
        {
            'position': (50.0, 50.0),
            'heading': np.pi/2,
            'speed': 2.0,
            'time': 80.0
        },
        {
            'position': (0.0, 50.0),
            'heading': np.pi,
            'speed': 2.0,
            'time': 120.0
        },
        {
            'position': (0.0, 0.0),
            'heading': -np.pi/2,
            'speed': 2.0,
            'time': 160.0
        },
        {
            'position': (0.0, 0.0),
            'heading': 0.0,
            'speed': 0.0,
            'time': 180.0
        }
    ]

    # Define controllers to compare
    controllers = [
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

    # Run the comparison
    simulator.compare_controllers(controllers, waypoint_trajectory)

    print("Waypoint following test completed.")


def station_keeping_test():
    """
    Test the controllers in a station keeping scenario with disturbances.
    """
    print("Running station keeping test...")

    # Define common simulation parameters
    sim_params = {
        'dt': 0.1,
        'simulation_time': 100.0,
        'x0': np.array([10.0, 10.0, np.pi/4, 0.0, 0.0, 0.0]),  # Start away from target
        'vessel_params': {
            'm': 50.0,    # mass (kg)
            'Iz': 20.0,   # moment of inertia (kg.m^2)
            'Xu': -25.0,  # surge linear damping
            'Yv': -40.0,  # sway linear damping
            'Nr': -10.0,  # yaw linear damping
        },
        'env_params': {
            'wind': {
                'speed': 5.0,           # Stronger wind (m/s)
                'direction': np.pi/2,   # Wind direction (rad)
                'gust_intensity': 0.3,  # Add gusts
            },
            'wave': {
                'significant_height': 0.5,  # Larger waves (m)
                'peak_period': 4.0,         # Wave period (s)
                'direction': np.pi/2,        # Wave direction (rad)
            },
            'current': {
                'speed': 0.5,           # Stronger current (m/s)
                'direction': np.pi/2,   # Current direction (rad)
            }
        }
    }

    # Create simulator
    simulator = USVSimulator(**sim_params)

    # Define station keeping target
    station_target = [
        {
            'position': (0.0, 0.0),
            'heading': 0.0,
            'speed': 0.0
        }
    ]

    # Define controllers to compare
    controllers = [
        {
            'type': 'pid',
            'name': 'PID Controller',
            'params': {
                'Kp_heading': 8.0,  # Increased for disturbance rejection
                'Ki_heading': 0.05,
                'Kd_heading': 4.0,
                'Kp_speed': 15.0,   # Increased for disturbance rejection
                'Ki_speed': 0.2,
                'Kd_speed': 2.0,
                'Kp_position': 3.0
            }
        },
        {
            'type': 'lqr',
            'name': 'LQR Controller',
            'params': {
                'Q': {
                    'x': 20.0, 'y': 20.0, 'psi': 30.0,  # Increased for station keeping
                    'u': 2.0, 'v': 2.0, 'r': 8.0
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
                'prediction_horizon': 15,  # Longer horizon for disturbance anticipation
                'Q': {
                    'x': 20.0, 'y': 20.0, 'psi': 30.0,  # Increased for station keeping
                    'u': 2.0, 'v': 2.0, 'r': 8.0
                },
                'R': {
                    'tau_u': 1.0, 'tau_r': 1.0
                }
            }
        }
    ]

    # Run the comparison
    simulator.compare_controllers(controllers, station_target)

    print("Station keeping test completed.")


def disturbance_rejection_test():
    """
    Test the controllers in a path following scenario with varying disturbances.
    """
    print("Running disturbance rejection test...")

    # Define common simulation parameters
    sim_params = {
        'dt': 0.1,
        'simulation_time': 200.0,
        'x0': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),  # Start at origin
        'vessel_params': {
            'm': 50.0,    # mass (kg)
            'Iz': 20.0,   # moment of inertia (kg.m^2)
            'Xu': -25.0,  # surge linear damping
            'Yv': -40.0,  # sway linear damping
            'Nr': -10.0,  # yaw linear damping
        },
        'env_params': {
            'wind': {
                'speed': 3.0,           # Moderate wind (m/s)
                'direction': 0.0,       # Wind from the east
                'gust_intensity': 0.4,  # Significant gusts
                'gust_period': 8.0,     # Frequent gusts
            },
            'wave': {
                'significant_height': 0.4,  # Moderate waves (m)
                'peak_period': 5.0,         # Wave period (s)
                'direction': 0.0,           # Waves from the east
            },
            'current': {
                'speed': 0.3,           # Moderate current (m/s)
                'direction': 0.0,       # Current from the east
                'variation_speed': 0.2, # Varying current
                'variation_period': 30.0,
            }
        }
    }

    # Create simulator
    simulator = USVSimulator(**sim_params)

    # Define a straight path with constant speed
    straight_path = [
        {
            'position': (0.0, 0.0),
            'heading': np.pi/2,  # North direction
            'speed': 0.0,
            'time': 0.0
        },
        {
            'position': (0.0, 0.0),
            'heading': np.pi/2,
            'speed': 2.0,
            'time': 10.0
        },
        {
            'position': (0.0, 100.0),
            'heading': np.pi/2,
            'speed': 2.0,
            'time': 100.0
        },
        {
            'position': (0.0, 100.0),
            'heading': np.pi/2,
            'speed': 0.0,
            'time': 110.0
        }
    ]

    # Define controllers to compare
    controllers = [
        {
            'type': 'pid',
            'name': 'PID Controller',
            'params': {
                'Kp_heading': 8.0,
                'Ki_heading': 0.1,
                'Kd_heading': 4.0,
                'Kp_speed': 15.0,
                'Ki_speed': 0.3,
                'Kd_speed': 2.0,
                'Kp_position': 3.0
            }
        },
        {
            'type': 'lqr',
            'name': 'LQR Controller',
            'params': {
                'Q': {
                    'x': 15.0, 'y': 15.0, 'psi': 25.0,
                    'u': 2.0, 'v': 2.0, 'r': 8.0
                },
                'R': {
                    'tau_u': 0.8, 'tau_r': 0.8  # Slightly lower control cost for better disturbance rejection
                }
            }
        },
        {
            'type': 'mpc',
            'name': 'MPC Controller',
            'params': {
                'prediction_horizon': 20,  # Extended horizon for disturbance prediction
                'Q': {
                    'x': 15.0, 'y': 15.0, 'psi': 25.0,
                    'u': 2.0, 'v': 2.0, 'r': 8.0
                },
                'R': {
                    'tau_u': 0.8, 'tau_r': 0.8  # Slightly lower control cost for better disturbance rejection
                }
            }
        }
    ]

    # Run the comparison
    simulator.compare_controllers(controllers, straight_path)

    print("Disturbance rejection test completed.")


def high_speed_maneuver_test():
    """
    Test the controllers in a high speed maneuvering scenario.
    """
    print("Running high speed maneuver test...")

    # Define common simulation parameters
    sim_params = {
        'dt': 0.1,
        'simulation_time': 150.0,
        'x0': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),  # Start at origin
        'vessel_params': {
            'm': 50.0,    # mass (kg)
            'Iz': 20.0,   # moment of inertia (kg.m^2)
            'Xu': -25.0,  # surge linear damping
            'Yv': -40.0,  # sway linear damping
            'Nr': -10.0,  # yaw linear damping
        },
        'env_params': {
            'wind': {
                'speed': 1.0,           # Light wind (m/s)
                'direction': np.pi/4,   # Wind direction
            },
            'wave': {
                'significant_height': 0.2,  # Small waves (m)
                'peak_period': 4.0,         # Wave period (s)
                'direction': np.pi/4,        # Wave direction
            },
            'current': {
                'speed': 0.1,           # Light current (m/s)
                'direction': np.pi/4,   # Current direction
            }
        }
    }

    # Create simulator
    simulator = USVSimulator(**sim_params)

    # Define a path with high speed and rapid turns
    high_speed_path = [
        {
            'position': (0.0, 0.0),
            'heading': 0.0,
            'speed': 0.0,
            'time': 0.0
        },
        {
            'position': (0.0, 0.0),
            'heading': 0.0,
            'speed': 4.0,  # High speed
            'time': 10.0
        },
        {
            'position': (50.0, 0.0),
            'heading': 0.0,
            'speed': 4.0,
            'time': 30.0
        },
        {
            'position': (50.0, 0.0),
            'heading': np.pi/2,  # 90-degree turn
            'speed': 4.0,
            'time': 35.0  # Fast turn
        },
        {
            'position': (50.0, 50.0),
            'heading': np.pi/2,
            'speed': 4.0,
            'time': 55.0
        },
        {
            'position': (50.0, 50.0),
            'heading': np.pi,  # 90-degree turn
            'speed': 4.0,
            'time': 60.0  # Fast turn
        },
        {
            'position': (0.0, 50.0),
            'heading': np.pi,
            'speed': 4.0,
            'time': 80.0
        },
        {
            'position': (0.0, 50.0),
            'heading': -np.pi/2,  # 90-degree turn
            'speed': 4.0,
            'time': 85.0  # Fast turn
        },
        {
            'position': (0.0, 0.0),
            'heading': -np.pi/2,
            'speed': 4.0,
            'time': 105.0
        },
        {
            'position': (0.0, 0.0),
            'heading': 0.0,  # 90-degree turn
            'speed': 0.0,  # Final stop
            'time': 110.0
        }
    ]

    # Define controllers to compare
    controllers = [
        {
            'type': 'pid',
            'name': 'PID Controller',
            'params': {
                'Kp_heading': 10.0,  # Increased for faster response
                'Ki_heading': 0.05,
                'Kd_heading': 5.0,   # Increased for faster response
                'Kp_speed': 20.0,    # Increased for faster response
                'Ki_speed': 0.2,
                'Kd_speed': 3.0,     # Increased for faster response
                'Kp_position': 4.0    # Increased for better tracking
            }
        },
        {
            'type': 'lqr',
            'name': 'LQR Controller',
            'params': {
                'Q': {
                    'x': 20.0, 'y': 20.0, 'psi': 40.0,  # Increased for better tracking
                    'u': 4.0, 'v': 4.0, 'r': 10.0      # Increased for better control
                },
                'R': {
                    'tau_u': 0.5, 'tau_r': 0.5  # Lower control cost for more aggressive control
                }
            }
        },
        {
            'type': 'mpc',
            'name': 'MPC Controller',
            'params': {
                'prediction_horizon': 15,
                'Q': {
                    'x': 20.0, 'y': 20.0, 'psi': 40.0,  # Increased for better tracking
                    'u': 4.0, 'v': 4.0, 'r': 10.0      # Increased for better control
                },
                'R': {
                    'tau_u': 0.5, 'tau_r': 0.5  # Lower control cost for more aggressive control
                }
            }
        }
    ]

    # Run the comparison
    simulator.compare_controllers(controllers, high_speed_path)

    print("High speed maneuver test completed.")


def obstacle_avoidance_path():
    """
    Create a path that simulates obstacle avoidance.
    """
    # Initial segment
    path = [
        {
            'position': (0.0, 0.0),
            'heading': 0.0,
            'speed': 0.0,
            'time': 0.0
        },
        {
            'position': (0.0, 0.0),
            'heading': 0.0,
            'speed': 2.0,
            'time': 10.0
        },
        {
            'position': (40.0, 0.0),
            'heading': 0.0,
            'speed': 2.0,
            'time': 50.0
        }
    ]
    
    # Add obstacle avoidance maneuver (imagine obstacle at (50, 0))
    # Deviation to avoid obstacle
    path.append({
        'position': (45.0, 10.0),
        'heading': np.pi/4,  # 45-degree heading
        'speed': 2.0,
        'time': 65.0
    })
    
    # Pass by the obstacle
    path.append({
        'position': (55.0, 10.0),
        'heading': 0.0,  # Back to original heading
        'speed': 2.0,
        'time': 80.0
    })
    
    # Return to original path
    path.append({
        'position': (60.0, 0.0),
        'heading': -np.pi/4,  # -45-degree heading
        'speed': 2.0,
        'time': 95.0
    })
    
    # Continue on original path
    path.append({
        'position': (100.0, 0.0),
        'heading': 0.0,
        'speed': 2.0,
        'time': 135.0
    })
    
    # Final stop
    path.append({
        'position': (100.0, 0.0),
        'heading': 0.0,
        'speed': 0.0,
        'time': 145.0
    })
    
    return path


def obstacle_avoidance_test():
    """
    Test the controllers in a scenario that simulates obstacle avoidance.
    """
    print("Running obstacle avoidance test...")

    # Define common simulation parameters
    sim_params = {
        'dt': 0.1,
        'simulation_time': 150.0,
        'x0': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),  # Start at origin
        'vessel_params': {
            'm': 50.0,    # mass (kg)
            'Iz': 20.0,   # moment of inertia (kg.m^2)
            'Xu': -25.0,  # surge linear damping
            'Yv': -40.0,  # sway linear damping
            'Nr': -10.0,  # yaw linear damping
        },
        'env_params': {
            'wind': {
                'speed': 2.0,
                'direction': np.pi/2,
            },
            'wave': {
                'significant_height': 0.3,
                'peak_period': 4.0,
                'direction': np.pi/2,
            },
            'current': {
                'speed': 0.2,
                'direction': np.pi/2,
            }
        }
    }

    # Create simulator
    simulator = USVSimulator(**sim_params)

    # Get the obstacle avoidance path
    path = obstacle_avoidance_path()

    # Define controllers to compare
    controllers = [
        {
            'type': 'pid',
            'name': 'PID Controller',
            'params': {
                'Kp_heading': 8.0,
                'Ki_heading': 0.05,
                'Kd_heading': 4.0,
                'Kp_speed': 15.0,
                'Ki_speed': 0.2,
                'Kd_speed': 2.0,
                'Kp_position': 3.0
            }
        },
        {
            'type': 'lqr',
            'name': 'LQR Controller',
            'params': {
                'Q': {
                    'x': 15.0, 'y': 15.0, 'psi': 25.0,
                    'u': 2.0, 'v': 2.0, 'r': 8.0
                },
                'R': {
                    'tau_u': 0.8, 'tau_r': 0.8
                }
            }
        },
        {
            'type': 'mpc',
            'name': 'MPC Controller',
            'params': {
                'prediction_horizon': 15,
                'Q': {
                    'x': 15.0, 'y': 15.0, 'psi': 25.0,
                    'u': 2.0, 'v': 2.0, 'r': 8.0
                },
                'R': {
                    'tau_u': 0.8, 'tau_r': 0.8
                }
            }
        }
    ]

    # Run the comparison
    simulator.compare_controllers(controllers, path)

    print("Obstacle avoidance test completed.")


def main():
    """
    Main function to run all tests.
    """
    # Create output directory for plots if it doesn't exist
    output_dir = os.path.join(os.path.dirname(__file__), 'outputs')
    os.makedirs(output_dir, exist_ok=True)

    # Run tests
    waypoint_following_test()
    station_keeping_test()
    disturbance_rejection_test()
    high_speed_maneuver_test()
    obstacle_avoidance_test()

    print("All tests completed.")


if __name__ == "__main__":
    main() 