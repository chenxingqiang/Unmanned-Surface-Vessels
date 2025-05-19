# Intelligent Navigation and Control System for Unmanned Surface Vessels

This repository contains a Python implementation of an intelligent navigation and control system for Unmanned Surface Vessels (USVs).

## Features

- USV mathematical modeling (kinematic and dynamic)
- Path planning (global and local)
- Obstacle detection and avoidance
- Motion controllers (PID, LQR, MPC)
- Simulation environment
- Visualization tools
- Configurable test scenarios

## Project Structure

```
usv_system/
│
├── navigation/             # Path planning algorithms
│   ├── global_planner.py   # Global path planning (A*, Dijkstra)
│   └── local_planner.py    # Local path planning (DWA, RRT*)
│
├── control/                # Control algorithms
│   ├── pid_controller.py   # PID controller implementation
│   ├── lqr_controller.py   # Linear Quadratic Regulator
│   └── mpc_controller.py   # Model Predictive Control
│
├── obstacle_avoidance/     # Obstacle detection and avoidance
│   ├── detector.py         # Object detection algorithms
│   └── avoidance_strategy.py # Collision avoidance strategies
│
├── models/                 # Mathematical models
│   ├── usv_model.py        # USV dynamics model
│   └── environment.py      # Environmental model (wind, waves, currents)
│
├── simulation/             # Simulation environment
│   └── simulator.py        # Main simulation framework
│
├── utils/                  # Utility functions
│   └── visualization.py    # Data visualization tools
│
├── tests/                  # Test cases
│   ├── run_waypoint_test.py    # Simple waypoint following test
│   ├── run_comparison_test.py  # Comparative controller tests
│   ├── test_navigation.py      # Tests for navigation algorithms
│   └── test_controllers.py     # Tests for control algorithms
│
├── config/                 # Configuration files
│   └── default_config.yaml # Default simulation configuration
│
├── main.py                 # Main entry point for the application
└── requirements.txt        # Project dependencies
```

## Installation

1. Clone this repository
2. Install the required dependencies:

```bash
pip install -r requirements.txt
```

## Usage

### Running Simulations

To run the simulation with default settings:

```bash
python main.py
```

### Command-Line Options

The main script supports various command-line options:

```bash
python main.py --config <config_file> --scenario <scenario_name> --controller <controller_type> [options]
```

Options:
- `--config`: Path to configuration file (default: config/default_config.yaml)
- `--scenario`: Test scenario to run ('waypoint', 'station', 'disturbance', 'highspeed', 'obstacle', or 'all')
- `--controller`: Controller to use ('pid', 'lqr', 'mpc', or 'all')
- `--verbose`: Enable verbose output
- `--visualize`: Enable real-time visualization
- `--save-results`: Save simulation results
- `--simulation-time`: Override simulation time (seconds)

Examples:

```bash
# Run all scenarios with all controllers
python main.py --scenario all --controller all --save-results

# Run waypoint following with PID controller only
python main.py --scenario waypoint --controller pid --verbose

# Run obstacle avoidance with MPC controller and real-time visualization
python main.py --scenario obstacle --controller mpc --visualize

# Run with a custom configuration file
python main.py --config my_config.yaml --scenario station
```

### Configuration

The system behavior can be configured through YAML configuration files. The default configuration is in `config/default_config.yaml`.

You can create your own configuration files with custom parameters for:
- Simulation settings (time step, duration, initial state)
- Vessel parameters (mass, dimensions, drag coefficients)
- Environment conditions (wind, currents, obstacles)
- Controller tuning parameters
- Test scenarios

## Tests

Run the test suite:

```bash
pytest tests/
```

Or run specific test scenarios:

```bash
# Run waypoint following test
python usv_system/tests/run_waypoint_test.py

# Run comparison test between controllers
python usv_system/tests/run_comparison_test.py
```

## License

MIT 