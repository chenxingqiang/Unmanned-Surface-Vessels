#!/usr/bin/env python3
"""
Main entry point for the USV navigation and control system.

This script initializes the USV simulation environment, loads configuration,
and runs specified test scenarios.
"""

import os
import sys
import argparse
import yaml
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# Add the parent directory to the system path
sys.path.append(os.path.abspath(os.path.dirname(os.path.dirname(__file__))))

from usv_system.simulation.simulator import USVSimulator
from usv_system.tests.run_comparison_test import run_scenario
from usv_system.tests.test_controllers import waypoint_following_test


def load_config(config_path):
    """
    Load configuration from YAML file.
    
    Args:
        config_path: Path to the configuration file
        
    Returns:
        Configuration dictionary
    """
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        return config
    except Exception as e:
        print(f"Error loading configuration: {e}")
        sys.exit(1)


def setup_output_dir():
    """
    Create output directory for test results if it doesn't exist.
    
    Returns:
        Path to the output directory
    """
    output_dir = Path(__file__).parent / "tests" / "outputs"
    output_dir.mkdir(exist_ok=True, parents=True)
    return output_dir


def parse_arguments():
    """
    Parse command line arguments.
    
    Returns:
        Parsed arguments
    """
    parser = argparse.ArgumentParser(description='USV Navigation and Control System')
    
    parser.add_argument('--config', type=str, default='config/default_config.yaml',
                        help='Path to configuration file')
    
    parser.add_argument('--scenario', type=str, default='waypoint',
                        choices=['waypoint', 'station', 'disturbance', 'highspeed', 'obstacle', 'all'],
                        help='Test scenario to run')
    
    parser.add_argument('--controller', type=str, default='all',
                        choices=['pid', 'lqr', 'mpc', 'all'],
                        help='Controller to use')
    
    parser.add_argument('--verbose', action='store_true',
                        help='Enable verbose output')
    
    parser.add_argument('--visualize', action='store_true',
                        help='Enable real-time visualization')
    
    parser.add_argument('--save-results', action='store_true',
                        help='Save simulation results')
    
    parser.add_argument('--simulation-time', type=float, default=None,
                        help='Override simulation time (seconds)')
    
    return parser.parse_args()


def run_single_scenario(scenario, controller_type, config, verbose=False, visualize=False, 
                       save_results=False, simulation_time=None):
    """
    Run a single test scenario with a specific controller.
    
    Args:
        scenario: Scenario name
        controller_type: Controller type
        config: Configuration dictionary
        verbose: Enable verbose output
        visualize: Enable real-time visualization
        save_results: Save simulation results
        simulation_time: Override simulation time
    """
    print(f"\n{'='*80}\nRunning scenario: {scenario} with controller: {controller_type}\n{'='*80}")
    
    # Override simulation time if specified
    if simulation_time is not None:
        config['simulation']['simulation_time'] = simulation_time
    
    # Create simulator
    simulator = USVSimulator(
        dt=config['simulation']['dt'],
        simulation_time=config['simulation']['simulation_time'],
        x0=np.array(config['simulation']['initial_state']),
        vessel_params=config['vessel'],
        env_params=config['environment'],
        controller_type=controller_type,
        controller_params=config['controllers'][controller_type]
    )
    
    # Load scenario-specific configuration
    scenario_config = config['scenarios'][scenario]
    
    # Set reference trajectory
    simulator.set_reference_trajectory(scenario_config['reference_trajectory'])
    
    # Run simulation
    simulator.run_simulation(verbose=verbose, visualize=visualize)
    
    # Save results if requested
    if save_results:
        output_dir = setup_output_dir()
        save_path = output_dir / f"{scenario}_{controller_type}.png"
        simulator.visualize_results(save_path=str(save_path))
        print(f"Results saved to: {save_path}")
    
    # Return key metrics
    return {
        'scenario': scenario,
        'controller': controller_type,
        'path_error': np.mean([e for e in simulator.path_error_history if e is not None]),
        'heading_error': np.mean([abs(e) for e in simulator.heading_error_history if e is not None]),
        'control_energy': np.sum([np.sum(c**2) for c in simulator.control_history]),
        'completion_time': simulator.time_history[-1] if simulator.time_history else None,
        'success': not simulator.collision_occurred and simulator.mission_completed
    }


def run_comparison(scenario, config, args):
    """
    Run comparison of all controllers for a specific scenario.
    
    Args:
        scenario: Scenario name
        config: Configuration dictionary
        args: Command line arguments
    """
    # Set up output directory
    output_dir = setup_output_dir()
    
    # Run the comparison test
    run_scenario(scenario, str(output_dir), 
                controllers=['pid', 'lqr', 'mpc'],
                config=config,
                verbose=args.verbose)


def main():
    """Main function to run the USV system."""
    # Parse command line arguments
    args = parse_arguments()
    
    # Load configuration
    config_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', args.config))
    config = load_config(config_path)
    
    # Setup output directory
    output_dir = setup_output_dir()
    print(f"Output directory: {output_dir}")
    
    # Set Matplotlib backend to avoid GUI issues
    if args.save_results and not args.visualize:
        plt.switch_backend('agg')
    
    # Handle 'all' scenarios
    scenarios = config['scenarios'].keys() if args.scenario == 'all' else [args.scenario]
    
    # Handle 'all' controllers
    controllers = ['pid', 'lqr', 'mpc'] if args.controller == 'all' else [args.controller]
    
    # Collect results
    results = []
    
    # Run each scenario
    for scenario in scenarios:
        if args.controller == 'all':
            # Run comparison test for all controllers
            print(f"\n{'='*80}\nRunning comparison for scenario: {scenario}\n{'='*80}")
            run_comparison(scenario, config, args)
        else:
            # Run individual tests for each controller
            for controller in controllers:
                result = run_single_scenario(
                    scenario=scenario,
                    controller_type=controller,
                    config=config,
                    verbose=args.verbose,
                    visualize=args.visualize,
                    save_results=args.save_results,
                    simulation_time=args.simulation_time
                )
                results.append(result)
    
    # Print summary of results
    if results:
        print("\n" + "="*80)
        print("Results Summary:")
        print("="*80)
        
        # Print table header
        print(f"{'Scenario':<15} {'Controller':<10} {'Path Error':<15} {'Heading Error':<15} " 
              f"{'Control Energy':<15} {'Time':<10} {'Success':<10}")
        print("-"*80)
        
        # Print each result
        for result in results:
            print(f"{result['scenario']:<15} {result['controller']:<10} "
                  f"{result['path_error']:<15.3f} {result['heading_error']:<15.3f} "
                  f"{result['control_energy']:<15.0f} {result['completion_time']:<10.1f} "
                  f"{result['success']:<10}")
    
    print("\nDone.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExecution interrupted by user.")
    except Exception as e:
        print(f"\nError during execution: {e}")
        import traceback
        traceback.print_exc()
