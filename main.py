#!/usr/bin/env python3
"""
Main entry point for the Exploration Drone Simulation Application.
Provides command-line interface for simulation control and execution.
"""

import argparse
import sys
import json
import os
from pathlib import Path

from src.simulator import DroneSimulator
from src.utils import load_config, setup_logging

def main():
    """Main function to handle command line arguments and run simulation."""
    parser = argparse.ArgumentParser(
        description='Exploration Drone Simulation Application',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  python main.py --drone mars_quadrotor --environment mars --mission reconnaissance
  python main.py --drone lunar_lander --environment moon --mission sample_transport --duration 300
  python main.py --list-configs
        '''
    )
    
    # Configuration options
    parser.add_argument('--drone', '-d', type=str, 
                       help='Drone model to use (see config/drone_models.json)')
    parser.add_argument('--environment', '-e', type=str,
                       help='Environment to simulate (earth, mars, moon)')
    parser.add_argument('--mission', '-m', type=str,
                       help='Mission type (reconnaissance, sample_transport, monitoring)')
    
    # Simulation parameters
    parser.add_argument('--duration', '-t', type=float, default=600.0,
                       help='Simulation duration in seconds (default: 600)')
    parser.add_argument('--timestep', type=float, default=0.01,
                       help='Simulation timestep in seconds (default: 0.01)')
    parser.add_argument('--realtime', action='store_true',
                       help='Run simulation in real-time instead of accelerated')
    
    # Output options
    parser.add_argument('--output', '-o', type=str, default='simulation_output',
                       help='Output directory for telemetry data (default: simulation_output)')
    parser.add_argument('--format', choices=['csv', 'json'], default='csv',
                       help='Output format for telemetry data (default: csv)')
    parser.add_argument('--plot', action='store_true',
                       help='Generate plots after simulation')
    parser.add_argument('--no-log', action='store_true',
                       help='Disable telemetry logging')
    
    # Utility options
    parser.add_argument('--list-configs', action='store_true',
                       help='List available configurations')
    parser.add_argument('--web', action='store_true',
                       help='Start web interface')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Enable verbose logging')
    
    args = parser.parse_args()
    
    # Setup logging
    setup_logging(verbose=args.verbose)
    
    # Handle utility commands
    if args.list_configs:
        list_configurations()
        return
    
    if args.web:
        start_web_interface()
        return
    
    # Validate required arguments
    if not all([args.drone, args.environment, args.mission]):
        parser.error("drone, environment, and mission are required unless using --list-configs or --web")
    
    try:
        # Create output directory
        output_dir = Path(args.output)
        output_dir.mkdir(exist_ok=True)
        
        # Initialize simulator
        simulator = DroneSimulator(
            drone_model=args.drone,
            environment=args.environment,
            mission_type=args.mission,
            timestep=args.timestep,
            output_dir=output_dir,
            output_format=args.format,
            enable_logging=not args.no_log
        )
        
        # Run simulation
        print(f"Starting simulation...")
        print(f"Drone: {args.drone}")
        print(f"Environment: {args.environment}")
        print(f"Mission: {args.mission}")
        print(f"Duration: {args.duration}s")
        print(f"Timestep: {args.timestep}s")
        print(f"Output: {args.output}")
        print("-" * 50)
        
        results = simulator.run_simulation(
            duration=args.duration,
            realtime=args.realtime
        )
        
        print(f"\nSimulation completed successfully!")
        print(f"Mission status: {results['mission_status']}")
        print(f"Final position: {results['final_position']}")
        print(f"Flight time: {results['flight_time']:.2f}s")
        print(f"Distance covered: {results['distance_covered']:.2f}m")
        
        if results['telemetry_file']:
            print(f"Telemetry saved to: {results['telemetry_file']}")
        
        # Generate plots if requested
        if args.plot:
            print("Generating plots...")
            simulator.generate_plots()
            print("Plots saved to output directory")
            
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

def list_configurations():
    """List available drone models, environments, and missions."""
    print("Available Configurations:")
    print("=" * 50)
    
    # List drone models
    try:
        drones = load_config('config/drone_models.json')
        print("\nDrone Models:")
        for name, drone in drones.items():
            print(f"  {name}: {drone.get('description', 'No description')}")
    except Exception as e:
        print(f"Error loading drone models: {e}")
    
    # List environments
    try:
        environments = load_config('config/environments.json')
        print("\nEnvironments:")
        for name, env in environments.items():
            print(f"  {name}: {env.get('description', 'No description')}")
    except Exception as e:
        print(f"Error loading environments: {e}")
    
    # List missions
    try:
        missions = load_config('config/missions.json')
        print("\nMission Types:")
        for name, mission in missions.items():
            print(f"  {name}: {mission.get('description', 'No description')}")
    except Exception as e:
        print(f"Error loading missions: {e}")

def start_web_interface():
    """Start the web interface."""
    print("Starting web interface...")
    print("Open your browser and navigate to http://localhost:5000")
    
    try:
        from web_server import app
        app.run(host='0.0.0.0', port=5000, debug=False)
    except ImportError:
        print("Error: Web interface dependencies not available")
        sys.exit(1)
    except Exception as e:
        print(f"Error starting web interface: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
