"""
Simulation Controller Module

Main simulation controller that coordinates all components and manages
the simulation execution process.
"""

import time
import numpy as np
from typing import Dict, List, Any, Optional, Tuple
from pathlib import Path
import logging

from .drone_model import DroneModel
from .physics_engine import PhysicsEngine
from .environment import Environment
from .mission_manager import MissionManager
from .data_logger import DataLogger
from .visualization import Visualizer


class DroneSimulator:
    """
    Main drone simulation controller.
    
    Coordinates all simulation components:
    - Drone model and physics
    - Environmental conditions
    - Mission execution
    - Data logging
    - Visualization
    """
    
    def __init__(self, drone_model: str, environment: str, mission_type: str,
                 timestep: float = 0.01, output_dir: Path = None,
                 output_format: str = 'csv', enable_logging: bool = True):
        """
        Initialize simulator.
        
        Args:
            drone_model: Name of drone model to use
            environment: Environment name (earth, mars, moon)
            mission_type: Mission type (reconnaissance, sample_transport, monitoring)
            timestep: Simulation timestep in seconds
            output_dir: Output directory for logs and plots
            output_format: Output format for data (csv, json)
            enable_logging: Enable data logging
        """
        self.drone_model_name = drone_model
        self.environment_name = environment
        self.mission_type = mission_type
        self.timestep = timestep
        self.output_dir = output_dir or Path("simulation_output")
        self.output_format = output_format
        self.enable_logging = enable_logging
        
        # Simulation state
        self.current_time = 0.0
        self.simulation_running = False
        self.simulation_paused = False
        self.performance_metrics = {}
        
        # Initialize components
        self.logger = logging.getLogger(__name__)
        self._initialize_components()
        
        # Control system
        self.control_gains = {
            'position': {'kp': 1.0, 'ki': 0.1, 'kd': 0.5},
            'attitude': {'kp': 2.0, 'ki': 0.2, 'kd': 0.8},
            'velocity': {'kp': 0.8, 'ki': 0.05, 'kd': 0.3}
        }
        
        # Control state
        self.position_integral = np.zeros(3)
        self.attitude_integral = np.zeros(3)
        self.velocity_integral = np.zeros(3)
        self.previous_position_error = np.zeros(3)
        self.previous_attitude_error = np.zeros(3)
        self.previous_velocity_error = np.zeros(3)
    
    def _initialize_components(self):
        """Initialize all simulation components."""
        try:
            # Initialize drone model
            self.drone = DroneModel(self.drone_model_name)
            self.logger.info(f"Initialized drone model: {self.drone_model_name}")
            
            # Initialize environment
            self.environment = Environment(self.environment_name)
            self.logger.info(f"Initialized environment: {self.environment_name}")
            
            # Initialize physics engine
            self.physics = PhysicsEngine(self.drone, self.environment)
            self.logger.info("Initialized physics engine")
            
            # Initialize mission manager
            self.mission = MissionManager(self.mission_type)
            self.logger.info(f"Initialized mission: {self.mission_type}")
            
            # Initialize data logger
            self.data_logger = DataLogger(
                self.output_dir, 
                self.output_format, 
                self.enable_logging
            )
            self.logger.info("Initialized data logger")
            
            # Initialize visualizer
            self.visualizer = Visualizer(self.output_dir / "plots")
            self.logger.info("Initialized visualizer")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize components: {e}")
            raise
    
    def run_simulation(self, duration: float, realtime: bool = False) -> Dict[str, Any]:
        """
        Run the simulation for specified duration.
        
        Args:
            duration: Simulation duration in seconds
            realtime: Run in real-time vs accelerated
            
        Returns:
            Simulation results dictionary
        """
        self.logger.info(f"Starting simulation for {duration}s")
        
        # Start mission
        self.mission.start_mission(self.current_time)
        
        # Initialize drone at first waypoint if available
        first_waypoint = self.mission.get_current_waypoint()
        if first_waypoint:
            self.drone.state.position = first_waypoint.position.copy()
        
        # Simulation loop
        self.simulation_running = True
        start_real_time = time.time()
        
        try:
            while self.simulation_running and self.current_time < duration:
                # Check if mission is complete or failed
                if self.mission.status.value in ['completed', 'failed', 'aborted']:
                    break
                
                # Check if drone is operational
                if not self.drone.is_operational():
                    self.logger.warning("Drone is no longer operational")
                    break
                
                # Update simulation step
                self._simulation_step()
                
                # Real-time synchronization
                if realtime:
                    self._synchronize_realtime(start_real_time)
                
                # Progress reporting
                if int(self.current_time * 10) % 100 == 0:  # Every 10 seconds
                    progress = (self.current_time / duration) * 100
                    self.logger.info(f"Simulation progress: {progress:.1f}%")
        
        except KeyboardInterrupt:
            self.logger.info("Simulation interrupted by user")
            self.simulation_running = False
        
        except Exception as e:
            self.logger.error(f"Simulation error: {e}")
            raise
        
        finally:
            # Finalize simulation
            self.simulation_running = False
            self._finalize_simulation()
        
        # Return results
        return self._get_simulation_results()
    
    def _simulation_step(self):
        """Execute one simulation step."""
        # Get current navigation command
        nav_command = self.mission.get_navigation_command(self.drone.state.position)
        
        # Calculate control inputs
        control_inputs = self._calculate_control_inputs(nav_command)
        
        # Get environmental conditions
        env_conditions = self.environment.get_environmental_effects(
            self.drone.state.position, self.current_time
        )
        
        # Physics simulation step
        physics_results = self.physics.step(self.timestep, control_inputs)
        
        # Update mission
        mission_update = self.mission.update_mission(
            self.current_time, 
            self.drone.state.position,
            self.drone.get_status_dict()
        )
        
        # Log data
        if self.enable_logging:
            self._log_simulation_data(control_inputs, env_conditions, mission_update)
        
        # Update time
        self.current_time += self.timestep
    
    def _calculate_control_inputs(self, nav_command: Dict[str, Any]) -> Dict[str, Any]:
        """
        Calculate control inputs based on navigation command.
        
        Args:
            nav_command: Navigation command from mission manager
            
        Returns:
            Control inputs dictionary
        """
        control_inputs = {
            'thrust': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'thrust_x': 0.0,
            'thrust_y': 0.0,
            'thrust_z': 0.0
        }
        
        if nav_command['command'] == 'hover':
            # Hover control
            control_inputs.update(self._hover_control())
            
        elif nav_command['command'] == 'navigate':
            # Navigation control
            control_inputs.update(self._navigation_control(nav_command))
            
        else:
            # Default hover
            control_inputs.update(self._hover_control())
        
        return control_inputs
    
    def _hover_control(self) -> Dict[str, Any]:
        """Calculate control inputs for hovering."""
        # Altitude hold
        target_altitude = self.drone.state.position[2]
        altitude_error = target_altitude - self.drone.state.position[2]
        
        # Thrust to counteract gravity
        thrust_to_weight = self.environment.gravity / self.drone.get_max_thrust()
        thrust_cmd = thrust_to_weight + 0.1 * altitude_error
        thrust_cmd = np.clip(thrust_cmd, 0.0, 1.0)
        
        # Attitude stabilization
        attitude_cmd = self._attitude_stabilization()
        
        if self.drone.specs.propulsion_type == 'thrusters':
            # For thrusters, use direct thrust control
            return {
                'thrust_x': 0.0,
                'thrust_y': 0.0,
                'thrust_z': thrust_cmd,
                'roll': attitude_cmd['roll'],
                'pitch': attitude_cmd['pitch'],
                'yaw': attitude_cmd['yaw']
            }
        else:
            # For rotorcraft
            return {
                'thrust': thrust_cmd,
                'roll': attitude_cmd['roll'],
                'pitch': attitude_cmd['pitch'],
                'yaw': attitude_cmd['yaw']
            }
    
    def _navigation_control(self, nav_command: Dict[str, Any]) -> Dict[str, Any]:
        """Calculate control inputs for navigation."""
        # Position control
        position_cmd = self._position_control(nav_command)
        
        # Attitude control
        attitude_cmd = self._attitude_control(nav_command)
        
        # Velocity control
        velocity_cmd = self._velocity_control(nav_command)
        
        # Combine commands
        if self.drone.specs.propulsion_type == 'thrusters':
            return {
                'thrust_x': position_cmd['thrust_x'],
                'thrust_y': position_cmd['thrust_y'],
                'thrust_z': position_cmd['thrust_z'],
                'roll': attitude_cmd['roll'],
                'pitch': attitude_cmd['pitch'],
                'yaw': attitude_cmd['yaw']
            }
        else:
            return {
                'thrust': position_cmd['thrust'],
                'roll': attitude_cmd['roll'],
                'pitch': attitude_cmd['pitch'],
                'yaw': attitude_cmd['yaw']
            }
    
    def _position_control(self, nav_command: Dict[str, Any]) -> Dict[str, Any]:
        """PID position control."""
        target_position = nav_command['target']
        current_position = self.drone.state.position
        
        # Position error
        position_error = target_position - current_position
        
        # PID control
        gains = self.control_gains['position']
        
        # Proportional term
        p_term = gains['kp'] * position_error
        
        # Integral term
        self.position_integral += position_error * self.timestep
        i_term = gains['ki'] * self.position_integral
        
        # Derivative term
        d_term = gains['kd'] * (position_error - self.previous_position_error) / self.timestep
        self.previous_position_error = position_error
        
        # Total control
        control_output = p_term + i_term + d_term
        
        if self.drone.specs.propulsion_type == 'thrusters':
            # For thrusters, use direct thrust control
            thrust_x = np.clip(control_output[0], -1.0, 1.0)
            thrust_y = np.clip(control_output[1], -1.0, 1.0)
            thrust_z = np.clip(control_output[2], -1.0, 1.0)
            
            return {
                'thrust_x': thrust_x,
                'thrust_y': thrust_y,
                'thrust_z': thrust_z
            }
        else:
            # For rotorcraft, convert to thrust and attitude commands
            thrust_cmd = np.clip(np.linalg.norm(control_output), 0.0, 1.0)
            
            return {'thrust': thrust_cmd}
    
    def _attitude_control(self, nav_command: Dict[str, Any]) -> Dict[str, Any]:
        """PID attitude control."""
        # Calculate desired attitude from navigation
        direction = nav_command.get('direction', np.array([1, 0, 0]))
        
        # Desired yaw angle
        desired_yaw = np.arctan2(direction[1], direction[0])
        desired_attitude = np.array([0, 0, desired_yaw])
        
        # Current attitude
        current_attitude = self.drone.state.attitude
        
        # Attitude error
        attitude_error = desired_attitude - current_attitude
        
        # Normalize angles
        for i in range(3):
            while attitude_error[i] > np.pi:
                attitude_error[i] -= 2 * np.pi
            while attitude_error[i] < -np.pi:
                attitude_error[i] += 2 * np.pi
        
        # PID control
        gains = self.control_gains['attitude']
        
        # Proportional term
        p_term = gains['kp'] * attitude_error
        
        # Integral term
        self.attitude_integral += attitude_error * self.timestep
        i_term = gains['ki'] * self.attitude_integral
        
        # Derivative term
        d_term = gains['kd'] * (attitude_error - self.previous_attitude_error) / self.timestep
        self.previous_attitude_error = attitude_error
        
        # Total control
        control_output = p_term + i_term + d_term
        
        return {
            'roll': np.clip(control_output[0], -1.0, 1.0),
            'pitch': np.clip(control_output[1], -1.0, 1.0),
            'yaw': np.clip(control_output[2], -1.0, 1.0)
        }
    
    def _attitude_stabilization(self) -> Dict[str, Any]:
        """Attitude stabilization control."""
        # Target attitude is level flight
        target_attitude = np.zeros(3)
        current_attitude = self.drone.state.attitude
        
        # Simple proportional control
        attitude_error = target_attitude - current_attitude
        
        # Normalize angles
        for i in range(3):
            while attitude_error[i] > np.pi:
                attitude_error[i] -= 2 * np.pi
            while attitude_error[i] < -np.pi:
                attitude_error[i] += 2 * np.pi
        
        gain = 0.5
        control_output = gain * attitude_error
        
        return {
            'roll': np.clip(control_output[0], -1.0, 1.0),
            'pitch': np.clip(control_output[1], -1.0, 1.0),
            'yaw': np.clip(control_output[2], -1.0, 1.0)
        }
    
    def _velocity_control(self, nav_command: Dict[str, Any]) -> Dict[str, Any]:
        """PID velocity control."""
        # Desired velocity
        max_speed = nav_command.get('speed', 10.0)
        direction = nav_command.get('direction', np.array([1, 0, 0]))
        desired_velocity = direction * max_speed
        
        # Current velocity
        current_velocity = self.drone.state.velocity
        
        # Velocity error
        velocity_error = desired_velocity - current_velocity
        
        # PID control
        gains = self.control_gains['velocity']
        
        # Proportional term
        p_term = gains['kp'] * velocity_error
        
        # Integral term
        self.velocity_integral += velocity_error * self.timestep
        i_term = gains['ki'] * self.velocity_integral
        
        # Derivative term
        d_term = gains['kd'] * (velocity_error - self.previous_velocity_error) / self.timestep
        self.previous_velocity_error = velocity_error
        
        # Total control (not used directly but could be for advanced control)
        control_output = p_term + i_term + d_term
        
        return {'velocity_cmd': control_output}
    
    def _synchronize_realtime(self, start_real_time: float):
        """Synchronize simulation with real time."""
        expected_real_time = start_real_time + self.current_time
        current_real_time = time.time()
        
        if current_real_time < expected_real_time:
            time.sleep(expected_real_time - current_real_time)
    
    def _log_simulation_data(self, control_inputs: Dict[str, Any], 
                           env_conditions: Dict[str, Any],
                           mission_update: Dict[str, Any]):
        """Log simulation data."""
        # Log telemetry
        self.data_logger.log_telemetry(
            self.current_time,
            self.drone.get_status_dict(),
            control_inputs,
            env_conditions
        )
        
        # Log mission events
        if mission_update.get('waypoint_completed', False):
            self.data_logger.log_mission_event(
                self.current_time,
                'waypoint_completed',
                self.mission.current_waypoint_index - 1,
                mission_update.get('action', 'unknown'),
                self.drone.state.position,
                0.0,
                mission_update.get('progress', 0.0),
                f"Waypoint {self.mission.current_waypoint_index - 1} completed",
                mission_update.get('action_result', {})
            )
    
    def _finalize_simulation(self):
        """Finalize simulation and save results."""
        # Calculate final performance metrics
        self.performance_metrics = self._calculate_performance_metrics()
        
        # Log final metrics
        self.data_logger.log_performance_metrics(self.performance_metrics)
        
        # Finalize data logging
        self.data_logger.finalize_logging()
        
        self.logger.info("Simulation finalized")
    
    def _calculate_performance_metrics(self) -> Dict[str, Any]:
        """Calculate performance metrics."""
        mission_metrics = self.mission.get_mission_metrics()
        
        return {
            'simulation_time': self.current_time,
            'mission_metrics': mission_metrics,
            'drone_status': self.drone.get_status_dict(),
            'environment_status': self.environment.get_status_dict(),
            'final_position': self.drone.state.position.tolist(),
            'final_velocity': self.drone.state.velocity.tolist(),
            'final_attitude': self.drone.state.attitude.tolist(),
            'telemetry_summary': self.data_logger.get_telemetry_summary()
        }
    
    def _get_simulation_results(self) -> Dict[str, Any]:
        """Get simulation results."""
        return {
            'mission_status': self.mission.status.value,
            'final_position': self.drone.state.position.tolist(),
            'flight_time': self.current_time,
            'distance_covered': self.mission.distance_traveled,
            'telemetry_file': self.data_logger.get_file_paths()['telemetry'],
            'mission_file': self.data_logger.get_file_paths()['mission'],
            'metrics_file': self.data_logger.get_file_paths()['metrics'],
            'performance_metrics': self.performance_metrics
        }
    
    def generate_plots(self) -> List[str]:
        """Generate visualization plots."""
        try:
            # Get data for plotting
            telemetry_data = self.data_logger.telemetry_data
            mission_events = self.data_logger.mission_events
            mission_metrics = self.performance_metrics.get('mission_metrics', {})
            
            # Get waypoints
            waypoints = []
            for wp in self.mission.waypoints:
                waypoints.append({
                    'x': wp.position[0],
                    'y': wp.position[1],
                    'z': wp.position[2],
                    'action': wp.action.value
                })
            
            # Generate comprehensive report
            plot_files = self.visualizer.generate_comprehensive_report(
                telemetry_data, mission_events, mission_metrics, waypoints
            )
            
            # Generate summary dashboard
            dashboard_file = self.visualizer.create_summary_dashboard(
                telemetry_data, mission_metrics
            )
            plot_files.append(dashboard_file)
            
            return plot_files
            
        except Exception as e:
            self.logger.error(f"Error generating plots: {e}")
            return []
    
    def pause_simulation(self):
        """Pause the simulation."""
        self.simulation_paused = True
        self.logger.info("Simulation paused")
    
    def resume_simulation(self):
        """Resume the simulation."""
        self.simulation_paused = False
        self.logger.info("Simulation resumed")
    
    def stop_simulation(self):
        """Stop the simulation."""
        self.simulation_running = False
        self.logger.info("Simulation stopped")
    
    def get_status(self) -> Dict[str, Any]:
        """Get current simulation status."""
        return {
            'running': self.simulation_running,
            'paused': self.simulation_paused,
            'current_time': self.current_time,
            'drone_status': self.drone.get_status_dict(),
            'mission_status': self.mission.get_status_dict(),
            'environment_status': self.environment.get_status_dict()
        }
