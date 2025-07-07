"""
Data Logger Module

Handles telemetry data collection, storage, and export in various formats.
Provides comprehensive logging of simulation data for analysis.
"""

import json
import csv
import os
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Any, Optional, Union
import numpy as np


class DataLogger:
    """
    Data logger for simulation telemetry and mission data.
    
    Features:
    - Real-time telemetry logging
    - Multiple export formats (CSV, JSON)
    - Configurable logging parameters
    - Performance metrics tracking
    - Mission event logging
    """
    
    def __init__(self, output_dir: Union[str, Path], 
                 output_format: str = 'csv',
                 enable_logging: bool = True):
        """
        Initialize data logger.
        
        Args:
            output_dir: Directory to save log files
            output_format: Output format ('csv' or 'json')
            enable_logging: Enable/disable logging
        """
        self.output_dir = Path(output_dir)
        self.output_format = output_format.lower()
        self.enable_logging = enable_logging
        
        # Create output directory if it doesn't exist
        if self.enable_logging:
            self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize data storage
        self.telemetry_data = []
        self.mission_events = []
        self.performance_metrics = {}
        
        # Logging configuration
        self.log_interval = 0.1  # seconds
        self.last_log_time = 0.0
        
        # File paths
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.telemetry_file = None
        self.mission_file = None
        self.metrics_file = None
        
        # Initialize log files
        if self.enable_logging:
            self._initialize_log_files()
    
    def _initialize_log_files(self):
        """Initialize log files with headers."""
        base_name = f"simulation_{self.session_id}"
        
        if self.output_format == 'csv':
            self.telemetry_file = self.output_dir / f"{base_name}_telemetry.csv"
            self.mission_file = self.output_dir / f"{base_name}_mission.csv"
            self.metrics_file = self.output_dir / f"{base_name}_metrics.json"
            
            # Write CSV headers
            self._write_csv_headers()
            
        elif self.output_format == 'json':
            self.telemetry_file = self.output_dir / f"{base_name}_telemetry.json"
            self.mission_file = self.output_dir / f"{base_name}_mission.json"
            self.metrics_file = self.output_dir / f"{base_name}_metrics.json"
    
    def _write_csv_headers(self):
        """Write CSV headers for telemetry and mission logs."""
        # Telemetry headers
        telemetry_headers = [
            'timestamp', 'simulation_time',
            'pos_x', 'pos_y', 'pos_z',
            'vel_x', 'vel_y', 'vel_z',
            'acc_x', 'acc_y', 'acc_z',
            'roll', 'pitch', 'yaw',
            'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
            'angular_acc_x', 'angular_acc_y', 'angular_acc_z',
            'thrust_cmd', 'roll_cmd', 'pitch_cmd', 'yaw_cmd',
            'current_mass', 'fuel_level', 'battery_level',
            'power_consumption', 'payload_mass',
            'altitude', 'airspeed', 'ground_speed',
            'air_density', 'air_pressure', 'temperature',
            'wind_speed', 'wind_direction'
        ]
        
        # Mission headers
        mission_headers = [
            'timestamp', 'simulation_time', 'event_type',
            'waypoint_index', 'action', 'pos_x', 'pos_y', 'pos_z',
            'distance_to_waypoint', 'mission_progress',
            'description', 'parameters'
        ]
        
        # Write headers
        with open(self.telemetry_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(telemetry_headers)
        
        with open(self.mission_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(mission_headers)
    
    def log_telemetry(self, current_time: float, drone_state: Dict[str, Any],
                     control_inputs: Dict[str, Any], environment_data: Dict[str, Any]):
        """
        Log telemetry data.
        
        Args:
            current_time: Current simulation time
            drone_state: Current drone state
            control_inputs: Control inputs
            environment_data: Environmental data
        """
        if not self.enable_logging:
            return
        
        # Check if enough time has passed since last log
        if current_time - self.last_log_time < self.log_interval:
            return
        
        timestamp = datetime.now().isoformat()
        
        # Extract data from drone state
        position = drone_state.get('position', [0, 0, 0])
        velocity = drone_state.get('velocity', [0, 0, 0])
        acceleration = drone_state.get('acceleration', [0, 0, 0])
        attitude = drone_state.get('attitude', [0, 0, 0])
        angular_velocity = drone_state.get('angular_velocity', [0, 0, 0])
        angular_acceleration = drone_state.get('angular_acceleration', [0, 0, 0])
        
        # Calculate derived values
        altitude = position[2]
        airspeed = np.linalg.norm(velocity)
        ground_speed = np.linalg.norm(velocity[:2])
        
        # Environmental data
        air_density = environment_data.get('air_density', 0.0)
        air_pressure = environment_data.get('air_pressure', 0.0)
        temperature = environment_data.get('temperature', 0.0)
        wind_velocity = environment_data.get('wind_velocity', [0, 0, 0])
        wind_speed = np.linalg.norm(wind_velocity)
        wind_direction = np.arctan2(wind_velocity[1], wind_velocity[0])
        
        # Create telemetry record
        telemetry_record = {
            'timestamp': timestamp,
            'simulation_time': current_time,
            'position': position,
            'velocity': velocity,
            'acceleration': acceleration,
            'attitude': attitude,
            'angular_velocity': angular_velocity,
            'angular_acceleration': angular_acceleration,
            'control_inputs': control_inputs,
            'current_mass': drone_state.get('current_mass', 0.0),
            'fuel_level': drone_state.get('fuel_level', 0.0),
            'battery_level': drone_state.get('battery_level', 0.0),
            'power_consumption': drone_state.get('power_consumption', 0.0),
            'payload_mass': drone_state.get('payload_mass', 0.0),
            'altitude': altitude,
            'airspeed': airspeed,
            'ground_speed': ground_speed,
            'air_density': air_density,
            'air_pressure': air_pressure,
            'temperature': temperature,
            'wind_speed': wind_speed,
            'wind_direction': wind_direction
        }
        
        # Store in memory
        self.telemetry_data.append(telemetry_record)
        
        # Write to file
        self._write_telemetry_record(telemetry_record)
        
        self.last_log_time = current_time
    
    def log_mission_event(self, current_time: float, event_type: str,
                         waypoint_index: int, action: str, position: np.ndarray,
                         distance_to_waypoint: float, mission_progress: float,
                         description: str, parameters: Dict[str, Any]):
        """
        Log mission event.
        
        Args:
            current_time: Current simulation time
            event_type: Type of event
            waypoint_index: Current waypoint index
            action: Current action
            position: Current position
            distance_to_waypoint: Distance to current waypoint
            mission_progress: Mission progress percentage
            description: Event description
            parameters: Additional parameters
        """
        if not self.enable_logging:
            return
        
        timestamp = datetime.now().isoformat()
        
        # Create mission event record
        event_record = {
            'timestamp': timestamp,
            'simulation_time': current_time,
            'event_type': event_type,
            'waypoint_index': waypoint_index,
            'action': action,
            'position': position.tolist(),
            'distance_to_waypoint': distance_to_waypoint,
            'mission_progress': mission_progress,
            'description': description,
            'parameters': parameters
        }
        
        # Store in memory
        self.mission_events.append(event_record)
        
        # Write to file
        self._write_mission_event(event_record)
    
    def _write_telemetry_record(self, record: Dict[str, Any]):
        """Write telemetry record to file."""
        if self.output_format == 'csv':
            self._write_telemetry_csv(record)
        elif self.output_format == 'json':
            self._write_telemetry_json(record)
    
    def _write_telemetry_csv(self, record: Dict[str, Any]):
        """Write telemetry record to CSV file."""
        position = record['position']
        velocity = record['velocity']
        acceleration = record['acceleration']
        attitude = record['attitude']
        angular_velocity = record['angular_velocity']
        angular_acceleration = record['angular_acceleration']
        control_inputs = record['control_inputs']
        
        row = [
            record['timestamp'],
            record['simulation_time'],
            position[0], position[1], position[2],
            velocity[0], velocity[1], velocity[2],
            acceleration[0], acceleration[1], acceleration[2],
            attitude[0], attitude[1], attitude[2],
            angular_velocity[0], angular_velocity[1], angular_velocity[2],
            angular_acceleration[0], angular_acceleration[1], angular_acceleration[2],
            control_inputs.get('thrust', 0.0),
            control_inputs.get('roll', 0.0),
            control_inputs.get('pitch', 0.0),
            control_inputs.get('yaw', 0.0),
            record['current_mass'],
            record['fuel_level'],
            record['battery_level'],
            record['power_consumption'],
            record['payload_mass'],
            record['altitude'],
            record['airspeed'],
            record['ground_speed'],
            record['air_density'],
            record['air_pressure'],
            record['temperature'],
            record['wind_speed'],
            record['wind_direction']
        ]
        
        with open(self.telemetry_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
    
    def _write_telemetry_json(self, record: Dict[str, Any]):
        """Write telemetry record to JSON file."""
        # For JSON, we'll write all records at once when finalizing
        pass
    
    def _write_mission_event(self, event: Dict[str, Any]):
        """Write mission event to file."""
        if self.output_format == 'csv':
            self._write_mission_csv(event)
        elif self.output_format == 'json':
            self._write_mission_json(event)
    
    def _write_mission_csv(self, event: Dict[str, Any]):
        """Write mission event to CSV file."""
        position = event['position']
        
        row = [
            event['timestamp'],
            event['simulation_time'],
            event['event_type'],
            event['waypoint_index'],
            event['action'],
            position[0], position[1], position[2],
            event['distance_to_waypoint'],
            event['mission_progress'],
            event['description'],
            json.dumps(event['parameters'])
        ]
        
        with open(self.mission_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
    
    def _write_mission_json(self, event: Dict[str, Any]):
        """Write mission event to JSON file."""
        # For JSON, we'll write all events at once when finalizing
        pass
    
    def log_performance_metrics(self, metrics: Dict[str, Any]):
        """Log performance metrics."""
        if not self.enable_logging:
            return
        
        self.performance_metrics.update(metrics)
        
        # Write to file immediately
        with open(self.metrics_file, 'w') as f:
            json.dump(self.performance_metrics, f, indent=2)
    
    def finalize_logging(self):
        """Finalize logging and write any remaining data."""
        if not self.enable_logging:
            return
        
        # Write JSON files if using JSON format
        if self.output_format == 'json':
            # Write telemetry data
            with open(self.telemetry_file, 'w') as f:
                json.dump(self.telemetry_data, f, indent=2)
            
            # Write mission events
            with open(self.mission_file, 'w') as f:
                json.dump(self.mission_events, f, indent=2)
        
        # Write final metrics
        if self.metrics_file:
            with open(self.metrics_file, 'w') as f:
                json.dump(self.performance_metrics, f, indent=2)
    
    def get_telemetry_summary(self) -> Dict[str, Any]:
        """Get summary of telemetry data."""
        if not self.telemetry_data:
            return {}
        
        # Calculate summary statistics
        positions = [record['position'] for record in self.telemetry_data]
        velocities = [record['velocity'] for record in self.telemetry_data]
        altitudes = [record['altitude'] for record in self.telemetry_data]
        airspeeds = [record['airspeed'] for record in self.telemetry_data]
        
        positions_array = np.array(positions)
        velocities_array = np.array(velocities)
        
        summary = {
            'total_records': len(self.telemetry_data),
            'simulation_duration': self.telemetry_data[-1]['simulation_time'] - self.telemetry_data[0]['simulation_time'],
            'position_stats': {
                'min': positions_array.min(axis=0).tolist(),
                'max': positions_array.max(axis=0).tolist(),
                'mean': positions_array.mean(axis=0).tolist(),
                'std': positions_array.std(axis=0).tolist()
            },
            'velocity_stats': {
                'min': velocities_array.min(axis=0).tolist(),
                'max': velocities_array.max(axis=0).tolist(),
                'mean': velocities_array.mean(axis=0).tolist(),
                'std': velocities_array.std(axis=0).tolist()
            },
            'altitude_stats': {
                'min': min(altitudes),
                'max': max(altitudes),
                'mean': np.mean(altitudes),
                'std': np.std(altitudes)
            },
            'airspeed_stats': {
                'min': min(airspeeds),
                'max': max(airspeeds),
                'mean': np.mean(airspeeds),
                'std': np.std(airspeeds)
            },
            'distance_traveled': self._calculate_total_distance(),
            'flight_envelope': self._calculate_flight_envelope()
        }
        
        return summary
    
    def _calculate_total_distance(self) -> float:
        """Calculate total distance traveled."""
        if len(self.telemetry_data) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(1, len(self.telemetry_data)):
            pos1 = np.array(self.telemetry_data[i-1]['position'])
            pos2 = np.array(self.telemetry_data[i]['position'])
            total_distance += np.linalg.norm(pos2 - pos1)
        
        return total_distance
    
    def _calculate_flight_envelope(self) -> Dict[str, Any]:
        """Calculate flight envelope statistics."""
        if not self.telemetry_data:
            return {}
        
        altitudes = [record['altitude'] for record in self.telemetry_data]
        airspeeds = [record['airspeed'] for record in self.telemetry_data]
        
        return {
            'max_altitude': max(altitudes),
            'min_altitude': min(altitudes),
            'max_airspeed': max(airspeeds),
            'service_ceiling': max(altitudes),
            'operational_envelope': {
                'altitude_range': [min(altitudes), max(altitudes)],
                'speed_range': [min(airspeeds), max(airspeeds)]
            }
        }
    
    def export_data(self, format_type: str = None, filename: str = None) -> str:
        """
        Export data in specified format.
        
        Args:
            format_type: Export format ('csv' or 'json')
            filename: Output filename
            
        Returns:
            Path to exported file
        """
        if not self.enable_logging:
            return ""
        
        if format_type is None:
            format_type = self.output_format
        
        if filename is None:
            filename = f"export_{self.session_id}"
        
        export_path = self.output_dir / f"{filename}.{format_type}"
        
        if format_type == 'csv':
            # Export combined data as CSV
            self._export_csv(export_path)
        elif format_type == 'json':
            # Export combined data as JSON
            self._export_json(export_path)
        
        return str(export_path)
    
    def _export_csv(self, filepath: Path):
        """Export all data to CSV format."""
        # This would combine telemetry and mission data
        # For now, just copy the existing telemetry file
        if self.telemetry_file:
            import shutil
            shutil.copy(self.telemetry_file, filepath)
    
    def _export_json(self, filepath: Path):
        """Export all data to JSON format."""
        export_data = {
            'telemetry': self.telemetry_data,
            'mission_events': self.mission_events,
            'performance_metrics': self.performance_metrics,
            'summary': self.get_telemetry_summary()
        }
        
        with open(filepath, 'w') as f:
            json.dump(export_data, f, indent=2)
    
    def get_file_paths(self) -> Dict[str, str]:
        """Get paths to all log files."""
        return {
            'telemetry': str(self.telemetry_file) if self.telemetry_file else "",
            'mission': str(self.mission_file) if self.mission_file else "",
            'metrics': str(self.metrics_file) if self.metrics_file else "",
            'output_dir': str(self.output_dir)
        }
