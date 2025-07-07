#!/usr/bin/env python3
"""
Web Server for Drone Simulation Interface

Provides a Flask-based web interface for controlling and monitoring
the drone simulation system.
"""

import os
import json
import asyncio
import threading
from pathlib import Path
from typing import Dict, Any, Optional
from datetime import datetime

from flask import Flask, render_template, request, jsonify, send_from_directory
from flask_socketio import SocketIO, emit
import logging

# Import simulation components
from src.simulator import DroneSimulator
from src.utils import load_config, setup_logging
from src.drone_model import DroneModel
from src.environment import Environment
from src.mission_manager import MissionManager


class SimulationWebServer:
    """
    Web server for drone simulation interface.
    
    Provides:
    - REST API endpoints for simulation control
    - WebSocket connections for real-time updates
    - Static file serving for web interface
    - Integration with simulation components
    """
    
    def __init__(self, host='0.0.0.0', port=5000, debug=False):
        """
        Initialize web server.
        
        Args:
            host: Server host address
            port: Server port
            debug: Enable debug mode
        """
        self.host = host
        self.port = port
        self.debug = debug
        
        # Initialize Flask app
        self.app = Flask(__name__, 
                        template_folder='web',
                        static_folder='web',
                        static_url_path='')
        self.app.config['SECRET_KEY'] = os.environ.get('SECRET_KEY', 'drone-sim-secret-key')
        
        # Initialize SocketIO
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", async_mode='threading')
        
        # Simulation state
        self.simulator: Optional[DroneSimulator] = None
        self.simulation_thread: Optional[threading.Thread] = None
        self.simulation_running = False
        self.simulation_config = {}
        
        # Setup logging
        self.logger = setup_logging(verbose=debug)
        
        # Register routes
        self._register_routes()
        self._register_socketio_events()
        
        # Start background tasks
        self._start_background_tasks()
    
    def _register_routes(self):
        """Register Flask routes."""
        
        @self.app.route('/')
        def index():
            """Serve main interface."""
            return send_from_directory('web', 'index.html')
        
        @self.app.route('/api/configurations/drones')
        def get_drone_configurations():
            """Get available drone configurations."""
            try:
                drones = load_config('config/drone_models.json')
                return jsonify(drones)
            except Exception as e:
                self.logger.error(f"Error loading drone configurations: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/configurations/environments')
        def get_environment_configurations():
            """Get available environment configurations."""
            try:
                environments = load_config('config/environments.json')
                return jsonify(environments)
            except Exception as e:
                self.logger.error(f"Error loading environment configurations: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/configurations/missions')
        def get_mission_configurations():
            """Get available mission configurations."""
            try:
                missions = load_config('config/missions.json')
                return jsonify(missions)
            except Exception as e:
                self.logger.error(f"Error loading mission configurations: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/simulation/start', methods=['POST'])
        def start_simulation():
            """Start simulation with given configuration."""
            try:
                if self.simulation_running:
                    return jsonify({'error': 'Simulation already running'}), 400
                
                config = request.get_json()
                if not self._validate_simulation_config(config):
                    return jsonify({'error': 'Invalid configuration'}), 400
                
                self.simulation_config = config
                self._start_simulation()
                
                return jsonify({
                    'success': True,
                    'message': 'Simulation started',
                    'config': config
                })
                
            except Exception as e:
                self.logger.error(f"Error starting simulation: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/simulation/stop', methods=['POST'])
        def stop_simulation():
            """Stop running simulation."""
            try:
                if not self.simulation_running:
                    return jsonify({'error': 'No simulation running'}), 400
                
                self._stop_simulation()
                
                return jsonify({
                    'success': True,
                    'message': 'Simulation stopped'
                })
                
            except Exception as e:
                self.logger.error(f"Error stopping simulation: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/simulation/pause', methods=['POST'])
        def pause_simulation():
            """Pause/resume running simulation."""
            try:
                if not self.simulation_running or not self.simulator:
                    return jsonify({'error': 'No simulation running'}), 400
                
                # Toggle pause state
                if hasattr(self.simulator, 'simulation_paused'):
                    if self.simulator.simulation_paused:
                        self.simulator.resume_simulation()
                        message = 'Simulation resumed'
                    else:
                        self.simulator.pause_simulation()
                        message = 'Simulation paused'
                else:
                    return jsonify({'error': 'Pause not supported'}), 400
                
                return jsonify({
                    'success': True,
                    'message': message
                })
                
            except Exception as e:
                self.logger.error(f"Error pausing simulation: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/simulation/status')
        def get_simulation_status():
            """Get current simulation status."""
            try:
                if not self.simulator:
                    return jsonify({
                        'simulation_status': 'ready',
                        'simulation_running': False
                    })
                
                status = self.simulator.get_status()
                status['simulation_running'] = self.simulation_running
                
                return jsonify(status)
                
            except Exception as e:
                self.logger.error(f"Error getting simulation status: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/simulation/telemetry')
        def get_telemetry_data():
            """Get latest telemetry data."""
            try:
                if not self.simulator or not hasattr(self.simulator, 'data_logger'):
                    return jsonify([])
                
                # Return last 100 telemetry points
                telemetry = self.simulator.data_logger.telemetry_data[-100:]
                return jsonify(telemetry)
                
            except Exception as e:
                self.logger.error(f"Error getting telemetry data: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/simulation/mission')
        def get_mission_data():
            """Get mission progress and events."""
            try:
                if not self.simulator:
                    return jsonify({})
                
                mission_metrics = self.simulator.mission.get_mission_metrics()
                return jsonify(mission_metrics)
                
            except Exception as e:
                self.logger.error(f"Error getting mission data: {e}")
                return jsonify({'error': str(e)}), 500
    
    def _register_socketio_events(self):
        """Register SocketIO event handlers."""
        
        @self.socketio.on('connect')
        def handle_connect():
            """Handle client connection."""
            self.logger.info('Client connected to WebSocket')
            emit('status', {'message': 'Connected to simulation server'})
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            """Handle client disconnection."""
            self.logger.info('Client disconnected from WebSocket')
        
        @self.socketio.on('request_status')
        def handle_status_request():
            """Handle status request from client."""
            try:
                if self.simulator:
                    status = self.simulator.get_status()
                    emit('status_update', status)
                else:
                    emit('status_update', {'simulation_status': 'ready'})
            except Exception as e:
                self.logger.error(f"Error handling status request: {e}")
    
    def _validate_simulation_config(self, config: Dict[str, Any]) -> bool:
        """
        Validate simulation configuration.
        
        Args:
            config: Configuration dictionary
            
        Returns:
            True if valid, False otherwise
        """
        required_fields = ['drone_model', 'environment', 'mission_type', 'duration']
        
        for field in required_fields:
            if field not in config or not config[field]:
                self.logger.error(f"Missing required field: {field}")
                return False
        
        # Validate ranges
        if config['duration'] <= 0 or config['duration'] > 3600:
            self.logger.error("Duration must be between 0 and 3600 seconds")
            return False
        
        if config.get('timestep', 0.01) <= 0 or config.get('timestep', 0.01) > 0.1:
            self.logger.error("Timestep must be between 0 and 0.1 seconds")
            return False
        
        return True
    
    def _start_simulation(self):
        """Start simulation in background thread."""
        if self.simulation_running:
            return
        
        self.simulation_running = True
        
        # Create simulator instance
        try:
            self.simulator = DroneSimulator(
                drone_model=self.simulation_config['drone_model'],
                environment=self.simulation_config['environment'],
                mission_type=self.simulation_config['mission_type'],
                timestep=self.simulation_config.get('timestep', 0.01),
                output_dir=Path('web_simulation_output'),
                output_format='json',
                enable_logging=True
            )
            
            # Start simulation in background thread
            self.simulation_thread = threading.Thread(
                target=self._run_simulation,
                daemon=True
            )
            self.simulation_thread.start()
            
            self.logger.info("Simulation started successfully")
            
        except Exception as e:
            self.logger.error(f"Failed to start simulation: {e}")
            self.simulation_running = False
            raise
    
    def _run_simulation(self):
        """Run simulation in background thread."""
        try:
            duration = self.simulation_config['duration']
            realtime = self.simulation_config.get('realtime', False)
            
            # Run simulation with progress callbacks
            self._run_simulation_with_updates(duration, realtime)
            
        except Exception as e:
            self.logger.error(f"Simulation error: {e}")
        finally:
            self.simulation_running = False
            self._broadcast_message('simulation_completed', {
                'message': 'Simulation completed'
            })
    
    def _run_simulation_with_updates(self, duration: float, realtime: bool):
        """Run simulation with real-time updates."""
        # Initialize simulation
        self.simulator.mission.start_mission(0.0)
        
        # Set initial position if waypoints available
        first_waypoint = self.simulator.mission.get_current_waypoint()
        if first_waypoint:
            self.simulator.drone.state.position = first_waypoint.position.copy()
        
        # Main simulation loop
        while (self.simulation_running and 
               self.simulator.current_time < duration and
               self.simulator.mission.status.value not in ['completed', 'failed', 'aborted']):
            
            # Check if drone is operational
            if not self.simulator.drone.is_operational():
                self.logger.warning("Drone is no longer operational")
                break
            
            # Store current time for telemetry
            current_time = self.simulator.current_time
            
            # Execute simulation step
            self.simulator._simulation_step()
            
            # Broadcast updates every 10 steps (0.1 second intervals)
            if int(current_time * 100) % 10 == 0:
                self._broadcast_telemetry_update()
            
            # Broadcast mission events
            mission_update = self.simulator.mission.update_mission(
                current_time,
                self.simulator.drone.state.position,
                self.simulator.drone.get_status_dict()
            )
            
            if mission_update.get('waypoint_completed'):
                self._broadcast_mission_event(mission_update)
            
            # Real-time delay if requested
            if realtime:
                import time
                time.sleep(self.simulator.timestep)
        
        # Finalize simulation
        self.simulator._finalize_simulation()
        self.logger.info("Simulation completed")
    
    def _stop_simulation(self):
        """Stop running simulation."""
        if self.simulator:
            self.simulator.stop_simulation()
        
        self.simulation_running = False
        
        if self.simulation_thread and self.simulation_thread.is_alive():
            self.simulation_thread.join(timeout=5.0)
        
        self.simulator = None
        self.logger.info("Simulation stopped")
    
    def _broadcast_telemetry_update(self):
        """Broadcast telemetry update to connected clients."""
        if not self.simulator:
            return
        
        try:
            # Get current drone state
            drone_status = self.simulator.drone.get_status_dict()
            
            # Get environmental data
            env_effects = self.simulator.environment.get_environmental_effects(
                self.simulator.drone.state.position,
                self.simulator.current_time
            )
            
            # Combine telemetry data
            telemetry = {
                'simulation_time': self.simulator.current_time,
                'position': drone_status['position'],
                'velocity': drone_status['velocity'],
                'attitude': drone_status['attitude'],
                'angular_velocity': [0, 0, 0],  # Placeholder
                'current_mass': drone_status['current_mass'],
                'fuel_level': drone_status['fuel_level'],
                'battery_level': drone_status['battery_level'],
                'power_consumption': drone_status['power_consumption'],
                'altitude': drone_status['position'][2],
                'airspeed': np.linalg.norm(drone_status['velocity']),
                'air_density': env_effects['air_density'],
                'air_pressure': env_effects['air_pressure'],
                'temperature': env_effects['temperature'],
                'wind_speed': np.linalg.norm(env_effects['wind_velocity']),
                'wind_direction': 0.0  # Placeholder
            }
            
            self.socketio.emit('telemetry_update', {
                'type': 'telemetry',
                'payload': telemetry
            })
            
        except Exception as e:
            self.logger.error(f"Error broadcasting telemetry: {e}")
    
    def _broadcast_mission_event(self, mission_update: Dict[str, Any]):
        """Broadcast mission event to connected clients."""
        try:
            event_data = {
                'simulation_time': self.simulator.current_time,
                'event_type': 'waypoint_completed',
                'action': mission_update.get('action', 'unknown'),
                'description': f"Waypoint completed: {mission_update.get('action', 'unknown')}",
                'mission_progress': mission_update.get('progress', 0.0)
            }
            
            self.socketio.emit('mission_event', {
                'type': 'mission_event',
                'payload': event_data
            })
            
        except Exception as e:
            self.logger.error(f"Error broadcasting mission event: {e}")
    
    def _broadcast_status_update(self):
        """Broadcast status update to connected clients."""
        try:
            if self.simulator:
                status = self.simulator.get_status()
                status['simulation_running'] = self.simulation_running
            else:
                status = {
                    'simulation_status': 'ready',
                    'simulation_running': False
                }
            
            self.socketio.emit('status_update', {
                'type': 'status',
                'payload': status
            })
            
        except Exception as e:
            self.logger.error(f"Error broadcasting status: {e}")
    
    def _broadcast_message(self, message_type: str, data: Dict[str, Any]):
        """Broadcast generic message to connected clients."""
        try:
            self.socketio.emit('message', {
                'type': message_type,
                'payload': data,
                'timestamp': datetime.now().isoformat()
            })
        except Exception as e:
            self.logger.error(f"Error broadcasting message: {e}")
    
    def _start_background_tasks(self):
        """Start background tasks for periodic updates."""
        def status_update_task():
            """Periodic status update task."""
            while True:
                try:
                    self._broadcast_status_update()
                    self.socketio.sleep(2)  # Update every 2 seconds
                except Exception as e:
                    self.logger.error(f"Status update task error: {e}")
                    self.socketio.sleep(5)
        
        # Start status update task
        self.socketio.start_background_task(status_update_task)
    
    def run(self):
        """Run the web server."""
        self.logger.info(f"Starting drone simulation web server on {self.host}:{self.port}")
        
        # Ensure web directory exists
        Path('web').mkdir(exist_ok=True)
        
        # Run with SocketIO
        self.socketio.run(
            self.app,
            host=self.host,
            port=self.port,
            debug=self.debug,
            allow_unsafe_werkzeug=True
        )


# Import numpy for calculations
import numpy as np


def create_app():
    """Create Flask application factory."""
    return SimulationWebServer().app


if __name__ == '__main__':
    # Create and run server
    server = SimulationWebServer(
        host='0.0.0.0',
        port=5000,
        debug=True
    )
    
    try:
        server.run()
    except KeyboardInterrupt:
        print("\nShutting down web server...")
        if server.simulation_running:
            server._stop_simulation()
