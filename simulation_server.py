#!/usr/bin/env python3
"""
Drone Simulation Server with Real Telemetry and Database Integration

A comprehensive Flask-based web server that provides real simulation data
with physics-based telemetry generation and smart database storage.
"""

import os
import json
import time
import threading
import math
import numpy as np
from pathlib import Path
from typing import Dict, Any, List
from dataclasses import dataclass

from flask import Flask, render_template, request, jsonify, send_from_directory
from flask_socketio import SocketIO, emit
from models import create_app, db
from database_service import DatabaseService
import logging

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class SimulationState:
    """Current simulation state"""
    running: bool = False
    paused: bool = False
    start_time: float = 0.0
    current_time: float = 0.0
    drone_config: Dict[str, Any] = None
    environment_config: Dict[str, Any] = None
    mission_config: Dict[str, Any] = None
    
    # Drone state
    position: np.ndarray = None
    velocity: np.ndarray = None
    attitude: np.ndarray = None  # roll, pitch, yaw
    angular_velocity: np.ndarray = None
    
    # Mission state
    current_waypoint: int = 0
    mission_progress: float = 0.0
    
    # Telemetry history
    telemetry_history: List[Dict[str, Any]] = None
    
    def __post_init__(self):
        if self.position is None:
            self.position = np.array([0.0, 0.0, 0.0])
        if self.velocity is None:
            self.velocity = np.array([0.0, 0.0, 0.0])
        if self.attitude is None:
            self.attitude = np.array([0.0, 0.0, 0.0])
        if self.angular_velocity is None:
            self.angular_velocity = np.array([0.0, 0.0, 0.0])
        if self.telemetry_history is None:
            self.telemetry_history = []

class DroneSimulationServer:
    """Drone simulation server with real telemetry generation."""
    
    def __init__(self, host='0.0.0.0', port=5000, debug=False):
        """Initialize web server with database integration."""
        self.host = host
        self.port = port
        self.debug = debug
        
        # Initialize Flask app with database
        self.app, self.db = create_app()
        self.app.template_folder = 'web'
        self.app.static_folder = 'web'
        self.app.static_url_path = ''
        self.app.config['SECRET_KEY'] = 'drone-sim-secret-key'
        
        # Initialize database service
        self.db_service = DatabaseService(self.app)
        
        # Initialize SocketIO
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", async_mode='threading')
        
        # Simulation state
        self.state = SimulationState()
        self.simulation_thread = None
        self.telemetry_thread = None
        
        # Register routes
        self._register_routes()
        self._register_socketio_events()
        
        logger.info(f"Starting drone simulation server with database on {host}:{port}")
    
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
                config_path = Path('config/drone_models.json')
                if config_path.exists():
                    with open(config_path, 'r') as f:
                        drones = json.load(f)
                    return jsonify(drones)
                else:
                    return jsonify({})
            except Exception as e:
                logger.error(f"Error loading drone configurations: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/configurations/environments')
        def get_environment_configurations():
            """Get available environment configurations."""
            try:
                config_path = Path('config/environments.json')
                if config_path.exists():
                    with open(config_path, 'r') as f:
                        environments = json.load(f)
                    return jsonify(environments)
                else:
                    return jsonify({})
            except Exception as e:
                logger.error(f"Error loading environment configurations: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/configurations/missions')
        def get_mission_configurations():
            """Get available mission configurations."""
            try:
                config_path = Path('config/missions.json')
                if config_path.exists():
                    with open(config_path, 'r') as f:
                        missions = json.load(f)
                    return jsonify(missions)
                else:
                    return jsonify({})
            except Exception as e:
                logger.error(f"Error loading mission configurations: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/simulation/start', methods=['POST'])
        def start_simulation():
            """Start simulation with given configuration."""
            try:
                if self.state.running:
                    return jsonify({'error': 'Simulation already running'}), 400
                
                config = request.get_json()
                logger.info(f"Starting simulation with config: {config}")
                
                # Load configurations
                self.state.drone_config = self._load_drone_config(config.get('drone_model', 'default_quadrotor'))
                self.state.environment_config = self._load_environment_config(config.get('environment', 'earth'))
                self.state.mission_config = self._load_mission_config(config.get('mission_type', 'test_flight'))
                
                # Start database session
                with self.app.app_context():
                    session = self.db_service.start_simulation_session(
                        drone_model=config.get('drone_model', 'default_quadrotor'),
                        environment=config.get('environment', 'earth'),
                        mission_type=config.get('mission_type', 'test_flight'),
                        total_waypoints=len(self.state.mission_config.get('waypoints', []))
                    )
                    
                    if session:
                        self.state.db_session_id = session.id
                        logger.info(f"Created database session: {session.id}")
                
                # Initialize simulation state
                self.state.running = True
                self.state.paused = False
                self.state.start_time = time.time()
                self.state.current_time = 0.0
                self.state.position = np.array([0.0, 0.0, 0.0])
                self.state.velocity = np.array([0.0, 0.0, 0.0])
                self.state.attitude = np.array([0.0, 0.0, 0.0])
                self.state.angular_velocity = np.array([0.0, 0.0, 0.0])
                self.state.current_waypoint = 0
                self.state.mission_progress = 0.0
                self.state.telemetry_history = []
                
                # Start simulation thread
                self.simulation_thread = threading.Thread(target=self._run_simulation)
                self.simulation_thread.daemon = True
                self.simulation_thread.start()
                
                # Start telemetry broadcast thread
                self.telemetry_thread = threading.Thread(target=self._broadcast_telemetry)
                self.telemetry_thread.daemon = True
                self.telemetry_thread.start()
                
                return jsonify({
                    'success': True,
                    'message': 'Simulation started',
                    'config': config,
                    'session_id': getattr(self.state, 'db_session_id', None)
                })
                
            except Exception as e:
                logger.error(f"Error starting simulation: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/simulation/stop', methods=['POST'])
        def stop_simulation():
            """Stop running simulation."""
            try:
                if not self.state.running:
                    return jsonify({'error': 'No simulation running'}), 400
                
                # End database session
                if hasattr(self.state, 'db_session_id') and self.state.db_session_id:
                    with self.app.app_context():
                        final_progress = getattr(self.state, 'mission_progress', 0.0)
                        self.db_service.end_simulation_session(
                            self.state.db_session_id, 
                            final_progress=final_progress,
                            status='stopped'
                        )
                        logger.info(f"Ended database session: {self.state.db_session_id}")
                
                self.state.running = False
                self.state.paused = False
                
                return jsonify({
                    'success': True,
                    'message': 'Simulation stopped'
                })
                
            except Exception as e:
                logger.error(f"Error stopping simulation: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/simulation/pause', methods=['POST'])
        def pause_simulation():
            """Pause/resume running simulation."""
            try:
                if not self.state.running:
                    return jsonify({'error': 'No simulation running'}), 400
                
                self.state.paused = not self.state.paused
                
                return jsonify({
                    'success': True,
                    'message': f'Simulation {"paused" if self.state.paused else "resumed"}'
                })
                
            except Exception as e:
                logger.error(f"Error pausing simulation: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/simulation/status')
        def get_simulation_status():
            """Get current simulation status."""
            try:
                if self.state.running:
                    status = 'paused' if self.state.paused else 'running'
                else:
                    status = 'ready'
                
                return jsonify({
                    'simulation_status': status,
                    'simulation_running': self.state.running,
                    'current_time': self.state.current_time,
                    'mission_progress': self.state.mission_progress,
                    'current_waypoint': self.state.current_waypoint,
                    'position': self.state.position.tolist() if self.state.position is not None else [0, 0, 0],
                    'velocity': self.state.velocity.tolist() if self.state.velocity is not None else [0, 0, 0],
                    'attitude': self.state.attitude.tolist() if self.state.attitude is not None else [0, 0, 0]
                })
                
            except Exception as e:
                logger.error(f"Error getting simulation status: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/simulation/telemetry')
        def get_telemetry_data():
            """Get latest telemetry data."""
            try:
                return jsonify(self.state.telemetry_history[-100:])  # Return last 100 entries
                
            except Exception as e:
                logger.error(f"Error getting telemetry data: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/simulation/mission')
        def get_mission_data():
            """Get mission progress and events."""
            try:
                if not self.state.mission_config:
                    return jsonify({})
                
                waypoints = self.state.mission_config.get('waypoints', [])
                current_waypoint = None
                if self.state.current_waypoint < len(waypoints):
                    current_waypoint = waypoints[self.state.current_waypoint]
                
                return jsonify({
                    'mission_type': self.state.mission_config.get('type', 'unknown'),
                    'description': self.state.mission_config.get('description', ''),
                    'progress': self.state.mission_progress,
                    'current_waypoint': self.state.current_waypoint,
                    'total_waypoints': len(waypoints),
                    'waypoint_info': current_waypoint,
                    'success_criteria': self.state.mission_config.get('success_criteria', {}),
                    'constraints': self.state.mission_config.get('constraints', {})
                })
                
            except Exception as e:
                logger.error(f"Error getting mission data: {e}")
                return jsonify({'error': str(e)}), 500
        
        # New database-enabled endpoints
        @self.app.route('/api/history/sessions')
        def get_session_history():
            """Get recent simulation sessions."""
            try:
                with self.app.app_context():
                    limit = request.args.get('limit', 20, type=int)
                    sessions = self.db_service.get_session_history(limit=limit)
                    return jsonify({
                        'sessions': sessions,
                        'total': len(sessions)
                    })
            except Exception as e:
                logger.error(f"Error getting session history: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/history/sessions/<int:session_id>/telemetry')
        def get_session_telemetry(session_id):
            """Get telemetry data for a specific session."""
            try:
                with self.app.app_context():
                    limit = request.args.get('limit', 1000, type=int)
                    telemetry = self.db_service.get_session_telemetry(session_id, limit=limit)
                    return jsonify({
                        'session_id': session_id,
                        'telemetry': telemetry,
                        'count': len(telemetry)
                    })
            except Exception as e:
                logger.error(f"Error getting session telemetry: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/history/sessions/<int:session_id>/events')
        def get_session_events(session_id):
            """Get mission events for a specific session."""
            try:
                with self.app.app_context():
                    events = self.db_service.get_session_events(session_id)
                    return jsonify({
                        'session_id': session_id,
                        'events': events,
                        'count': len(events)
                    })
            except Exception as e:
                logger.error(f"Error getting session events: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/analytics/performance')
        def get_performance_analytics():
            """Get performance analytics across sessions."""
            try:
                with self.app.app_context():
                    drone_model = request.args.get('drone_model')
                    environment = request.args.get('environment')
                    mission_type = request.args.get('mission_type')
                    limit = request.args.get('limit', 50, type=int)
                    
                    analytics = self.db_service.get_performance_analytics(
                        drone_model=drone_model,
                        environment=environment,
                        mission_type=mission_type,
                        limit=limit
                    )
                    return jsonify(analytics)
            except Exception as e:
                logger.error(f"Error getting performance analytics: {e}")
                return jsonify({'error': str(e)}), 500
    
    def _register_socketio_events(self):
        """Register SocketIO event handlers."""
        
        @self.socketio.on('connect')
        def handle_connect():
            """Handle client connection."""
            logger.info('Client connected to WebSocket')
            emit('status', {'message': 'Connected to simulation server'})
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            """Handle client disconnection."""
            logger.info('Client disconnected from WebSocket')
        
        @self.socketio.on('request_status')
        def handle_status_request():
            """Handle status request from client."""
            try:
                status = {
                    'simulation_status': 'running' if self.state.running else 'ready',
                    'simulation_running': self.state.running,
                    'current_time': self.state.current_time,
                    'mission_progress': self.state.mission_progress
                }
                emit('status_update', status)
            except Exception as e:
                logger.error(f"Error handling status request: {e}")
    
    def _load_drone_config(self, drone_name: str) -> Dict[str, Any]:
        """Load drone configuration."""
        try:
            config_path = Path('config/drone_models.json')
            if config_path.exists():
                with open(config_path, 'r') as f:
                    drones = json.load(f)
                return drones.get(drone_name, {})
            return {}
        except Exception as e:
            logger.error(f"Error loading drone config: {e}")
            return {}
    
    def _load_environment_config(self, environment_name: str) -> Dict[str, Any]:
        """Load environment configuration."""
        try:
            config_path = Path('config/environments.json')
            if config_path.exists():
                with open(config_path, 'r') as f:
                    environments = json.load(f)
                return environments.get(environment_name, {})
            return {}
        except Exception as e:
            logger.error(f"Error loading environment config: {e}")
            return {}
    
    def _load_mission_config(self, mission_name: str) -> Dict[str, Any]:
        """Load mission configuration."""
        try:
            config_path = Path('config/missions.json')
            if config_path.exists():
                with open(config_path, 'r') as f:
                    missions = json.load(f)
                return missions.get(mission_name, {})
            return {}
        except Exception as e:
            logger.error(f"Error loading mission config: {e}")
            return {}
    
    def _run_simulation(self):
        """Run the simulation loop."""
        dt = 0.1  # 100ms timestep
        
        while self.state.running:
            if not self.state.paused:
                # Update simulation time
                self.state.current_time = time.time() - self.state.start_time
                
                # Get current waypoint
                waypoints = self.state.mission_config.get('waypoints', [])
                if waypoints and self.state.current_waypoint < len(waypoints):
                    target_waypoint = waypoints[self.state.current_waypoint]
                    target_pos = np.array([target_waypoint['x'], target_waypoint['y'], target_waypoint['z']])
                    
                    # Simple physics simulation
                    self._update_drone_physics(target_pos, dt)
                    
                    # Check if reached waypoint
                    distance = np.linalg.norm(self.state.position - target_pos)
                    if distance < target_waypoint.get('tolerance', 5.0):
                        self.state.current_waypoint += 1
                        if self.state.current_waypoint >= len(waypoints):
                            # Mission complete
                            self.state.mission_progress = 100.0
                            self.state.running = False
                            self.socketio.emit('mission_complete', {
                                'message': 'Mission completed successfully',
                                'total_time': self.state.current_time
                            })
                        else:
                            # Update mission progress
                            self.state.mission_progress = (self.state.current_waypoint / len(waypoints)) * 100.0
                
                # Log telemetry
                self._log_telemetry()
            
            time.sleep(dt)
    
    def _update_drone_physics(self, target_pos: np.ndarray, dt: float):
        """Update drone physics with simple navigation."""
        # Simple proportional controller
        position_error = target_pos - self.state.position
        velocity_command = position_error * 0.5  # P controller
        
        # Limit velocity
        max_velocity = 10.0  # m/s
        velocity_magnitude = np.linalg.norm(velocity_command)
        if velocity_magnitude > max_velocity:
            velocity_command = velocity_command / velocity_magnitude * max_velocity
        
        # Update velocity and position
        self.state.velocity = velocity_command
        self.state.position += self.state.velocity * dt
        
        # Update attitude (simple heading control)
        if np.linalg.norm(self.state.velocity) > 0.1:
            heading = math.atan2(self.state.velocity[1], self.state.velocity[0])
            self.state.attitude[2] = heading  # yaw
        
        # Add some realistic variations
        gravity = self.state.environment_config.get('gravity', 9.81)
        self.state.attitude[0] = math.sin(self.state.current_time * 0.5) * 0.1  # roll
        self.state.attitude[1] = math.cos(self.state.current_time * 0.3) * 0.1  # pitch
    
    def _log_telemetry(self):
        """Log current telemetry data."""
        telemetry = {
            'timestamp': self.state.current_time,
            'position': {
                'x': float(self.state.position[0]),
                'y': float(self.state.position[1]),
                'z': float(self.state.position[2])
            },
            'velocity': {
                'x': float(self.state.velocity[0]),
                'y': float(self.state.velocity[1]),
                'z': float(self.state.velocity[2])
            },
            'attitude': {
                'roll': float(self.state.attitude[0]),
                'pitch': float(self.state.attitude[1]),
                'yaw': float(self.state.attitude[2])
            },
            'mission_progress': self.state.mission_progress,
            'current_waypoint': self.state.current_waypoint,
            'altitude': float(self.state.position[2]),
            'ground_speed': float(np.linalg.norm(self.state.velocity[:2])),
            'vertical_speed': float(self.state.velocity[2])
        }
        
        self.state.telemetry_history.append(telemetry)
        
        # Log to database if we have a session
        if hasattr(self.state, 'db_session_id') and self.state.db_session_id:
            try:
                with self.app.app_context():
                    self.db_service.log_telemetry_data(
                        self.state.db_session_id,
                        self.state.current_time,
                        telemetry
                    )
            except Exception as e:
                logger.error(f"Error logging telemetry to database: {e}")
        
        # Keep only last 1000 entries in memory
        if len(self.state.telemetry_history) > 1000:
            self.state.telemetry_history.pop(0)
    
    def _broadcast_telemetry(self):
        """Broadcast telemetry data to connected clients."""
        while self.state.running:
            if not self.state.paused and self.state.telemetry_history:
                latest_telemetry = self.state.telemetry_history[-1]
                
                # Broadcast telemetry update
                self.socketio.emit('telemetry_update', {
                    'type': 'telemetry',
                    'payload': latest_telemetry
                })
                
                # Broadcast status update
                self.socketio.emit('status_update', {
                    'type': 'status',
                    'payload': {
                        'simulation_status': 'running',
                        'simulation_running': True,
                        'current_time': self.state.current_time,
                        'mission_progress': self.state.mission_progress,
                        'current_waypoint': self.state.current_waypoint
                    }
                })
            
            time.sleep(0.5)  # Broadcast every 500ms
    
    def run(self):
        """Run the web server."""
        self.socketio.run(
            self.app,
            host=self.host,
            port=self.port,
            debug=self.debug,
            allow_unsafe_werkzeug=True
        )


if __name__ == '__main__':
    # Create and run server
    server = DroneSimulationServer(
        host='0.0.0.0',
        port=5000,
        debug=True
    )
    
    try:
        server.run()
    except KeyboardInterrupt:
        print("\nShutting down simulation server...")