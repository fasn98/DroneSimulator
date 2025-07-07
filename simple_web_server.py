#!/usr/bin/env python3
"""
Simple Web Server for Drone Simulation Interface

Provides a minimal Flask-based web interface for demonstrating
the drone simulation system.
"""

import os
import json
from pathlib import Path
from typing import Dict, Any

from flask import Flask, render_template, request, jsonify, send_from_directory
from flask_socketio import SocketIO, emit
import logging

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SimpleSimulationServer:
    """Simple web server for drone simulation demonstration."""
    
    def __init__(self, host='0.0.0.0', port=5000, debug=False):
        """Initialize web server."""
        self.host = host
        self.port = port
        self.debug = debug
        
        # Initialize Flask app
        self.app = Flask(__name__, 
                        template_folder='web',
                        static_folder='web',
                        static_url_path='')
        self.app.config['SECRET_KEY'] = 'drone-sim-secret-key'
        
        # Initialize SocketIO
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", async_mode='threading')
        
        # Simulation state
        self.simulation_running = False
        
        # Register routes
        self._register_routes()
        self._register_socketio_events()
        
        logger.info(f"Starting simple drone simulation web server on {host}:{port}")
    
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
                    return jsonify({
                        "default_quadrotor": {
                            "description": "Default quadrotor configuration",
                            "type": "quadrotor"
                        }
                    })
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
                    return jsonify({
                        "earth": {
                            "description": "Earth standard atmosphere",
                            "gravity": 9.81
                        }
                    })
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
                    return jsonify({
                        "test_flight": {
                            "description": "Simple test flight",
                            "type": "test"
                        }
                    })
            except Exception as e:
                logger.error(f"Error loading mission configurations: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/simulation/start', methods=['POST'])
        def start_simulation():
            """Start simulation with given configuration."""
            try:
                if self.simulation_running:
                    return jsonify({'error': 'Simulation already running'}), 400
                
                config = request.get_json()
                logger.info(f"Starting simulation with config: {config}")
                
                self.simulation_running = True
                
                # Broadcast simulation started
                self.socketio.emit('status_update', {
                    'type': 'status',
                    'payload': {
                        'simulation_status': 'running',
                        'simulation_running': True
                    }
                })
                
                return jsonify({
                    'success': True,
                    'message': 'Simulation started',
                    'config': config
                })
                
            except Exception as e:
                logger.error(f"Error starting simulation: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/simulation/stop', methods=['POST'])
        def stop_simulation():
            """Stop running simulation."""
            try:
                if not self.simulation_running:
                    return jsonify({'error': 'No simulation running'}), 400
                
                self.simulation_running = False
                
                # Broadcast simulation stopped
                self.socketio.emit('status_update', {
                    'type': 'status',
                    'payload': {
                        'simulation_status': 'ready',
                        'simulation_running': False
                    }
                })
                
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
                if not self.simulation_running:
                    return jsonify({'error': 'No simulation running'}), 400
                
                # For demo purposes, just return success
                return jsonify({
                    'success': True,
                    'message': 'Simulation paused/resumed'
                })
                
            except Exception as e:
                logger.error(f"Error pausing simulation: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/simulation/status')
        def get_simulation_status():
            """Get current simulation status."""
            try:
                status = {
                    'simulation_status': 'running' if self.simulation_running else 'ready',
                    'simulation_running': self.simulation_running,
                    'current_time': 0.0,
                    'mission_progress': 0.0
                }
                return jsonify(status)
                
            except Exception as e:
                logger.error(f"Error getting simulation status: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/simulation/telemetry')
        def get_telemetry_data():
            """Get latest telemetry data."""
            try:
                # Return empty for now
                return jsonify([])
                
            except Exception as e:
                logger.error(f"Error getting telemetry data: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/simulation/mission')
        def get_mission_data():
            """Get mission progress and events."""
            try:
                # Return empty for now
                return jsonify({})
                
            except Exception as e:
                logger.error(f"Error getting mission data: {e}")
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
                    'simulation_status': 'running' if self.simulation_running else 'ready',
                    'simulation_running': self.simulation_running
                }
                emit('status_update', status)
            except Exception as e:
                logger.error(f"Error handling status request: {e}")
    
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
    server = SimpleSimulationServer(
        host='0.0.0.0',
        port=5000,
        debug=True
    )
    
    try:
        server.run()
    except KeyboardInterrupt:
        print("\nShutting down web server...")