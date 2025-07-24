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
from src.google_maps_integration import GoogleMapsIntegration
from src.ai_environment_generator import AIEnvironmentGenerator
from src.video_export_system import VideoExportSystem
from src.post_simulation_video_creator import PostSimulationVideoCreator
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
        
        # Initialize database service (lazy initialization)
        self.db_service = None
        self.db_initialized = False
        
        # Initialize enhanced features (lazy initialization for faster startup)
        self.google_maps = None
        self.ai_env_generator = None
        self.video_export = None
        self.post_video_creator = None
        self._enhanced_features_initialized = False
        
        # Initialize SocketIO
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", async_mode='threading')
        
        # Simulation state
        self.state = SimulationState()
        self.simulation_thread = None
        self.telemetry_thread = None
        
        # Register routes and events
        self._register_routes()
        self._register_socketio_events()
        
        logger.info(f"Drone simulation server initialized on {host}:{port}")
    
    def _init_database_service(self):
        """Initialize database service if not already initialized."""
        if not self.db_initialized:
            try:
                # Initialize database tables
                if hasattr(self.app, 'init_db'):
                    self.app.init_db()
                
                # Initialize database service
                self.db_service = DatabaseService(self.app)
                self.db_initialized = True
                logger.info("Database service initialized successfully")
            except Exception as e:
                logger.error(f"Failed to initialize database service: {e}")
                # Continue without database service for basic functionality
    
    def _init_enhanced_features(self):
        """Initialize enhanced features if not already initialized."""
        if not self._enhanced_features_initialized:
            try:
                # Initialize enhanced features
                self.google_maps = GoogleMapsIntegration()
                self.ai_env_generator = AIEnvironmentGenerator()
                self.video_export = VideoExportSystem()
                self.post_video_creator = PostSimulationVideoCreator()
                
                # Set API keys for enhanced features
                google_maps_key = os.environ.get('GOOGLE_MAPS_API_KEY')
                if google_maps_key:
                    self.google_maps.set_api_key(google_maps_key)
                    logger.info("Google Maps API key configured")
                else:
                    logger.warning("Google Maps API key not found in environment")
                    
                openai_key = os.environ.get('OPENAI_API_KEY')
                if openai_key:
                    if hasattr(self.ai_env_generator, 'set_api_key'):
                        self.ai_env_generator.set_api_key(openai_key)
                        logger.info("OpenAI API key configured")
                else:
                    logger.warning("OpenAI API key not found in environment")
                
                self._enhanced_features_initialized = True
                logger.info("Enhanced features initialized successfully")
            except Exception as e:
                logger.error(f"Failed to initialize enhanced features: {e}")
                # Continue without enhanced features for basic functionality
    
    def _register_routes(self):
        """Register Flask routes."""
        
        @self.app.route('/')
        def index():
            """Serve main interface."""
            return send_from_directory('web', 'index.html')
        
        @self.app.route('/health')
        def health_check():
            """Simple health check endpoint for deployment."""
            return jsonify({
                'status': 'healthy',
                'service': 'drone-simulation-server',
                'timestamp': time.time()
            }), 200
        
        @self.app.route('/script.js')
        def script():
            """Serve JavaScript file."""
            return send_from_directory('web', 'script.js')
            
        @self.app.route('/style.css')
        def style():
            """Serve CSS file."""
            return send_from_directory('web', 'style.css')
        
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
                
                # Use custom environment if available, otherwise load from config
                if hasattr(self, 'custom_environment_config') and self.custom_environment_config:
                    self.state.environment_config = self.custom_environment_config
                    logger.info("Using custom environment configuration")
                else:
                    self.state.environment_config = self._load_environment_config(config.get('environment', 'earth'))
                
                self.state.mission_config = self._load_mission_config(config.get('mission_type', 'test_flight'))
                
                # Initialize database service if needed
                self._init_database_service()
                
                # Start database session if database service is available
                if self.db_service:
                    with self.app.app_context():
                        session = self.db_service.start_simulation_session(
                            drone_model=config.get('drone_model', 'default_quadrotor'),
                            environment=config.get('environment', 'earth'),
                            mission_type=config.get('mission_type', 'test_flight'),
                            total_waypoints=len(self.state.mission_config.get('waypoints', []))
                        )
                        
                        if session:
                            self.current_session_id = session.id
                            logger.info(f"Created database session: {session.id}")
                else:
                    logger.warning("Database service not available, simulation will run without database logging")
                
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
                
                # Environment config is already stored in self.state.environment_config above
                
                # Start simulation thread
                logger.info("Starting simulation thread...")
                self.simulation_thread = threading.Thread(target=self._run_simulation)
                self.simulation_thread.daemon = True
                self.simulation_thread.start()
                
                # Start telemetry broadcast thread  
                logger.info("Starting telemetry broadcast thread...")
                self.telemetry_thread = threading.Thread(target=self._broadcast_telemetry)
                self.telemetry_thread.daemon = True
                self.telemetry_thread.start()
                
                logger.info(f"Simulation threads started. State running: {self.state.running}")
                
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
                self._complete_simulation('stopped')
                
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
                    # Initialize database service if needed
                    if not self.db_service:
                        self._init_database_service()
                    
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
                    # Initialize database service if needed
                    if not self.db_service:
                        self._init_database_service()
                    
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
        
        @self.app.route('/api/analytics/sessions')
        def get_analytics_sessions():
            """Get session history for Analytics tab."""
            try:
                with self.app.app_context():
                    # Initialize database service if needed
                    if not self.db_service:
                        self._init_database_service()
                    
                    limit = request.args.get('limit', 50, type=int)
                    sessions = self.db_service.get_session_history(limit=limit)
                    return jsonify({
                        'sessions': sessions,
                        'total': len(sessions)
                    })
            except Exception as e:
                logger.error(f"Error getting analytics sessions: {e}")
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
        
        # Enhanced Features API Endpoints
        
        @self.app.route('/api/maps/locations/popular')
        def get_popular_locations():
            """Get popular real-world locations for simulation."""
            try:
                self._init_enhanced_features()
                if self.google_maps:
                    locations = self.google_maps.get_popular_locations()
                    return jsonify({'locations': locations})
                else:
                    return jsonify({'error': 'Google Maps service not available'}), 503
            except Exception as e:
                logger.error(f"Error getting popular locations: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/maps/locations/search')
        def search_locations():
            """Search for real-world locations."""
            try:
                self._init_enhanced_features()
                query = request.args.get('query')
                if not query:
                    return jsonify({'error': 'Query parameter required'}), 400
                
                if self.google_maps:
                    results = self.google_maps.search_locations(query)
                    return jsonify({'results': results or []})
                else:
                    return jsonify({'error': 'Google Maps service not available'}), 503
            except Exception as e:
                logger.error(f"Error searching locations: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/maps/terrain/<float:latitude>/<float:longitude>')
        def get_terrain_data(latitude, longitude):
            """Get 3D terrain data for a location."""
            try:
                grid_size = int(request.args.get('grid_size', 20))
                
                terrain_model = self.google_maps.create_3d_terrain_model(
                    latitude, longitude, grid_size
                )
                
                if terrain_model:
                    return jsonify(terrain_model)
                else:
                    return jsonify({'error': 'Failed to generate terrain model'}), 500
                    
            except Exception as e:
                logger.error(f"Error getting terrain data: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/maps/terrain/generate', methods=['POST'])
        def generate_terrain_data():
            """Generate 3D terrain data for a location via POST request."""
            try:
                data = request.get_json() or {}
                latitude = float(data.get('latitude', 0))
                longitude = float(data.get('longitude', 0))
                grid_size = int(data.get('size', 20))
                
                terrain_model = self.google_maps.create_3d_terrain_model(
                    latitude, longitude, grid_size
                )
                
                if terrain_model:
                    return jsonify(terrain_model)
                else:
                    return jsonify({'error': 'Failed to generate terrain model'}), 500
                    
            except Exception as e:
                logger.error(f"Error generating terrain data: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/environments/ai/lunar')
        def generate_lunar_environment():
            """Generate AI-powered lunar environment."""
            try:
                region_type = request.args.get('region_type', 'highland')
                size_km = float(request.args.get('size_km', 10.0))
                
                environment = self.ai_env_generator.generate_lunar_environment(
                    region_type, size_km
                )
                
                if environment:
                    return jsonify(environment)
                else:
                    return jsonify({'error': 'Failed to generate lunar environment'}), 500
                    
            except Exception as e:
                logger.error(f"Error generating lunar environment: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/environments/ai/martian')
        def generate_martian_environment():
            """Generate AI-powered Martian environment."""
            try:
                region_type = request.args.get('region_type', 'plains')
                season = request.args.get('season', 'spring')
                
                environment = self.ai_env_generator.generate_martian_environment(
                    region_type, season
                )
                
                if environment:
                    return jsonify(environment)
                else:
                    return jsonify({'error': 'Failed to generate Martian environment'}), 500
                    
            except Exception as e:
                logger.error(f"Error generating Martian environment: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/environments/ai/presets')
        def get_environment_presets():
            """Get AI environment presets."""
            try:
                presets = self.ai_env_generator.get_environment_presets()
                return jsonify(presets)
            except Exception as e:
                logger.error(f"Error getting environment presets: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/video/status')
        def get_video_status():
            """Get current video recording status."""
            try:
                return jsonify({
                    'recording': self.video_export.recording if hasattr(self.video_export, 'recording') else False,
                    'session_id': getattr(self.video_export, 'session_id', None),
                    'frames_captured': len(self.video_export.frames) if hasattr(self.video_export, 'frames') else 0,
                    'start_time': getattr(self.video_export, 'start_time', None),
                    'duration': time.time() - getattr(self.video_export, 'start_time', time.time()) if hasattr(self.video_export, 'recording') and self.video_export.recording else 0
                })
            except Exception as e:
                logger.error(f"Error getting video status: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/video/files')
        def list_video_files():
            """List available video files."""
            try:
                video_dir = Path('video_exports')
                if not video_dir.exists():
                    return jsonify({'files': []})
                
                files = []
                for file_path in video_dir.glob('*.mp4'):
                    stat = file_path.stat()
                    files.append({
                        'filename': file_path.name,
                        'size_mb': stat.st_size / (1024 * 1024),
                        'created_at': stat.st_mtime,
                        'path': str(file_path)
                    })
                
                # Sort by creation time, newest first
                files.sort(key=lambda x: x['created_at'], reverse=True)
                
                return jsonify({'files': files})
                
            except Exception as e:
                logger.error(f"Error listing video files: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/video/export/start', methods=['POST'])
        def start_video_recording():
            """Start video recording for current simulation."""
            try:
                data = request.get_json() or {}
                session_id = getattr(self, 'current_session_id', 'unknown')
                export_settings = data.get('export_settings', {})
                
                success = self.video_export.start_recording(session_id, export_settings)
                
                return jsonify({
                    'success': success,
                    'session_id': session_id,
                    'message': 'Video recording started' if success else 'Failed to start recording'
                })
                
            except Exception as e:
                logger.error(f"Error starting video recording: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/video/export/stop', methods=['POST'])
        def stop_video_recording():
            """Stop video recording."""
            try:
                success = self.video_export.stop_recording()
                
                return jsonify({
                    'success': success,
                    'message': 'Video recording stopped' if success else 'Failed to stop recording'
                })
                
            except Exception as e:
                logger.error(f"Error stopping video recording: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/video/export/render', methods=['POST'])
        def export_video():
            """Export recorded video as MP4."""
            try:
                data = request.get_json() or {}
                session_id = getattr(self, 'current_session_id', 'unknown')
                
                # Create output directory if it doesn't exist
                output_dir = 'video_exports'
                os.makedirs(output_dir, exist_ok=True)
                
                # Generate filename
                timestamp = time.strftime('%Y%m%d_%H%M%S')
                filename = f"drone_flight_{session_id}_{timestamp}.mp4"
                output_path = os.path.join(output_dir, filename)
                
                export_options = data.get('export_options', {})
                
                # Export video
                result = self.video_export.export_video(output_path, export_options)
                
                return jsonify(result)
                
            except Exception as e:
                logger.error(f"Error exporting video: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/video/presets')
        def get_video_presets():
            """Get video export presets."""
            try:
                presets = self.video_export.get_export_presets()
                return jsonify(presets)
            except Exception as e:
                logger.error(f"Error getting video presets: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/video/download/<filename>')
        def download_video(filename):
            """Download exported video file."""
            try:
                output_dir = 'video_exports'
                
                return send_from_directory(output_dir, filename, as_attachment=True)
                
            except Exception as e:
                logger.error(f"Error downloading video: {e}")
                return jsonify({'error': str(e)}), 500
        
        # Post-simulation video creation endpoints
        @self.app.route('/api/video/templates')
        def get_video_templates():
            """Get available video templates."""
            try:
                templates = self.post_video_creator.get_available_templates()
                return jsonify({'templates': templates})
            except Exception as e:
                logger.error(f"Error getting templates: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/video/create', methods=['POST'])
        def create_video_from_session():
            """Create video from completed session using stored telemetry."""
            try:
                data = request.get_json() or {}
                session_id = data.get('session_id')
                template = data.get('template', 'professional')
                include_audio = data.get('include_audio', True)
                
                if not session_id:
                    return jsonify({'error': 'Session ID required'}), 400
                
                # Get telemetry data from database
                if not self.db_service:
                    self._init_database_service()
                
                telemetry_data = self.db_service.get_session_telemetry(session_id, limit=None)
                
                if not telemetry_data:
                    return jsonify({'error': 'No telemetry data found for session'}), 404
                
                # Get session info for context
                sessions = self.db_service.get_session_history(limit=100)
                session_info = next((s for s in sessions if s['id'] == session_id), None)
                
                # Create video
                result = self.post_video_creator.create_video_from_telemetry(
                    telemetry_data=telemetry_data,
                    template=template,
                    drone_config={'model': session_info.get('drone_model') if session_info else None},
                    environment_config={'name': session_info.get('environment') if session_info else None},
                    mission_config={'type': session_info.get('mission_type') if session_info else None},
                    include_audio=include_audio
                )
                
                return jsonify(result)
                
            except Exception as e:
                logger.error(f"Error creating video from session: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/video/create/current', methods=['POST']) 
        def create_video_from_current_session():
            """Create video from currently stored telemetry data."""
            try:
                data = request.get_json() or {}
                template = data.get('template', 'professional')
                include_audio = data.get('include_audio', True)
                
                # Get telemetry data from the most recent session in database
                if not self.db_service:
                    self._init_database_service()
                
                # Get the most recent session
                sessions = self.db_service.get_session_history(limit=1)
                if not sessions:
                    return jsonify({'error': 'No simulation sessions found'}), 404
                
                current_session = sessions[0]
                session_id = current_session['id']
                
                # Get all telemetry data for this session (no limit)
                telemetry_data = self.db_service.get_session_telemetry(session_id, limit=None)
                
                if not telemetry_data:
                    return jsonify({'error': 'No telemetry data available for current session'}), 404
                
                # Create video
                logger.info(f"Creating video with {len(telemetry_data)} telemetry points from session {session_id}")
                
                result = self.post_video_creator.create_video_from_telemetry(
                    telemetry_data=telemetry_data,
                    template=template,
                    drone_config={'model': current_session.get('drone_model')},
                    environment_config={'name': current_session.get('environment')},
                    mission_config={'type': current_session.get('mission_type')},
                    include_audio=include_audio
                )
                
                return jsonify(result)
                
            except Exception as e:
                logger.error(f"Error creating video from current session: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/video/create/real-world', methods=['POST'])
        def create_real_world_video():
            """Create video with Real World Google Maps terrain integration."""
            try:
                data = request.get_json() or {}
                template = data.get('template', 'professional')
                include_audio = data.get('include_audio', True)
                location = data.get('location', 'Grand Canyon, Arizona')
                
                # Set up a Real World simulation configuration
                config = {
                    'drone': 'quadcopter_x4',
                    'environment': 'earth',
                    'mission': 'reconnaissance',
                    'duration': 10,  # Short demo video
                    'timestep': 0.1,
                    'realtime': False
                }
                
                # Run a quick simulation with Google Maps terrain
                logger.info(f"Creating Real World video for location: {location}")
                
                # For Real World video, we'll use existing session data and create a video with location context
                # Get the telemetry data from the most recent session
                if not self.db_service:
                    self._init_database_service()
                
                sessions = self.db_service.get_session_history(limit=1)
                if not sessions:
                    return jsonify({'error': 'No simulation sessions found. Please run a simulation first.'}), 404
                
                session_id = sessions[0]['id']
                telemetry_data = self.db_service.get_session_telemetry(session_id, limit=None)
                
                if not telemetry_data:
                    return jsonify({'error': 'No telemetry data available. Please complete a simulation first.'}), 404
                
                # Create video with Google Maps terrain context
                result = self.post_video_creator.create_video_from_telemetry(
                    telemetry_data=telemetry_data,
                    template=template,
                    drone_config={'model': 'quadcopter_x4'},
                    environment_config={'name': 'earth', 'location': location},
                    mission_config={'type': 'reconnaissance'},
                    include_audio=include_audio
                )
                
                return jsonify(result)
                
            except Exception as e:
                logger.error(f"Error creating Real World video: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/api/environment/custom', methods=['POST'])
        def apply_custom_environment():
            """Apply custom environment configuration for next simulation."""
            try:
                custom_config = request.get_json() or {}
                
                # Validate required fields
                if 'gravity' not in custom_config:
                    return jsonify({'error': 'Gravity value required'}), 400
                
                # Store custom environment configuration
                self.custom_environment_config = custom_config
                
                logger.info(f"Custom environment applied: gravity={custom_config.get('gravity')} m/s²")
                
                return jsonify({
                    'success': True,
                    'message': 'Custom environment configuration applied',
                    'config': custom_config
                })
                
            except Exception as e:
                logger.error(f"Error applying custom environment: {e}")
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
        logger.info("Simulation loop started")
        dt = 0.1  # 100ms timestep
        loop_count = 0
        # Get mission duration from flight_time_max or default to 900 seconds
        mission_criteria = self.state.mission_config.get('success_criteria', {})
        simulation_duration = mission_criteria.get('flight_time_max', 900.0)
        mission_status = "executing"  # Change status from planning to executing
        total_distance = 0.0  # Track total distance traveled
        
        while self.state.running and self.state.current_time < simulation_duration:
            loop_count += 1
            if loop_count % 50 == 0:  # Log every 5 seconds
                logger.info(f"Simulation loop running - iteration {loop_count}, time: {self.state.current_time:.1f}s")
            if not self.state.paused:
                # Update simulation time (use simulation time, not real time)
                self.state.current_time += dt
                
                # Store previous position for distance calculation
                prev_position = self.state.position.copy()
                
                # Get current waypoint
                waypoints = self.state.mission_config.get('waypoints', [])
                if waypoints and self.state.current_waypoint < len(waypoints):
                    target_waypoint = waypoints[self.state.current_waypoint]
                    target_pos = np.array([target_waypoint['x'], target_waypoint['y'], target_waypoint['z']])
                    
                    # Simple physics simulation
                    self._update_drone_physics(target_pos, dt)
                    
                    # Calculate distance traveled this step
                    step_distance = np.linalg.norm(self.state.position - prev_position)
                    total_distance += step_distance
                    
                    # Check if reached waypoint with realistic timing
                    distance = np.linalg.norm(self.state.position - target_pos)
                    tolerance = target_waypoint.get('tolerance', 5.0)
                    
                    if distance < tolerance:
                        # Add minimum time at waypoint for realistic mission timing
                        if not hasattr(self.state, 'waypoint_arrival_time'):
                            self.state.waypoint_arrival_time = self.state.current_time
                        
                        # Stay at waypoint for the specified duration
                        waypoint_duration = target_waypoint.get('duration', 10.0)
                        time_at_waypoint = self.state.current_time - self.state.waypoint_arrival_time
                        
                        if time_at_waypoint >= waypoint_duration:
                            # Move to next waypoint
                            self.state.current_waypoint += 1
                            delattr(self.state, 'waypoint_arrival_time')  # Reset for next waypoint
                            
                            if self.state.current_waypoint >= len(waypoints):
                                # Mission complete
                                self.state.mission_progress = 100.0
                                self.state.running = False
                                
                                # End the session in database with completed status
                                self._complete_simulation('completed')
                                
                                self.socketio.emit('mission_complete', {
                                    'message': 'Mission completed successfully',
                                    'total_time': self.state.current_time
                                })
                            else:
                                # Update mission progress
                                self.state.mission_progress = (self.state.current_waypoint / len(waypoints)) * 100.0
                
                # Log telemetry with mission status and distance
                self._log_telemetry(mission_status, total_distance)
            
            time.sleep(dt)
        
        # Handle simulation completion due to timeout
        if self.state.current_time >= simulation_duration:
            logger.info(f"Simulation completed due to timeout at {simulation_duration}s")
            self.state.running = False
            self._complete_simulation('completed_timeout')
    
    def _complete_simulation(self, status='completed'):
        """Complete the current simulation session with proper cleanup."""
        if hasattr(self, 'current_session_id') and self.current_session_id:
            try:
                with self.app.app_context():
                    final_progress = getattr(self.state, 'mission_progress', 0.0)
                    success = self.db_service.end_simulation_session(
                        self.current_session_id, 
                        final_progress=final_progress,
                        status=status
                    )
                    if success:
                        logger.info(f"Session {self.current_session_id} ended with status: {status}")
                    else:
                        logger.error(f"Failed to end session {self.current_session_id}")
                        
                    # Clear the session ID
                    self.current_session_id = None
                
            except Exception as e:
                logger.error(f"Error completing simulation: {e}")
                import traceback
                traceback.print_exc()
    
    def _update_drone_physics(self, target_pos: np.ndarray, dt: float):
        """Update drone physics with realistic navigation timing."""
        # Calculate position error
        position_error = target_pos - self.state.position
        distance_to_target = np.linalg.norm(position_error)
        
        # Realistic drone speeds and acceleration
        max_velocity = 5.0  # m/s (realistic drone speed)
        max_acceleration = 2.0  # m/s^2 (realistic acceleration)
        
        # Proportional controller with realistic gains
        velocity_command = position_error * 0.2  # Lower gain for smoother movement
        
        # Limit velocity more realistically
        velocity_magnitude = np.linalg.norm(velocity_command)
        if velocity_magnitude > max_velocity:
            velocity_command = velocity_command / velocity_magnitude * max_velocity
        
        # Apply acceleration limits for realistic movement
        velocity_diff = velocity_command - self.state.velocity
        accel_magnitude = np.linalg.norm(velocity_diff) / dt
        if accel_magnitude > max_acceleration:
            velocity_diff = velocity_diff / accel_magnitude * max_acceleration * dt
        
        # Update velocity and position with realistic physics
        self.state.velocity += velocity_diff
        self.state.position += self.state.velocity * dt
        
        # Update attitude (simple heading control)
        if np.linalg.norm(self.state.velocity) > 0.1:
            heading = math.atan2(self.state.velocity[1], self.state.velocity[0])
            self.state.attitude[2] = heading  # yaw
        
        # Add some realistic variations
        gravity = self.state.environment_config.get('gravity', 9.81)
        self.state.attitude[0] = math.sin(self.state.current_time * 0.5) * 0.1  # roll
        self.state.attitude[1] = math.cos(self.state.current_time * 0.3) * 0.1  # pitch
    
    def _get_environment_data(self):
        """Get current environment data for display."""
        if not hasattr(self.state, 'environment_config'):
            return {}
        
        env_config = self.state.environment_config
        atmosphere = env_config.get('atmosphere', {})
        wind = env_config.get('wind', {})
        
        # Calculate altitude-dependent values
        altitude = float(self.state.position[2])
        gravity = env_config.get('gravity', 9.81)
        
        # Air density (simplified calculation)
        sea_level_density = atmosphere.get('sea_level_density', 1.225)
        if sea_level_density > 0:  # Not vacuum
            air_density = sea_level_density * math.exp(-altitude / 8400)  # Scale height approximation
        else:
            air_density = 0.0
            
        # Temperature (simplified calculation)
        sea_level_temp = atmosphere.get('sea_level_temperature', 288.15)
        lapse_rate = atmosphere.get('temperature_lapse_rate', -0.0065)
        temperature_k = sea_level_temp + lapse_rate * altitude
        temperature_c = temperature_k - 273.15
        
        # Wind speed
        wind_speed = wind.get('base_speed', 0.0) if wind.get('enabled', False) else 0.0
        
        return {
            'gravity': gravity,
            'air_density': air_density,
            'temperature': temperature_c,
            'wind_speed': wind_speed
        }
    
    def _log_telemetry(self, mission_status="executing", total_distance=0.0):
        """Log current telemetry data with enhanced mission information."""
        telemetry = {
            'timestamp': self.state.current_time,  # Use simulation time consistently
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
            'vertical_speed': float(self.state.velocity[2]),
            'vector_speed': float(np.linalg.norm(self.state.velocity)),  # Total 3D speed
            'mission_status': mission_status,
            'total_distance': total_distance
        }
        
        self.state.telemetry_history.append(telemetry)
        
        # Log to database if we have a session and database service
        if hasattr(self, 'current_session_id') and self.current_session_id and self.db_service:
            try:
                with self.app.app_context():
                    self.db_service.log_telemetry_data(
                        self.current_session_id,
                        self.state.current_time,
                        telemetry
                    )
            except Exception as e:
                logger.error(f"Error logging telemetry to database: {e}")
        
        # Add frame to video recording if active
        if self.video_export and hasattr(self.video_export, 'recording') and self.video_export.recording:
            try:
                self.video_export.add_frame(telemetry, view_type="chase")
            except Exception as e:
                logger.error(f"Error adding video frame: {e}")
        
        # Keep only last 1000 entries in memory
        if len(self.state.telemetry_history) > 1000:
            self.state.telemetry_history.pop(0)
    
    def _broadcast_telemetry(self):
        """Broadcast telemetry data to connected clients."""
        while self.state.running:
            if not self.state.paused and self.state.telemetry_history:
                latest_telemetry = self.state.telemetry_history[-1]
                
                # Calculate environment data
                env_data = self._get_environment_data()
                
                # Add environment data to telemetry
                latest_telemetry['environment'] = env_data
                
                # Broadcast telemetry update
                self.socketio.emit('telemetry_update', {
                    'type': 'telemetry',
                    'payload': latest_telemetry
                })
                
                # Calculate environment data
                env_data = self._get_environment_data()
                
                # Broadcast status update
                self.socketio.emit('status_update', {
                    'type': 'status',
                    'payload': {
                        'simulation_status': 'running',
                        'simulation_running': True,
                        'current_time': self.state.current_time,
                        'mission_progress': self.state.mission_progress,
                        'current_waypoint': self.state.current_waypoint,
                        'mission_status': latest_telemetry.get('mission_status', 'executing'),
                        'total_distance': latest_telemetry.get('total_distance', 0.0),
                        'environment': env_data
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
            use_reloader=False,
            log_output=True,
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