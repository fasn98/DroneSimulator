"""
Database Models for Drone Simulation System

Stores simulation sessions, telemetry data, mission logs, and performance metrics
for comprehensive historical analysis and smart insights.
"""

import os
from datetime import datetime
from flask import Flask
from flask_sqlalchemy import SQLAlchemy
from sqlalchemy.orm import DeclarativeBase
import json


class Base(DeclarativeBase):
    pass


db = SQLAlchemy(model_class=Base)


class SimulationSession(db.Model):
    """
    Main simulation session record containing overall mission information.
    """
    __tablename__ = 'simulation_sessions'
    
    id = db.Column(db.Integer, primary_key=True)
    start_time = db.Column(db.DateTime, default=datetime.utcnow, nullable=False)
    end_time = db.Column(db.DateTime, nullable=True)
    duration = db.Column(db.Float, nullable=True)  # seconds
    
    # Configuration
    drone_model = db.Column(db.String(100), nullable=False)
    environment = db.Column(db.String(100), nullable=False)
    mission_type = db.Column(db.String(100), nullable=False)
    
    # Mission Results
    status = db.Column(db.String(50), default='running')  # running, completed, failed, stopped
    mission_progress = db.Column(db.Float, default=0.0)  # 0-100%
    waypoints_completed = db.Column(db.Integer, default=0)
    total_waypoints = db.Column(db.Integer, nullable=False)
    
    # Performance Metrics
    total_distance = db.Column(db.Float, default=0.0)  # meters
    max_altitude = db.Column(db.Float, default=0.0)  # meters
    max_speed = db.Column(db.Float, default=0.0)  # m/s
    total_energy = db.Column(db.Float, default=0.0)  # Wh
    
    # Relationships
    telemetry_points = db.relationship('TelemetryData', backref='session', lazy='dynamic', cascade='all, delete-orphan')
    mission_events = db.relationship('MissionEvent', backref='session', lazy='dynamic', cascade='all, delete-orphan')
    
    def to_dict(self):
        return {
            'id': self.id,
            'start_time': self.start_time.isoformat() if self.start_time else None,
            'end_time': self.end_time.isoformat() if self.end_time else None,
            'duration': self.duration,
            'drone_model': self.drone_model,
            'environment': self.environment,
            'mission_type': self.mission_type,
            'status': self.status,
            'mission_progress': self.mission_progress,
            'waypoints_completed': self.waypoints_completed,
            'total_waypoints': self.total_waypoints,
            'total_distance': self.total_distance,
            'max_altitude': self.max_altitude,
            'max_speed': self.max_speed,
            'total_energy': self.total_energy
        }


class TelemetryData(db.Model):
    """
    Real-time telemetry data points during simulation.
    High-frequency data for detailed analysis and plotting.
    """
    __tablename__ = 'telemetry_data'
    
    id = db.Column(db.Integer, primary_key=True)
    session_id = db.Column(db.Integer, db.ForeignKey('simulation_sessions.id'), nullable=False)
    timestamp = db.Column(db.Float, nullable=False)  # simulation time in seconds
    recorded_at = db.Column(db.DateTime, default=datetime.utcnow, nullable=False)
    
    # Position
    position_x = db.Column(db.Float, nullable=False)
    position_y = db.Column(db.Float, nullable=False)
    position_z = db.Column(db.Float, nullable=False)
    
    # Velocity
    velocity_x = db.Column(db.Float, nullable=False)
    velocity_y = db.Column(db.Float, nullable=False)
    velocity_z = db.Column(db.Float, nullable=False)
    ground_speed = db.Column(db.Float, nullable=False)
    vertical_speed = db.Column(db.Float, nullable=False)
    
    # Attitude (in radians)
    roll = db.Column(db.Float, nullable=False)
    pitch = db.Column(db.Float, nullable=False)
    yaw = db.Column(db.Float, nullable=False)
    
    # Angular velocity
    angular_velocity_x = db.Column(db.Float, nullable=False)
    angular_velocity_y = db.Column(db.Float, nullable=False)
    angular_velocity_z = db.Column(db.Float, nullable=False)
    
    # Derived metrics
    altitude = db.Column(db.Float, nullable=False)
    mission_progress = db.Column(db.Float, nullable=False)
    current_waypoint = db.Column(db.Integer, nullable=False)
    
    # Power and energy
    power_consumption = db.Column(db.Float, nullable=False)  # Watts
    energy_consumed = db.Column(db.Float, nullable=False)    # Wh
    
    def to_dict(self):
        return {
            'id': self.id,
            'session_id': self.session_id,
            'timestamp': self.timestamp,
            'position': {
                'x': self.position_x,
                'y': self.position_y,
                'z': self.position_z
            },
            'velocity': {
                'x': self.velocity_x,
                'y': self.velocity_y,
                'z': self.velocity_z
            },
            'attitude': {
                'roll': self.roll,
                'pitch': self.pitch,
                'yaw': self.yaw
            },
            'angular_velocity': {
                'x': self.angular_velocity_x,
                'y': self.angular_velocity_y,
                'z': self.angular_velocity_z
            },
            'ground_speed': self.ground_speed,
            'vertical_speed': self.vertical_speed,
            'altitude': self.altitude,
            'mission_progress': self.mission_progress,
            'current_waypoint': self.current_waypoint,
            'power_consumption': self.power_consumption,
            'energy_consumed': self.energy_consumed
        }


class MissionEvent(db.Model):
    """
    Discrete mission events and waypoint achievements.
    """
    __tablename__ = 'mission_events'
    
    id = db.Column(db.Integer, primary_key=True)
    session_id = db.Column(db.Integer, db.ForeignKey('simulation_sessions.id'), nullable=False)
    timestamp = db.Column(db.Float, nullable=False)  # simulation time
    recorded_at = db.Column(db.DateTime, default=datetime.utcnow, nullable=False)
    
    event_type = db.Column(db.String(50), nullable=False)  # waypoint_reached, mission_start, mission_complete, etc.
    waypoint_index = db.Column(db.Integer, nullable=True)
    action = db.Column(db.String(50), nullable=True)  # takeoff, land, navigate, photo, etc.
    
    # Position when event occurred
    position_x = db.Column(db.Float, nullable=False)
    position_y = db.Column(db.Float, nullable=False)
    position_z = db.Column(db.Float, nullable=False)
    
    mission_progress = db.Column(db.Float, nullable=False)
    description = db.Column(db.Text, nullable=True)
    
    # Additional parameters as JSON
    parameters = db.Column(db.Text, nullable=True)  # JSON string for flexible data
    
    def get_parameters(self):
        if self.parameters:
            return json.loads(self.parameters)
        return {}
    
    def set_parameters(self, params_dict):
        self.parameters = json.dumps(params_dict) if params_dict else None
    
    def to_dict(self):
        return {
            'id': self.id,
            'session_id': self.session_id,
            'timestamp': self.timestamp,
            'event_type': self.event_type,
            'waypoint_index': self.waypoint_index,
            'action': self.action,
            'position': {
                'x': self.position_x,
                'y': self.position_y,
                'z': self.position_z
            },
            'mission_progress': self.mission_progress,
            'description': self.description,
            'parameters': self.get_parameters()
        }


class PerformanceMetrics(db.Model):
    """
    Aggregated performance metrics for analysis and comparison.
    """
    __tablename__ = 'performance_metrics'
    
    id = db.Column(db.Integer, primary_key=True)
    session_id = db.Column(db.Integer, db.ForeignKey('simulation_sessions.id'), nullable=False)
    calculated_at = db.Column(db.DateTime, default=datetime.utcnow, nullable=False)
    
    # Flight envelope
    min_altitude = db.Column(db.Float, nullable=False)
    max_altitude = db.Column(db.Float, nullable=False)
    avg_altitude = db.Column(db.Float, nullable=False)
    
    min_speed = db.Column(db.Float, nullable=False)
    max_speed = db.Column(db.Float, nullable=False)
    avg_speed = db.Column(db.Float, nullable=False)
    
    # Efficiency metrics
    energy_efficiency = db.Column(db.Float, nullable=False)  # Wh/km
    time_efficiency = db.Column(db.Float, nullable=False)    # actual_time/planned_time
    path_efficiency = db.Column(db.Float, nullable=False)    # direct_distance/actual_distance
    
    # Stability metrics
    attitude_variance = db.Column(db.Float, nullable=False)  # variance in attitude changes
    speed_variance = db.Column(db.Float, nullable=False)     # variance in speed changes
    
    # Mission-specific metrics
    mission_success_score = db.Column(db.Float, nullable=False)  # 0-100
    waypoint_accuracy = db.Column(db.Float, nullable=False)      # average distance error
    
    session = db.relationship('SimulationSession', backref='performance_metrics')
    
    def to_dict(self):
        return {
            'id': self.id,
            'session_id': self.session_id,
            'calculated_at': self.calculated_at.isoformat(),
            'flight_envelope': {
                'min_altitude': self.min_altitude,
                'max_altitude': self.max_altitude,
                'avg_altitude': self.avg_altitude,
                'min_speed': self.min_speed,
                'max_speed': self.max_speed,
                'avg_speed': self.avg_speed
            },
            'efficiency': {
                'energy_efficiency': self.energy_efficiency,
                'time_efficiency': self.time_efficiency,
                'path_efficiency': self.path_efficiency
            },
            'stability': {
                'attitude_variance': self.attitude_variance,
                'speed_variance': self.speed_variance
            },
            'mission_metrics': {
                'success_score': self.mission_success_score,
                'waypoint_accuracy': self.waypoint_accuracy
            }
        }


def create_app():
    """Create Flask app with database configuration."""
    app = Flask(__name__)
    
    # Database configuration with fallback
    database_url = os.environ.get('DATABASE_URL')
    if not database_url:
        # Fallback to environment variables
        host = os.environ.get('PGHOST', 'localhost')
        port = os.environ.get('PGPORT', '5432')
        user = os.environ.get('PGUSER', 'postgres')
        password = os.environ.get('PGPASSWORD', '')
        database = os.environ.get('PGDATABASE', 'postgres')
        database_url = f"postgresql://{user}:{password}@{host}:{port}/{database}"
    
    app.config['SQLALCHEMY_DATABASE_URI'] = database_url
    app.config['SQLALCHEMY_ENGINE_OPTIONS'] = {
        'pool_recycle': 300,
        'pool_pre_ping': True,
    }
    app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
    
    # Initialize database
    db.init_app(app)
    
    with app.app_context():
        try:
            db.create_all()
            print("Database tables created successfully!")
        except Exception as e:
            print(f"Error creating database tables: {e}")
    
    return app, db


if __name__ == "__main__":
    # Create tables when run directly
    app, database = create_app()
    print("Database tables created successfully!")