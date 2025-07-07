"""
Database Service for Drone Simulation System

Provides high-level database operations for storing and retrieving
simulation data, telemetry, and performance metrics.
"""

from models import db, SimulationSession, TelemetryData, MissionEvent, PerformanceMetrics
from datetime import datetime
import numpy as np
import json


class DatabaseService:
    """
    Service class for database operations related to drone simulations.
    """
    
    def __init__(self, app=None):
        self.app = app
        self.current_session_id = None
    
    def start_simulation_session(self, drone_model, environment, mission_type, total_waypoints):
        """
        Create a new simulation session in the database.
        
        Args:
            drone_model: Name of drone model
            environment: Environment name
            mission_type: Type of mission
            total_waypoints: Total number of waypoints in mission
            
        Returns:
            SimulationSession: The created session object
        """
        try:
            session = SimulationSession(
                drone_model=drone_model,
                environment=environment,
                mission_type=mission_type,
                total_waypoints=total_waypoints,
                status='running'
            )
            
            db.session.add(session)
            db.session.commit()
            
            self.current_session_id = session.id
            print(f"Started new simulation session: {session.id}")
            
            # Log mission start event
            self.log_mission_event(
                session.id,
                0.0,  # timestamp
                'mission_start',
                description=f"Started {mission_type} mission with {drone_model} in {environment}",
                position=(0, 0, 0),
                mission_progress=0.0
            )
            
            return session
            
        except Exception as e:
            print(f"Error starting simulation session: {e}")
            db.session.rollback()
            return None
    
    def end_simulation_session(self, session_id, final_progress=100.0, status='completed'):
        """
        End a simulation session and calculate final metrics.
        
        Args:
            session_id: Session ID to end
            final_progress: Final mission progress percentage
            status: Final status (completed, failed, stopped)
        """
        try:
            session = SimulationSession.query.get(session_id)
            if not session:
                print(f"Session {session_id} not found")
                return False
            
            session.end_time = datetime.utcnow()
            session.status = status
            session.mission_progress = final_progress
            
            # Calculate duration
            if session.start_time:
                duration = (session.end_time - session.start_time).total_seconds()
                session.duration = duration
            
            # Calculate performance metrics from telemetry data
            self._calculate_session_metrics(session)
            
            db.session.commit()
            
            # Log mission completion event
            self.log_mission_event(
                session_id,
                duration if 'duration' in locals() else 0,
                'mission_complete',
                description=f"Mission {status} with {final_progress:.1f}% progress",
                position=(0, 0, 0),  # Will be updated with actual final position
                mission_progress=final_progress
            )
            
            print(f"Ended simulation session {session_id} with status: {status}")
            
            if session_id == self.current_session_id:
                self.current_session_id = None
                
            return True
            
        except Exception as e:
            print(f"Error ending simulation session: {e}")
            db.session.rollback()
            return False
    
    def log_telemetry_data(self, session_id, timestamp, telemetry):
        """
        Store telemetry data point in database.
        
        Args:
            session_id: Session ID
            timestamp: Simulation timestamp
            telemetry: Dictionary containing telemetry data
        """
        try:
            # Extract data from telemetry dictionary
            position = telemetry.get('position', {})
            velocity = telemetry.get('velocity', {})
            attitude = telemetry.get('attitude', {})
            angular_velocity = telemetry.get('angular_velocity', {})
            
            # Calculate power consumption (basic model)
            speed = telemetry.get('ground_speed', 0)
            altitude = telemetry.get('altitude', 0)
            power = 50 + (speed * 2) + (altitude * 0.1)  # Watts
            
            # Calculate cumulative energy (simplified)
            energy = telemetry.get('energy_consumed', power * timestamp / 3600)  # Wh
            
            telemetry_point = TelemetryData(
                session_id=session_id,
                timestamp=timestamp,
                position_x=position.get('x', 0),
                position_y=position.get('y', 0),
                position_z=position.get('z', 0),
                velocity_x=velocity.get('x', 0),
                velocity_y=velocity.get('y', 0),
                velocity_z=velocity.get('z', 0),
                ground_speed=telemetry.get('ground_speed', 0),
                vertical_speed=telemetry.get('vertical_speed', 0),
                roll=attitude.get('roll', 0),
                pitch=attitude.get('pitch', 0),
                yaw=attitude.get('yaw', 0),
                angular_velocity_x=angular_velocity.get('x', 0),
                angular_velocity_y=angular_velocity.get('y', 0),
                angular_velocity_z=angular_velocity.get('z', 0),
                altitude=telemetry.get('altitude', 0),
                mission_progress=telemetry.get('mission_progress', 0),
                current_waypoint=telemetry.get('current_waypoint', 0),
                power_consumption=power,
                energy_consumed=energy
            )
            
            db.session.add(telemetry_point)
            
            # Update session metrics
            self._update_session_realtime_metrics(session_id, telemetry)
            
            # Commit every 10 data points to improve performance
            if int(timestamp * 10) % 10 == 0:
                db.session.commit()
                
        except Exception as e:
            print(f"Error logging telemetry data: {e}")
            db.session.rollback()
    
    def log_mission_event(self, session_id, timestamp, event_type, waypoint_index=None, 
                         action=None, position=(0, 0, 0), mission_progress=0.0, 
                         description=None, parameters=None):
        """
        Log a mission event.
        
        Args:
            session_id: Session ID
            timestamp: Simulation timestamp
            event_type: Type of event
            waypoint_index: Waypoint index if applicable
            action: Action being performed
            position: Current position tuple (x, y, z)
            mission_progress: Current mission progress
            description: Event description
            parameters: Additional parameters as dictionary
        """
        try:
            event = MissionEvent(
                session_id=session_id,
                timestamp=timestamp,
                event_type=event_type,
                waypoint_index=waypoint_index,
                action=action,
                position_x=position[0],
                position_y=position[1],
                position_z=position[2],
                mission_progress=mission_progress,
                description=description
            )
            
            if parameters:
                event.set_parameters(parameters)
            
            db.session.add(event)
            db.session.commit()
            
        except Exception as e:
            print(f"Error logging mission event: {e}")
            db.session.rollback()
    
    def get_session_history(self, limit=50):
        """
        Get recent simulation sessions.
        
        Args:
            limit: Maximum number of sessions to return
            
        Returns:
            List of session dictionaries
        """
        try:
            sessions = SimulationSession.query.order_by(
                SimulationSession.start_time.desc()
            ).limit(limit).all()
            
            return [session.to_dict() for session in sessions]
            
        except Exception as e:
            print(f"Error getting session history: {e}")
            return []
    
    def get_session_telemetry(self, session_id, limit=None):
        """
        Get telemetry data for a specific session.
        
        Args:
            session_id: Session ID
            limit: Optional limit on number of points
            
        Returns:
            List of telemetry dictionaries
        """
        try:
            query = TelemetryData.query.filter_by(session_id=session_id).order_by(TelemetryData.timestamp)
            
            if limit:
                query = query.limit(limit)
            
            telemetry_points = query.all()
            return [point.to_dict() for point in telemetry_points]
            
        except Exception as e:
            print(f"Error getting session telemetry: {e}")
            return []
    
    def get_session_events(self, session_id):
        """
        Get mission events for a specific session.
        
        Args:
            session_id: Session ID
            
        Returns:
            List of event dictionaries
        """
        try:
            events = MissionEvent.query.filter_by(session_id=session_id).order_by(MissionEvent.timestamp).all()
            return [event.to_dict() for event in events]
            
        except Exception as e:
            print(f"Error getting session events: {e}")
            return []
    
    def get_performance_analytics(self, drone_model=None, environment=None, mission_type=None, limit=20):
        """
        Get performance analytics across multiple sessions.
        
        Args:
            drone_model: Filter by drone model
            environment: Filter by environment
            mission_type: Filter by mission type
            limit: Maximum sessions to analyze
            
        Returns:
            Dictionary with analytics data
        """
        try:
            query = SimulationSession.query
            
            if drone_model:
                query = query.filter_by(drone_model=drone_model)
            if environment:
                query = query.filter_by(environment=environment)
            if mission_type:
                query = query.filter_by(mission_type=mission_type)
            
            sessions = query.order_by(SimulationSession.start_time.desc()).limit(limit).all()
            
            if not sessions:
                return {}
            
            # Calculate aggregate statistics
            total_sessions = len(sessions)
            completed_sessions = len([s for s in sessions if s.status == 'completed'])
            success_rate = (completed_sessions / total_sessions) * 100 if total_sessions > 0 else 0
            
            avg_duration = np.mean([s.duration for s in sessions if s.duration])
            avg_distance = np.mean([s.total_distance for s in sessions if s.total_distance])
            avg_energy = np.mean([s.total_energy for s in sessions if s.total_energy])
            avg_progress = np.mean([s.mission_progress for s in sessions])
            
            return {
                'total_sessions': total_sessions,
                'success_rate': success_rate,
                'completed_sessions': completed_sessions,
                'average_metrics': {
                    'duration': float(avg_duration) if not np.isnan(avg_duration) else 0,
                    'distance': float(avg_distance) if not np.isnan(avg_distance) else 0,
                    'energy': float(avg_energy) if not np.isnan(avg_energy) else 0,
                    'progress': float(avg_progress) if not np.isnan(avg_progress) else 0
                },
                'recent_sessions': [s.to_dict() for s in sessions[:10]]
            }
            
        except Exception as e:
            print(f"Error getting performance analytics: {e}")
            return {}
    
    def _update_session_realtime_metrics(self, session_id, telemetry):
        """
        Update session metrics in real-time as telemetry comes in.
        """
        try:
            session = SimulationSession.query.get(session_id)
            if not session:
                return
            
            # Update max values
            altitude = telemetry.get('altitude', 0)
            speed = telemetry.get('ground_speed', 0)
            progress = telemetry.get('mission_progress', 0)
            waypoint = telemetry.get('current_waypoint', 0)
            
            if altitude > session.max_altitude:
                session.max_altitude = altitude
            
            if speed > session.max_speed:
                session.max_speed = speed
            
            session.mission_progress = progress
            session.waypoints_completed = waypoint
            
            # Calculate energy (simplified)
            power = 50 + (speed * 2) + (altitude * 0.1)
            session.total_energy = telemetry.get('energy_consumed', power * telemetry.get('timestamp', 0) / 3600)
            
        except Exception as e:
            print(f"Error updating session metrics: {e}")
    
    def _calculate_session_metrics(self, session):
        """
        Calculate final performance metrics for a completed session.
        """
        try:
            # Get all telemetry for this session
            telemetry_points = TelemetryData.query.filter_by(session_id=session.id).all()
            
            if not telemetry_points:
                print(f"No telemetry data found for session {session.id}")
                return
            
            # Calculate total distance
            total_distance = 0
            for i in range(1, len(telemetry_points)):
                prev = telemetry_points[i-1]
                curr = telemetry_points[i]
                
                dx = curr.position_x - prev.position_x
                dy = curr.position_y - prev.position_y
                dz = curr.position_z - prev.position_z
                
                total_distance += np.sqrt(dx*dx + dy*dy + dz*dz)
            
            session.total_distance = float(total_distance)
            
            # Update final energy
            if telemetry_points:
                session.total_energy = telemetry_points[-1].energy_consumed
            
            # Create performance metrics record
            if len(telemetry_points) > 1:
                self._create_performance_metrics(session, telemetry_points)
                
        except Exception as e:
            print(f"Error calculating session metrics: {e}")
    
    def _create_performance_metrics(self, session, telemetry_points):
        """
        Create detailed performance metrics for analysis.
        """
        try:
            altitudes = [p.altitude for p in telemetry_points]
            speeds = [p.ground_speed for p in telemetry_points]
            
            metrics = PerformanceMetrics(
                session_id=session.id,
                min_altitude=min(altitudes),
                max_altitude=max(altitudes),
                avg_altitude=float(np.mean(altitudes)),
                min_speed=min(speeds),
                max_speed=max(speeds),
                avg_speed=float(np.mean(speeds)),
                energy_efficiency=session.total_energy / max(session.total_distance, 1) * 1000,  # Wh/km
                time_efficiency=100.0,  # TODO: Calculate based on planned vs actual time
                path_efficiency=100.0,  # TODO: Calculate based on optimal vs actual path
                attitude_variance=float(np.var([p.roll for p in telemetry_points])),
                speed_variance=float(np.var(speeds)),
                mission_success_score=session.mission_progress,
                waypoint_accuracy=5.0  # TODO: Calculate actual waypoint accuracy
            )
            
            db.session.add(metrics)
            
        except Exception as e:
            print(f"Error creating performance metrics: {e}")
    
    def cleanup_old_sessions(self, days_to_keep=30):
        """
        Clean up old simulation data to manage database size.
        
        Args:
            days_to_keep: Number of days of data to retain
        """
        try:
            cutoff_date = datetime.utcnow() - timedelta(days=days_to_keep)
            
            old_sessions = SimulationSession.query.filter(
                SimulationSession.start_time < cutoff_date
            ).all()
            
            for session in old_sessions:
                db.session.delete(session)
            
            db.session.commit()
            print(f"Cleaned up {len(old_sessions)} old sessions")
            
        except Exception as e:
            print(f"Error cleaning up old sessions: {e}")
            db.session.rollback()