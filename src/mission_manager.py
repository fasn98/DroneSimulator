"""
Mission Manager Module

Handles mission planning, execution, and monitoring for different
types of drone missions (reconnaissance, sample transport, monitoring).
"""

import json
import numpy as np
import math
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum


class MissionStatus(Enum):
    """Mission status enumeration."""
    PLANNING = "planning"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"
    ABORTED = "aborted"


class WaypointAction(Enum):
    """Waypoint action enumeration."""
    TAKEOFF = "takeoff"
    LAND = "land"
    NAVIGATE = "navigate"
    PHOTO = "photo"
    PICKUP = "pickup"
    DELIVERY = "delivery"
    MONITOR = "monitor"
    ASCEND = "ascend"
    DESCEND = "descend"
    HOVER = "hover"


@dataclass
class Waypoint:
    """Waypoint data class."""
    position: np.ndarray
    action: WaypointAction
    parameters: Dict[str, Any]
    tolerance: float = 1.0
    duration: float = 0.0  # Duration in seconds for time-based actions
    completed: bool = False
    start_time: Optional[float] = None  # When the waypoint action was started
    arrived_at_position: bool = False  # Whether we've reached the waypoint position


@dataclass
class MissionObjective:
    """Mission objective data class."""
    name: str
    description: str
    target_value: float
    current_value: float = 0.0
    completed: bool = False
    weight: float = 1.0


class MissionManager:
    """
    Mission manager class for planning and executing drone missions.
    
    Supports:
    - Mission planning and waypoint generation
    - Mission execution monitoring
    - Objective tracking and completion
    - Performance metrics calculation
    """
    
    def __init__(self, mission_type: str, config_path: str = "config/missions.json"):
        """
        Initialize mission manager.
        
        Args:
            mission_type: Type of mission (reconnaissance, sample_transport, monitoring)
            config_path: Path to mission configuration file
        """
        self.mission_type = mission_type
        self.config_path = config_path
        self.status = MissionStatus.PLANNING
        
        # Mission parameters
        self.waypoints: List[Waypoint] = []
        self.objectives: List[MissionObjective] = []
        self.current_waypoint_index = 0
        self.success_criteria = {}
        self.constraints = {}
        
        # Mission tracking
        self.start_time = 0.0
        self.completion_time = 0.0
        self.distance_traveled = 0.0
        self.area_covered = 0.0
        self.samples_collected = 0
        self.photos_taken = 0
        self.monitoring_points = []
        
        # Previous position for distance calculation
        self.previous_position = np.zeros(3)
        
        # Load mission configuration
        self._load_configuration()
        self._create_waypoints()
        self._create_objectives()
    
    def _load_configuration(self):
        """Load mission configuration from JSON file."""
        try:
            with open(self.config_path, 'r') as f:
                configs = json.load(f)
            
            if self.mission_type not in configs:
                raise ValueError(f"Mission type '{self.mission_type}' not found in configuration")
            
            config = configs[self.mission_type]
            
            # Load mission parameters
            self.mission_config = config
            self.success_criteria = config.get('success_criteria', {})
            self.constraints = config.get('constraints', {})
            
        except Exception as e:
            raise RuntimeError(f"Error loading mission configuration: {e}")
    
    def _create_waypoints(self):
        """Create waypoints from mission configuration."""
        waypoint_configs = self.mission_config.get('waypoints', [])
        
        for wp_config in waypoint_configs:
            position = np.array([wp_config['x'], wp_config['y'], wp_config['z']])
            action = WaypointAction(wp_config['action'])
            parameters = wp_config.get('parameters', {})
            tolerance = wp_config.get('tolerance', 1.0)
            duration = wp_config.get('duration', 0.0)  # Add duration from config
            
            waypoint = Waypoint(
                position=position,
                action=action,
                parameters=parameters,
                tolerance=tolerance,
                duration=duration
            )
            
            self.waypoints.append(waypoint)
    
    def _create_objectives(self):
        """Create mission objectives from configuration."""
        objectives_config = self.mission_config.get('objectives', {})
        
        if self.mission_type == 'reconnaissance':
            # Area coverage objective
            target_area = objectives_config.get('area_coverage', 10000.0)
            self.objectives.append(MissionObjective(
                name="area_coverage",
                description="Cover specified area",
                target_value=target_area,
                weight=1.0
            ))
            
            # Photo objective
            min_photos = self.success_criteria.get('photos_taken_min', 10)
            self.objectives.append(MissionObjective(
                name="photos_taken",
                description="Take minimum number of photos",
                target_value=min_photos,
                weight=0.5
            ))
            
        elif self.mission_type == 'sample_transport':
            # Sample collection objective
            target_samples = len(objectives_config.get('pickup_locations', []))
            self.objectives.append(MissionObjective(
                name="samples_collected",
                description="Collect all samples",
                target_value=target_samples,
                weight=1.0
            ))
            
            # Delivery precision objective
            precision_req = objectives_config.get('precision_required', 0.5)
            self.objectives.append(MissionObjective(
                name="delivery_precision",
                description="Deliver samples with required precision",
                target_value=precision_req,
                weight=0.8
            ))
            
        elif self.mission_type == 'monitoring':
            # Monitoring time objective
            target_time = objectives_config.get('endurance_target', 3600.0)
            self.objectives.append(MissionObjective(
                name="monitoring_time",
                description="Monitor for specified duration",
                target_value=target_time,
                weight=1.0
            ))
            
            # Data collection objective
            target_points = self.success_criteria.get('data_points_min', 100)
            self.objectives.append(MissionObjective(
                name="data_points",
                description="Collect minimum data points",
                target_value=target_points,
                weight=0.7
            ))
    
    def start_mission(self, start_time: float):
        """
        Start mission execution.
        
        Args:
            start_time: Mission start time in seconds
        """
        self.start_time = start_time
        self.status = MissionStatus.EXECUTING
        self.current_waypoint_index = 0
        
        # Reset tracking variables
        self.distance_traveled = 0.0
        self.area_covered = 0.0
        self.samples_collected = 0
        self.photos_taken = 0
        self.monitoring_points = []
    
    def update_mission(self, current_time: float, drone_position: np.ndarray, 
                      drone_state: Dict[str, Any]) -> Dict[str, Any]:
        """
        Update mission progress and check waypoint completion.
        
        Args:
            current_time: Current simulation time
            drone_position: Current drone position
            drone_state: Current drone state dictionary
            
        Returns:
            Dictionary with mission update information
        """
        if self.status != MissionStatus.EXECUTING:
            return {"status": self.status.value, "action": "none"}
        
        # Update distance traveled
        if np.linalg.norm(self.previous_position) > 0:
            self.distance_traveled += np.linalg.norm(drone_position - self.previous_position)
        self.previous_position = drone_position.copy()
        
        # Check current waypoint completion
        current_waypoint = self.get_current_waypoint()
        if current_waypoint and not current_waypoint.completed:
            distance_to_waypoint = np.linalg.norm(drone_position - current_waypoint.position)
            
            # Check if we've reached the waypoint position
            if distance_to_waypoint <= current_waypoint.tolerance and not current_waypoint.arrived_at_position:
                current_waypoint.arrived_at_position = True
                current_waypoint.start_time = current_time
                
                # Execute action immediately for instant actions
                action_result = self._execute_waypoint_action(current_waypoint, current_time, drone_position)
                
                # For actions without duration, complete immediately
                if current_waypoint.duration <= 0:
                    current_waypoint.completed = True
                    self.current_waypoint_index += 1
                    
                    return {
                        "status": self.status.value,
                        "action": current_waypoint.action.value,
                        "waypoint_completed": True,
                        "action_result": action_result
                    }
            
            # Check if duration-based waypoint is complete
            elif current_waypoint.arrived_at_position and current_waypoint.duration > 0:
                elapsed_time = current_time - current_waypoint.start_time
                if elapsed_time >= current_waypoint.duration:
                    current_waypoint.completed = True
                    self.current_waypoint_index += 1
                    
                    return {
                        "status": self.status.value,
                        "action": current_waypoint.action.value,
                        "waypoint_completed": True,
                        "action_result": {"success": True, "message": f"Duration-based action completed after {elapsed_time:.1f}s"}
                    }
        
        # Update objectives
        self._update_objectives(current_time, drone_position)
        
        # Check mission completion
        if self.current_waypoint_index >= len(self.waypoints):
            self._complete_mission(current_time)
        
        # Check mission failure conditions
        self._check_failure_conditions(current_time, drone_state)
        
        return {
            "status": self.status.value,
            "action": "continue",
            "waypoint_completed": False,
            "progress": self.get_mission_progress()
        }
    
    def _execute_waypoint_action(self, waypoint: Waypoint, current_time: float, 
                               position: np.ndarray) -> Dict[str, Any]:
        """
        Execute waypoint action.
        
        Args:
            waypoint: Waypoint to execute
            current_time: Current time
            position: Current position
            
        Returns:
            Action execution result
        """
        action_result = {"success": True, "message": ""}
        
        if waypoint.action == WaypointAction.PHOTO:
            self.photos_taken += 1
            action_result["message"] = f"Photo taken at {position}"
            
        elif waypoint.action == WaypointAction.PICKUP:
            self.samples_collected += 1
            sample_mass = waypoint.parameters.get('sample_mass', 1.0)
            action_result["message"] = f"Sample collected: {sample_mass} kg"
            
        elif waypoint.action == WaypointAction.DELIVERY:
            action_result["message"] = "Samples delivered"
            
        elif waypoint.action == WaypointAction.MONITOR:
            # Record monitoring point
            self.monitoring_points.append({
                'time': current_time,
                'position': position.copy(),
                'data': waypoint.parameters
            })
            action_result["message"] = f"Monitoring point recorded"
            
        elif waypoint.action == WaypointAction.TAKEOFF:
            action_result["message"] = "Takeoff initiated"
            
        elif waypoint.action == WaypointAction.LAND:
            action_result["message"] = "Landing initiated"
            
        else:
            action_result["message"] = f"Action {waypoint.action.value} executed"
        
        return action_result
    
    def _update_objectives(self, current_time: float, position: np.ndarray):
        """Update mission objectives based on current progress."""
        for objective in self.objectives:
            if objective.name == "area_coverage":
                # Update area coverage (simplified calculation)
                objective.current_value = self._calculate_area_coverage(position)
                
            elif objective.name == "photos_taken":
                objective.current_value = self.photos_taken
                
            elif objective.name == "samples_collected":
                objective.current_value = self.samples_collected
                
            elif objective.name == "delivery_precision":
                # Calculate delivery precision if applicable
                if self.mission_type == 'sample_transport':
                    objective.current_value = self._calculate_delivery_precision()
                    
            elif objective.name == "monitoring_time":
                objective.current_value = current_time - self.start_time
                
            elif objective.name == "data_points":
                objective.current_value = len(self.monitoring_points)
            
            # Check if objective is completed
            if objective.current_value >= objective.target_value:
                objective.completed = True
    
    def _calculate_area_coverage(self, position: np.ndarray) -> float:
        """Calculate area coverage based on flight path."""
        # Simplified area calculation
        # In reality, this would consider sensor field of view and overlap
        sensor_range = 50.0  # meters
        coverage_radius = sensor_range / 2.0
        
        # Approximate coverage as sum of circles at each position
        coverage_area = len(self.monitoring_points) * math.pi * coverage_radius**2
        
        return coverage_area
    
    def _calculate_delivery_precision(self) -> float:
        """Calculate delivery precision for sample transport missions."""
        if not self.waypoints:
            return 0.0
        
        # Find delivery waypoint
        delivery_waypoint = None
        for wp in self.waypoints:
            if wp.action == WaypointAction.DELIVERY:
                delivery_waypoint = wp
                break
        
        if not delivery_waypoint:
            return 0.0
        
        # Calculate distance from target (simplified)
        # In reality, this would track actual delivery position
        return 0.1  # Placeholder value
    
    def _complete_mission(self, current_time: float):
        """Complete the mission and calculate final metrics."""
        self.completion_time = current_time
        self.status = MissionStatus.COMPLETED
        
        # Check if all critical objectives are met
        critical_objectives_met = all(
            obj.completed for obj in self.objectives if obj.weight >= 1.0
        )
        
        if not critical_objectives_met:
            self.status = MissionStatus.FAILED
    
    def _check_failure_conditions(self, current_time: float, drone_state: Dict[str, Any]):
        """Check for mission failure conditions."""
        mission_time = current_time - self.start_time
        
        # Check time limits
        max_time = self.success_criteria.get('flight_time_max', float('inf'))
        if mission_time > max_time:
            self.status = MissionStatus.FAILED
            return
        
        # Check fuel/battery levels
        if not drone_state.get('operational', True):
            self.status = MissionStatus.FAILED
            return
        
        # Check altitude constraints
        altitude = drone_state.get('position', [0, 0, 0])[2]
        max_altitude = self.constraints.get('max_altitude', float('inf'))
        min_altitude = self.constraints.get('min_altitude', 0.0)
        
        if altitude > max_altitude or altitude < min_altitude:
            self.status = MissionStatus.FAILED
            return
    
    def get_current_waypoint(self) -> Optional[Waypoint]:
        """Get the current waypoint."""
        if 0 <= self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        return None
    
    def get_next_waypoint(self) -> Optional[Waypoint]:
        """Get the next waypoint."""
        next_index = self.current_waypoint_index + 1
        if 0 <= next_index < len(self.waypoints):
            return self.waypoints[next_index]
        return None
    
    def get_mission_progress(self) -> float:
        """Get mission progress as percentage."""
        if not self.waypoints:
            return 0.0
        
        completed_waypoints = sum(1 for wp in self.waypoints if wp.completed)
        return (completed_waypoints / len(self.waypoints)) * 100.0
    
    def get_navigation_command(self, current_position: np.ndarray) -> Dict[str, Any]:
        """
        Get navigation command for current waypoint.
        
        Args:
            current_position: Current drone position
            
        Returns:
            Navigation command dictionary
        """
        current_waypoint = self.get_current_waypoint()
        if not current_waypoint:
            return {"command": "hover", "target": current_position}
        
        # Calculate direction to waypoint
        direction = current_waypoint.position - current_position
        distance = np.linalg.norm(direction)
        
        if distance > 0:
            direction_unit = direction / distance
        else:
            direction_unit = np.zeros(3)
        
        # Determine appropriate speed based on distance
        max_speed = self.constraints.get('max_speed', 20.0)
        if distance < 10.0:
            speed = max_speed * 0.3  # Slow down when approaching
        else:
            speed = max_speed
        
        return {
            "command": "navigate",
            "target": current_waypoint.position,
            "direction": direction_unit,
            "distance": distance,
            "speed": speed,
            "action": current_waypoint.action.value
        }
    
    def get_mission_metrics(self) -> Dict[str, Any]:
        """Get mission performance metrics."""
        total_time = self.completion_time - self.start_time if self.completion_time > 0 else 0.0
        
        metrics = {
            "mission_type": self.mission_type,
            "status": self.status.value,
            "total_time": total_time,
            "distance_traveled": self.distance_traveled,
            "area_covered": self.area_covered,
            "samples_collected": self.samples_collected,
            "photos_taken": self.photos_taken,
            "monitoring_points": len(self.monitoring_points),
            "waypoints_completed": sum(1 for wp in self.waypoints if wp.completed),
            "total_waypoints": len(self.waypoints),
            "progress_percentage": self.get_mission_progress(),
            "objectives_status": [
                {
                    "name": obj.name,
                    "completed": obj.completed,
                    "progress": min(obj.current_value / obj.target_value, 1.0) * 100.0,
                    "target": obj.target_value,
                    "current": obj.current_value
                }
                for obj in self.objectives
            ]
        }
        
        return metrics
    
    def abort_mission(self, reason: str = "User requested"):
        """Abort the mission."""
        self.status = MissionStatus.ABORTED
        return {"status": "aborted", "reason": reason}
    
    def get_status_dict(self) -> Dict[str, Any]:
        """Get current mission status as dictionary."""
        return {
            "mission_type": self.mission_type,
            "status": self.status.value,
            "current_waypoint": self.current_waypoint_index,
            "total_waypoints": len(self.waypoints),
            "progress": self.get_mission_progress(),
            "objectives_completed": sum(1 for obj in self.objectives if obj.completed),
            "total_objectives": len(self.objectives),
            "distance_traveled": self.distance_traveled,
            "area_covered": self.area_covered,
            "samples_collected": self.samples_collected,
            "photos_taken": self.photos_taken,
            "monitoring_points": len(self.monitoring_points)
        }
