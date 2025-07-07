"""
Drone Model Module

Handles loading, parsing, and representation of drone physical parameters
and specifications from configuration files.
"""

import json
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, Any, Optional


@dataclass
class DroneSpecs:
    """Data class for drone specifications."""
    # Mass properties
    empty_weight: float = 0.0
    mtow: float = 0.0
    fuel_capacity: float = 0.0
    current_mass: float = 0.0
    
    # Dimensions
    wingspan: float = 0.0
    length: float = 0.0
    height: float = 0.0
    rotor_diameter: float = 0.0
    
    # Propulsion
    propulsion_type: str = "electric_rotors"
    motor_count: int = 4
    max_thrust_per_motor: float = 0.0
    power_consumption: float = 0.0
    efficiency: float = 0.85
    battery_capacity: float = 0.0
    specific_impulse: float = 0.0
    
    # Aerodynamics
    drag_coefficient: float = 0.3
    lift_coefficient: float = 0.8
    reference_area: float = 0.2
    
    # Control limits
    max_roll_rate: float = 180.0
    max_pitch_rate: float = 180.0
    max_yaw_rate: float = 90.0
    
    # Flight envelope
    max_altitude: float = 5000.0
    max_speed: float = 30.0
    service_ceiling: float = 10000.0


@dataclass
class DroneState:
    """Data class for drone state variables."""
    # Position (m)
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    # Velocity (m/s)
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    # Acceleration (m/s²)
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    # Attitude (rad) - roll, pitch, yaw
    attitude: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    # Angular velocity (rad/s)
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    # Angular acceleration (rad/s²)
    angular_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    # Thrust commands (N)
    thrust_commands: np.ndarray = field(default_factory=lambda: np.zeros(4))
    
    # Current mass (kg)
    current_mass: float = 0.0
    
    # Fuel/battery level (%)
    fuel_level: float = 1.0
    battery_level: float = 1.0
    
    # Power consumption (W)
    power_consumption: float = 0.0
    
    # Payload mass (kg)
    payload_mass: float = 0.0


class DroneModel:
    """
    Drone model class that manages drone specifications and state.
    
    This class handles:
    - Loading drone configurations from JSON files
    - Managing drone physical parameters
    - Tracking drone state during simulation
    - Calculating mass properties and moments of inertia
    """
    
    def __init__(self, model_name: str, config_path: str = "config/drone_models.json"):
        """
        Initialize drone model.
        
        Args:
            model_name: Name of the drone model to load
            config_path: Path to the drone configuration file
        """
        self.model_name = model_name
        self.config_path = config_path
        self.specs = DroneSpecs()
        self.state = DroneState()
        self.inertia_matrix = np.eye(3)
        
        # Load configuration
        self._load_configuration()
        self._calculate_inertia_matrix()
        
        # Initialize state
        self.state.current_mass = self.specs.empty_weight
        self.state.fuel_level = 1.0 if self.specs.fuel_capacity > 0 else 0.0
        self.state.battery_level = 1.0 if self.specs.battery_capacity > 0 else 0.0
    
    def _load_configuration(self):
        """Load drone configuration from JSON file."""
        try:
            with open(self.config_path, 'r') as f:
                configs = json.load(f)
            
            if self.model_name not in configs:
                raise ValueError(f"Drone model '{self.model_name}' not found in configuration")
            
            config = configs[self.model_name]
            
            # Load mass properties
            mass_config = config.get('mass', {})
            self.specs.empty_weight = mass_config.get('empty_weight', 35.0)
            self.specs.mtow = mass_config.get('mtow', 70.0)
            self.specs.fuel_capacity = mass_config.get('fuel_capacity', 0.0)
            
            # Load dimensions
            dim_config = config.get('dimensions', {})
            self.specs.wingspan = dim_config.get('wingspan', 2.0)
            self.specs.length = dim_config.get('length', 0.8)
            self.specs.height = dim_config.get('height', 0.8)
            self.specs.rotor_diameter = dim_config.get('rotor_diameter', 0.5)
            
            # Load propulsion
            prop_config = config.get('propulsion', {})
            self.specs.propulsion_type = prop_config.get('type', 'electric_rotors')
            self.specs.motor_count = prop_config.get('motor_count', 4)
            self.specs.max_thrust_per_motor = prop_config.get('max_thrust_per_motor', 20.0)
            self.specs.power_consumption = prop_config.get('power_consumption', 1500.0)
            self.specs.efficiency = prop_config.get('efficiency', 0.85)
            self.specs.battery_capacity = prop_config.get('battery_capacity', 10000.0)
            self.specs.specific_impulse = prop_config.get('specific_impulse', 0.0)
            
            # Load aerodynamics
            aero_config = config.get('aerodynamics', {})
            self.specs.drag_coefficient = aero_config.get('drag_coefficient', 0.3)
            self.specs.lift_coefficient = aero_config.get('lift_coefficient', 0.8)
            self.specs.reference_area = aero_config.get('reference_area', 0.2)
            
            # Load control limits
            control_config = config.get('control', {})
            self.specs.max_roll_rate = control_config.get('max_roll_rate', 180.0)
            self.specs.max_pitch_rate = control_config.get('max_pitch_rate', 180.0)
            self.specs.max_yaw_rate = control_config.get('max_yaw_rate', 90.0)
            
            # Load flight envelope
            envelope_config = config.get('flight_envelope', {})
            self.specs.max_altitude = envelope_config.get('max_altitude', 5000.0)
            self.specs.max_speed = envelope_config.get('max_speed', 30.0)
            self.specs.service_ceiling = envelope_config.get('service_ceiling', 10000.0)
            
        except Exception as e:
            raise RuntimeError(f"Error loading drone configuration: {e}")
    
    def _calculate_inertia_matrix(self):
        """Calculate moment of inertia matrix based on drone geometry."""
        # Simplified inertia calculation for typical drone geometries
        mass = self.specs.empty_weight
        
        if self.specs.propulsion_type in ['electric_rotors', 'thrusters']:
            # For multirotor drones - approximate as thin disk
            radius = self.specs.wingspan / 2.0
            height = self.specs.height
            
            # Moments of inertia for a thin disk
            Ixx = mass * (3 * radius**2 + height**2) / 12.0
            Iyy = mass * (3 * radius**2 + height**2) / 12.0
            Izz = mass * radius**2 / 2.0
            
        elif self.specs.propulsion_type == 'hybrid':
            # For hybrid drones - approximate as rectangular box
            length = self.specs.length
            width = self.specs.wingspan
            height = self.specs.height
            
            # Moments of inertia for a rectangular box
            Ixx = mass * (width**2 + height**2) / 12.0
            Iyy = mass * (length**2 + height**2) / 12.0
            Izz = mass * (length**2 + width**2) / 12.0
            
        else:
            # Default case
            Ixx = mass * 0.1
            Iyy = mass * 0.1
            Izz = mass * 0.2
        
        self.inertia_matrix = np.array([
            [Ixx, 0, 0],
            [0, Iyy, 0],
            [0, 0, Izz]
        ])
    
    def get_current_mass(self) -> float:
        """Get current drone mass including fuel and payload."""
        base_mass = self.specs.empty_weight
        fuel_mass = self.specs.fuel_capacity * self.state.fuel_level
        payload_mass = self.state.payload_mass
        
        return base_mass + fuel_mass + payload_mass
    
    def get_max_thrust(self) -> float:
        """Get maximum total thrust available."""
        return self.specs.max_thrust_per_motor * self.specs.motor_count
    
    def get_thrust_to_weight_ratio(self, gravity: float = 9.81) -> float:
        """Calculate thrust-to-weight ratio."""
        max_thrust = self.get_max_thrust()
        weight = self.get_current_mass() * gravity
        return max_thrust / weight if weight > 0 else 0.0
    
    def update_fuel_consumption(self, power_used: float, dt: float):
        """
        Update fuel/battery consumption based on power usage.
        
        Args:
            power_used: Power consumption in watts
            dt: Time step in seconds
        """
        if self.specs.propulsion_type == 'thrusters':
            # For thrusters, calculate fuel consumption based on thrust
            if self.specs.specific_impulse > 0:
                # Simplified fuel consumption model
                fuel_flow_rate = power_used / (self.specs.specific_impulse * 9.81)
                fuel_consumed = fuel_flow_rate * dt
                if self.specs.fuel_capacity > 0:
                    self.state.fuel_level -= fuel_consumed / self.specs.fuel_capacity
                    self.state.fuel_level = max(0.0, self.state.fuel_level)
        else:
            # For electric propulsion, calculate battery consumption
            if self.specs.battery_capacity > 0:
                energy_consumed = power_used * dt / 3600.0  # Wh
                self.state.battery_level -= energy_consumed / self.specs.battery_capacity
                self.state.battery_level = max(0.0, self.state.battery_level)
    
    def add_payload(self, mass: float):
        """Add payload mass."""
        total_mass = self.get_current_mass() + mass
        if total_mass <= self.specs.mtow:
            self.state.payload_mass += mass
            return True
        return False
    
    def remove_payload(self, mass: float):
        """Remove payload mass."""
        if self.state.payload_mass >= mass:
            self.state.payload_mass -= mass
            return True
        return False
    
    def is_operational(self) -> bool:
        """Check if drone is operational (has fuel/battery)."""
        if self.specs.propulsion_type == 'thrusters':
            return self.state.fuel_level > 0.0
        else:
            return self.state.battery_level > 0.0
    
    def get_status_dict(self) -> Dict[str, Any]:
        """Get current drone status as dictionary."""
        return {
            'model_name': self.model_name,
            'position': self.state.position.tolist(),
            'velocity': self.state.velocity.tolist(),
            'attitude': self.state.attitude.tolist(),
            'current_mass': self.get_current_mass(),
            'fuel_level': self.state.fuel_level,
            'battery_level': self.state.battery_level,
            'payload_mass': self.state.payload_mass,
            'power_consumption': self.state.power_consumption,
            'operational': self.is_operational(),
            'thrust_to_weight_ratio': self.get_thrust_to_weight_ratio()
        }
