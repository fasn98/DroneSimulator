"""
Environment Module

Manages environmental conditions including atmosphere, gravity, and weather
for different planetary environments (Earth, Mars, Moon).
"""

import json
import numpy as np
import math
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass


@dataclass
class AtmosphereModel:
    """Atmospheric model parameters."""
    sea_level_pressure: float = 101325.0  # Pa
    sea_level_density: float = 1.225      # kg/m³
    sea_level_temperature: float = 288.15  # K
    temperature_lapse_rate: float = -0.0065  # K/m
    gas_constant: float = 287.0           # J/(kg·K)
    
    def get_properties(self, altitude: float) -> Tuple[float, float, float]:
        """
        Get atmospheric properties at given altitude.
        
        Args:
            altitude: Altitude in meters
            
        Returns:
            Tuple of (pressure, density, temperature)
        """
        # Standard atmosphere model
        if altitude < 11000:  # Troposphere
            temperature = self.sea_level_temperature + self.temperature_lapse_rate * altitude
            pressure = self.sea_level_pressure * (temperature / self.sea_level_temperature) ** (-9.81 / (self.gas_constant * self.temperature_lapse_rate))
        else:  # Stratosphere (simplified)
            temperature = 216.65  # K
            pressure = 22632.0 * math.exp(-9.81 * (altitude - 11000) / (self.gas_constant * temperature))
        
        density = pressure / (self.gas_constant * temperature)
        
        return pressure, density, temperature


class Environment:
    """
    Environmental simulation class.
    
    Manages:
    - Atmospheric conditions (pressure, density, temperature)
    - Gravitational fields
    - Weather conditions (wind, temperature variations)
    - Environmental factors specific to different planets
    """
    
    def __init__(self, environment_name: str, config_path: str = "config/environments.json"):
        """
        Initialize environment.
        
        Args:
            environment_name: Name of the environment (earth, mars, moon)
            config_path: Path to environment configuration file
        """
        self.name = environment_name
        self.config_path = config_path
        self.gravity = 9.81
        self.atmosphere = None
        self.wind_model = None
        self.environmental_factors = {}
        
        # Load configuration
        self._load_configuration()
        
        # Initialize atmospheric model
        self._initialize_atmosphere()
        
        # Initialize wind model
        self._initialize_wind_model()
    
    def _load_configuration(self):
        """Load environment configuration from JSON file."""
        try:
            with open(self.config_path, 'r') as f:
                configs = json.load(f)
            
            if self.name not in configs:
                raise ValueError(f"Environment '{self.name}' not found in configuration")
            
            config = configs[self.name]
            
            # Load gravity
            self.gravity = config.get('gravity', 9.81)
            
            # Load atmospheric configuration
            self.atmosphere_config = config.get('atmosphere', {})
            
            # Load wind configuration
            self.wind_config = config.get('wind', {})
            
            # Load environmental factors
            self.environmental_factors = config.get('environmental_factors', {})
            
        except Exception as e:
            raise RuntimeError(f"Error loading environment configuration: {e}")
    
    def _initialize_atmosphere(self):
        """Initialize atmospheric model based on configuration."""
        if self.atmosphere_config.get('type') == 'vacuum':
            # No atmosphere (Moon)
            self.atmosphere = None
        else:
            # Create atmosphere model
            self.atmosphere = AtmosphereModel(
                sea_level_pressure=self.atmosphere_config.get('sea_level_pressure', 101325.0),
                sea_level_density=self.atmosphere_config.get('sea_level_density', 1.225),
                sea_level_temperature=self.atmosphere_config.get('sea_level_temperature', 288.15),
                temperature_lapse_rate=self.atmosphere_config.get('temperature_lapse_rate', -0.0065),
                gas_constant=self.atmosphere_config.get('gas_constant', 287.0)
            )
    
    def _initialize_wind_model(self):
        """Initialize wind model."""
        if self.wind_config.get('enabled', False):
            self.wind_model = {
                'base_speed': self.wind_config.get('base_speed', 0.0),
                'gust_factor': self.wind_config.get('gust_factor', 1.0),
                'direction_variability': self.wind_config.get('direction_variability', 0.0),
                'current_direction': 0.0,
                'current_speed': self.wind_config.get('base_speed', 0.0)
            }
        else:
            self.wind_model = None
    
    def has_atmosphere(self) -> bool:
        """Check if environment has atmosphere."""
        return self.atmosphere is not None
    
    def get_air_density(self, altitude: float) -> float:
        """
        Get air density at given altitude.
        
        Args:
            altitude: Altitude in meters
            
        Returns:
            Air density in kg/m³
        """
        if not self.has_atmosphere():
            return 0.0
        
        _, density, _ = self.atmosphere.get_properties(altitude)
        return density
    
    def get_air_pressure(self, altitude: float) -> float:
        """
        Get air pressure at given altitude.
        
        Args:
            altitude: Altitude in meters
            
        Returns:
            Air pressure in Pa
        """
        if not self.has_atmosphere():
            return 0.0
        
        pressure, _, _ = self.atmosphere.get_properties(altitude)
        return pressure
    
    def get_temperature(self, altitude: float) -> float:
        """
        Get temperature at given altitude.
        
        Args:
            altitude: Altitude in meters
            
        Returns:
            Temperature in K
        """
        if not self.has_atmosphere():
            return self.environmental_factors.get('sea_level_temperature', 250.0)
        
        _, _, temperature = self.atmosphere.get_properties(altitude)
        return temperature
    
    def get_wind_velocity(self, position: np.ndarray, time: float) -> np.ndarray:
        """
        Get wind velocity at given position and time.
        
        Args:
            position: Position vector [x, y, z]
            time: Current time in seconds
            
        Returns:
            Wind velocity vector [vx, vy, vz]
        """
        if not self.wind_model:
            return np.zeros(3)
        
        # Update wind model
        self._update_wind_model(time)
        
        # Calculate wind velocity
        wind_speed = self.wind_model['current_speed']
        wind_direction = self.wind_model['current_direction']
        
        # Wind velocity in horizontal plane
        wind_velocity = np.array([
            wind_speed * math.cos(wind_direction),
            wind_speed * math.sin(wind_direction),
            0.0  # No vertical wind component for now
        ])
        
        # Add altitude effects
        altitude = max(0, position[2])
        altitude_factor = 1.0 + 0.1 * math.sqrt(altitude / 1000.0)  # Wind increases with altitude
        wind_velocity[:2] *= altitude_factor
        
        return wind_velocity
    
    def _update_wind_model(self, time: float):
        """Update wind model parameters based on time."""
        if not self.wind_model:
            return
        
        # Update wind direction with some variability
        direction_noise = 0.1 * math.sin(time * 0.1) * math.radians(self.wind_model['direction_variability'])
        self.wind_model['current_direction'] += direction_noise
        
        # Update wind speed with gusts
        base_speed = self.wind_model['base_speed']
        gust_factor = self.wind_model['gust_factor']
        gust_noise = 0.2 * math.sin(time * 0.5) * (gust_factor - 1.0)
        self.wind_model['current_speed'] = base_speed * (1.0 + gust_noise)
    
    def get_gravity_vector(self, position: np.ndarray) -> np.ndarray:
        """
        Get gravity vector at given position.
        
        Args:
            position: Position vector [x, y, z]
            
        Returns:
            Gravity vector [gx, gy, gz]
        """
        # For planetary surfaces, gravity is approximately constant
        # and points toward the center (negative z direction)
        return np.array([0.0, 0.0, -self.gravity])
    
    def get_environmental_effects(self, position: np.ndarray, time: float) -> Dict[str, Any]:
        """
        Get environmental effects at given position and time.
        
        Args:
            position: Position vector [x, y, z]
            time: Current time in seconds
            
        Returns:
            Dictionary of environmental effects
        """
        altitude = max(0, position[2])
        
        effects = {
            'gravity': self.gravity,
            'air_density': self.get_air_density(altitude),
            'air_pressure': self.get_air_pressure(altitude),
            'temperature': self.get_temperature(altitude),
            'wind_velocity': self.get_wind_velocity(position, time),
            'visibility': self.environmental_factors.get('visibility', 10000.0)
        }
        
        # Add environment-specific effects
        if self.name == 'mars':
            effects['dust_opacity'] = self.environmental_factors.get('dust_opacity', 0.5)
            effects['dust_storm_probability'] = self._calculate_dust_storm_probability(time)
        elif self.name == 'moon':
            effects['radiation_level'] = self.environmental_factors.get('radiation', 'high')
            effects['thermal_variation'] = self.environmental_factors.get('temperature_variation', 280.0)
        elif self.name == 'earth':
            effects['humidity'] = self.environmental_factors.get('humidity', 0.6)
            effects['weather_condition'] = self._get_weather_condition(time)
        
        return effects
    
    def _calculate_dust_storm_probability(self, time: float) -> float:
        """Calculate dust storm probability for Mars."""
        # Simplified model - higher probability during certain seasons
        seasonal_factor = 0.5 + 0.3 * math.sin(time * 2 * math.pi / (687 * 24 * 3600))  # Martian year
        base_probability = 0.1
        return base_probability * seasonal_factor
    
    def _get_weather_condition(self, time: float) -> str:
        """Get weather condition for Earth."""
        # Simplified weather model
        weather_factor = math.sin(time * 0.001) + 0.5 * math.sin(time * 0.0001)
        
        if weather_factor > 0.5:
            return "clear"
        elif weather_factor > 0.0:
            return "partly_cloudy"
        elif weather_factor > -0.5:
            return "cloudy"
        else:
            return "stormy"
    
    def is_safe_for_flight(self, position: np.ndarray, time: float) -> Tuple[bool, str]:
        """
        Check if conditions are safe for flight.
        
        Args:
            position: Position vector [x, y, z]
            time: Current time in seconds
            
        Returns:
            Tuple of (is_safe, reason)
        """
        effects = self.get_environmental_effects(position, time)
        
        # Check altitude limits
        altitude = position[2]
        if altitude < 0:
            return False, "Below ground level"
        
        # Check wind conditions
        wind_velocity = effects['wind_velocity']
        wind_speed = np.linalg.norm(wind_velocity)
        if wind_speed > 20.0:  # m/s
            return False, f"Wind speed too high: {wind_speed:.1f} m/s"
        
        # Environment-specific checks
        if self.name == 'mars':
            dust_prob = effects.get('dust_storm_probability', 0.0)
            if dust_prob > 0.7:
                return False, "High dust storm probability"
        
        # Check visibility
        visibility = effects.get('visibility', 10000.0)
        if visibility < 100.0:
            return False, f"Poor visibility: {visibility:.1f} m"
        
        return True, "Conditions safe for flight"
    
    def get_status_dict(self) -> Dict[str, Any]:
        """Get current environment status as dictionary."""
        return {
            'name': self.name,
            'gravity': self.gravity,
            'has_atmosphere': self.has_atmosphere(),
            'atmosphere_type': self.atmosphere_config.get('type', 'standard'),
            'wind_enabled': self.wind_model is not None,
            'environmental_factors': self.environmental_factors
        }
