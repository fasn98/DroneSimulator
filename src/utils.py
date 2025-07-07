"""
Utility Functions Module

Common utility functions for configuration loading, logging setup,
and other helper functions used throughout the simulation.
"""

import json
import logging
import os
import sys
from pathlib import Path
from typing import Dict, Any, Optional, Union
import numpy as np
from datetime import datetime


def load_config(config_path: Union[str, Path]) -> Dict[str, Any]:
    """
    Load configuration from JSON file.
    
    Args:
        config_path: Path to configuration file
        
    Returns:
        Configuration dictionary
        
    Raises:
        FileNotFoundError: If config file doesn't exist
        json.JSONDecodeError: If config file is invalid JSON
    """
    config_path = Path(config_path)
    
    if not config_path.exists():
        raise FileNotFoundError(f"Configuration file not found: {config_path}")
    
    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
        return config
    except json.JSONDecodeError as e:
        raise json.JSONDecodeError(f"Invalid JSON in config file {config_path}: {e}")


def save_config(config: Dict[str, Any], config_path: Union[str, Path]):
    """
    Save configuration to JSON file.
    
    Args:
        config: Configuration dictionary
        config_path: Path to save configuration file
    """
    config_path = Path(config_path)
    config_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(config_path, 'w') as f:
        json.dump(config, f, indent=2)


def setup_logging(log_level: str = "INFO", log_file: Optional[str] = None, 
                 verbose: bool = False) -> logging.Logger:
    """
    Setup logging configuration.
    
    Args:
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_file: Optional log file path
        verbose: Enable verbose logging
        
    Returns:
        Configured logger instance
    """
    # Set log level
    if verbose:
        log_level = "DEBUG"
    
    numeric_level = getattr(logging, log_level.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError(f'Invalid log level: {log_level}')
    
    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # Configure root logger
    logger = logging.getLogger()
    logger.setLevel(numeric_level)
    
    # Remove existing handlers
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)
    
    # Add console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(numeric_level)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    # Add file handler if specified
    if log_file:
        log_path = Path(log_file)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(numeric_level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    
    return logger


def validate_drone_config(config: Dict[str, Any]) -> bool:
    """
    Validate drone configuration.
    
    Args:
        config: Drone configuration dictionary
        
    Returns:
        True if valid, False otherwise
    """
    required_fields = ['mass', 'dimensions', 'propulsion']
    
    for field in required_fields:
        if field not in config:
            return False
    
    # Validate mass properties
    mass_config = config['mass']
    if not all(key in mass_config for key in ['empty_weight', 'mtow']):
        return False
    
    if mass_config['empty_weight'] >= mass_config['mtow']:
        return False
    
    # Validate dimensions
    dimensions_config = config['dimensions']
    required_dims = ['length', 'height']
    if not all(key in dimensions_config for key in required_dims):
        return False
    
    # Validate propulsion
    propulsion_config = config['propulsion']
    if 'type' not in propulsion_config:
        return False
    
    return True


def validate_environment_config(config: Dict[str, Any]) -> bool:
    """
    Validate environment configuration.
    
    Args:
        config: Environment configuration dictionary
        
    Returns:
        True if valid, False otherwise
    """
    required_fields = ['gravity', 'atmosphere']
    
    for field in required_fields:
        if field not in config:
            return False
    
    # Validate gravity
    if config['gravity'] <= 0:
        return False
    
    # Validate atmosphere
    atmosphere_config = config['atmosphere']
    if 'type' not in atmosphere_config:
        return False
    
    return True


def validate_mission_config(config: Dict[str, Any]) -> bool:
    """
    Validate mission configuration.
    
    Args:
        config: Mission configuration dictionary
        
    Returns:
        True if valid, False otherwise
    """
    required_fields = ['type', 'waypoints']
    
    for field in required_fields:
        if field not in config:
            return False
    
    # Validate waypoints
    waypoints = config['waypoints']
    if not isinstance(waypoints, list) or len(waypoints) == 0:
        return False
    
    for waypoint in waypoints:
        if not all(key in waypoint for key in ['x', 'y', 'z', 'action']):
            return False
    
    return True


def calculate_distance(pos1: np.ndarray, pos2: np.ndarray) -> float:
    """
    Calculate Euclidean distance between two positions.
    
    Args:
        pos1: First position vector
        pos2: Second position vector
        
    Returns:
        Distance in meters
    """
    return np.linalg.norm(pos2 - pos1)


def calculate_bearing(pos1: np.ndarray, pos2: np.ndarray) -> float:
    """
    Calculate bearing from pos1 to pos2.
    
    Args:
        pos1: Start position vector
        pos2: End position vector
        
    Returns:
        Bearing in radians
    """
    delta = pos2 - pos1
    return np.arctan2(delta[1], delta[0])


def rotate_vector(vector: np.ndarray, angles: np.ndarray) -> np.ndarray:
    """
    Rotate vector by given angles (roll, pitch, yaw).
    
    Args:
        vector: 3D vector to rotate
        angles: Rotation angles [roll, pitch, yaw] in radians
        
    Returns:
        Rotated vector
    """
    roll, pitch, yaw = angles
    
    # Rotation matrices
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation matrix
    R = R_z @ R_y @ R_x
    
    return R @ vector


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-π, π] range.
    
    Args:
        angle: Angle in radians
        
    Returns:
        Normalized angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def create_directory_structure(base_path: Union[str, Path]):
    """
    Create standard directory structure for simulation output.
    
    Args:
        base_path: Base directory path
    """
    base_path = Path(base_path)
    
    directories = [
        'logs',
        'plots',
        'data',
        'config',
        'results'
    ]
    
    for directory in directories:
        (base_path / directory).mkdir(parents=True, exist_ok=True)


def format_time(seconds: float) -> str:
    """
    Format time in seconds to human-readable format.
    
    Args:
        seconds: Time in seconds
        
    Returns:
        Formatted time string
    """
    if seconds < 60:
        return f"{seconds:.1f}s"
    elif seconds < 3600:
        minutes = seconds // 60
        remaining_seconds = seconds % 60
        return f"{int(minutes)}m {remaining_seconds:.1f}s"
    else:
        hours = seconds // 3600
        remaining_minutes = (seconds % 3600) // 60
        remaining_seconds = seconds % 60
        return f"{int(hours)}h {int(remaining_minutes)}m {remaining_seconds:.1f}s"


def format_distance(meters: float) -> str:
    """
    Format distance in meters to human-readable format.
    
    Args:
        meters: Distance in meters
        
    Returns:
        Formatted distance string
    """
    if meters < 1000:
        return f"{meters:.1f}m"
    else:
        kilometers = meters / 1000
        return f"{kilometers:.2f}km"


def format_speed(mps: float) -> str:
    """
    Format speed in m/s to human-readable format.
    
    Args:
        mps: Speed in meters per second
        
    Returns:
        Formatted speed string
    """
    return f"{mps:.1f}m/s ({mps * 3.6:.1f}km/h)"


def get_system_info() -> Dict[str, Any]:
    """
    Get system information for logging.
    
    Returns:
        System information dictionary
    """
    import platform
    import psutil
    
    return {
        'platform': platform.platform(),
        'python_version': platform.python_version(),
        'cpu_count': psutil.cpu_count(),
        'memory_gb': psutil.virtual_memory().total / (1024**3),
        'timestamp': datetime.now().isoformat()
    }


def create_default_configs():
    """Create default configuration files if they don't exist."""
    config_dir = Path("config")
    config_dir.mkdir(exist_ok=True)
    
    # Check if configs already exist
    if (config_dir / "drone_models.json").exists():
        return
    
    # Create minimal default configs
    default_drone = {
        "default_quadrotor": {
            "description": "Default quadrotor configuration",
            "type": "quadrotor",
            "mass": {"empty_weight": 2.0, "mtow": 5.0, "fuel_capacity": 0.0},
            "dimensions": {"wingspan": 0.5, "length": 0.5, "height": 0.2, "rotor_diameter": 0.2},
            "propulsion": {
                "type": "electric_rotors",
                "motor_count": 4,
                "max_thrust_per_motor": 15.0,
                "power_consumption": 500.0,
                "efficiency": 0.8,
                "battery_capacity": 5000.0
            },
            "aerodynamics": {"drag_coefficient": 0.3, "lift_coefficient": 0.8, "reference_area": 0.1},
            "control": {"max_roll_rate": 180.0, "max_pitch_rate": 180.0, "max_yaw_rate": 90.0},
            "sensors": {"camera": {"fov": 60, "range": 1000}},
            "flight_envelope": {"max_altitude": 1000.0, "max_speed": 20.0, "service_ceiling": 2000.0}
        }
    }
    
    default_environment = {
        "earth": {
            "description": "Earth standard atmosphere",
            "gravity": 9.81,
            "atmosphere": {
                "type": "standard",
                "sea_level_pressure": 101325.0,
                "sea_level_density": 1.225,
                "temperature_lapse_rate": -0.0065,
                "sea_level_temperature": 288.15,
                "gas_constant": 287.0
            },
            "wind": {"enabled": false, "base_speed": 0.0}
        }
    }
    
    default_mission = {
        "test_flight": {
            "description": "Simple test flight",
            "type": "test",
            "waypoints": [
                {"x": 0, "y": 0, "z": 10, "action": "takeoff"},
                {"x": 10, "y": 0, "z": 10, "action": "navigate"},
                {"x": 0, "y": 0, "z": 0, "action": "land"}
            ],
            "success_criteria": {"flight_time_max": 300.0},
            "constraints": {"max_altitude": 50.0, "max_speed": 10.0}
        }
    }
    
    # Save default configs
    save_config(default_drone, config_dir / "drone_models.json")
    save_config(default_environment, config_dir / "environments.json")
    save_config(default_mission, config_dir / "missions.json")


class ConfigurationError(Exception):
    """Configuration error exception."""
    pass


class SimulationError(Exception):
    """Simulation error exception."""
    pass


class ValidationError(Exception):
    """Validation error exception."""
    pass


def handle_exception(exc_type, exc_value, exc_traceback):
    """Global exception handler."""
    if issubclass(exc_type, KeyboardInterrupt):
        sys.__excepthook__(exc_type, exc_value, exc_traceback)
        return
    
    logger = logging.getLogger(__name__)
    logger.error("Uncaught exception", exc_info=(exc_type, exc_value, exc_traceback))


# Set global exception handler
sys.excepthook = handle_exception
