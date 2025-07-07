"""
Exploration Drone Simulation Package

This package provides a comprehensive simulation environment for testing
UAV performance across Earth, Mars, and Moon environments.

Main modules:
- drone_model: Drone configuration and physical parameters
- physics_engine: 6-DoF flight dynamics simulation
- environment: Environmental modeling (atmosphere, gravity, etc.)
- mission_manager: Mission planning and execution
- data_logger: Telemetry collection and storage
- visualization: Data plotting and analysis
- simulator: Main simulation controller
"""

__version__ = "1.0.0"
__author__ = "Drone Simulation Team"
__email__ = "simulation@example.com"

from .simulator import DroneSimulator
from .drone_model import DroneModel
from .physics_engine import PhysicsEngine
from .environment import Environment
from .mission_manager import MissionManager
from .data_logger import DataLogger
from .visualization import Visualizer

__all__ = [
    'DroneSimulator',
    'DroneModel',
    'PhysicsEngine',
    'Environment',
    'MissionManager',
    'DataLogger',
    'Visualizer'
]
