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
- physics: Twin v2 physics core (density-dependent rotors, quaternion 6-DoF, RK4)
"""

__version__ = "1.0.0"
__author__ = "Drone Simulation Team"
__email__ = "simulation@example.com"

# Lazy imports: `import src.physics` must not pull in matplotlib, pygame, etc.
_LAZY = {
    'DroneSimulator': '.simulator',
    'DroneModel': '.drone_model',
    'PhysicsEngine': '.physics_engine',
    'Environment': '.environment',
    'MissionManager': '.mission_manager',
    'DataLogger': '.data_logger',
    'Visualizer': '.visualization',
}


def __getattr__(name):
    if name in _LAZY:
        import importlib
        return getattr(importlib.import_module(_LAZY[name], __name__), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    'DroneSimulator',
    'DroneModel',
    'PhysicsEngine',
    'Environment',
    'MissionManager',
    'DataLogger',
    'Visualizer'
]
