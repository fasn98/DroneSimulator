"""
Drone Optimization Module

Independent module for creating optimized drone configurations with advanced
aerodynamic and acoustic characteristics. This module implements the concepts
from acoustic optimization research, including uneven step angles for noise
reduction while maintaining equivalent lift characteristics.

Features:
- Acoustic-optimized rotor designs with uneven step angles
- Aerodynamic performance maintenance
- Professional drone configurations for safe flight
- CFD and CAA integration capabilities
- Manufacturing tolerance analysis
"""

__version__ = '1.0.0'

from .optimized_drones import OptimizedDroneFactory
from .acoustic_optimizer import AcousticOptimizer
from .aerodynamic_analyzer import AerodynamicAnalyzer
from .rotor_designer import RotorDesigner

__all__ = [
    'OptimizedDroneFactory',
    'AcousticOptimizer', 
    'AerodynamicAnalyzer',
    'RotorDesigner'
]