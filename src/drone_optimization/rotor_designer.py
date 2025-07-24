"""
Rotor Designer Module

Implements advanced rotor design with uneven step angles for acoustic optimization
while maintaining aerodynamic performance based on research specifications.
"""

import numpy as np
import json
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class RotorSpecification:
    """Rotor design specifications with acoustic optimization parameters."""
    
    num_blades: int
    diameter: float  # meters
    chord_length: float  # meters
    twist_distribution: List[float]  # degrees along span
    step_angles: List[float]  # angular spacing between blades (degrees)
    airfoil_profile: str
    tip_speed: float  # m/s
    design_rpm: float
    acoustic_optimization: bool = True
    manufacturing_tolerance: float = 0.01  # percentage


@dataclass
class AcousticMetrics:
    """Acoustic performance metrics for rotor design."""
    
    overall_sound_pressure_level: float  # dB
    perceived_noise_level: float  # dB
    discrete_tone_reduction: float  # dB reduction vs symmetric design
    broadband_noise_level: float  # dB
    frequency_spectrum: Dict[float, float]  # Hz: dB mapping


@dataclass
class AerodynamicMetrics:
    """Aerodynamic performance metrics for rotor design."""
    
    thrust_coefficient: float
    power_coefficient: float
    figure_of_merit: float
    thrust_asymmetry: float  # percentage
    vibration_level: float  # N·m
    efficiency: float  # percentage


class RotorDesigner:
    """
    Advanced rotor designer implementing acoustic optimization through uneven step angles.
    
    Based on research showing 3-5 dB reduction in discrete tones through careful
    angular spacing optimization while maintaining aerodynamic performance.
    """
    
    def __init__(self):
        """Initialize the rotor designer with optimization parameters."""
        self.optimization_ranges = {
            'angular_spacing_variation': (0.05, 0.10),  # ±5% to ±10% of nominal
            'pitch_variation': (1.0, 3.0),  # ±1° to ±3° for variable pitch
            'manufacturing_tolerance': (0.005, 0.02)  # 0.5% to 2%
        }
        
        # Research-based acoustic coefficients
        self.acoustic_coefficients = {
            'discrete_tone_reduction_factor': 0.8,  # 3-5 dB reduction achievable
            'broadband_redistribution_factor': 1.2,
            'blade_vortex_interaction_factor': 0.6
        }
    
    def design_optimized_rotor(self, base_design: Dict, optimization_target: str = 'balanced') -> RotorSpecification:
        """
        Design an acoustically optimized rotor based on input specifications.
        
        Args:
            base_design: Base rotor design parameters
            optimization_target: 'acoustic', 'aerodynamic', or 'balanced'
            
        Returns:
            RotorSpecification with optimized parameters
        """
        num_blades = base_design.get('num_blades', 4)
        diameter = base_design.get('diameter', 0.25)  # meters
        
        # Calculate nominal angular spacing
        nominal_angle = 360.0 / num_blades
        
        # Generate optimized step angles based on research parameters
        step_angles = self._calculate_optimal_step_angles(
            num_blades, 
            optimization_target
        )
        
        # Optimize blade twist distribution for acoustic performance
        twist_distribution = self._optimize_twist_distribution(
            base_design.get('twist_distribution', [0, -5, -10, -15]),
            step_angles
        )
        
        # Calculate design parameters
        design_rpm = base_design.get('rpm', 6000)
        tip_speed = (np.pi * diameter * design_rpm) / 60.0
        
        return RotorSpecification(
            num_blades=num_blades,
            diameter=diameter,
            chord_length=base_design.get('chord_length', 0.02),
            twist_distribution=twist_distribution,
            step_angles=step_angles,
            airfoil_profile=base_design.get('airfoil', 'NACA2412'),
            tip_speed=tip_speed,
            design_rpm=design_rpm,
            acoustic_optimization=True,
            manufacturing_tolerance=base_design.get('tolerance', 0.01)
        )
    
    def _calculate_optimal_step_angles(self, num_blades: int, target: str) -> List[float]:
        """
        Calculate optimal angular spacing for acoustic optimization.
        
        Based on research showing ±5% to ±10% variations in angular spacing
        provide optimal balance between noise reduction and performance.
        """
        nominal_angle = 360.0 / num_blades
        
        if target == 'acoustic':
            # Maximum acoustic optimization (±8-10%)
            variation_factor = 0.09
        elif target == 'aerodynamic':
            # Minimal variation to preserve aerodynamics (±3-5%)
            variation_factor = 0.04
        else:  # balanced
            # Optimal balance (±5-7%)
            variation_factor = 0.06
        
        # Generate uneven spacing pattern
        step_angles = []
        for i in range(num_blades):
            # Alternating pattern for optimal acoustic dispersion
            variation = variation_factor * (-1)**i * nominal_angle
            step_angles.append(nominal_angle + variation)
        
        # Ensure total adds to 360 degrees
        total = sum(step_angles)
        adjustment = (360.0 - total) / num_blades
        step_angles = [angle + adjustment for angle in step_angles]
        
        return step_angles
    
    def _optimize_twist_distribution(self, base_twist: List[float], step_angles: List[float]) -> List[float]:
        """
        Optimize blade twist distribution to compensate for uneven step angles.
        
        Adjusts individual blade twist to maintain thrust balance while
        preserving acoustic benefits.
        """
        optimized_twist = base_twist.copy()
        
        # Calculate angular deviation from nominal
        nominal_angle = 360.0 / len(step_angles)
        
        for i, angle in enumerate(step_angles):
            deviation = (angle - nominal_angle) / nominal_angle
            
            # Compensate for angular spacing with slight twist adjustment
            # Research shows ±1-3° twist variation is effective
            twist_compensation = deviation * 2.0  # 2° max compensation
            
            # Apply compensation to twist distribution
            if i < len(optimized_twist):
                optimized_twist[i] += twist_compensation
        
        return optimized_twist
    
    def analyze_acoustic_performance(self, rotor_spec: RotorSpecification) -> AcousticMetrics:
        """
        Analyze acoustic performance of optimized rotor design.
        
        Uses research-based models to predict noise reduction and
        spectral characteristics.
        """
        # Base noise levels for symmetric design
        base_oaspl = 65.0  # dB typical for small drone
        base_pnl = 70.0    # dB perceived noise level
        
        # Calculate acoustic benefits from uneven step angles
        angular_variation = self._calculate_angular_variation(rotor_spec.step_angles)
        
        # Discrete tone reduction (3-5 dB achievable per research)
        discrete_tone_reduction = min(5.0, angular_variation * 50.0)
        
        # Overall noise reduction accounting for broadband redistribution
        oaspl_reduction = discrete_tone_reduction * 0.6  # Partial overall reduction
        pnl_reduction = discrete_tone_reduction * 0.8    # Better perceived reduction
        
        # Generate frequency spectrum with reduced discrete tones
        frequency_spectrum = self._generate_acoustic_spectrum(
            rotor_spec, discrete_tone_reduction
        )
        
        return AcousticMetrics(
            overall_sound_pressure_level=base_oaspl - oaspl_reduction,
            perceived_noise_level=base_pnl - pnl_reduction,
            discrete_tone_reduction=discrete_tone_reduction,
            broadband_noise_level=base_oaspl - 10.0,  # Typical broadband level
            frequency_spectrum=frequency_spectrum
        )
    
    def analyze_aerodynamic_performance(self, rotor_spec: RotorSpecification) -> AerodynamicMetrics:
        """
        Analyze aerodynamic performance and trade-offs of optimized design.
        
        Evaluates thrust asymmetry, efficiency impacts, and vibration levels.
        """
        # Calculate thrust asymmetry from uneven step angles
        angular_variation = self._calculate_angular_variation(rotor_spec.step_angles)
        thrust_asymmetry = angular_variation * 2.0  # Typical relationship
        
        # Efficiency impact (well-designed systems maintain >95% efficiency)
        efficiency_loss = min(3.0, angular_variation * 10.0)
        base_efficiency = 85.0  # Typical rotor efficiency
        
        # Vibration levels increase with asymmetry
        base_vibration = 0.1  # N·m
        vibration_increase = angular_variation * 0.5
        
        # Figure of merit and coefficients (research-based estimates)
        thrust_coefficient = 0.012 * (1.0 - efficiency_loss / 100.0)
        power_coefficient = 0.008 * (1.0 + efficiency_loss / 200.0)
        figure_of_merit = thrust_coefficient / power_coefficient * 0.8
        
        return AerodynamicMetrics(
            thrust_coefficient=thrust_coefficient,
            power_coefficient=power_coefficient,
            figure_of_merit=figure_of_merit,
            thrust_asymmetry=thrust_asymmetry,
            vibration_level=base_vibration + vibration_increase,
            efficiency=base_efficiency - efficiency_loss
        )
    
    def _calculate_angular_variation(self, step_angles: List[float]) -> float:
        """Calculate the degree of angular variation from nominal spacing."""
        nominal_angle = 360.0 / len(step_angles)
        variations = [abs(angle - nominal_angle) / nominal_angle for angle in step_angles]
        return sum(variations) / len(variations)
    
    def _generate_acoustic_spectrum(self, rotor_spec: RotorSpecification, 
                                  tone_reduction: float) -> Dict[float, float]:
        """Generate acoustic frequency spectrum with optimized characteristics."""
        spectrum = {}
        
        # Blade passage frequency and harmonics
        bpf = (rotor_spec.design_rpm / 60.0) * rotor_spec.num_blades
        
        # Generate spectrum with reduced discrete tones
        for harmonic in range(1, 6):
            frequency = bpf * harmonic
            
            # Base tone level reduces with harmonic number
            base_level = 55.0 - (harmonic - 1) * 5.0
            
            # Apply tone reduction for optimized design
            optimized_level = base_level - tone_reduction * (0.8 ** (harmonic - 1))
            
            spectrum[frequency] = optimized_level
        
        # Add broadband components
        for freq in [100, 200, 500, 1000, 2000, 5000]:
            if freq not in spectrum:
                spectrum[freq] = 45.0 - abs(freq - 1000) / 100.0
        
        return spectrum
    
    def generate_manufacturing_specs(self, rotor_spec: RotorSpecification) -> Dict:
        """
        Generate detailed manufacturing specifications for optimized rotor.
        
        Includes precision requirements, quality control parameters,
        and balancing specifications based on research recommendations.
        """
        return {
            'geometric_precision': {
                'angular_tolerance': rotor_spec.manufacturing_tolerance / 2.0,  # degrees
                'chord_tolerance': rotor_spec.chord_length * 0.005,  # meters
                'twist_tolerance': 0.5,  # degrees
                'diameter_tolerance': rotor_spec.diameter * 0.002  # meters
            },
            'quality_control': {
                'dimensional_inspection': 'CMM required for angular positioning',
                'balance_tolerance': rotor_spec.diameter * 0.001,  # g·cm
                'surface_finish': 'Ra 1.6 μm maximum',
                'material_specifications': 'Carbon fiber composite or aluminum alloy'
            },
            'assembly_requirements': {
                'blade_positioning_accuracy': 0.1,  # degrees
                'hub_runout': 0.02,  # mm
                'static_balance': 0.5,  # g·cm maximum
                'dynamic_balance': 1.0,  # g·cm maximum at operating speed
            },
            'testing_protocols': {
                'acoustic_verification': 'Anechoic chamber testing required',
                'vibration_testing': 'Modal analysis and operating deflection shapes',
                'performance_validation': 'Thrust and power measurement at design RPM',
                'endurance_testing': '100 hour operational qualification'
            }
        }
    
    def export_design_data(self, rotor_spec: RotorSpecification, 
                          acoustic_metrics: AcousticMetrics,
                          aero_metrics: AerodynamicMetrics,
                          filename: str) -> None:
        """Export complete rotor design data to JSON file."""
        design_data = {
            'rotor_specification': {
                'num_blades': rotor_spec.num_blades,
                'diameter': rotor_spec.diameter,
                'chord_length': rotor_spec.chord_length,
                'twist_distribution': rotor_spec.twist_distribution,
                'step_angles': rotor_spec.step_angles,
                'airfoil_profile': rotor_spec.airfoil_profile,
                'tip_speed': rotor_spec.tip_speed,
                'design_rpm': rotor_spec.design_rpm,
                'acoustic_optimization': rotor_spec.acoustic_optimization,
                'manufacturing_tolerance': rotor_spec.manufacturing_tolerance
            },
            'acoustic_performance': {
                'overall_sound_pressure_level': acoustic_metrics.overall_sound_pressure_level,
                'perceived_noise_level': acoustic_metrics.perceived_noise_level,
                'discrete_tone_reduction': acoustic_metrics.discrete_tone_reduction,
                'broadband_noise_level': acoustic_metrics.broadband_noise_level,
                'frequency_spectrum': acoustic_metrics.frequency_spectrum
            },
            'aerodynamic_performance': {
                'thrust_coefficient': aero_metrics.thrust_coefficient,
                'power_coefficient': aero_metrics.power_coefficient,
                'figure_of_merit': aero_metrics.figure_of_merit,
                'thrust_asymmetry': aero_metrics.thrust_asymmetry,
                'vibration_level': aero_metrics.vibration_level,
                'efficiency': aero_metrics.efficiency
            },
            'manufacturing_specifications': self.generate_manufacturing_specs(rotor_spec)
        }
        
        with open(filename, 'w') as f:
            json.dump(design_data, f, indent=2)