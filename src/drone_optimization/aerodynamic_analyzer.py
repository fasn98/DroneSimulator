"""
Aerodynamic Analyzer Module

Analyzes aerodynamic performance of optimized drone rotors and provides
trade-off analysis between acoustic optimization and flight performance.
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class AerodynamicAnalysisResult:
    """Results from aerodynamic analysis of a rotor configuration."""
    
    thrust_coefficient: float
    power_coefficient: float
    figure_of_merit: float
    thrust_asymmetry_factor: float
    vibration_amplitude: float  # N·m
    efficiency_percentage: float
    performance_degradation: float  # % relative to symmetric design
    recommended_operating_range: Tuple[float, float]  # RPM range


@dataclass
class FlightEnvelopeLimit:
    """Flight envelope limitations due to aerodynamic characteristics."""
    
    max_forward_speed: float  # m/s
    max_climb_rate: float    # m/s
    max_descent_rate: float  # m/s
    max_wind_resistance: float  # m/s
    service_ceiling: float   # m
    payload_limitation: float  # kg


class AerodynamicAnalyzer:
    """
    Aerodynamic performance analyzer for optimized drone rotors.
    
    Evaluates the aerodynamic trade-offs of acoustic optimization and
    provides flight performance predictions for optimized designs.
    """
    
    def __init__(self):
        """Initialize aerodynamic analyzer with performance models."""
        
        # Standard atmospheric conditions
        self.atmosphere = {
            'air_density': 1.225,      # kg/m³ at sea level
            'kinematic_viscosity': 1.48e-5,  # m²/s
            'speed_of_sound': 343.0,   # m/s
            'temperature': 288.15      # K (15°C)
        }
        
        # Aerodynamic coefficients for rotor analysis
        self.aero_coefficients = {
            'ideal_efficiency': 0.85,     # Maximum theoretical efficiency
            'profile_drag_factor': 0.008,  # Profile drag coefficient
            'induced_loss_factor': 1.15,   # Induced power factor
            'tip_loss_factor': 0.97       # Tip loss correction
        }
        
        # Asymmetry impact factors (based on research)
        self.asymmetry_factors = {
            'thrust_variation_gain': 2.0,    # Angular variation to thrust variation
            'vibration_gain': 1.5,           # Angular variation to vibration
            'efficiency_loss_gain': 1.2      # Angular variation to efficiency loss
        }
    
    def analyze_rotor_performance(self,
                                angular_spacing: List[float],
                                rotor_diameter: float,
                                design_rpm: float,
                                num_blades: int,
                                chord_length: float,
                                twist_distribution: List[float]) -> AerodynamicAnalysisResult:
        """
        Perform comprehensive aerodynamic analysis of optimized rotor.
        
        Args:
            angular_spacing: Blade angular positions in degrees
            rotor_diameter: Rotor diameter in meters
            design_rpm: Design operating RPM
            num_blades: Number of rotor blades
            chord_length: Blade chord length in meters
            twist_distribution: Blade twist along span in degrees
            
        Returns:
            Detailed aerodynamic analysis results
        """
        # Calculate basic rotor parameters
        rotor_area = np.pi * (rotor_diameter / 2) ** 2
        tip_speed = np.pi * rotor_diameter * design_rpm / 60.0
        
        # Calculate angular spacing asymmetry
        asymmetry_factor = self._calculate_asymmetry_factor(angular_spacing)
        
        # Calculate thrust and power coefficients
        thrust_coeff, power_coeff = self._calculate_rotor_coefficients(
            rotor_diameter, tip_speed, num_blades, chord_length, asymmetry_factor
        )
        
        # Calculate figure of merit
        figure_of_merit = self._calculate_figure_of_merit(thrust_coeff, power_coeff)
        
        # Calculate thrust asymmetry effects
        thrust_asymmetry = self._calculate_thrust_asymmetry(
            angular_spacing, thrust_coeff
        )
        
        # Calculate vibration amplitude
        vibration_amplitude = self._calculate_vibration_amplitude(
            thrust_asymmetry, rotor_diameter, design_rpm
        )
        
        # Calculate efficiency and performance degradation
        efficiency, degradation = self._calculate_efficiency_degradation(
            figure_of_merit, asymmetry_factor
        )
        
        # Determine recommended operating range
        operating_range = self._determine_operating_range(
            design_rpm, asymmetry_factor, tip_speed
        )
        
        return AerodynamicAnalysisResult(
            thrust_coefficient=thrust_coeff,
            power_coefficient=power_coeff,
            figure_of_merit=figure_of_merit,
            thrust_asymmetry_factor=thrust_asymmetry,
            vibration_amplitude=vibration_amplitude,
            efficiency_percentage=efficiency * 100,
            performance_degradation=degradation * 100,
            recommended_operating_range=operating_range
        )
    
    def evaluate_flight_envelope(self,
                               aero_results: AerodynamicAnalysisResult,
                               drone_weight: float,
                               rotor_diameter: float,
                               num_rotors: int) -> FlightEnvelopeLimit:
        """
        Evaluate flight envelope limitations based on aerodynamic analysis.
        
        Args:
            aero_results: Results from aerodynamic analysis
            drone_weight: Total drone weight in kg
            rotor_diameter: Individual rotor diameter in meters
            num_rotors: Number of rotors on drone
            
        Returns:
            Flight envelope limitations
        """
        total_rotor_area = num_rotors * np.pi * (rotor_diameter / 2) ** 2
        disc_loading = (drone_weight * 9.81) / total_rotor_area  # N/m²
        
        # Calculate performance limitations
        max_forward_speed = self._calculate_max_forward_speed(
            aero_results, disc_loading
        )
        
        max_climb_rate = self._calculate_max_climb_rate(
            aero_results, disc_loading
        )
        
        max_wind_resistance = self._calculate_wind_resistance(
            aero_results, max_forward_speed
        )
        
        service_ceiling = self._calculate_service_ceiling(
            aero_results, disc_loading
        )
        
        payload_limitation = self._calculate_payload_limitation(
            aero_results, drone_weight, total_rotor_area
        )
        
        max_descent_rate = min(5.0, max_climb_rate * 0.8)  # Conservative limit
        
        return FlightEnvelopeLimit(
            max_forward_speed=max_forward_speed,
            max_climb_rate=max_climb_rate,
            max_descent_rate=max_descent_rate,
            max_wind_resistance=max_wind_resistance,
            service_ceiling=service_ceiling,
            payload_limitation=payload_limitation
        )
    
    def _calculate_asymmetry_factor(self, angular_spacing: List[float]) -> float:
        """Calculate rotor asymmetry factor from angular spacing."""
        nominal_angle = 360.0 / len(angular_spacing)
        
        # Calculate coefficient of variation
        deviations = [abs(angle - nominal_angle) for angle in angular_spacing]
        mean_deviation = np.mean(deviations)
        std_deviation = np.std(deviations)
        
        # Asymmetry factor (0 = symmetric, 1 = maximum reasonable asymmetry)
        asymmetry = (mean_deviation / nominal_angle) + (std_deviation / nominal_angle)
        
        return min(1.0, asymmetry)
    
    def _calculate_rotor_coefficients(self,
                                    rotor_diameter: float,
                                    tip_speed: float,
                                    num_blades: int,
                                    chord_length: float,
                                    asymmetry_factor: float) -> Tuple[float, float]:
        """Calculate thrust and power coefficients for the rotor."""
        
        # Baseline coefficients for symmetric rotor
        solidity = (num_blades * chord_length) / (np.pi * rotor_diameter / 2)
        
        # Thrust coefficient (based on blade element momentum theory)
        base_ct = 0.6 * solidity * 0.1  # Simplified relationship
        
        # Power coefficient 
        base_cp = base_ct ** 1.5 / (2 ** 0.5) + 0.008 * solidity  # Ideal + profile power
        
        # Apply asymmetry corrections
        ct_correction = 1.0 - asymmetry_factor * 0.05  # Up to 5% thrust loss
        cp_correction = 1.0 + asymmetry_factor * 0.08  # Up to 8% power increase
        
        thrust_coeff = base_ct * ct_correction
        power_coeff = base_cp * cp_correction
        
        return thrust_coeff, power_coeff
    
    def _calculate_figure_of_merit(self, thrust_coeff: float, power_coeff: float) -> float:
        """Calculate rotor figure of merit."""
        # Figure of merit = ideal power / actual power
        ideal_power_coeff = thrust_coeff ** 1.5 / (2 ** 0.5)
        
        if power_coeff > 0:
            figure_of_merit = ideal_power_coeff / power_coeff
        else:
            figure_of_merit = 0.0
        
        # Limit to reasonable range
        return min(0.9, max(0.3, figure_of_merit))
    
    def _calculate_thrust_asymmetry(self,
                                  angular_spacing: List[float],
                                  thrust_coeff: float) -> float:
        """Calculate thrust asymmetry factor from uneven spacing."""
        nominal_angle = 360.0 / len(angular_spacing)
        
        # Calculate thrust variation for each blade position
        thrust_variations = []
        for angle in angular_spacing:
            # Thrust varies with angular position due to uneven spacing
            angular_deviation = (angle - nominal_angle) / nominal_angle
            thrust_variation = angular_deviation * self.asymmetry_factors['thrust_variation_gain']
            thrust_variations.append(abs(thrust_variation))
        
        # RMS asymmetry factor
        asymmetry = np.sqrt(np.mean(np.array(thrust_variations) ** 2))
        
        return min(0.2, asymmetry)  # Limit to 20% maximum asymmetry
    
    def _calculate_vibration_amplitude(self,
                                     thrust_asymmetry: float,
                                     rotor_diameter: float,
                                     design_rpm: float) -> float:
        """Calculate vibration amplitude from thrust asymmetry."""
        
        # Base vibration from symmetric rotor
        base_vibration = 0.01 * (rotor_diameter ** 2) * ((design_rpm / 1000) ** 2)
        
        # Additional vibration from asymmetry
        asymmetry_vibration = thrust_asymmetry * self.asymmetry_factors['vibration_gain']
        
        total_vibration = base_vibration * (1.0 + asymmetry_vibration)
        
        return total_vibration  # N·m
    
    def _calculate_efficiency_degradation(self,
                                        figure_of_merit: float,
                                        asymmetry_factor: float) -> Tuple[float, float]:
        """Calculate efficiency and performance degradation."""
        
        # Base efficiency from figure of merit
        base_efficiency = figure_of_merit * self.aero_coefficients['ideal_efficiency']
        
        # Degradation due to asymmetry
        degradation = asymmetry_factor * self.asymmetry_factors['efficiency_loss_gain'] * 0.05
        
        actual_efficiency = base_efficiency * (1.0 - degradation)
        
        return actual_efficiency, degradation
    
    def _determine_operating_range(self,
                                 design_rpm: float,
                                 asymmetry_factor: float,
                                 tip_speed: float) -> Tuple[float, float]:
        """Determine recommended operating RPM range."""
        
        # Baseline operating range (±20% of design RPM)
        base_range = 0.20
        
        # Reduce range for higher asymmetry (more critical)
        range_reduction = asymmetry_factor * 0.1  # Up to 10% range reduction
        operating_range = base_range - range_reduction
        
        # Ensure tip speed doesn't exceed limits (typically 200 m/s for efficiency)
        max_tip_speed = 200.0
        max_rpm_tip_limit = max_tip_speed * 60 / (np.pi * tip_speed / design_rpm)
        
        min_rpm = max(design_rpm * (1 - operating_range), design_rpm * 0.6)
        max_rpm = min(design_rpm * (1 + operating_range), max_rpm_tip_limit)
        
        return (min_rpm, max_rpm)
    
    def _calculate_max_forward_speed(self,
                                   aero_results: AerodynamicAnalysisResult,
                                   disc_loading: float) -> float:
        """Calculate maximum forward flight speed."""
        
        # Base speed from disc loading (empirical relationship)
        base_speed = 25.0 - disc_loading * 0.1  # m/s
        
        # Reduction due to performance degradation
        speed_reduction = aero_results.performance_degradation / 100 * 0.3
        
        max_speed = base_speed * (1.0 - speed_reduction)
        
        return max(5.0, max_speed)  # Minimum 5 m/s
    
    def _calculate_max_climb_rate(self,
                                aero_results: AerodynamicAnalysisResult,
                                disc_loading: float) -> float:
        """Calculate maximum climb rate."""
        
        # Base climb rate from figure of merit and disc loading
        base_climb_rate = aero_results.figure_of_merit * 8.0 - disc_loading * 0.02
        
        # Reduction due to thrust asymmetry
        asymmetry_reduction = aero_results.thrust_asymmetry_factor * 0.5
        
        max_climb_rate = base_climb_rate * (1.0 - asymmetry_reduction)
        
        return max(1.0, max_climb_rate)  # Minimum 1 m/s
    
    def _calculate_wind_resistance(self,
                                 aero_results: AerodynamicAnalysisResult,
                                 max_forward_speed: float) -> float:
        """Calculate maximum wind resistance capability."""
        
        # Base wind resistance (typically 50% of max speed)
        base_wind_resistance = max_forward_speed * 0.5
        
        # Reduction due to vibration and asymmetry
        vibration_factor = min(0.2, aero_results.vibration_amplitude * 10)
        
        wind_resistance = base_wind_resistance * (1.0 - vibration_factor)
        
        return max(3.0, wind_resistance)  # Minimum 3 m/s wind resistance
    
    def _calculate_service_ceiling(self,
                                 aero_results: AerodynamicAnalysisResult,
                                 disc_loading: float) -> float:
        """Calculate service ceiling based on performance."""
        
        # Base ceiling from efficiency and disc loading
        base_ceiling = aero_results.efficiency_percentage * 10 - disc_loading * 2
        
        # Reduction due to performance degradation
        ceiling_reduction = aero_results.performance_degradation * 2
        
        service_ceiling = base_ceiling - ceiling_reduction
        
        return max(100.0, service_ceiling)  # Minimum 100m ceiling
    
    def _calculate_payload_limitation(self,
                                    aero_results: AerodynamicAnalysisResult,
                                    drone_weight: float,
                                    total_rotor_area: float) -> float:
        """Calculate payload limitation due to performance degradation."""
        
        # Base payload capacity (thrust margin)
        thrust_margin = (aero_results.thrust_coefficient * total_rotor_area * 1000) / 9.81
        base_payload = thrust_margin - drone_weight
        
        # Reduction due to asymmetry and degradation
        payload_reduction = (
            aero_results.performance_degradation / 100 * 0.5 +
            aero_results.thrust_asymmetry_factor * 0.3
        )
        
        limited_payload = base_payload * (1.0 - payload_reduction)
        
        return max(0.0, limited_payload)
    
    def compare_with_baseline(self,
                            optimized_results: AerodynamicAnalysisResult,
                            baseline_angular_spacing: List[float],
                            rotor_diameter: float,
                            design_rpm: float,
                            num_blades: int,
                            chord_length: float,
                            twist_distribution: List[float]) -> Dict:
        """
        Compare optimized design with symmetric baseline.
        
        Returns comparison metrics showing trade-offs.
        """
        # Analyze baseline (symmetric) configuration
        baseline_results = self.analyze_rotor_performance(
            baseline_angular_spacing,
            rotor_diameter,
            design_rpm,
            num_blades,
            chord_length,
            twist_distribution
        )
        
        # Calculate performance differences
        comparison = {
            'thrust_coefficient_change': (
                (optimized_results.thrust_coefficient - baseline_results.thrust_coefficient) /
                baseline_results.thrust_coefficient * 100
            ),
            'power_coefficient_change': (
                (optimized_results.power_coefficient - baseline_results.power_coefficient) /
                baseline_results.power_coefficient * 100
            ),
            'figure_of_merit_change': (
                (optimized_results.figure_of_merit - baseline_results.figure_of_merit) /
                baseline_results.figure_of_merit * 100
            ),
            'efficiency_change': (
                optimized_results.efficiency_percentage - baseline_results.efficiency_percentage
            ),
            'vibration_increase': (
                (optimized_results.vibration_amplitude - baseline_results.vibration_amplitude) /
                baseline_results.vibration_amplitude * 100
            ),
            'thrust_asymmetry_introduced': optimized_results.thrust_asymmetry_factor * 100,
            'overall_performance_impact': optimized_results.performance_degradation,
            'recommended_rpm_range_change': {
                'baseline_range': baseline_results.recommended_operating_range,
                'optimized_range': optimized_results.recommended_operating_range,
                'range_reduction_percent': (
                    (baseline_results.recommended_operating_range[1] - 
                     baseline_results.recommended_operating_range[0]) -
                    (optimized_results.recommended_operating_range[1] - 
                     optimized_results.recommended_operating_range[0])
                ) / (baseline_results.recommended_operating_range[1] - 
                     baseline_results.recommended_operating_range[0]) * 100
            }
        }
        
        return comparison
    
    def generate_performance_recommendations(self,
                                           aero_results: AerodynamicAnalysisResult,
                                           flight_envelope: FlightEnvelopeLimit) -> Dict:
        """Generate operational recommendations based on analysis results."""
        
        recommendations = {
            'operational_guidelines': {
                'preferred_rpm_range': f"{aero_results.recommended_operating_range[0]:.0f}-{aero_results.recommended_operating_range[1]:.0f} RPM",
                'max_safe_forward_speed': f"{flight_envelope.max_forward_speed:.1f} m/s",
                'recommended_cruise_speed': f"{flight_envelope.max_forward_speed * 0.7:.1f} m/s",
                'max_wind_conditions': f"{flight_envelope.max_wind_resistance:.1f} m/s",
                'service_ceiling': f"{flight_envelope.service_ceiling:.0f} m"
            },
            'performance_optimizations': {
                'flight_controller_tuning': 'Increase integral gains by 10-20% to compensate for thrust asymmetry',
                'vibration_management': 'Install vibration isolation for sensitive payloads' if aero_results.vibration_amplitude > 0.05 else 'Standard mounting acceptable',
                'power_management': f'Budget {aero_results.performance_degradation:.1f}% additional power consumption',
                'maintenance_schedule': 'Increased inspection frequency for dynamic components' if aero_results.thrust_asymmetry_factor > 0.05 else 'Standard maintenance schedule'
            },
            'design_trade_offs': {
                'acoustic_benefit': 'Estimated 3-5 dB noise reduction',
                'aerodynamic_cost': f"{aero_results.performance_degradation:.1f}% performance reduction",
                'efficiency_impact': f"{100 - aero_results.efficiency_percentage:.1f}% efficiency loss",
                'operational_complexity': 'Moderate increase in flight control complexity',
                'manufacturing_requirements': 'High precision manufacturing required'
            },
            'suitability_assessment': {
                'urban_operations': 'Excellent - noise reduction beneficial',
                'payload_missions': 'Good' if flight_envelope.payload_limitation > 1.0 else 'Limited',
                'long_endurance_flights': 'Good' if aero_results.efficiency_percentage > 75 else 'Fair',
                'precision_applications': 'Fair - vibration monitoring recommended',
                'commercial_viability': 'High for noise-sensitive applications'
            }
        }
        
        return recommendations