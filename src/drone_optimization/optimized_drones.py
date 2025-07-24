"""
Optimized Drone Factory

Creates professional drone configurations with advanced acoustic and aerodynamic
optimization features. Generates drone models suitable for safe flight operations
with reduced noise signatures.
"""

import json
from typing import Dict, List, Optional
from .rotor_designer import RotorDesigner, RotorSpecification
from .acoustic_optimizer import AcousticOptimizer
from .aerodynamic_analyzer import AerodynamicAnalyzer


class OptimizedDroneFactory:
    """
    Factory class for creating acoustically and aerodynamically optimized drone configurations.
    
    Generates professional drone models with research-based optimization for noise reduction
    while maintaining safe flight characteristics and performance.
    """
    
    def __init__(self):
        """Initialize the drone factory with optimization components."""
        self.rotor_designer = RotorDesigner()
        self.acoustic_optimizer = AcousticOptimizer()
        self.aerodynamic_analyzer = AerodynamicAnalyzer()
        
        # Professional drone templates
        self.drone_templates = {
            'professional_quadcopter': {
                'num_rotors': 4,
                'frame_size': 'medium',
                'payload_capacity': 2.0,  # kg
                'flight_time': 25.0,     # minutes
                'max_speed': 15.0,       # m/s
                'service_ceiling': 500.0  # meters
            },
            'acoustic_optimized_hex': {
                'num_rotors': 6,
                'frame_size': 'large',
                'payload_capacity': 5.0,
                'flight_time': 30.0,
                'max_speed': 12.0,
                'service_ceiling': 400.0
            },
            'precision_octocopter': {
                'num_rotors': 8,
                'frame_size': 'large',
                'payload_capacity': 8.0,
                'flight_time': 35.0,
                'max_speed': 10.0,
                'service_ceiling': 300.0
            }
        }
    
    def create_optimized_drone(self, template_name: str, optimization_profile: str = 'balanced') -> Dict:
        """
        Create an optimized drone configuration based on template and optimization profile.
        
        Args:
            template_name: Name of drone template ('professional_quadcopter', etc.)
            optimization_profile: 'acoustic', 'aerodynamic', or 'balanced'
            
        Returns:
            Complete drone configuration dictionary
        """
        if template_name not in self.drone_templates:
            raise ValueError(f"Unknown template: {template_name}")
        
        template = self.drone_templates[template_name]
        
        # Design optimized rotors
        rotor_specs = self._design_rotors_for_template(template, optimization_profile)
        
        # Calculate performance characteristics
        performance = self._calculate_drone_performance(template, rotor_specs)
        
        # Generate flight control parameters
        flight_control = self._generate_flight_control_params(template, rotor_specs)
        
        # Create complete drone configuration
        drone_config = {
            'model_name': f"{template_name}_optimized_{optimization_profile}",
            'description': f"Professional {template_name} with {optimization_profile} optimization",
            'template_base': template_name,
            'optimization_profile': optimization_profile,
            'physical_properties': self._generate_physical_properties(template),
            'rotor_specifications': self._convert_rotor_specs_to_dict(rotor_specs),
            'performance_characteristics': performance,
            'flight_control_parameters': flight_control,
            'acoustic_metrics': self._calculate_acoustic_metrics(rotor_specs),
            'safety_features': self._generate_safety_features(template),
            'operational_envelope': self._define_operational_envelope(template, performance)
        }
        
        return drone_config
    
    def _design_rotors_for_template(self, template: Dict, optimization_profile: str) -> List[RotorSpecification]:
        """Design optimized rotors for the given drone template."""
        num_rotors = template['num_rotors']
        
        # Base rotor design parameters based on template
        base_rotor_design = self._get_base_rotor_design(template)
        
        # Create optimized rotor specifications
        rotor_specs = []
        for i in range(num_rotors):
            spec = self.rotor_designer.design_optimized_rotor(
                base_rotor_design, 
                optimization_profile
            )
            rotor_specs.append(spec)
        
        return rotor_specs
    
    def _get_base_rotor_design(self, template: Dict) -> Dict:
        """Generate base rotor design parameters from drone template."""
        frame_size = template['frame_size']
        payload = template['payload_capacity']
        
        # Scale rotor parameters based on frame size and payload
        if frame_size == 'medium':
            diameter = 0.25 if payload <= 3.0 else 0.30
            rpm = 6000 if payload <= 3.0 else 5500
            chord = 0.020
        elif frame_size == 'large':
            diameter = 0.35 if payload <= 6.0 else 0.40
            rpm = 5000 if payload <= 6.0 else 4500
            chord = 0.025
        else:  # small
            diameter = 0.20
            rpm = 7000
            chord = 0.015
        
        return {
            'num_blades': 4,  # Standard for efficiency
            'diameter': diameter,
            'chord_length': chord,
            'rpm': rpm,
            'twist_distribution': [0, -3, -6, -9, -12],  # Linear twist
            'airfoil': 'NACA2412',
            'tolerance': 0.008  # High precision for professional applications
        }
    
    def _calculate_drone_performance(self, template: Dict, rotor_specs: List[RotorSpecification]) -> Dict:
        """Calculate overall drone performance based on optimized rotors."""
        # Estimate total thrust capability
        single_rotor_thrust = self._estimate_rotor_thrust(rotor_specs[0])
        total_thrust = single_rotor_thrust * len(rotor_specs)
        
        # Calculate power requirements
        single_rotor_power = self._estimate_rotor_power(rotor_specs[0])
        total_power = single_rotor_power * len(rotor_specs)
        
        # Performance calculations
        hover_thrust_to_weight = 2.0  # Safety factor
        estimated_weight = total_thrust / hover_thrust_to_weight
        
        return {
            'total_thrust_capacity': round(total_thrust, 2),  # N
            'hover_power_required': round(total_power, 1),    # W
            'estimated_weight': round(estimated_weight, 2),   # kg
            'thrust_to_weight_ratio': round(hover_thrust_to_weight, 2),
            'power_loading': round(total_thrust / total_power * 1000, 1),  # g/W
            'disc_loading': round(estimated_weight * 9.81 / (len(rotor_specs) * 3.14159 * (rotor_specs[0].diameter/2)**2), 1),  # N/m²
            'figure_of_merit': 0.75,  # Optimized rotors achieve higher FoM
            'efficiency_rating': 'High',
            'noise_reduction_level': '3-5 dB below standard designs'
        }
    
    def _estimate_rotor_thrust(self, rotor_spec: RotorSpecification) -> float:
        """Estimate single rotor thrust capacity."""
        # Simplified thrust estimation based on disc area and tip speed
        disc_area = 3.14159 * (rotor_spec.diameter / 2) ** 2
        tip_speed_factor = min(1.0, rotor_spec.tip_speed / 200.0)  # Limit for efficiency
        
        # Thrust coefficient based on rotor design
        thrust_coefficient = 0.012  # Typical for optimized rotors
        air_density = 1.225  # kg/m³ at sea level
        
        thrust = thrust_coefficient * air_density * disc_area * rotor_spec.tip_speed ** 2
        return thrust
    
    def _estimate_rotor_power(self, rotor_spec: RotorSpecification) -> float:
        """Estimate single rotor power requirement."""
        thrust = self._estimate_rotor_thrust(rotor_spec)
        disc_area = 3.14159 * (rotor_spec.diameter / 2) ** 2
        
        # Momentum theory power calculation
        air_density = 1.225
        ideal_power = thrust ** 1.5 / (2 * air_density * disc_area) ** 0.5
        
        # Account for losses and figure of merit
        figure_of_merit = 0.75  # Optimized design
        actual_power = ideal_power / figure_of_merit
        
        return actual_power
    
    def _generate_flight_control_params(self, template: Dict, rotor_specs: List[RotorSpecification]) -> Dict:
        """Generate flight control parameters optimized for uneven rotor spacing."""
        # Account for acoustic optimization in flight control
        has_uneven_spacing = any(
            abs(max(spec.step_angles) - min(spec.step_angles)) > 1.0 
            for spec in rotor_specs
        )
        
        base_gains = {
            'roll_kp': 0.8,
            'roll_ki': 0.1,
            'roll_kd': 0.05,
            'pitch_kp': 0.8,
            'pitch_ki': 0.1,
            'pitch_kd': 0.05,
            'yaw_kp': 1.2,
            'yaw_ki': 0.15,
            'yaw_kd': 0.02
        }
        
        # Adjust gains for uneven rotor spacing
        if has_uneven_spacing:
            # Slightly increase integral gains to handle asymmetric forces
            base_gains['roll_ki'] *= 1.2
            base_gains['pitch_ki'] *= 1.2
            base_gains['yaw_ki'] *= 1.1
        
        return {
            'control_gains': base_gains,
            'rate_limits': {
                'max_roll_rate': 200.0,   # deg/s
                'max_pitch_rate': 200.0,  # deg/s  
                'max_yaw_rate': 150.0     # deg/s
            },
            'attitude_limits': {
                'max_tilt_angle': 30.0,   # degrees
                'max_vertical_speed': 5.0, # m/s
                'max_horizontal_speed': template['max_speed']
            },
            'stabilization_features': {
                'vibration_compensation': has_uneven_spacing,
                'dynamic_balancing': True,
                'wind_resistance': 'enhanced',
                'gps_hold_precision': '±0.5m'
            }
        }
    
    def _convert_rotor_specs_to_dict(self, rotor_specs: List[RotorSpecification]) -> List[Dict]:
        """Convert rotor specifications to dictionary format."""
        rotor_dicts = []
        for i, spec in enumerate(rotor_specs):
            rotor_dict = {
                'rotor_id': i + 1,
                'num_blades': spec.num_blades,
                'diameter': spec.diameter,
                'chord_length': spec.chord_length,
                'twist_distribution': spec.twist_distribution,
                'step_angles': spec.step_angles,
                'airfoil_profile': spec.airfoil_profile,
                'tip_speed': spec.tip_speed,
                'design_rpm': spec.design_rpm,
                'acoustic_optimization': spec.acoustic_optimization,
                'manufacturing_tolerance': spec.manufacturing_tolerance
            }
            rotor_dicts.append(rotor_dict)
        
        return rotor_dicts
    
    def _generate_physical_properties(self, template: Dict) -> Dict:
        """Generate physical properties for the drone."""
        frame_size = template['frame_size']
        
        # Scale dimensions based on frame size
        if frame_size == 'small':
            dimensions = {'length': 0.35, 'width': 0.35, 'height': 0.12}
            weight = 1.2
        elif frame_size == 'medium':
            dimensions = {'length': 0.55, 'width': 0.55, 'height': 0.18}
            weight = 2.5
        else:  # large
            dimensions = {'length': 0.85, 'width': 0.85, 'height': 0.25}
            weight = 4.5
        
        return {
            'dimensions': dimensions,  # meters
            'dry_weight': weight,      # kg
            'max_takeoff_weight': weight + template['payload_capacity'],
            'center_of_gravity': {'x': 0.0, 'y': 0.0, 'z': -0.05},
            'moment_of_inertia': {
                'Ixx': weight * 0.1,
                'Iyy': weight * 0.1, 
                'Izz': weight * 0.15
            },
            'materials': {
                'frame': 'Carbon fiber composite',
                'rotors': 'Carbon fiber with aluminum hub',
                'electronics_housing': 'Aluminum alloy'
            }
        }
    
    def _calculate_acoustic_metrics(self, rotor_specs: List[RotorSpecification]) -> Dict:
        """Calculate overall acoustic performance metrics."""
        # Analyze acoustic performance of each rotor
        total_noise_reduction = 0.0
        max_perceived_noise = 0.0
        
        for spec in rotor_specs:
            acoustic_metrics = self.rotor_designer.analyze_acoustic_performance(spec)
            total_noise_reduction += acoustic_metrics.discrete_tone_reduction
            max_perceived_noise = max(max_perceived_noise, acoustic_metrics.perceived_noise_level)
        
        avg_noise_reduction = total_noise_reduction / len(rotor_specs)
        
        return {
            'average_discrete_tone_reduction': round(avg_noise_reduction, 1),  # dB
            'maximum_perceived_noise_level': round(max_perceived_noise, 1),   # dB
            'noise_signature': 'Optimized broadband distribution',
            'acoustic_certification': 'Suitable for urban operations',
            'comparison_to_standard': f"{avg_noise_reduction:.1f} dB quieter than conventional designs"
        }
    
    def _generate_safety_features(self, template: Dict) -> Dict:
        """Generate safety features appropriate for the drone configuration."""
        return {
            'redundancy_systems': {
                'dual_gps': True,
                'backup_imu': True,
                'failsafe_landing': True,
                'motor_failure_compensation': template['num_rotors'] > 4
            },
            'autonomous_features': {
                'return_to_home': True,
                'obstacle_avoidance': True,
                'low_battery_landing': True,
                'geofencing': True
            },
            'monitoring_systems': {
                'real_time_telemetry': True,
                'vibration_monitoring': True,
                'motor_health_tracking': True,
                'battery_management': 'Advanced BMS with cell monitoring'
            },
            'emergency_procedures': {
                'auto_landing_on_failure': True,
                'emergency_stop': True,
                'manual_override': True,
                'parachute_system': template['payload_capacity'] > 3.0
            }
        }
    
    def _define_operational_envelope(self, template: Dict, performance: Dict) -> Dict:
        """Define safe operational envelope for the optimized drone."""
        return {
            'altitude_limits': {
                'minimum_altitude': 5.0,     # meters AGL
                'maximum_altitude': template['service_ceiling'],
                'recommended_cruise': template['service_ceiling'] * 0.6
            },
            'speed_limits': {
                'maximum_horizontal': template['max_speed'],
                'maximum_vertical_up': 5.0,
                'maximum_vertical_down': 3.0,
                'recommended_cruise': template['max_speed'] * 0.7
            },
            'environmental_limits': {
                'max_wind_speed': 12.0,      # m/s
                'operating_temperature': {'min': -10, 'max': 50},  # °C
                'maximum_humidity': 90,      # %
                'precipitation': 'Light rain acceptable with proper sealing'
            },
            'payload_limits': {
                'maximum_payload': template['payload_capacity'],
                'center_of_gravity_tolerance': '±5cm from geometric center',
                'vibration_sensitive_payloads': 'Suitable due to acoustic optimization'
            },
            'flight_time_performance': {
                'hover_endurance': template['flight_time'],
                'cruise_endurance': template['flight_time'] * 1.3,
                'range_at_cruise': template['flight_time'] * 1.3 * template['max_speed'] * 0.7 / 60,  # km
                'battery_reserve': '20% minimum for safety'
            }
        }
    
    def save_drone_configuration(self, drone_config: Dict, filename: str) -> None:
        """Save drone configuration to JSON file."""
        with open(filename, 'w') as f:
            json.dump(drone_config, f, indent=2)
    
    def get_available_templates(self) -> List[str]:
        """Get list of available drone templates."""
        return list(self.drone_templates.keys())
    
    def get_optimization_profiles(self) -> List[str]:
        """Get list of available optimization profiles."""
        return ['acoustic', 'aerodynamic', 'balanced']