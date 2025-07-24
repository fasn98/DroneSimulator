"""
Advanced Aerodynamic Analysis Module

Implements sophisticated aerodynamic simulation and monitoring techniques
for high-efficiency drone performance optimization as outlined in the
technical specifications document.

Features:
- CL vs CD drag polar analysis
- Flight envelope efficiency mapping  
- Geometric impact assessment
- Stability and controllability derivatives
- Environmental simulation (wind gusts, Martian conditions)
"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as opt
from scipy import interpolate
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
import json


@dataclass
class AerodynamicCoefficients:
    """Aerodynamic coefficients for different flight conditions."""
    CL: float  # Lift coefficient
    CD: float  # Drag coefficient
    CM: float  # Moment coefficient
    L_D_ratio: float  # Lift-to-drag ratio
    alpha: float  # Angle of attack (degrees)


@dataclass
class StabilityDerivatives:
    """Stability and control derivatives."""
    CL_alpha: float  # Lift curve slope
    CD_alpha: float  # Drag curve slope
    CM_alpha: float  # Moment curve slope (static stability)
    CL_q: float     # Pitch damping derivative
    CM_q: float     # Pitch damping moment
    control_effectiveness: Dict[str, float]  # Control surface effectiveness


@dataclass
class FlightEnvelope:
    """Flight envelope characteristics."""
    mach_numbers: np.ndarray
    reynolds_numbers: np.ndarray
    efficiency_map: np.ndarray  # L/D efficiency surface
    max_efficiency_curve: np.ndarray
    compressibility_effects: Dict[str, float]


@dataclass
class WindGustProfile:
    """Wind gust modeling parameters."""
    time_array: np.ndarray
    velocity_profile: np.ndarray
    gust_type: str
    peak_velocity: float
    duration: float


class AdvancedAerodynamicAnalyzer:
    """Advanced aerodynamic analysis for high-efficiency drone simulation."""
    
    def __init__(self):
        self.analysis_results = {}
        self.polar_data = {}
        self.stability_data = {}
        
    def analyze_drag_polar(self, 
                          alpha_range: Tuple[float, float] = (-5, 20),
                          alpha_points: int = 26,
                          mach_number: float = 0.1,
                          reynolds_number: float = 1e6) -> List[AerodynamicCoefficients]:
        """
        Analyze CL vs CD relationship across angle of attack range.
        
        Args:
            alpha_range: Range of angles of attack (min, max) in degrees
            alpha_points: Number of analysis points
            mach_number: Mach number for analysis
            reynolds_number: Reynolds number for analysis
            
        Returns:
            List of aerodynamic coefficients for each angle of attack
        """
        alphas = np.linspace(alpha_range[0], alpha_range[1], alpha_points)
        coefficients = []
        
        for alpha in alphas:
            # Advanced aerodynamic modeling with realistic curves
            CL = self._calculate_lift_coefficient(alpha, mach_number, reynolds_number)
            CD = self._calculate_drag_coefficient(alpha, CL, mach_number, reynolds_number)
            CM = self._calculate_moment_coefficient(alpha, CL, mach_number)
            
            L_D_ratio = CL / CD if CD > 0 else 0
            
            coeff = AerodynamicCoefficients(
                CL=CL, CD=CD, CM=CM, L_D_ratio=L_D_ratio, alpha=alpha
            )
            coefficients.append(coeff)
        
        # Find optimal efficiency point
        max_ld_idx = np.argmax([c.L_D_ratio for c in coefficients])
        optimal_alpha = coefficients[max_ld_idx].alpha
        max_ld_ratio = coefficients[max_ld_idx].L_D_ratio
        
        self.polar_data[f"M{mach_number}_Re{reynolds_number:.0e}"] = {
            'coefficients': coefficients,
            'optimal_alpha': optimal_alpha,
            'max_ld_ratio': max_ld_ratio,
            'stall_alpha': self._find_stall_angle(coefficients),
            'zero_lift_alpha': self._find_zero_lift_angle(coefficients)
        }
        
        return coefficients
    
    def _calculate_lift_coefficient(self, alpha: float, mach: float, reynolds: float) -> float:
        """Calculate lift coefficient with advanced modeling."""
        # Linear region
        CL_alpha = 2 * np.pi  # Per radian, theoretical value
        alpha_rad = np.radians(alpha)
        alpha_0L = np.radians(-2.0)  # Zero-lift angle
        
        CL_linear = CL_alpha * (alpha_rad - alpha_0L)
        
        # Stall effects
        alpha_stall = np.radians(16.0)  # Typical stall angle
        if alpha_rad > alpha_stall:
            stall_factor = 1.0 - 0.8 * ((alpha_rad - alpha_stall) / np.radians(5.0))**2
            stall_factor = max(stall_factor, 0.2)
            CL_linear *= stall_factor
        
        # Compressibility effects (simplified)
        mach_factor = 1.0 / np.sqrt(1 - mach**2) if mach < 0.8 else 1.0 / np.sqrt(1 - 0.64)
        
        # Reynolds number effects
        re_factor = 1.0 + 0.1 * np.log10(reynolds / 1e6)
        
        return CL_linear * mach_factor * re_factor
    
    def _calculate_drag_coefficient(self, alpha: float, CL: float, mach: float, reynolds: float) -> float:
        """Calculate drag coefficient with advanced modeling."""
        # Zero-lift drag
        CD0 = 0.008 + 0.002 * mach**2  # Base drag with compressibility
        
        # Induced drag
        AR = 8.0  # Aspect ratio assumption
        e = 0.85  # Oswald efficiency factor
        CDi = (CL**2) / (np.pi * AR * e)
        
        # Viscous drag (Reynolds effects)
        re_factor = 1.0 - 0.05 * np.log10(reynolds / 1e6)
        re_factor = max(re_factor, 0.8)
        
        # Profile drag increase at high alpha
        alpha_rad = np.radians(abs(alpha))
        profile_factor = 1.0 + 0.5 * (alpha_rad / np.radians(15))**2
        
        # Wave drag (compressibility)
        if mach > 0.7:
            wave_drag = 0.01 * ((mach - 0.7) / 0.1)**2
        else:
            wave_drag = 0.0
        
        total_CD = (CD0 * profile_factor + CDi) * re_factor + wave_drag
        return max(total_CD, 0.005)  # Minimum drag floor
    
    def _calculate_moment_coefficient(self, alpha: float, CL: float, mach: float) -> float:
        """Calculate pitching moment coefficient."""
        # Basic moment coefficient (typically negative for stability)
        CM0 = -0.05  # Zero-lift moment
        CM_alpha = -0.05  # Moment curve slope (static stability)
        
        alpha_rad = np.radians(alpha)
        CM = CM0 + CM_alpha * alpha_rad
        
        # CL dependency (center of pressure movement)
        CM_CL = -0.08
        CM += CM_CL * CL
        
        return CM
    
    def calculate_stability_derivatives(self, 
                                     coefficients: List[AerodynamicCoefficients]) -> StabilityDerivatives:
        """Calculate stability and control derivatives from polar data."""
        alphas = [c.alpha for c in coefficients]
        CLs = [c.CL for c in coefficients]
        CDs = [c.CD for c in coefficients]
        CMs = [c.CM for c in coefficients]
        
        # Calculate derivatives using finite differences
        alpha_rad = np.radians(alphas)
        
        # Lift curve slope (per radian)
        CL_alpha = np.gradient(CLs, alpha_rad)
        CL_alpha_avg = np.mean(CL_alpha[5:15])  # Average in linear region
        
        # Drag curve slope
        CD_alpha = np.gradient(CDs, alpha_rad)
        CD_alpha_avg = np.mean(CD_alpha[5:15])
        
        # Moment curve slope (static stability indicator)
        CM_alpha = np.gradient(CMs, alpha_rad)
        CM_alpha_avg = np.mean(CM_alpha[5:15])
        
        # Pitch damping derivatives (estimated)
        CL_q = -CL_alpha_avg * 0.5  # Typical relationship
        CM_q = -CM_alpha_avg * 2.0  # Pitch damping
        
        # Control effectiveness (estimated for typical control surfaces)
        control_effectiveness = {
            'elevator': -0.6,  # Elevator effectiveness (CM per degree)
            'aileron': 0.08,   # Aileron effectiveness (CL per degree)
            'rudder': 0.05     # Rudder effectiveness (CY per degree)
        }
        
        return StabilityDerivatives(
            CL_alpha=CL_alpha_avg,
            CD_alpha=CD_alpha_avg,
            CM_alpha=CM_alpha_avg,
            CL_q=CL_q,
            CM_q=CM_q,
            control_effectiveness=control_effectiveness
        )
    
    def generate_flight_envelope(self, 
                               mach_range: Tuple[float, float] = (0.05, 0.8),
                               reynolds_range: Tuple[float, float] = (1e5, 1e7),
                               grid_points: int = 20) -> FlightEnvelope:
        """Generate flight envelope efficiency map."""
        mach_numbers = np.linspace(mach_range[0], mach_range[1], grid_points)
        reynolds_numbers = np.logspace(np.log10(reynolds_range[0]), 
                                     np.log10(reynolds_range[1]), grid_points)
        
        efficiency_map = np.zeros((len(reynolds_numbers), len(mach_numbers)))
        
        for i, re in enumerate(reynolds_numbers):
            for j, mach in enumerate(mach_numbers):
                # Calculate optimal L/D for this condition
                coeffs = self.analyze_drag_polar(mach_number=mach, reynolds_number=re)
                max_ld = max([c.L_D_ratio for c in coeffs])
                efficiency_map[i, j] = max_ld
        
        # Find maximum efficiency curve
        max_efficiency_curve = np.max(efficiency_map, axis=0)
        
        # Compressibility effects analysis
        compressibility_effects = {
            'critical_mach': 0.75,  # Mach number where drag rise begins
            'drag_rise_factor': 2.0,  # Factor by which drag increases
            'efficiency_loss': 0.3   # Fraction of efficiency lost
        }
        
        return FlightEnvelope(
            mach_numbers=mach_numbers,
            reynolds_numbers=reynolds_numbers,
            efficiency_map=efficiency_map,
            max_efficiency_curve=max_efficiency_curve,
            compressibility_effects=compressibility_effects
        )
    
    def generate_wind_gust_profile(self,
                                 total_time: float = 10.0,
                                 dt: float = 0.01,
                                 mean_velocity: float = 20.0,
                                 gust_amplitude: float = 15.0,
                                 gust_start_time: float = 2.0,
                                 gust_duration: float = 3.0,
                                 gust_type: str = "1-cosine") -> WindGustProfile:
        """Generate wind gust velocity profile for simulation."""
        time_array = np.arange(0, total_time, dt)
        velocity_profile = np.full_like(time_array, mean_velocity)
        
        gust_end_time = gust_start_time + gust_duration
        
        for i, t in enumerate(time_array):
            if gust_start_time <= t <= gust_end_time:
                if gust_type == "1-cosine":
                    # 1-cosine gust model
                    gust_velocity = (gust_amplitude / 2) * (
                        1 - np.cos(2 * np.pi * (t - gust_start_time) / gust_duration)
                    )
                elif gust_type == "ramp":
                    # Ramp gust model
                    ramp_factor = (t - gust_start_time) / gust_duration
                    gust_velocity = gust_amplitude * ramp_factor
                else:
                    # Step gust
                    gust_velocity = gust_amplitude
                
                velocity_profile[i] += gust_velocity
        
        peak_velocity = np.max(velocity_profile)
        
        return WindGustProfile(
            time_array=time_array,
            velocity_profile=velocity_profile,
            gust_type=gust_type,
            peak_velocity=peak_velocity,
            duration=gust_duration
        )
    
    def simulate_martian_environment(self,
                                   base_velocity: float = 30.0,
                                   dust_density: float = 0.001,  # kg/m³
                                   particle_size: float = 50e-6,  # 50 microns
                                   storm_intensity: float = 0.8) -> Dict:
        """Simulate Martian atmospheric conditions with dust storm."""
        # Martian atmospheric properties
        martian_properties = {
            'pressure': 610.0,  # Pa (0.6% of Earth)
            'density': 0.02,    # kg/m³ (1.6% of Earth)
            'gravity': 3.71,    # m/s² (38% of Earth)
            'temperature': 210, # K (-63°C average)
            'composition': {'CO2': 0.95, 'N2': 0.027, 'Ar': 0.016},
            'scale_height': 11.1  # km
        }
        
        # Dust storm effects
        dust_effects = {
            'visibility_reduction': storm_intensity * 0.9,  # Fraction
            'drag_increase_factor': 1.0 + storm_intensity * 0.3,
            'heating_effects': storm_intensity * 50,  # Additional K
            'particle_loading': dust_density * storm_intensity,
            'aerodynamic_roughness': particle_size * 1e6  # Roughness in microns
        }
        
        # Modified aerodynamic coefficients for dust storm
        modified_coefficients = {
            'CD_increase': storm_intensity * 0.15,  # Additional drag
            'CL_decrease': storm_intensity * 0.05,  # Lift reduction
            'surface_roughness_factor': 1.0 + storm_intensity * 0.1
        }
        
        # Particle dynamics (simplified Lagrangian model)
        particle_dynamics = {
            'terminal_velocity': self._calculate_particle_terminal_velocity(
                particle_size, dust_density, martian_properties['density']
            ),
            'drag_coefficient': self._calculate_particle_drag(particle_size),
            'settling_time': self._calculate_settling_time(particle_size, 1000)  # 1km altitude
        }
        
        return {
            'atmospheric_properties': martian_properties,
            'dust_effects': dust_effects,
            'modified_coefficients': modified_coefficients,
            'particle_dynamics': particle_dynamics,
            'effective_wind_velocity': base_velocity * (1 + storm_intensity * 0.4)
        }
    
    def _calculate_particle_terminal_velocity(self, diameter: float, 
                                            particle_density: float, 
                                            fluid_density: float) -> float:
        """Calculate terminal velocity of dust particles."""
        g = 3.71  # Martian gravity
        cd_sphere = 0.44  # Drag coefficient for sphere
        
        v_terminal = np.sqrt(
            (4 * g * diameter * particle_density) / 
            (3 * cd_sphere * fluid_density)
        )
        return v_terminal
    
    def _calculate_particle_drag(self, diameter: float) -> float:
        """Calculate drag coefficient for dust particles."""
        # Simplified drag model for small particles
        if diameter < 100e-6:  # Less than 100 microns
            return 24.0  # Stokes flow
        else:
            return 0.44  # Sphere in turbulent flow
    
    def _calculate_settling_time(self, diameter: float, height: float) -> float:
        """Calculate time for particles to settle from given height."""
        v_terminal = self._calculate_particle_terminal_velocity(
            diameter, 2500, 0.02  # Typical values
        )
        return height / v_terminal
    
    def _find_stall_angle(self, coefficients: List[AerodynamicCoefficients]) -> float:
        """Find stall angle from coefficient data."""
        CLs = [c.CL for c in coefficients]
        max_cl_idx = np.argmax(CLs)
        return coefficients[max_cl_idx].alpha
    
    def _find_zero_lift_angle(self, coefficients: List[AerodynamicCoefficients]) -> float:
        """Find zero-lift angle of attack."""
        CLs = [c.CL for c in coefficients]
        alphas = [c.alpha for c in coefficients]
        
        # Interpolate to find zero crossing
        f = interpolate.interp1d(CLs, alphas, kind='linear')
        try:
            return float(f(0.0))
        except:
            return -2.0  # Default estimate
    
    def plot_drag_polar(self, coefficients: List[AerodynamicCoefficients], 
                       save_path: str = None) -> None:
        """Plot CL vs CD drag polar."""
        CLs = [c.CL for c in coefficients]
        CDs = [c.CD for c in coefficients]
        alphas = [c.alpha for c in coefficients]
        
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))
        
        # Drag polar
        ax1.plot(CDs, CLs, 'b-o', linewidth=2, markersize=4)
        ax1.set_xlabel('CD (Drag Coefficient)')
        ax1.set_ylabel('CL (Lift Coefficient)')
        ax1.set_title('Drag Polar')
        ax1.grid(True, alpha=0.3)
        
        # L/D ratio vs alpha
        ld_ratios = [c.L_D_ratio for c in coefficients]
        ax2.plot(alphas, ld_ratios, 'r-o', linewidth=2, markersize=4)
        ax2.set_xlabel('Angle of Attack (degrees)')
        ax2.set_ylabel('L/D Ratio')
        ax2.set_title('Aerodynamic Efficiency')
        ax2.grid(True, alpha=0.3)
        
        # Mark maximum L/D
        max_ld_idx = np.argmax(ld_ratios)
        ax2.plot(alphas[max_ld_idx], ld_ratios[max_ld_idx], 'ro', 
                markersize=8, label=f'Max L/D = {ld_ratios[max_ld_idx]:.1f}')
        ax2.legend()
        
        # CL and CD vs alpha
        ax3.plot(alphas, CLs, 'b-', label='CL', linewidth=2)
        ax3.plot(alphas, CDs, 'r-', label='CD', linewidth=2)
        ax3.set_xlabel('Angle of Attack (degrees)')
        ax3.set_ylabel('Coefficient')
        ax3.set_title('CL and CD vs Alpha')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_flight_envelope(self, envelope: FlightEnvelope, save_path: str = None) -> None:
        """Plot flight envelope efficiency map."""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # 2D efficiency contour map
        M, Re = np.meshgrid(envelope.mach_numbers, envelope.reynolds_numbers)
        contour = ax1.contourf(M, Re, envelope.efficiency_map, levels=20, cmap='viridis')
        ax1.set_xlabel('Mach Number')
        ax1.set_ylabel('Reynolds Number')
        ax1.set_title('Flight Envelope Efficiency Map (L/D)')
        ax1.set_yscale('log')
        plt.colorbar(contour, ax=ax1, label='L/D Ratio')
        
        # Maximum efficiency curve
        ax2.plot(envelope.mach_numbers, envelope.max_efficiency_curve, 'b-o', linewidth=2)
        ax2.set_xlabel('Mach Number')
        ax2.set_ylabel('Maximum L/D Ratio')
        ax2.set_title('Peak Efficiency vs Mach Number')
        ax2.grid(True, alpha=0.3)
        
        # Mark critical Mach number
        critical_mach = envelope.compressibility_effects['critical_mach']
        ax2.axvline(critical_mach, color='r', linestyle='--', 
                   label=f'Critical Mach = {critical_mach}')
        ax2.legend()
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.show()
    
    def export_analysis_results(self, filename: str = "advanced_aerodynamic_analysis.json") -> None:
        """Export analysis results to JSON file."""
        export_data = {
            'polar_data': {},
            'analysis_metadata': {
                'analysis_type': 'advanced_aerodynamic',
                'created_by': 'AdvancedAerodynamicAnalyzer',
                'version': '1.0'
            }
        }
        
        # Convert complex data structures to serializable format
        for key, data in self.polar_data.items():
            export_data['polar_data'][key] = {
                'optimal_alpha': data['optimal_alpha'],
                'max_ld_ratio': data['max_ld_ratio'],
                'stall_alpha': data['stall_alpha'],
                'zero_lift_alpha': data['zero_lift_alpha'],
                'coefficients': [
                    {
                        'CL': c.CL,
                        'CD': c.CD,
                        'CM': c.CM,
                        'L_D_ratio': c.L_D_ratio,
                        'alpha': c.alpha
                    } for c in data['coefficients']
                ]
            }
        
        with open(filename, 'w') as f:
            json.dump(export_data, f, indent=2)
        
        print(f"Advanced aerodynamic analysis results exported to {filename}")


# Demonstration script for advanced aerodynamic analysis
def demonstrate_advanced_aerodynamics():
    """Demonstrate advanced aerodynamic analysis capabilities."""
    print("ADVANCED AERODYNAMIC ANALYSIS DEMONSTRATION")
    print("=" * 55)
    
    analyzer = AdvancedAerodynamicAnalyzer()
    
    # 1. Drag Polar Analysis
    print("1. Performing CL vs CD drag polar analysis...")
    coefficients = analyzer.analyze_drag_polar(
        alpha_range=(-5, 20),
        alpha_points=26,
        mach_number=0.15,
        reynolds_number=2e6
    )
    
    # Find key performance points
    max_ld_coeff = max(coefficients, key=lambda c: c.L_D_ratio)
    print(f"   ✓ Maximum L/D ratio: {max_ld_coeff.L_D_ratio:.1f} at α = {max_ld_coeff.alpha:.1f}°")
    print(f"   ✓ Optimal cruise efficiency at CL = {max_ld_coeff.CL:.3f}")
    
    # 2. Stability Analysis
    print("\n2. Calculating stability derivatives...")
    stability = analyzer.calculate_stability_derivatives(coefficients)
    print(f"   ✓ Lift curve slope: {stability.CL_alpha:.3f} /rad")
    print(f"   ✓ Static stability (CM_α): {stability.CM_alpha:.4f} /rad")
    print(f"   ✓ Pitch damping (CM_q): {stability.CM_q:.3f}")
    
    stability_status = "STABLE" if stability.CM_alpha < 0 else "UNSTABLE"
    print(f"   ✓ Stability status: {stability_status}")
    
    # 3. Flight Envelope
    print("\n3. Generating flight envelope efficiency map...")
    envelope = analyzer.generate_flight_envelope(
        mach_range=(0.05, 0.6),
        reynolds_range=(5e5, 5e6),
        grid_points=15
    )
    
    peak_efficiency = np.max(envelope.max_efficiency_curve)
    optimal_mach = envelope.mach_numbers[np.argmax(envelope.max_efficiency_curve)]
    print(f"   ✓ Peak efficiency: L/D = {peak_efficiency:.1f}")
    print(f"   ✓ Optimal Mach number: {optimal_mach:.2f}")
    print(f"   ✓ Critical Mach number: {envelope.compressibility_effects['critical_mach']}")
    
    # 4. Wind Gust Analysis
    print("\n4. Generating wind gust profile...")
    gust_profile = analyzer.generate_wind_gust_profile(
        total_time=8.0,
        mean_velocity=25.0,
        gust_amplitude=12.0,
        gust_duration=2.5,
        gust_type="1-cosine"
    )
    print(f"   ✓ Base wind speed: {gust_profile.velocity_profile[0]:.1f} m/s")
    print(f"   ✓ Peak gust speed: {gust_profile.peak_velocity:.1f} m/s")
    print(f"   ✓ Gust intensity: {(gust_profile.peak_velocity/25.0-1)*100:.1f}% above baseline")
    
    # 5. Martian Environment Simulation
    print("\n5. Simulating Martian dust storm conditions...")
    martian_sim = analyzer.simulate_martian_environment(
        base_velocity=40.0,
        storm_intensity=0.7
    )
    
    mars_props = martian_sim['atmospheric_properties']
    dust_effects = martian_sim['dust_effects']
    
    print(f"   ✓ Martian atmospheric density: {mars_props['density']:.3f} kg/m³")
    print(f"   ✓ Atmospheric pressure: {mars_props['pressure']:.0f} Pa")
    print(f"   ✓ Visibility reduction: {dust_effects['visibility_reduction']*100:.0f}%")
    print(f"   ✓ Drag increase: {(dust_effects['drag_increase_factor']-1)*100:.0f}%")
    print(f"   ✓ Effective wind speed: {martian_sim['effective_wind_velocity']:.1f} m/s")
    
    # 6. Generate Visualizations
    print("\n6. Generating analysis visualizations...")
    analyzer.plot_drag_polar(coefficients, save_path="advanced_drag_polar.png")
    analyzer.plot_flight_envelope(envelope, save_path="advanced_flight_envelope.png")
    
    # 7. Export Results
    print("\n7. Exporting analysis results...")
    analyzer.export_analysis_results("advanced_aerodynamic_analysis.json")
    
    print("\n" + "=" * 55)
    print("ADVANCED AERODYNAMIC ANALYSIS COMPLETE")
    print("=" * 55)
    print("Key Features Demonstrated:")
    print("✓ High-fidelity CL vs CD drag polar analysis")
    print("✓ Stability and controllability derivatives")
    print("✓ Multi-dimensional flight envelope mapping")
    print("✓ Environmental simulation (wind gusts)")
    print("✓ Martian atmospheric conditions with dust storms")
    print("✓ Professional visualization and data export")
    
    return analyzer


if __name__ == "__main__":
    analyzer = demonstrate_advanced_aerodynamics()