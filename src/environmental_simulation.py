"""
Environmental Simulation Module

Advanced environmental simulation capabilities for extreme conditions
including wind gusts, climate changes, and Martian dust storms as
outlined in the technical specifications.

Features:
- Dynamic wind gust modeling
- Martian atmospheric conditions with dust storms
- Time-dependent environmental parameters
- Particle dynamics simulation
- Environmental impact on aerodynamic performance
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
import json


@dataclass
class EnvironmentalConditions:
    """Environmental conditions for simulation."""
    temperature: float  # K
    pressure: float    # Pa
    density: float     # kg/m³
    gravity: float     # m/s²
    wind_velocity: np.ndarray  # m/s [x, y, z]
    visibility: float  # m
    humidity: float    # fraction


@dataclass
class ParticleProperties:
    """Properties of airborne particles (dust, sand, etc.)."""
    diameter: float          # m
    density: float          # kg/m³
    concentration: float    # particles/m³
    terminal_velocity: float # m/s
    drag_coefficient: float


@dataclass
class StormParameters:
    """Parameters for storm simulation."""
    intensity: float        # 0-1 scale
    particle_loading: float # kg/m³
    wind_speed_factor: float
    visibility_factor: float
    duration: float         # seconds


class EnvironmentalSimulator:
    """Advanced environmental simulation for drone testing."""
    
    def __init__(self):
        self.current_conditions = None
        self.time_history = []
        self.environmental_effects = {}
        
    def create_martian_atmosphere(self, 
                                altitude: float = 0.0,
                                season: str = "winter",
                                dust_storm: bool = False,
                                storm_intensity: float = 0.5) -> EnvironmentalConditions:
        """Create Martian atmospheric conditions."""
        
        # Base Martian atmospheric properties
        base_pressure = 610.0  # Pa at surface
        base_density = 0.020   # kg/m³ at surface
        base_temperature = 210 # K (-63°C)
        
        # Altitude scaling (exponential atmosphere)
        scale_height = 11100  # m for Mars
        pressure = base_pressure * np.exp(-altitude / scale_height)
        density = base_density * np.exp(-altitude / scale_height)
        
        # Temperature variation with altitude and season
        lapse_rate = -0.0045  # K/m (Mars troposphere)
        seasonal_variation = {
            "spring": 10,
            "summer": 25,
            "autumn": 5,
            "winter": -15
        }
        
        temperature = base_temperature + lapse_rate * altitude + seasonal_variation.get(season, 0)
        
        # Base wind conditions
        base_wind = np.array([15.0, 2.0, 0.5])  # m/s [x, y, z]
        
        # Dust storm effects
        if dust_storm:
            storm_effects = self._apply_dust_storm_effects(
                pressure, density, temperature, base_wind, storm_intensity
            )
            pressure *= storm_effects['pressure_factor']
            density *= storm_effects['density_factor']
            temperature += storm_effects['temperature_increase']
            base_wind *= storm_effects['wind_factor']
            visibility = storm_effects['visibility']
        else:
            visibility = 50000.0  # Clear conditions, 50 km
        
        return EnvironmentalConditions(
            temperature=temperature,
            pressure=pressure,
            density=density,
            gravity=3.71,  # Mars gravity
            wind_velocity=base_wind,
            visibility=visibility,
            humidity=0.001  # Very low humidity on Mars
        )
    
    def _apply_dust_storm_effects(self, pressure: float, density: float, 
                                temperature: float, wind: np.ndarray, 
                                intensity: float) -> Dict:
        """Apply dust storm effects to atmospheric conditions."""
        
        # Storm intensity effects (0.0 to 1.0 scale)
        effects = {
            'pressure_factor': 1.0 + intensity * 0.05,  # Slight pressure increase
            'density_factor': 1.0 + intensity * 0.15,   # Dust loading
            'temperature_increase': intensity * 80,      # Heating from dust
            'wind_factor': 1.0 + intensity * 2.0,       # Wind speed increase
            'visibility': 10000 * (1.0 - intensity * 0.95)  # Visibility reduction
        }
        
        return effects
    
    def simulate_dust_particles(self, 
                              storm_params: StormParameters,
                              atmospheric_conditions: EnvironmentalConditions) -> ParticleProperties:
        """Simulate dust particle behavior in Martian atmosphere."""
        
        # Typical Martian dust properties
        particle_diameter = 2.0e-6  # 2 microns (fine dust)
        particle_density = 2500.0   # kg/m³ (basalt-like)
        
        # Calculate terminal velocity in Martian atmosphere
        cd_particle = self._calculate_particle_drag_coefficient(
            particle_diameter, atmospheric_conditions.density
        )
        
        terminal_velocity = np.sqrt(
            (4 * atmospheric_conditions.gravity * particle_diameter * particle_density) /
            (3 * cd_particle * atmospheric_conditions.density)
        )
        
        # Particle concentration based on storm intensity
        base_concentration = 1e6  # particles/m³ in light dust
        concentration = base_concentration * (1 + storm_params.intensity * 100)
        
        return ParticleProperties(
            diameter=particle_diameter,
            density=particle_density,
            concentration=concentration,
            terminal_velocity=terminal_velocity,
            drag_coefficient=cd_particle
        )
    
    def _calculate_particle_drag_coefficient(self, diameter: float, fluid_density: float) -> float:
        """Calculate drag coefficient for spherical particles."""
        # Reynolds number for particle
        viscosity = 1.3e-5  # Martian atmospheric viscosity (approximation)
        velocity = 10.0     # Typical relative velocity
        
        re_particle = fluid_density * velocity * diameter / viscosity
        
        # Drag coefficient correlation for spheres
        if re_particle < 0.1:
            cd = 24.0 / re_particle  # Stokes flow
        elif re_particle < 1000:
            cd = 24.0 / re_particle * (1 + 0.15 * re_particle**0.687)
        else:
            cd = 0.44  # Turbulent flow
        
        return cd
    
    def generate_dynamic_wind_field(self,
                                  duration: float = 30.0,
                                  dt: float = 0.1,
                                  base_wind: np.ndarray = np.array([20.0, 5.0, 1.0]),
                                  turbulence_intensity: float = 0.2,
                                  gust_probability: float = 0.1) -> Dict:
        """Generate time-varying wind field with gusts and turbulence."""
        
        time_steps = np.arange(0, duration, dt)
        wind_history = np.zeros((len(time_steps), 3))
        gust_events = []
        
        for i, t in enumerate(time_steps):
            # Base wind with slow variations
            wind_variation = 1.0 + 0.1 * np.sin(2 * np.pi * t / 10.0)  # 10-second cycle
            current_wind = base_wind * wind_variation
            
            # Add turbulence (high-frequency fluctuations)
            turbulence = np.random.normal(0, turbulence_intensity * np.linalg.norm(base_wind), 3)
            current_wind += turbulence
            
            # Random gust events
            if np.random.random() < gust_probability * dt:
                gust_magnitude = np.random.uniform(0.5, 2.0) * np.linalg.norm(base_wind)
                gust_direction = np.random.uniform(-np.pi, np.pi)
                gust_duration = np.random.uniform(1.0, 5.0)
                
                gust_events.append({
                    'start_time': t,
                    'duration': gust_duration,
                    'magnitude': gust_magnitude,
                    'direction': gust_direction
                })
            
            # Apply active gusts
            for gust in gust_events:
                if gust['start_time'] <= t <= gust['start_time'] + gust['duration']:
                    # Cosine-shaped gust profile
                    gust_phase = (t - gust['start_time']) / gust['duration']
                    gust_factor = 0.5 * (1 - np.cos(2 * np.pi * gust_phase))
                    
                    gust_velocity = gust['magnitude'] * gust_factor
                    gust_vector = np.array([
                        gust_velocity * np.cos(gust['direction']),
                        gust_velocity * np.sin(gust['direction']),
                        0.1 * gust_velocity  # Small vertical component
                    ])
                    
                    current_wind += gust_vector
            
            wind_history[i] = current_wind
        
        return {
            'time_steps': time_steps,
            'wind_history': wind_history,
            'gust_events': gust_events,
            'statistics': {
                'mean_wind_speed': np.mean(np.linalg.norm(wind_history, axis=1)),
                'max_wind_speed': np.max(np.linalg.norm(wind_history, axis=1)),
                'turbulence_intensity': np.std(np.linalg.norm(wind_history, axis=1)) / np.mean(np.linalg.norm(wind_history, axis=1)),
                'gust_count': len(gust_events)
            }
        }
    
    def calculate_environmental_aerodynamic_effects(self,
                                                  conditions: EnvironmentalConditions,
                                                  particles: ParticleProperties = None) -> Dict:
        """Calculate how environmental conditions affect aerodynamic performance."""
        
        # Density effects on lift and drag
        density_ratio = conditions.density / 1.225  # Ratio to sea-level Earth
        
        # Reynolds number effects
        reference_velocity = 25.0  # m/s
        reference_chord = 0.2      # m
        kinematic_viscosity = self._calculate_kinematic_viscosity(
            conditions.temperature, conditions.pressure
        )
        
        reynolds_number = reference_velocity * reference_chord / kinematic_viscosity
        earth_reynolds = reference_velocity * reference_chord / 1.5e-5  # Earth comparison
        reynolds_ratio = reynolds_number / earth_reynolds
        
        # Base aerodynamic effects
        aerodynamic_effects = {
            'lift_factor': density_ratio,  # Direct proportionality
            'drag_factor': density_ratio,  # Direct proportionality
            'reynolds_effects': {
                'reynolds_number': reynolds_number,
                'reynolds_ratio': reynolds_ratio,
                'boundary_layer_factor': 1.0 + 0.1 * np.log10(reynolds_ratio)
            },
            'compressibility_effects': {
                'mach_number': reference_velocity / self._calculate_speed_of_sound(conditions),
                'compressibility_factor': 1.0  # Placeholder for advanced calculation
            }
        }
        
        # Particle effects (if dust storm)
        if particles:
            particle_effects = self._calculate_particle_aerodynamic_effects(
                conditions, particles
            )
            aerodynamic_effects['particle_effects'] = particle_effects
        
        # Wind effects on relative velocity
        wind_magnitude = np.linalg.norm(conditions.wind_velocity)
        aerodynamic_effects['wind_effects'] = {
            'wind_magnitude': wind_magnitude,
            'relative_velocity_factor': 1.0 + wind_magnitude / reference_velocity,
            'crosswind_component': conditions.wind_velocity[1],  # Lateral wind
            'headwind_component': conditions.wind_velocity[0],   # Longitudinal wind
            'updraft_component': conditions.wind_velocity[2]     # Vertical wind
        }
        
        return aerodynamic_effects
    
    def _calculate_particle_aerodynamic_effects(self,
                                              conditions: EnvironmentalConditions,
                                              particles: ParticleProperties) -> Dict:
        """Calculate aerodynamic effects of suspended particles."""
        
        # Mass loading effect on density
        particle_mass_fraction = (particles.concentration * particles.density * 
                                 (4/3 * np.pi * (particles.diameter/2)**3))
        effective_density_increase = particle_mass_fraction / conditions.density
        
        # Surface roughness effect
        roughness_height = particles.diameter * 0.1  # Equivalent sand grain roughness
        roughness_factor = 1.0 + 0.3 * np.log10(1 + roughness_height * 1e6)  # Factor for CD increase
        
        # Erosion and fouling effects
        erosion_factor = 1.0 + 0.05 * np.log10(1 + particles.concentration / 1e6)
        
        return {
            'density_increase': effective_density_increase,
            'roughness_factor': roughness_factor,
            'erosion_factor': erosion_factor,
            'additional_drag': 0.02 * effective_density_increase,  # Simplified model
            'particle_loading': particle_mass_fraction
        }
    
    def _calculate_kinematic_viscosity(self, temperature: float, pressure: float) -> float:
        """Calculate kinematic viscosity from temperature and pressure."""
        # Sutherland's law for viscosity (simplified for CO2-dominant atmosphere)
        T0 = 273.15  # Reference temperature
        mu0 = 1.716e-5  # Reference viscosity for air
        S = 110.4  # Sutherland constant
        
        dynamic_viscosity = mu0 * (temperature / T0)**(3/2) * (T0 + S) / (temperature + S)
        
        # Calculate density from ideal gas law
        R_specific = 188.9  # Specific gas constant for CO2 (J/kg·K)
        density = pressure / (R_specific * temperature)
        
        return dynamic_viscosity / density
    
    def _calculate_speed_of_sound(self, conditions: EnvironmentalConditions) -> float:
        """Calculate speed of sound in given atmospheric conditions."""
        gamma = 1.30  # Heat capacity ratio for CO2
        R_specific = 188.9  # Specific gas constant for CO2
        
        return np.sqrt(gamma * R_specific * conditions.temperature)
    
    def simulate_extreme_weather_scenario(self,
                                        scenario_type: str = "martian_dust_storm",
                                        duration: float = 600.0,  # 10 minutes
                                        dt: float = 1.0) -> Dict:
        """Simulate extreme weather scenarios for drone testing."""
        
        time_steps = np.arange(0, duration, dt)
        scenario_data = {
            'time_steps': time_steps,
            'conditions_history': [],
            'aerodynamic_effects_history': [],
            'scenario_type': scenario_type
        }
        
        for i, t in enumerate(time_steps):
            if scenario_type == "martian_dust_storm":
                # Progressive dust storm intensity
                storm_intensity = min(0.8, 0.1 + 0.7 * t / duration)
                
                conditions = self.create_martian_atmosphere(
                    altitude=1000.0,
                    season="summer",
                    dust_storm=True,
                    storm_intensity=storm_intensity
                )
                
                storm_params = StormParameters(
                    intensity=storm_intensity,
                    particle_loading=0.001 * storm_intensity,
                    wind_speed_factor=1 + 2 * storm_intensity,
                    visibility_factor=1 - 0.9 * storm_intensity,
                    duration=duration
                )
                
                particles = self.simulate_dust_particles(storm_params, conditions)
                
            elif scenario_type == "earth_severe_turbulence":
                # Earth atmosphere with severe turbulence
                base_conditions = EnvironmentalConditions(
                    temperature=288.15,  # 15°C
                    pressure=101325.0,   # Sea level
                    density=1.225,       # kg/m³
                    gravity=9.81,
                    wind_velocity=np.array([25.0, 15.0, 5.0]),
                    visibility=5000.0,   # 5 km in severe weather
                    humidity=0.8
                )
                
                # Add time-varying turbulence
                turbulence_factor = 1.0 + 0.5 * np.sin(2 * np.pi * t / 30.0)  # 30-second cycles
                base_conditions.wind_velocity *= turbulence_factor
                
                conditions = base_conditions
                particles = None
                
            else:
                raise ValueError(f"Unknown scenario type: {scenario_type}")
            
            # Calculate aerodynamic effects
            aero_effects = self.calculate_environmental_aerodynamic_effects(
                conditions, particles
            )
            
            scenario_data['conditions_history'].append(conditions)
            scenario_data['aerodynamic_effects_history'].append(aero_effects)
        
        return scenario_data
    
    def plot_environmental_analysis(self, scenario_data: Dict, save_path: str = None) -> None:
        """Plot environmental simulation results."""
        
        time_steps = scenario_data['time_steps']
        conditions = scenario_data['conditions_history']
        aero_effects = scenario_data['aerodynamic_effects_history']
        
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        
        # Extract time series data
        temperatures = [c.temperature for c in conditions]
        pressures = [c.pressure for c in conditions]
        densities = [c.density for c in conditions]
        wind_speeds = [np.linalg.norm(c.wind_velocity) for c in conditions]
        visibilities = [c.visibility for c in conditions]
        lift_factors = [ae['lift_factor'] for ae in aero_effects]
        
        # Temperature
        axes[0, 0].plot(time_steps, temperatures, 'r-', linewidth=2)
        axes[0, 0].set_title('Temperature')
        axes[0, 0].set_ylabel('Temperature (K)')
        axes[0, 0].grid(True, alpha=0.3)
        
        # Pressure
        axes[0, 1].plot(time_steps, pressures, 'b-', linewidth=2)
        axes[0, 1].set_title('Atmospheric Pressure')
        axes[0, 1].set_ylabel('Pressure (Pa)')
        axes[0, 1].grid(True, alpha=0.3)
        
        # Density
        axes[0, 2].plot(time_steps, densities, 'g-', linewidth=2)
        axes[0, 2].set_title('Atmospheric Density')
        axes[0, 2].set_ylabel('Density (kg/m³)')
        axes[0, 2].grid(True, alpha=0.3)
        
        # Wind Speed
        axes[1, 0].plot(time_steps, wind_speeds, 'm-', linewidth=2)
        axes[1, 0].set_title('Wind Speed')
        axes[1, 0].set_ylabel('Wind Speed (m/s)')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].grid(True, alpha=0.3)
        
        # Visibility
        axes[1, 1].plot(time_steps, visibilities, 'orange', linewidth=2)
        axes[1, 1].set_title('Visibility')
        axes[1, 1].set_ylabel('Visibility (m)')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].grid(True, alpha=0.3)
        
        # Aerodynamic Impact
        axes[1, 2].plot(time_steps, lift_factors, 'purple', linewidth=2)
        axes[1, 2].set_title('Aerodynamic Performance Factor')
        axes[1, 2].set_ylabel('Lift Factor')
        axes[1, 2].set_xlabel('Time (s)')
        axes[1, 2].grid(True, alpha=0.3)
        
        plt.suptitle(f'Environmental Simulation: {scenario_data["scenario_type"]}', 
                    fontsize=14, fontweight='bold')
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.show()
    
    def export_environmental_data(self, scenario_data: Dict, filename: str) -> None:
        """Export environmental simulation data to JSON."""
        
        # Convert numpy arrays and complex objects to JSON-serializable format
        export_data = {
            'scenario_type': scenario_data['scenario_type'],
            'time_steps': scenario_data['time_steps'].tolist(),
            'conditions_summary': {
                'temperature_range': [
                    float(min(c.temperature for c in scenario_data['conditions_history'])),
                    float(max(c.temperature for c in scenario_data['conditions_history']))
                ],
                'pressure_range': [
                    float(min(c.pressure for c in scenario_data['conditions_history'])),
                    float(max(c.pressure for c in scenario_data['conditions_history']))
                ],
                'wind_speed_range': [
                    float(min(np.linalg.norm(c.wind_velocity) for c in scenario_data['conditions_history'])),
                    float(max(np.linalg.norm(c.wind_velocity) for c in scenario_data['conditions_history']))
                ]
            },
            'aerodynamic_impact_summary': {
                'lift_factor_range': [
                    float(min(ae['lift_factor'] for ae in scenario_data['aerodynamic_effects_history'])),
                    float(max(ae['lift_factor'] for ae in scenario_data['aerodynamic_effects_history']))
                ],
                'drag_factor_range': [
                    float(min(ae['drag_factor'] for ae in scenario_data['aerodynamic_effects_history'])),
                    float(max(ae['drag_factor'] for ae in scenario_data['aerodynamic_effects_history']))
                ]
            }
        }
        
        with open(filename, 'w') as f:
            json.dump(export_data, f, indent=2)
        
        print(f"Environmental simulation data exported to {filename}")


# Demonstration function
def demonstrate_environmental_simulation():
    """Demonstrate environmental simulation capabilities."""
    print("ENVIRONMENTAL SIMULATION DEMONSTRATION")
    print("=" * 45)
    
    simulator = EnvironmentalSimulator()
    
    # 1. Martian Dust Storm Simulation
    print("1. Simulating Martian dust storm scenario...")
    martian_scenario = simulator.simulate_extreme_weather_scenario(
        scenario_type="martian_dust_storm",
        duration=300.0,  # 5 minutes
        dt=5.0
    )
    
    final_conditions = martian_scenario['conditions_history'][-1]
    final_effects = martian_scenario['aerodynamic_effects_history'][-1]
    
    print(f"   ✓ Final storm conditions:")
    print(f"     - Temperature: {final_conditions.temperature:.1f} K")
    print(f"     - Pressure: {final_conditions.pressure:.0f} Pa")
    print(f"     - Wind speed: {np.linalg.norm(final_conditions.wind_velocity):.1f} m/s")
    print(f"     - Visibility: {final_conditions.visibility:.0f} m")
    print(f"   ✓ Aerodynamic impact:")
    print(f"     - Lift factor: {final_effects['lift_factor']:.3f}")
    print(f"     - Drag factor: {final_effects['drag_factor']:.3f}")
    
    # 2. Dynamic Wind Field
    print("\n2. Generating dynamic wind field...")
    wind_field = simulator.generate_dynamic_wind_field(
        duration=60.0,
        turbulence_intensity=0.3,
        gust_probability=0.15
    )
    
    stats = wind_field['statistics']
    print(f"   ✓ Wind statistics:")
    print(f"     - Mean wind speed: {stats['mean_wind_speed']:.1f} m/s")
    print(f"     - Maximum wind speed: {stats['max_wind_speed']:.1f} m/s")
    print(f"     - Turbulence intensity: {stats['turbulence_intensity']:.3f}")
    print(f"     - Gust events: {stats['gust_count']}")
    
    # 3. Earth Severe Turbulence
    print("\n3. Simulating Earth severe turbulence...")
    earth_scenario = simulator.simulate_extreme_weather_scenario(
        scenario_type="earth_severe_turbulence",
        duration=120.0,
        dt=2.0
    )
    
    max_wind = max(np.linalg.norm(c.wind_velocity) for c in earth_scenario['conditions_history'])
    min_visibility = min(c.visibility for c in earth_scenario['conditions_history'])
    
    print(f"   ✓ Severe weather conditions:")
    print(f"     - Maximum wind speed: {max_wind:.1f} m/s")
    print(f"     - Minimum visibility: {min_visibility:.0f} m")
    
    # 4. Generate Visualizations
    print("\n4. Generating analysis plots...")
    simulator.plot_environmental_analysis(
        martian_scenario, 
        save_path="martian_dust_storm_analysis.png"
    )
    
    # 5. Export Data
    print("\n5. Exporting simulation data...")
    simulator.export_environmental_data(
        martian_scenario, 
        "martian_dust_storm_data.json"
    )
    
    print("\n" + "=" * 45)
    print("ENVIRONMENTAL SIMULATION COMPLETE")
    print("=" * 45)
    print("Capabilities Demonstrated:")
    print("✓ Martian atmospheric modeling with dust storms")
    print("✓ Dynamic wind field generation with gusts")
    print("✓ Particle dynamics simulation")
    print("✓ Aerodynamic performance impact analysis")
    print("✓ Extreme weather scenario testing")
    print("✓ Professional visualization and data export")
    
    return simulator


if __name__ == "__main__":
    simulator = demonstrate_environmental_simulation()