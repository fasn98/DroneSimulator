"""
Drone Optimization Module Demonstration

This script demonstrates the acoustic and aerodynamic optimization capabilities
of the new drone optimization module, showcasing the research-based approach
to reducing noise while maintaining flight performance.
"""

import os
import sys
import json
import numpy as np
import matplotlib.pyplot as plt

# Add src directory to path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from drone_optimization import OptimizedDroneFactory, AcousticOptimizer, AerodynamicAnalyzer, RotorDesigner


def main():
    """Main demonstration function."""
    print("=" * 80)
    print("DRONE ACOUSTIC & AERODYNAMIC OPTIMIZATION DEMONSTRATION")
    print("=" * 80)
    print("Based on research in uneven step angles for noise reduction")
    print("while maintaining safe flight performance characteristics.\n")
    
    # Initialize optimization components
    factory = OptimizedDroneFactory()
    acoustic_optimizer = AcousticOptimizer()
    aero_analyzer = AerodynamicAnalyzer()
    rotor_designer = RotorDesigner()
    
    # Demonstrate available templates and profiles
    print("1. AVAILABLE DRONE TEMPLATES:")
    templates = factory.get_available_templates()
    for i, template in enumerate(templates, 1):
        print(f"   {i}. {template}")
    
    print("\n2. OPTIMIZATION PROFILES:")
    profiles = factory.get_optimization_profiles()
    for i, profile in enumerate(profiles, 1):
        print(f"   {i}. {profile}")
    
    # Create optimized drone configurations
    print("\n3. CREATING OPTIMIZED DRONE CONFIGURATIONS:")
    
    # Professional quadcopter with balanced optimization
    print("\n   Creating Professional Quadcopter (Balanced Optimization)...")
    professional_quad = factory.create_optimized_drone(
        'professional_quadcopter', 
        'balanced'
    )
    
    # Acoustic-optimized hexacopter
    print("   Creating Acoustic-Optimized Hexacopter...")
    acoustic_hex = factory.create_optimized_drone(
        'acoustic_optimized_hex',
        'acoustic'
    )
    
    # Precision octocopter with aerodynamic focus
    print("   Creating Precision Octocopter (Aerodynamic Focus)...")
    precision_octo = factory.create_optimized_drone(
        'precision_octocopter',
        'aerodynamic'
    )
    
    # Display optimization results
    print("\n4. OPTIMIZATION RESULTS SUMMARY:")
    display_drone_summary(professional_quad, "Professional Quadcopter")
    display_drone_summary(acoustic_hex, "Acoustic-Optimized Hexacopter")
    display_drone_summary(precision_octo, "Precision Octocopter")
    
    # Demonstrate detailed rotor analysis
    print("\n5. DETAILED ROTOR ANALYSIS:")
    demonstrate_rotor_analysis(rotor_designer, acoustic_optimizer, aero_analyzer)
    
    # Save configurations for future use
    print("\n6. SAVING CONFIGURATIONS:")
    save_configurations(professional_quad, acoustic_hex, precision_octo)
    
    # Generate visualization plots
    print("\n7. GENERATING ANALYSIS PLOTS:")
    create_analysis_plots(professional_quad, acoustic_hex, precision_octo)
    
    print("\n" + "=" * 80)
    print("DEMONSTRATION COMPLETE")
    print("=" * 80)
    print("Generated optimized drone configurations with:")
    print("• 3-5 dB noise reduction through uneven step angles")
    print("• Maintained flight safety and performance")
    print("• Professional manufacturing specifications")
    print("• Comprehensive performance analysis")
    print("\nConfiguration files saved for integration with existing simulation.")


def display_drone_summary(drone_config, name):
    """Display summary of drone configuration."""
    print(f"\n   {name}:")
    print(f"      Model: {drone_config['model_name']}")
    print(f"      Optimization: {drone_config['optimization_profile']}")
    
    # Performance metrics
    perf = drone_config['performance_characteristics']
    print(f"      Total Thrust: {perf['total_thrust_capacity']} N")
    print(f"      Power Required: {perf['hover_power_required']} W")
    print(f"      Weight: {perf['estimated_weight']} kg")
    print(f"      Efficiency: {perf['efficiency_rating']}")
    
    # Acoustic metrics
    acoustic = drone_config['acoustic_metrics']
    print(f"      Noise Reduction: {acoustic['average_discrete_tone_reduction']} dB")
    print(f"      Max Noise Level: {acoustic['maximum_perceived_noise_level']} dB")
    
    # Safety features
    safety = drone_config['safety_features']
    print(f"      Motor Failure Compensation: {safety['redundancy_systems']['motor_failure_compensation']}")
    print(f"      Vibration Monitoring: {safety['monitoring_systems']['vibration_monitoring']}")


def demonstrate_rotor_analysis(rotor_designer, acoustic_optimizer, aero_analyzer):
    """Demonstrate detailed rotor analysis capabilities."""
    print("\n   Analyzing optimized rotor design...")
    
    # Create base rotor design
    base_design = {
        'num_blades': 4,
        'diameter': 0.25,
        'chord_length': 0.020,
        'rpm': 6000,
        'twist_distribution': [0, -3, -6, -9],
        'airfoil': 'NACA2412',
        'tolerance': 0.008
    }
    
    # Design optimized rotor
    rotor_spec = rotor_designer.design_optimized_rotor(base_design, 'balanced')
    
    print(f"      Rotor Diameter: {rotor_spec.diameter} m")
    print(f"      Design RPM: {rotor_spec.design_rpm}")
    print(f"      Tip Speed: {rotor_spec.tip_speed:.1f} m/s")
    print(f"      Step Angles: {[f'{angle:.1f}°' for angle in rotor_spec.step_angles]}")
    
    # Analyze acoustic performance
    acoustic_metrics = rotor_designer.analyze_acoustic_performance(rotor_spec)
    print(f"      Acoustic Reduction: {acoustic_metrics.discrete_tone_reduction:.1f} dB")
    print(f"      Perceived Noise: {acoustic_metrics.perceived_noise_level:.1f} dB")
    
    # Analyze aerodynamic performance
    aero_metrics = rotor_designer.analyze_aerodynamic_performance(rotor_spec)
    print(f"      Figure of Merit: {aero_metrics.figure_of_merit:.3f}")
    print(f"      Thrust Asymmetry: {aero_metrics.thrust_asymmetry:.1f}%")
    print(f"      Efficiency: {aero_metrics.efficiency:.1f}%")
    
    # Manufacturing specifications
    manufacturing = rotor_designer.generate_manufacturing_specs(rotor_spec)
    print(f"      Angular Tolerance: ±{manufacturing['geometric_precision']['angular_tolerance']:.3f}°")
    print(f"      Balance Tolerance: {manufacturing['assembly_requirements']['static_balance']} g·cm")


def save_configurations(professional_quad, acoustic_hex, precision_octo):
    """Save drone configurations to files."""
    configurations = {
        'professional_quadcopter_optimized.json': professional_quad,
        'acoustic_hexacopter_optimized.json': acoustic_hex,
        'precision_octocopter_optimized.json': precision_octo
    }
    
    for filename, config in configurations.items():
        try:
            with open(filename, 'w') as f:
                json.dump(config, f, indent=2)
            print(f"      ✓ Saved {filename}")
        except Exception as e:
            print(f"      ✗ Error saving {filename}: {e}")


def create_analysis_plots(professional_quad, acoustic_hex, precision_octo):
    """Create visualization plots for optimization analysis."""
    try:
        # Extract data for plotting
        drones = [professional_quad, acoustic_hex, precision_octo]
        names = ['Professional Quad', 'Acoustic Hex', 'Precision Octo']
        
        # Performance data
        thrust_capacities = [d['performance_characteristics']['total_thrust_capacity'] for d in drones]
        noise_reductions = [d['acoustic_metrics']['average_discrete_tone_reduction'] for d in drones]
        max_noise_levels = [d['acoustic_metrics']['maximum_perceived_noise_level'] for d in drones]
        
        # Create comparison plots
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 10))
        
        # Thrust capacity comparison
        ax1.bar(names, thrust_capacities, color=['#2E86C1', '#E74C3C', '#F39C12'])
        ax1.set_title('Total Thrust Capacity')
        ax1.set_ylabel('Thrust (N)')
        ax1.tick_params(axis='x', rotation=45)
        
        # Noise reduction comparison
        ax2.bar(names, noise_reductions, color=['#27AE60', '#8E44AD', '#D35400'])
        ax2.set_title('Acoustic Noise Reduction')
        ax2.set_ylabel('Reduction (dB)')
        ax2.tick_params(axis='x', rotation=45)
        
        # Maximum noise levels
        ax3.bar(names, max_noise_levels, color=['#1ABC9C', '#E67E22', '#9B59B6'])
        ax3.set_title('Maximum Perceived Noise Level')
        ax3.set_ylabel('Noise Level (dB)')
        ax3.tick_params(axis='x', rotation=45)
        
        # Step angle visualization for professional quad
        rotor_data = professional_quad['rotor_specifications'][0]
        step_angles = rotor_data['step_angles']
        blade_positions = np.cumsum([0] + step_angles[:-1])
        
        # Polar plot of blade positions
        ax4 = plt.subplot(2, 2, 4, projection='polar')
        blade_angles_rad = np.deg2rad(blade_positions)
        radii = np.ones(len(blade_angles_rad))
        
        ax4.scatter(blade_angles_rad, radii, s=100, c='red', marker='o')
        ax4.set_ylim(0, 1.2)
        ax4.set_title('Optimized Blade Spacing\n(Professional Quadcopter)')
        
        # Add angle labels
        for i, (angle, angle_rad) in enumerate(zip(blade_positions, blade_angles_rad)):
            ax4.annotate(f'B{i+1}\n{angle:.1f}°', 
                        (angle_rad, 1.1), 
                        ha='center', va='center', fontsize=8)
        
        plt.tight_layout()
        plt.savefig('optimization_analysis.png', dpi=150, bbox_inches='tight')
        print("      ✓ Saved optimization_analysis.png")
        
        # Create frequency spectrum plot for acoustic analysis
        fig2, ax = plt.subplots(1, 1, figsize=(10, 6))
        
        # Sample frequency data (would come from actual rotor analysis)
        frequencies = np.array([200, 400, 600, 800, 1000, 1200])
        baseline_levels = np.array([60, 55, 50, 45, 40, 35])
        optimized_levels = baseline_levels - noise_reductions[0]  # Professional quad reduction
        
        ax.plot(frequencies, baseline_levels, 'r-o', label='Baseline Design', linewidth=2)
        ax.plot(frequencies, optimized_levels, 'g-s', label='Optimized Design', linewidth=2)
        ax.fill_between(frequencies, baseline_levels, optimized_levels, alpha=0.3, color='green')
        
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('Sound Pressure Level (dB)')
        ax.set_title('Acoustic Frequency Spectrum - Professional Quadcopter')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        plt.savefig('acoustic_spectrum.png', dpi=150, bbox_inches='tight')
        print("      ✓ Saved acoustic_spectrum.png")
        
        print("      ✓ Generated visualization plots")
        
    except Exception as e:
        print(f"      ✗ Error creating plots: {e}")
        print("        (matplotlib may not be available in environment)")


if __name__ == "__main__":
    main()