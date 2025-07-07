"""
Visualization Module

Handles data plotting and visualization for drone simulation results.
Provides trajectory plots, performance analysis, and mission monitoring.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pandas as pd
from typing import Dict, List, Any, Optional, Tuple
from pathlib import Path
import json
import seaborn as sns

# Set matplotlib style for better plots
plt.style.use('seaborn-v0_8' if 'seaborn-v0_8' in plt.style.available else 'default')
sns.set_palette("husl")


class Visualizer:
    """
    Visualization class for drone simulation data.
    
    Provides:
    - 3D trajectory plotting
    - Performance metrics visualization
    - Mission progress tracking
    - Environmental data plotting
    - Comparative analysis charts
    """
    
    def __init__(self, output_dir: Path = None):
        """
        Initialize visualizer.
        
        Args:
            output_dir: Directory to save plots
        """
        self.output_dir = output_dir or Path("plots")
        self.output_dir.mkdir(exist_ok=True)
        
        # Plot configuration
        self.figure_size = (12, 8)
        self.dpi = 100
        self.font_size = 12
        
        # Configure matplotlib
        plt.rcParams.update({
            'font.size': self.font_size,
            'axes.labelsize': self.font_size,
            'axes.titlesize': self.font_size + 2,
            'xtick.labelsize': self.font_size - 1,
            'ytick.labelsize': self.font_size - 1,
            'legend.fontsize': self.font_size - 1,
            'figure.titlesize': self.font_size + 4
        })
    
    def plot_3d_trajectory(self, telemetry_data: List[Dict[str, Any]], 
                          waypoints: List[Dict[str, Any]] = None,
                          title: str = "Drone 3D Trajectory") -> str:
        """
        Plot 3D trajectory of drone flight.
        
        Args:
            telemetry_data: List of telemetry records
            waypoints: List of waypoint dictionaries
            title: Plot title
            
        Returns:
            Path to saved plot file
        """
        fig = plt.figure(figsize=self.figure_size, dpi=self.dpi)
        ax = fig.add_subplot(111, projection='3d')
        
        # Extract position data
        positions = np.array([record['position'] for record in telemetry_data])
        times = np.array([record['simulation_time'] for record in telemetry_data])
        
        # Create color map based on time
        colors = plt.cm.viridis(times / times.max())
        
        # Plot trajectory
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                'b-', linewidth=2, alpha=0.7, label='Flight Path')
        
        # Scatter plot with time-based colors
        scatter = ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], 
                           c=times, cmap='viridis', s=20, alpha=0.6)
        
        # Plot waypoints if provided
        if waypoints:
            waypoint_positions = np.array([[wp['x'], wp['y'], wp['z']] for wp in waypoints])
            ax.scatter(waypoint_positions[:, 0], waypoint_positions[:, 1], 
                      waypoint_positions[:, 2], c='red', s=100, marker='o', 
                      alpha=0.8, label='Waypoints')
            
            # Connect waypoints with lines
            ax.plot(waypoint_positions[:, 0], waypoint_positions[:, 1], 
                   waypoint_positions[:, 2], 'r--', alpha=0.5, linewidth=1)
        
        # Mark start and end points
        ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], 
                  c='green', s=150, marker='^', label='Start')
        ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], 
                  c='red', s=150, marker='v', label='End')
        
        # Add colorbar
        cbar = plt.colorbar(scatter, ax=ax, shrink=0.5)
        cbar.set_label('Time (s)')
        
        # Set labels and title
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Z Position (m)')
        ax.set_title(title)
        ax.legend()
        
        # Equal aspect ratio
        max_range = np.array([positions[:, 0].max() - positions[:, 0].min(),
                             positions[:, 1].max() - positions[:, 1].min(),
                             positions[:, 2].max() - positions[:, 2].min()]).max() / 2.0
        mid_x = (positions[:, 0].max() + positions[:, 0].min()) * 0.5
        mid_y = (positions[:, 1].max() + positions[:, 1].min()) * 0.5
        mid_z = (positions[:, 2].max() + positions[:, 2].min()) * 0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        # Save plot
        filename = f"trajectory_3d_{int(times.max())}.png"
        filepath = self.output_dir / filename
        plt.tight_layout()
        plt.savefig(filepath, dpi=self.dpi, bbox_inches='tight')
        plt.close()
        
        return str(filepath)
    
    def plot_altitude_profile(self, telemetry_data: List[Dict[str, Any]], 
                             title: str = "Altitude Profile") -> str:
        """
        Plot altitude vs time.
        
        Args:
            telemetry_data: List of telemetry records
            title: Plot title
            
        Returns:
            Path to saved plot file
        """
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=self.figure_size, dpi=self.dpi)
        
        # Extract data
        times = np.array([record['simulation_time'] for record in telemetry_data])
        altitudes = np.array([record['altitude'] for record in telemetry_data])
        velocities = np.array([record['velocity'] for record in telemetry_data])
        vertical_speeds = velocities[:, 2]
        
        # Plot altitude
        ax1.plot(times, altitudes, 'b-', linewidth=2, label='Altitude')
        ax1.fill_between(times, altitudes, alpha=0.3)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Altitude (m)')
        ax1.set_title(f'{title} - Altitude vs Time')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # Plot vertical speed
        ax2.plot(times, vertical_speeds, 'r-', linewidth=2, label='Vertical Speed')
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Vertical Speed (m/s)')
        ax2.set_title('Vertical Speed vs Time')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        # Save plot
        filename = f"altitude_profile_{int(times.max())}.png"
        filepath = self.output_dir / filename
        plt.tight_layout()
        plt.savefig(filepath, dpi=self.dpi, bbox_inches='tight')
        plt.close()
        
        return str(filepath)
    
    def plot_velocity_analysis(self, telemetry_data: List[Dict[str, Any]], 
                              title: str = "Velocity Analysis") -> str:
        """
        Plot velocity components and magnitude.
        
        Args:
            telemetry_data: List of telemetry records
            title: Plot title
            
        Returns:
            Path to saved plot file
        """
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10), dpi=self.dpi)
        
        # Extract data
        times = np.array([record['simulation_time'] for record in telemetry_data])
        velocities = np.array([record['velocity'] for record in telemetry_data])
        airspeeds = np.array([record['airspeed'] for record in telemetry_data])
        ground_speeds = np.array([record['ground_speed'] for record in telemetry_data])
        
        # Plot velocity components
        ax1.plot(times, velocities[:, 0], 'r-', label='Vx', linewidth=2)
        ax1.plot(times, velocities[:, 1], 'g-', label='Vy', linewidth=2)
        ax1.plot(times, velocities[:, 2], 'b-', label='Vz', linewidth=2)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Velocity (m/s)')
        ax1.set_title('Velocity Components')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot speed magnitudes
        ax2.plot(times, airspeeds, 'k-', label='Airspeed', linewidth=2)
        ax2.plot(times, ground_speeds, 'orange', label='Ground Speed', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Speed (m/s)')
        ax2.set_title('Speed Magnitudes')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Velocity vector field (2D projection)
        positions = np.array([record['position'] for record in telemetry_data])
        # Sample every 20th point for readability
        sample_idx = slice(0, len(times), max(1, len(times) // 20))
        ax3.quiver(positions[sample_idx, 0], positions[sample_idx, 1], 
                  velocities[sample_idx, 0], velocities[sample_idx, 1], 
                  airspeeds[sample_idx], cmap='viridis', alpha=0.7)
        ax3.set_xlabel('X Position (m)')
        ax3.set_ylabel('Y Position (m)')
        ax3.set_title('Velocity Vector Field (XY Plane)')
        ax3.axis('equal')
        
        # Speed histogram
        ax4.hist(airspeeds, bins=30, alpha=0.7, color='skyblue', edgecolor='black')
        ax4.axvline(airspeeds.mean(), color='red', linestyle='--', 
                   label=f'Mean: {airspeeds.mean():.1f} m/s')
        ax4.set_xlabel('Airspeed (m/s)')
        ax4.set_ylabel('Frequency')
        ax4.set_title('Airspeed Distribution')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # Save plot
        filename = f"velocity_analysis_{int(times.max())}.png"
        filepath = self.output_dir / filename
        plt.tight_layout()
        plt.savefig(filepath, dpi=self.dpi, bbox_inches='tight')
        plt.close()
        
        return str(filepath)
    
    def plot_attitude_analysis(self, telemetry_data: List[Dict[str, Any]], 
                              title: str = "Attitude Analysis") -> str:
        """
        Plot attitude angles and angular velocities.
        
        Args:
            telemetry_data: List of telemetry records
            title: Plot title
            
        Returns:
            Path to saved plot file
        """
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10), dpi=self.dpi)
        
        # Extract data
        times = np.array([record['simulation_time'] for record in telemetry_data])
        attitudes = np.array([record['attitude'] for record in telemetry_data])
        angular_velocities = np.array([record['angular_velocity'] for record in telemetry_data])
        
        # Convert to degrees
        attitudes_deg = np.degrees(attitudes)
        angular_velocities_deg = np.degrees(angular_velocities)
        
        # Plot attitude angles
        ax1.plot(times, attitudes_deg[:, 0], 'r-', label='Roll', linewidth=2)
        ax1.plot(times, attitudes_deg[:, 1], 'g-', label='Pitch', linewidth=2)
        ax1.plot(times, attitudes_deg[:, 2], 'b-', label='Yaw', linewidth=2)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Angle (degrees)')
        ax1.set_title('Attitude Angles')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot angular velocities
        ax2.plot(times, angular_velocities_deg[:, 0], 'r-', label='Roll Rate', linewidth=2)
        ax2.plot(times, angular_velocities_deg[:, 1], 'g-', label='Pitch Rate', linewidth=2)
        ax2.plot(times, angular_velocities_deg[:, 2], 'b-', label='Yaw Rate', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Angular Velocity (deg/s)')
        ax2.set_title('Angular Velocities')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Attitude vs altitude
        altitudes = np.array([record['altitude'] for record in telemetry_data])
        ax3.scatter(altitudes, attitudes_deg[:, 0], c='red', alpha=0.6, s=20, label='Roll')
        ax3.scatter(altitudes, attitudes_deg[:, 1], c='green', alpha=0.6, s=20, label='Pitch')
        ax3.set_xlabel('Altitude (m)')
        ax3.set_ylabel('Attitude (degrees)')
        ax3.set_title('Attitude vs Altitude')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Control surface usage (simplified)
        ax4.plot(times, np.abs(attitudes_deg[:, 0]), 'r-', alpha=0.7, label='|Roll|')
        ax4.plot(times, np.abs(attitudes_deg[:, 1]), 'g-', alpha=0.7, label='|Pitch|')
        ax4.plot(times, np.abs(attitudes_deg[:, 2]), 'b-', alpha=0.7, label='|Yaw|')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Absolute Angle (degrees)')
        ax4.set_title('Control Authority Usage')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # Save plot
        filename = f"attitude_analysis_{int(times.max())}.png"
        filepath = self.output_dir / filename
        plt.tight_layout()
        plt.savefig(filepath, dpi=self.dpi, bbox_inches='tight')
        plt.close()
        
        return str(filepath)
    
    def plot_power_analysis(self, telemetry_data: List[Dict[str, Any]], 
                           title: str = "Power Analysis") -> str:
        """
        Plot power consumption and energy usage.
        
        Args:
            telemetry_data: List of telemetry records
            title: Plot title
            
        Returns:
            Path to saved plot file
        """
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10), dpi=self.dpi)
        
        # Extract data
        times = np.array([record['simulation_time'] for record in telemetry_data])
        power_consumption = np.array([record['power_consumption'] for record in telemetry_data])
        battery_levels = np.array([record['battery_level'] for record in telemetry_data])
        fuel_levels = np.array([record['fuel_level'] for record in telemetry_data])
        
        # Plot power consumption
        ax1.plot(times, power_consumption, 'b-', linewidth=2, label='Power')
        ax1.fill_between(times, power_consumption, alpha=0.3)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Power (W)')
        ax1.set_title('Power Consumption')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # Plot battery/fuel levels
        if battery_levels.max() > 0:
            ax2.plot(times, battery_levels * 100, 'g-', linewidth=2, label='Battery')
        if fuel_levels.max() > 0:
            ax2.plot(times, fuel_levels * 100, 'r-', linewidth=2, label='Fuel')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Level (%)')
        ax2.set_title('Energy/Fuel Levels')
        ax2.set_ylim(0, 100)
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        # Energy consumption rate
        if len(times) > 1:
            dt = np.diff(times)
            energy_rate = np.diff(power_consumption) / dt
            ax3.plot(times[1:], energy_rate, 'orange', linewidth=2, label='Energy Rate')
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('dP/dt (W/s)')
            ax3.set_title('Power Consumption Rate')
            ax3.grid(True, alpha=0.3)
            ax3.legend()
        
        # Power vs speed correlation
        airspeeds = np.array([record['airspeed'] for record in telemetry_data])
        ax4.scatter(airspeeds, power_consumption, c=times, cmap='viridis', alpha=0.6)
        ax4.set_xlabel('Airspeed (m/s)')
        ax4.set_ylabel('Power (W)')
        ax4.set_title('Power vs Airspeed')
        ax4.grid(True, alpha=0.3)
        
        # Save plot
        filename = f"power_analysis_{int(times.max())}.png"
        filepath = self.output_dir / filename
        plt.tight_layout()
        plt.savefig(filepath, dpi=self.dpi, bbox_inches='tight')
        plt.close()
        
        return str(filepath)
    
    def plot_mission_progress(self, mission_events: List[Dict[str, Any]], 
                             mission_metrics: Dict[str, Any],
                             title: str = "Mission Progress") -> str:
        """
        Plot mission progress and waypoint completion.
        
        Args:
            mission_events: List of mission event records
            mission_metrics: Mission metrics dictionary
            title: Plot title
            
        Returns:
            Path to saved plot file
        """
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10), dpi=self.dpi)
        
        # Extract event data
        if mission_events:
            event_times = np.array([event['simulation_time'] for event in mission_events])
            progress_values = np.array([event['mission_progress'] for event in mission_events])
            
            # Plot progress over time
            ax1.plot(event_times, progress_values, 'b-', linewidth=2, marker='o', 
                    markersize=4, label='Progress')
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Progress (%)')
            ax1.set_title('Mission Progress Over Time')
            ax1.set_ylim(0, 100)
            ax1.grid(True, alpha=0.3)
            ax1.legend()
            
            # Plot event types
            event_types = [event['event_type'] for event in mission_events]
            event_counts = {}
            for event_type in event_types:
                event_counts[event_type] = event_counts.get(event_type, 0) + 1
            
            ax2.bar(event_counts.keys(), event_counts.values(), color='skyblue', alpha=0.7)
            ax2.set_xlabel('Event Type')
            ax2.set_ylabel('Count')
            ax2.set_title('Mission Events Summary')
            ax2.tick_params(axis='x', rotation=45)
        
        # Plot objectives status
        objectives = mission_metrics.get('objectives_status', [])
        if objectives:
            obj_names = [obj['name'] for obj in objectives]
            obj_progress = [obj['progress'] for obj in objectives]
            
            colors = ['green' if obj['completed'] else 'orange' for obj in objectives]
            bars = ax3.barh(obj_names, obj_progress, color=colors, alpha=0.7)
            ax3.set_xlabel('Progress (%)')
            ax3.set_title('Objectives Status')
            ax3.set_xlim(0, 100)
            
            # Add progress text
            for i, (bar, progress) in enumerate(zip(bars, obj_progress)):
                ax3.text(progress + 1, bar.get_y() + bar.get_height()/2, 
                        f'{progress:.1f}%', va='center')
        
        # Mission metrics summary
        metrics_text = []
        if 'total_time' in mission_metrics:
            metrics_text.append(f"Total Time: {mission_metrics['total_time']:.1f}s")
        if 'distance_traveled' in mission_metrics:
            metrics_text.append(f"Distance: {mission_metrics['distance_traveled']:.1f}m")
        if 'area_covered' in mission_metrics:
            metrics_text.append(f"Area Covered: {mission_metrics['area_covered']:.1f}m²")
        if 'samples_collected' in mission_metrics:
            metrics_text.append(f"Samples: {mission_metrics['samples_collected']}")
        if 'photos_taken' in mission_metrics:
            metrics_text.append(f"Photos: {mission_metrics['photos_taken']}")
        
        ax4.text(0.1, 0.9, '\n'.join(metrics_text), transform=ax4.transAxes, 
                fontsize=12, verticalalignment='top', 
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
        ax4.set_title('Mission Summary')
        ax4.axis('off')
        
        # Save plot
        filename = f"mission_progress_{int(mission_metrics.get('total_time', 0))}.png"
        filepath = self.output_dir / filename
        plt.tight_layout()
        plt.savefig(filepath, dpi=self.dpi, bbox_inches='tight')
        plt.close()
        
        return str(filepath)
    
    def plot_environmental_conditions(self, telemetry_data: List[Dict[str, Any]], 
                                    title: str = "Environmental Conditions") -> str:
        """
        Plot environmental conditions during flight.
        
        Args:
            telemetry_data: List of telemetry records
            title: Plot title
            
        Returns:
            Path to saved plot file
        """
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10), dpi=self.dpi)
        
        # Extract data
        times = np.array([record['simulation_time'] for record in telemetry_data])
        air_density = np.array([record['air_density'] for record in telemetry_data])
        air_pressure = np.array([record['air_pressure'] for record in telemetry_data])
        temperature = np.array([record['temperature'] for record in telemetry_data])
        wind_speed = np.array([record['wind_speed'] for record in telemetry_data])
        
        # Plot air density
        ax1.plot(times, air_density, 'b-', linewidth=2, label='Air Density')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Air Density (kg/m³)')
        ax1.set_title('Air Density Over Time')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # Plot air pressure
        ax2.plot(times, air_pressure / 1000, 'g-', linewidth=2, label='Air Pressure')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Air Pressure (kPa)')
        ax2.set_title('Air Pressure Over Time')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        # Plot temperature
        ax3.plot(times, temperature - 273.15, 'r-', linewidth=2, label='Temperature')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Temperature (°C)')
        ax3.set_title('Temperature Over Time')
        ax3.grid(True, alpha=0.3)
        ax3.legend()
        
        # Plot wind speed
        ax4.plot(times, wind_speed, 'orange', linewidth=2, label='Wind Speed')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Wind Speed (m/s)')
        ax4.set_title('Wind Speed Over Time')
        ax4.grid(True, alpha=0.3)
        ax4.legend()
        
        # Save plot
        filename = f"environmental_conditions_{int(times.max())}.png"
        filepath = self.output_dir / filename
        plt.tight_layout()
        plt.savefig(filepath, dpi=self.dpi, bbox_inches='tight')
        plt.close()
        
        return str(filepath)
    
    def generate_comprehensive_report(self, telemetry_data: List[Dict[str, Any]], 
                                    mission_events: List[Dict[str, Any]],
                                    mission_metrics: Dict[str, Any],
                                    waypoints: List[Dict[str, Any]] = None) -> List[str]:
        """
        Generate comprehensive visualization report.
        
        Args:
            telemetry_data: List of telemetry records
            mission_events: List of mission event records
            mission_metrics: Mission metrics dictionary
            waypoints: List of waypoint dictionaries
            
        Returns:
            List of paths to generated plot files
        """
        plot_files = []
        
        try:
            # Generate all plots
            plot_files.append(self.plot_3d_trajectory(telemetry_data, waypoints))
            plot_files.append(self.plot_altitude_profile(telemetry_data))
            plot_files.append(self.plot_velocity_analysis(telemetry_data))
            plot_files.append(self.plot_attitude_analysis(telemetry_data))
            plot_files.append(self.plot_power_analysis(telemetry_data))
            plot_files.append(self.plot_mission_progress(mission_events, mission_metrics))
            plot_files.append(self.plot_environmental_conditions(telemetry_data))
            
        except Exception as e:
            print(f"Error generating plots: {e}")
        
        return plot_files
    
    def create_summary_dashboard(self, telemetry_data: List[Dict[str, Any]], 
                               mission_metrics: Dict[str, Any],
                               title: str = "Flight Summary Dashboard") -> str:
        """
        Create a summary dashboard with key metrics.
        
        Args:
            telemetry_data: List of telemetry records
            mission_metrics: Mission metrics dictionary
            title: Dashboard title
            
        Returns:
            Path to saved dashboard file
        """
        fig = plt.figure(figsize=(16, 12), dpi=self.dpi)
        gs = fig.add_gridspec(3, 4, hspace=0.3, wspace=0.3)
        
        # Extract key data
        times = np.array([record['simulation_time'] for record in telemetry_data])
        positions = np.array([record['position'] for record in telemetry_data])
        velocities = np.array([record['velocity'] for record in telemetry_data])
        altitudes = positions[:, 2]
        airspeeds = np.array([record['airspeed'] for record in telemetry_data])
        
        # 3D trajectory (top left)
        ax1 = fig.add_subplot(gs[0, 0:2], projection='3d')
        ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=2)
        ax1.scatter(positions[0, 0], positions[0, 1], positions[0, 2], c='green', s=100, marker='^')
        ax1.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], c='red', s=100, marker='v')
        ax1.set_title('3D Trajectory')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        
        # Altitude profile (top right)
        ax2 = fig.add_subplot(gs[0, 2:4])
        ax2.plot(times, altitudes, 'b-', linewidth=2)
        ax2.fill_between(times, altitudes, alpha=0.3)
        ax2.set_title('Altitude Profile')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Altitude (m)')
        ax2.grid(True, alpha=0.3)
        
        # Speed analysis (middle left)
        ax3 = fig.add_subplot(gs[1, 0:2])
        ax3.plot(times, airspeeds, 'r-', linewidth=2, label='Airspeed')
        ax3.plot(times, np.linalg.norm(velocities[:, :2], axis=1), 'g-', linewidth=2, label='Ground Speed')
        ax3.set_title('Speed Analysis')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Speed (m/s)')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Mission metrics (middle right)
        ax4 = fig.add_subplot(gs[1, 2:4])
        metrics_text = [
            f"Mission: {mission_metrics.get('mission_type', 'Unknown')}",
            f"Status: {mission_metrics.get('status', 'Unknown')}",
            f"Duration: {mission_metrics.get('total_time', 0):.1f}s",
            f"Distance: {mission_metrics.get('distance_traveled', 0):.1f}m",
            f"Max Altitude: {altitudes.max():.1f}m",
            f"Max Speed: {airspeeds.max():.1f}m/s",
            f"Avg Speed: {airspeeds.mean():.1f}m/s",
            f"Progress: {mission_metrics.get('progress_percentage', 0):.1f}%"
        ]
        ax4.text(0.1, 0.9, '\n'.join(metrics_text), transform=ax4.transAxes, 
                fontsize=12, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))
        ax4.set_title('Mission Metrics')
        ax4.axis('off')
        
        # Performance indicators (bottom)
        ax5 = fig.add_subplot(gs[2, :])
        
        # Create performance bars
        metrics_names = ['Altitude\nUtilization', 'Speed\nUtilization', 'Mission\nProgress', 
                        'Fuel/Battery\nRemaining']
        
        # Calculate utilization percentages
        altitude_util = (altitudes.max() / 1000) * 100  # Assume 1000m max for visualization
        speed_util = (airspeeds.max() / 50) * 100  # Assume 50m/s max for visualization
        mission_progress = mission_metrics.get('progress_percentage', 0)
        
        # Get final battery/fuel level
        final_battery = telemetry_data[-1].get('battery_level', 0) * 100
        final_fuel = telemetry_data[-1].get('fuel_level', 0) * 100
        energy_remaining = max(final_battery, final_fuel)
        
        values = [min(altitude_util, 100), min(speed_util, 100), mission_progress, energy_remaining]
        colors = ['blue', 'green', 'orange', 'red']
        
        bars = ax5.bar(metrics_names, values, color=colors, alpha=0.7)
        ax5.set_ylabel('Percentage (%)')
        ax5.set_title('Performance Indicators')
        ax5.set_ylim(0, 100)
        
        # Add value labels on bars
        for bar, value in zip(bars, values):
            height = bar.get_height()
            ax5.text(bar.get_x() + bar.get_width()/2., height + 1,
                    f'{value:.1f}%', ha='center', va='bottom')
        
        fig.suptitle(title, fontsize=16)
        
        # Save dashboard
        filename = f"summary_dashboard_{int(times.max())}.png"
        filepath = self.output_dir / filename
        plt.savefig(filepath, dpi=self.dpi, bbox_inches='tight')
        plt.close()
        
        return str(filepath)
