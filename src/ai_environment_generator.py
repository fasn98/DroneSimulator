"""
AI Environment Generator for Fiction Planetary Environments

Creates immersive, scientifically-inspired environments for Moon and Mars
using generative AI to create realistic terrain, atmosphere, and visual elements.
"""

import os
import json
import numpy as np
from typing import Dict, List, Tuple, Optional, Any
import logging
from datetime import datetime
import random
import math

logger = logging.getLogger(__name__)

class AIEnvironmentGenerator:
    """
    Generates fictional but scientifically-plausible planetary environments
    for Moon and Mars using AI-driven procedural generation.
    """
    
    def __init__(self):
        """Initialize AI environment generator."""
        self.openai_api_key = None
        
    def set_openai_api_key(self, api_key: str):
        """Set OpenAI API key for generative features."""
        self.openai_api_key = api_key
    
    def generate_lunar_environment(self, region_type: str = "highland", 
                                 size_km: float = 10.0) -> Dict[str, Any]:
        """
        Generate a realistic lunar environment with terrain and features.
        
        Args:
            region_type: Type of lunar region (highland, mare, crater, polar)
            size_km: Size of environment in kilometers
            
        Returns:
            Dictionary containing environment data
        """
        try:
            # Base lunar parameters
            base_params = {
                'gravity': 1.62,  # m/s²
                'atmosphere_pressure': 0.0,  # No atmosphere
                'temperature_day': 127,  # °C
                'temperature_night': -173,  # °C
                'solar_irradiance': 1361,  # W/m² (same as Earth's orbital distance)
                'atmosphere_composition': {},
                'surface_material': 'regolith'
            }
            
            # Generate terrain based on region type
            terrain_data = self._generate_lunar_terrain(region_type, size_km)
            
            # Generate surface features
            surface_features = self._generate_lunar_surface_features(region_type, size_km)
            
            # Generate lighting conditions
            lighting = self._generate_lunar_lighting()
            
            # Generate visual description using AI if available
            visual_description = self._generate_environment_description(
                "lunar", region_type, terrain_data, surface_features
            )
            
            return {
                'environment_type': 'lunar',
                'region_type': region_type,
                'size_km': size_km,
                'parameters': base_params,
                'terrain': terrain_data,
                'surface_features': surface_features,
                'lighting': lighting,
                'visual_description': visual_description,
                'generated_at': datetime.now().isoformat()
            }
            
        except Exception as e:
            logger.error(f"Error generating lunar environment: {e}")
            return None
    
    def generate_martian_environment(self, region_type: str = "plains", 
                                   season: str = "spring") -> Dict[str, Any]:
        """
        Generate a realistic Martian environment with dynamic weather.
        
        Args:
            region_type: Type of Martian region (plains, canyon, polar, highland)
            season: Martian season affecting weather patterns
            
        Returns:
            Dictionary containing environment data
        """
        try:
            # Base Martian parameters
            base_params = {
                'gravity': 3.71,  # m/s²
                'atmosphere_pressure': 610,  # Pa (0.6% of Earth)
                'temperature_avg': -80,  # °C
                'temperature_range': (-140, 20),  # °C
                'solar_irradiance': 590,  # W/m² (43% of Earth's)
                'atmosphere_composition': {
                    'CO2': 95.32,
                    'N2': 2.7,
                    'Ar': 1.6,
                    'O2': 0.13,
                    'CO': 0.08
                },
                'surface_material': 'iron_oxide_dust'
            }
            
            # Generate terrain based on region type
            terrain_data = self._generate_martian_terrain(region_type)
            
            # Generate weather patterns
            weather_data = self._generate_martian_weather(season, region_type)
            
            # Generate surface features
            surface_features = self._generate_martian_surface_features(region_type)
            
            # Generate atmospheric effects
            atmospheric_effects = self._generate_martian_atmospheric_effects(weather_data)
            
            # Generate visual description using AI if available
            visual_description = self._generate_environment_description(
                "martian", region_type, terrain_data, surface_features, weather_data
            )
            
            return {
                'environment_type': 'martian',
                'region_type': region_type,
                'season': season,
                'parameters': base_params,
                'terrain': terrain_data,
                'weather': weather_data,
                'surface_features': surface_features,
                'atmospheric_effects': atmospheric_effects,
                'visual_description': visual_description,
                'generated_at': datetime.now().isoformat()
            }
            
        except Exception as e:
            logger.error(f"Error generating Martian environment: {e}")
            return None
    
    def _generate_lunar_terrain(self, region_type: str, size_km: float) -> Dict[str, Any]:
        """Generate lunar terrain data."""
        grid_size = 50
        terrain_grid = np.zeros((grid_size, grid_size))
        
        if region_type == "highland":
            # Heavily cratered highland terrain
            base_elevation = 2000  # meters
            for _ in range(20):  # Add multiple craters
                cx, cy = random.randint(5, grid_size-5), random.randint(5, grid_size-5)
                radius = random.randint(3, 8)
                depth = random.randint(50, 200)
                for i in range(grid_size):
                    for j in range(grid_size):
                        dist = math.sqrt((i-cx)**2 + (j-cy)**2)
                        if dist < radius:
                            terrain_grid[i][j] -= depth * (1 - dist/radius)
            
        elif region_type == "mare":
            # Smooth basaltic plains
            base_elevation = 0
            # Add subtle rolling hills
            for i in range(grid_size):
                for j in range(grid_size):
                    terrain_grid[i][j] = 10 * math.sin(i * 0.2) * math.cos(j * 0.2)
                    
        elif region_type == "crater":
            # Large crater with rim and central peak
            base_elevation = 0
            center = grid_size // 2
            crater_radius = grid_size // 3
            for i in range(grid_size):
                for j in range(grid_size):
                    dist = math.sqrt((i-center)**2 + (j-center)**2)
                    if dist < crater_radius:
                        # Crater bowl
                        terrain_grid[i][j] = -300 * (1 - (dist/crater_radius)**2)
                        # Central peak
                        if dist < crater_radius * 0.1:
                            terrain_grid[i][j] += 150
                    elif dist < crater_radius * 1.2:
                        # Crater rim
                        terrain_grid[i][j] = 100 * (1 - (dist - crater_radius)/(crater_radius * 0.2))
        
        terrain_grid += base_elevation
        
        return {
            'type': region_type,
            'elevation_grid': terrain_grid.tolist(),
            'grid_size': grid_size,
            'scale_meters_per_cell': (size_km * 1000) / grid_size,
            'min_elevation': float(np.min(terrain_grid)),
            'max_elevation': float(np.max(terrain_grid))
        }
    
    def _generate_martian_terrain(self, region_type: str) -> Dict[str, Any]:
        """Generate Martian terrain data."""
        grid_size = 50
        terrain_grid = np.zeros((grid_size, grid_size))
        
        if region_type == "plains":
            # Gently rolling plains with occasional hills
            base_elevation = 0
            for i in range(grid_size):
                for j in range(grid_size):
                    terrain_grid[i][j] = 50 * math.sin(i * 0.1) * math.cos(j * 0.1)
                    # Add some random hills
                    if random.random() < 0.1:
                        terrain_grid[i][j] += random.randint(20, 100)
                        
        elif region_type == "canyon":
            # Deep canyon system
            base_elevation = 1000
            canyon_depth = 2000
            for i in range(grid_size):
                for j in range(grid_size):
                    # Create winding canyon
                    canyon_center = grid_size // 2 + 5 * math.sin(i * 0.3)
                    dist_to_canyon = abs(j - canyon_center)
                    if dist_to_canyon < 10:
                        terrain_grid[i][j] = -canyon_depth * (1 - dist_to_canyon/10)
                        
        elif region_type == "polar":
            # Layered polar terrain with ice caps
            base_elevation = 500
            for i in range(grid_size):
                for j in range(grid_size):
                    # Create layered appearance
                    terrain_grid[i][j] = 50 * math.sin(i * 0.5) + 30 * math.cos(j * 0.3)
                    
        elif region_type == "highland":
            # Ancient highland terrain
            base_elevation = 2000
            for i in range(grid_size):
                for j in range(grid_size):
                    terrain_grid[i][j] = 200 * random.random() - 100
        
        terrain_grid += base_elevation
        
        return {
            'type': region_type,
            'elevation_grid': terrain_grid.tolist(),
            'grid_size': grid_size,
            'scale_meters_per_cell': 200,  # 200m per cell
            'min_elevation': float(np.min(terrain_grid)),
            'max_elevation': float(np.max(terrain_grid))
        }
    
    def _generate_lunar_surface_features(self, region_type: str, size_km: float) -> List[Dict]:
        """Generate surface features for lunar environment."""
        features = []
        
        if region_type == "highland":
            # Craters and boulders
            for _ in range(random.randint(5, 15)):
                features.append({
                    'type': 'crater',
                    'position': [random.uniform(0, size_km), random.uniform(0, size_km)],
                    'diameter': random.uniform(50, 500),
                    'depth': random.uniform(5, 50)
                })
            
            for _ in range(random.randint(20, 50)):
                features.append({
                    'type': 'boulder',
                    'position': [random.uniform(0, size_km), random.uniform(0, size_km)],
                    'size': random.uniform(1, 10)
                })
                
        elif region_type == "mare":
            # Wrinkle ridges and rilles
            for _ in range(random.randint(2, 5)):
                features.append({
                    'type': 'wrinkle_ridge',
                    'position': [random.uniform(0, size_km), random.uniform(0, size_km)],
                    'length': random.uniform(1000, 5000),
                    'height': random.uniform(20, 100)
                })
        
        return features
    
    def _generate_martian_surface_features(self, region_type: str) -> List[Dict]:
        """Generate surface features for Martian environment."""
        features = []
        
        if region_type == "plains":
            # Dust devils and rock formations
            for _ in range(random.randint(3, 8)):
                features.append({
                    'type': 'dust_devil',
                    'position': [random.uniform(0, 10), random.uniform(0, 10)],
                    'height': random.uniform(100, 1000),
                    'intensity': random.uniform(0.3, 0.8)
                })
                
        elif region_type == "canyon":
            # Layered rock formations
            for _ in range(random.randint(10, 20)):
                features.append({
                    'type': 'rock_layer',
                    'position': [random.uniform(0, 10), random.uniform(0, 10)],
                    'thickness': random.uniform(5, 50),
                    'composition': random.choice(['iron_oxide', 'sulfate', 'basalt'])
                })
        
        return features
    
    def _generate_lunar_lighting(self) -> Dict[str, Any]:
        """Generate lighting conditions for lunar environment."""
        # Lunar day is about 29.5 Earth days
        time_of_day = random.uniform(0, 29.5)
        
        if time_of_day < 14.75:  # Lunar day
            sun_angle = (time_of_day / 14.75) * 180
            lighting = {
                'sun_elevation': sun_angle,
                'sun_azimuth': random.uniform(0, 360),
                'ambient_light': 0.1,
                'direct_light': 1.0,
                'shadow_intensity': 0.95,
                'earth_shine': 0.05 if time_of_day > 7 else 0.1
            }
        else:  # Lunar night
            lighting = {
                'sun_elevation': -10,
                'sun_azimuth': 0,
                'ambient_light': 0.02,
                'direct_light': 0.0,
                'shadow_intensity': 0.0,
                'earth_shine': 0.2,
                'star_field': True
            }
        
        return lighting
    
    def _generate_martian_weather(self, season: str, region_type: str) -> Dict[str, Any]:
        """Generate weather patterns for Martian environment."""
        base_temp = {
            'spring': -60,
            'summer': -40,
            'fall': -70,
            'winter': -90
        }.get(season, -60)
        
        return {
            'temperature': base_temp + random.uniform(-20, 20),
            'wind_speed': random.uniform(5, 40),  # m/s
            'wind_direction': random.uniform(0, 360),
            'dust_opacity': random.uniform(0.1, 0.8),
            'pressure': 610 + random.uniform(-100, 100),
            'humidity': random.uniform(0, 0.03),  # Very low humidity
            'dust_storm_probability': 0.3 if season == 'spring' else 0.1
        }
    
    def _generate_martian_atmospheric_effects(self, weather_data: Dict) -> Dict[str, Any]:
        """Generate atmospheric visual effects for Mars."""
        return {
            'sky_color': {
                'daytime': [0.7, 0.5, 0.3],  # Butterscotch
                'sunset': [0.6, 0.4, 0.8],   # Blue sunset
                'dust_storm': [0.8, 0.6, 0.4]
            },
            'dust_particles': {
                'density': weather_data['dust_opacity'],
                'size_distribution': [0.1, 0.5, 1.0],
                'visibility_km': 50 / weather_data['dust_opacity']
            },
            'atmospheric_distortion': weather_data['dust_opacity'] * 0.5
        }
    
    def _generate_environment_description(self, planet_type: str, region_type: str, 
                                        terrain_data: Dict, surface_features: List, 
                                        weather_data: Dict = None) -> str:
        """Generate AI-powered environment description."""
        if not self.openai_api_key:
            # Fallback to template-based descriptions
            return self._generate_template_description(planet_type, region_type, terrain_data)
        
        try:
            # This would integrate with OpenAI API when available
            # For now, return detailed template description
            return self._generate_detailed_description(planet_type, region_type, terrain_data, 
                                                     surface_features, weather_data)
        except Exception as e:
            logger.error(f"Error generating AI description: {e}")
            return self._generate_template_description(planet_type, region_type, terrain_data)
    
    def _generate_template_description(self, planet_type: str, region_type: str, 
                                     terrain_data: Dict) -> str:
        """Generate template-based environment description."""
        descriptions = {
            'lunar_highland': "Ancient lunar highlands stretch before you, scarred by billions of years of meteorite impacts. The terrain is heavily cratered with steep-walled formations and boulder fields scattered across the regolith surface.",
            'lunar_mare': "The dark basaltic plains of the lunar mare create a smooth, rolling landscape. Ancient lava flows have created a relatively flat surface broken only by occasional wrinkle ridges and rilles.",
            'martian_plains': "The rust-colored Martian plains extend to the horizon, dotted with ancient impact craters and carved by dust devils. The thin atmosphere creates a butterscotch sky above the iron oxide landscape.",
            'martian_canyon': "A massive canyon system cuts through the Martian highlands, revealing layers of geological history in its walls. The stratified rock formations tell the story of Mars' ancient past."
        }
        
        key = f"{planet_type}_{region_type}"
        return descriptions.get(key, f"A {planet_type} {region_type} environment with diverse terrain features.")
    
    def _generate_detailed_description(self, planet_type: str, region_type: str, 
                                     terrain_data: Dict, surface_features: List, 
                                     weather_data: Dict = None) -> str:
        """Generate detailed environment description with all features."""
        elevation_range = terrain_data['max_elevation'] - terrain_data['min_elevation']
        feature_count = len(surface_features)
        
        base_desc = self._generate_template_description(planet_type, region_type, terrain_data)
        
        details = [
            f"Elevation varies from {terrain_data['min_elevation']:.0f}m to {terrain_data['max_elevation']:.0f}m",
            f"The landscape contains {feature_count} notable surface features"
        ]
        
        if weather_data:
            details.append(f"Current temperature: {weather_data['temperature']:.1f}°C")
            details.append(f"Wind speed: {weather_data['wind_speed']:.1f} m/s")
            if weather_data['dust_opacity'] > 0.5:
                details.append("Dust storm conditions reduce visibility")
        
        return base_desc + " " + ". ".join(details) + "."
    
    def get_environment_presets(self) -> Dict[str, List[Dict]]:
        """Get predefined environment presets for quick selection."""
        return {
            'lunar': [
                {'name': 'Apollo Landing Site', 'type': 'mare', 'description': 'Smooth basaltic plains similar to Apollo landing sites'},
                {'name': 'Lunar Highlands', 'type': 'highland', 'description': 'Ancient cratered highlands with rugged terrain'},
                {'name': 'Lunar South Pole', 'type': 'polar', 'description': 'Permanently shadowed craters with water ice'},
                {'name': 'Large Impact Crater', 'type': 'crater', 'description': 'Multi-ring impact crater with central peak'}
            ],
            'martian': [
                {'name': 'Amazonis Planitia', 'type': 'plains', 'description': 'Smooth volcanic plains with dust devil activity'},
                {'name': 'Valles Marineris', 'type': 'canyon', 'description': 'Massive canyon system with layered walls'},
                {'name': 'Olympus Mons', 'type': 'highland', 'description': 'Volcanic highland with steep escarpments'},
                {'name': 'Polar Ice Cap', 'type': 'polar', 'description': 'Layered ice deposits with seasonal changes'}
            ]
        }