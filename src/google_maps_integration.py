"""
Google Maps Integration for Enhanced 3D Environment Visualization

Provides real-world terrain data and satellite imagery for drone simulations
using Google Maps API for authentic geographic environments.
"""

import os
import requests
import json
import numpy as np
from typing import Dict, List, Tuple, Optional
import logging
from PIL import Image
import io
import base64

logger = logging.getLogger(__name__)

class GoogleMapsIntegration:
    """
    Integration with Google Maps API for real-world terrain and satellite data.
    """
    
    def __init__(self):
        """Initialize Google Maps integration."""
        # Get API key from environment variable
        self.api_key = os.environ.get('GOOGLE_MAPS_API_KEY')
        self.base_url = "https://maps.googleapis.com/maps/api"
        
    def set_api_key(self, api_key: str):
        """Set the Google Maps API key."""
        self.api_key = api_key
        
    def get_satellite_imagery(self, latitude: float, longitude: float, 
                            zoom: int = 18, size: str = "640x640") -> Optional[bytes]:
        """
        Get satellite imagery for a specific location.
        
        Args:
            latitude: Latitude coordinate
            longitude: Longitude coordinate  
            zoom: Zoom level (1-20)
            size: Image size (max 640x640 for free tier)
            
        Returns:
            Image data as bytes or None if error
        """
        if not self.api_key:
            logger.error("Google Maps API key not set")
            return None
            
        try:
            url = f"{self.base_url}/staticmap"
            params = {
                'center': f"{latitude},{longitude}",
                'zoom': zoom,
                'size': size,
                'maptype': 'satellite',
                'key': self.api_key
            }
            
            response = requests.get(url, params=params)
            response.raise_for_status()
            
            return response.content
            
        except Exception as e:
            logger.error(f"Error fetching satellite imagery: {e}")
            return None
    
    def get_elevation_data(self, latitude: float, longitude: float, 
                          grid_size: int = 10) -> Optional[np.ndarray]:
        """
        Get elevation data for terrain modeling.
        
        Args:
            latitude: Center latitude
            longitude: Center longitude
            grid_size: Size of elevation grid
            
        Returns:
            2D numpy array of elevation data
        """
        if not self.api_key:
            logger.error("Google Maps API key not set")
            return None
            
        try:
            # Create grid of coordinates around center point
            lat_offset = 0.001 * grid_size / 2  # Approximate offset
            lon_offset = 0.001 * grid_size / 2
            
            locations = []
            for i in range(grid_size):
                for j in range(grid_size):
                    lat = latitude - lat_offset + (i * 2 * lat_offset / (grid_size - 1))
                    lon = longitude - lon_offset + (j * 2 * lon_offset / (grid_size - 1))
                    locations.append(f"{lat},{lon}")
            
            # Google Elevation API has limit of 512 locations per request
            if len(locations) > 512:
                locations = locations[:512]
                grid_size = int(np.sqrt(512))
            
            url = f"{self.base_url}/elevation/json"
            params = {
                'locations': '|'.join(locations),
                'key': self.api_key
            }
            
            response = requests.get(url, params=params)
            response.raise_for_status()
            
            data = response.json()
            
            if data.get('status') == 'OK':
                elevations = [result['elevation'] for result in data['results']]
                return np.array(elevations).reshape(grid_size, grid_size)
            else:
                logger.error(f"Elevation API error: {data.get('error_message', 'Unknown error')}")
                return None
                
        except Exception as e:
            logger.error(f"Error fetching elevation data: {e}")
            return None
    
    def create_3d_terrain_model(self, latitude: float, longitude: float, 
                              grid_size: int = 20) -> Optional[Dict]:
        """
        Create a 3D terrain model combining satellite imagery and elevation data.
        
        Args:
            latitude: Center latitude
            longitude: Center longitude
            grid_size: Resolution of terrain grid
            
        Returns:
            Dictionary containing terrain model data
        """
        try:
            logger.info(f"Creating 3D terrain model for {latitude}, {longitude} with grid size {grid_size}")
            
            # For now, create a synthetic terrain model while we test the API
            # This ensures the web interface works while we debug the Google Maps API
            if not self.api_key:
                logger.warning("Google Maps API key not available, creating synthetic terrain")
                return self._create_synthetic_terrain(latitude, longitude, grid_size)
            
            # Get elevation data first (more critical for 3D visualization)
            elevation_data = self.get_elevation_data(latitude, longitude, grid_size)
            if elevation_data is None:
                logger.warning("Failed to get real elevation data, creating synthetic terrain")
                return self._create_synthetic_terrain(latitude, longitude, grid_size)
            
            # Try to get satellite imagery (optional)
            satellite_image = self.get_satellite_imagery(latitude, longitude, zoom=18)
            satellite_b64 = None
            if satellite_image:
                satellite_b64 = base64.b64encode(satellite_image).decode('utf-8')
            
            return {
                'location': {
                    'latitude': latitude,
                    'longitude': longitude
                },
                'satellite_image': satellite_b64,
                'elevation_data': elevation_data.tolist(),
                'grid_size': grid_size,
                'bounds': {
                    'min_elevation': float(np.min(elevation_data)),
                    'max_elevation': float(np.max(elevation_data))
                },
                'source': 'google_maps'
            }
            
        except Exception as e:
            logger.error(f"Error creating 3D terrain model: {e}")
            logger.info("Falling back to synthetic terrain model")
            return self._create_synthetic_terrain(latitude, longitude, grid_size)
    
    def _create_synthetic_terrain(self, latitude: float, longitude: float, grid_size: int) -> Dict:
        """Create a synthetic terrain model for demonstration purposes."""
        try:
            # Create realistic terrain based on known location characteristics
            if abs(latitude - 36.1069) < 0.1 and abs(longitude + 112.1129) < 0.1:
                # Grand Canyon - deep canyon terrain
                elevation_data = self._generate_canyon_terrain(grid_size)
                description = "Grand Canyon synthetic terrain with deep canyon features"
            elif abs(latitude - 46.8523) < 0.1 and abs(longitude + 121.7603) < 0.1:
                # Mount Rainier - mountain terrain  
                elevation_data = self._generate_mountain_terrain(grid_size)
                description = "Mount Rainier synthetic terrain with volcanic peaks"
            else:
                # General mountainous terrain
                elevation_data = self._generate_general_terrain(grid_size)
                description = "Synthetic mountainous terrain"
            
            return {
                'location': {
                    'latitude': latitude,
                    'longitude': longitude
                },
                'satellite_image': None,
                'elevation_data': elevation_data.tolist(),
                'grid_size': grid_size,
                'bounds': {
                    'min_elevation': float(np.min(elevation_data)),
                    'max_elevation': float(np.max(elevation_data))
                },
                'source': 'synthetic',
                'description': description
            }
            
        except Exception as e:
            logger.error(f"Error creating synthetic terrain: {e}")
            return None
    
    def _generate_canyon_terrain(self, grid_size: int) -> np.ndarray:
        """Generate canyon-like terrain."""
        x = np.linspace(-1, 1, grid_size)
        y = np.linspace(-1, 1, grid_size)
        X, Y = np.meshgrid(x, y)
        
        # Create canyon effect with sharp drops
        elevation = 2000 - 1500 * np.exp(-2 * (X**2 + Y**2))
        elevation += 300 * np.sin(3 * X) * np.cos(2 * Y)  # Add canyon walls
        return elevation
    
    def _generate_mountain_terrain(self, grid_size: int) -> np.ndarray:
        """Generate mountain-like terrain."""
        x = np.linspace(-1, 1, grid_size)
        y = np.linspace(-1, 1, grid_size)
        X, Y = np.meshgrid(x, y)
        
        # Create mountain peaks
        elevation = 3000 + 1000 * np.exp(-2 * (X**2 + Y**2))
        elevation += 500 * np.sin(2 * X) * np.cos(3 * Y)  # Add ridges
        return elevation
    
    def _generate_general_terrain(self, grid_size: int) -> np.ndarray:
        """Generate general hilly terrain."""
        x = np.linspace(-1, 1, grid_size)
        y = np.linspace(-1, 1, grid_size)
        X, Y = np.meshgrid(x, y)
        
        # Create rolling hills
        elevation = 1000 + 300 * np.sin(2 * X) * np.cos(2 * Y)
        elevation += 200 * np.sin(4 * X) + 150 * np.cos(3 * Y)
        return elevation
    
    def get_popular_locations(self) -> List[Dict]:
        """
        Get list of popular locations for drone simulation.
        
        Returns:
            List of location dictionaries with coordinates and descriptions
        """
        return [
            {
                'name': 'Grand Canyon, Arizona',
                'latitude': 36.1069,
                'longitude': -112.1129,
                'description': 'Spectacular canyon terrain with diverse elevation changes'
            },
            {
                'name': 'Yosemite Valley, California',
                'latitude': 37.7459,
                'longitude': -119.5936,
                'description': 'Mountain valley with granite cliffs and waterfalls'
            },
            {
                'name': 'Mount Rainier, Washington',
                'latitude': 46.8523,
                'longitude': -121.7603,
                'description': 'Volcanic mountain with glaciers and alpine terrain'
            },
            {
                'name': 'Sahara Desert, Morocco',
                'latitude': 31.0424,
                'longitude': -7.9999,
                'description': 'Desert dunes and arid landscape'
            },
            {
                'name': 'Swiss Alps, Switzerland',
                'latitude': 46.5197,
                'longitude': 7.6815,
                'description': 'High alpine terrain with peaks and valleys'
            },
            {
                'name': 'Patagonia, Argentina',
                'latitude': -50.9423,
                'longitude': -73.4068,
                'description': 'Rugged mountain landscape with glacial features'
            }
        ]
    
    def search_locations(self, query: str) -> Optional[List[Dict]]:
        """
        Search for locations using Google Places API.
        
        Args:
            query: Search query (e.g., "mountain", "desert", "canyon")
            
        Returns:
            List of location results
        """
        if not self.api_key:
            logger.error("Google Maps API key not set")
            return None
            
        try:
            url = f"{self.base_url}/place/textsearch/json"
            params = {
                'query': query,
                'key': self.api_key
            }
            
            response = requests.get(url, params=params)
            response.raise_for_status()
            
            data = response.json()
            
            if data.get('status') == 'OK':
                results = []
                for place in data['results'][:10]:  # Limit to top 10 results
                    location = place.get('geometry', {}).get('location', {})
                    results.append({
                        'name': place.get('name', 'Unknown'),
                        'latitude': location.get('lat'),
                        'longitude': location.get('lng'),
                        'description': place.get('formatted_address', ''),
                        'rating': place.get('rating'),
                        'types': place.get('types', [])
                    })
                return results
            else:
                logger.error(f"Places API error: {data.get('error_message', 'Unknown error')}")
                return None
                
        except Exception as e:
            logger.error(f"Error searching locations: {e}")
            return None