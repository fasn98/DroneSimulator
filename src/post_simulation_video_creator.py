"""
Post-Simulation Video Creator for Drone Flight Recording

Creates high-quality MP4 videos after simulation completion using stored telemetry data
and customizable templates for different visualization styles.
"""

import os
import cv2
import numpy as np
import json
import math
import random
import subprocess
from typing import Dict, List, Tuple, Optional, Any
import logging
from datetime import datetime
import threading
import time
import requests
from PIL import Image
import io

logger = logging.getLogger(__name__)

class PostSimulationVideoCreator:
    """
    Creates videos after simulation completion using stored telemetry data.
    """
    
    def __init__(self):
        """Initialize the post-simulation video creator."""
        self.video_directory = "video_exports"
        self.audio_directory = "audio_assets"
        self._ensure_directories()
        
        # Video templates
        self.templates = {
            "professional": {
                "name": "Professional View",
                "description": "Clean, technical view with telemetry overlays",
                "frame_size": (1920, 1080),
                "fps": 30,
                "background_color": (40, 40, 40),
                "show_telemetry": True,
                "show_trajectory": True,
                "camera_style": "smooth_follow"
            },
            "cinematic": {
                "name": "Cinematic View", 
                "description": "Dramatic camera angles with environment effects",
                "frame_size": (1920, 1080),
                "fps": 24,
                "background_color": (20, 30, 40),
                "show_telemetry": False,
                "show_trajectory": False,
                "camera_style": "dynamic_angles"
            },
            "technical": {
                "name": "Technical Analysis",
                "description": "Detailed technical data with graphs and metrics",
                "frame_size": (1920, 1080),
                "fps": 30,
                "background_color": (0, 0, 0),
                "show_telemetry": True,
                "show_trajectory": True,
                "camera_style": "fixed_overview"
            }
        }
        
    def _ensure_directories(self):
        """Ensure required directories exist."""
        for directory in [self.video_directory, self.audio_directory]:
            if not os.path.exists(directory):
                os.makedirs(directory)
    
    def create_video_from_telemetry(self, telemetry_data: List[Dict], template: str = "professional", 
                                   drone_config: Dict = None, environment_config: Dict = None,
                                   mission_config: Dict = None, include_audio: bool = True) -> Dict:
        """
        Create video from stored telemetry data using specified template.
        
        Args:
            telemetry_data: List of telemetry data points
            template: Template name to use
            drone_config: Drone configuration
            environment_config: Environment configuration  
            mission_config: Mission configuration
            include_audio: Whether to include sci-fi audio
            
        Returns:
            Dictionary with video creation results
        """
        try:
            if not telemetry_data:
                return {"success": False, "error": "No telemetry data provided"}
            
            if template not in self.templates:
                template = "professional"
            
            template_config = self.templates[template]
            
            # Generate unique filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            video_filename = f"drone_flight_{template}_{timestamp}.mp4"
            video_path = os.path.join(self.video_directory, video_filename)
            
            logger.info(f"Creating video from {len(telemetry_data)} telemetry points")
            
            # Create video frames
            success = self._create_video_frames(
                telemetry_data, video_path, template_config,
                drone_config, environment_config, mission_config
            )
            
            if not success:
                return {"success": False, "error": "Failed to create video frames"}
            
            # Add audio if requested
            if include_audio:
                audio_video_path = video_path.replace(".mp4", "_with_audio.mp4")
                if self._add_scifi_audio(video_path, audio_video_path):
                    # Replace original with audio version
                    os.replace(audio_video_path, video_path)
                    logger.info("Added sci-fi audio to video")
            
            # Get video file info
            file_size = os.path.getsize(video_path) / (1024 * 1024)  # MB
            
            return {
                "success": True,
                "filename": video_filename,
                "path": video_path,
                "size_mb": round(file_size, 2),
                "duration": len(telemetry_data) / template_config["fps"],
                "template": template,
                "frames": len(telemetry_data)
            }
            
        except Exception as e:
            logger.error(f"Error creating video from telemetry: {e}")
            return {"success": False, "error": str(e)}
    
    def _create_video_frames(self, telemetry_data: List[Dict], video_path: str, 
                           template_config: Dict, drone_config: Dict = None,
                           environment_config: Dict = None, mission_config: Dict = None) -> bool:
        """
        Create video frames from telemetry data.
        
        Args:
            telemetry_data: List of telemetry data points
            video_path: Output video path
            template_config: Template configuration
            drone_config: Drone configuration
            environment_config: Environment configuration
            mission_config: Mission configuration
            
        Returns:
            Boolean indicating success
        """
        try:
            # Initialize video writer
            # Use XVID codec for initial creation, then convert to H.264
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            frame_size = template_config["frame_size"]
            fps = template_config["fps"]
            
            # Create with .avi extension first to avoid MP4 corruption
            temp_video_path = video_path.replace('.mp4', '_raw.avi')
            out = cv2.VideoWriter(temp_video_path, fourcc, fps, frame_size)
            
            if not out.isOpened():
                logger.error("Failed to open video writer")
                return False
            
            # Calculate trajectory bounds for scaling
            positions = [(point.get("position", {}).get("x", 0), 
                         point.get("position", {}).get("y", 0),
                         point.get("position", {}).get("z", 0)) for point in telemetry_data]
            
            if not positions:
                logger.error("No position data found in telemetry")
                return False
            
            x_coords = [pos[0] for pos in positions]
            y_coords = [pos[1] for pos in positions]
            z_coords = [pos[2] for pos in positions]
            
            # Calculate bounds with padding
            x_min, x_max = min(x_coords), max(x_coords)
            y_min, y_max = min(y_coords), max(y_coords)
            z_min, z_max = min(z_coords), max(z_coords)
            
            # Add padding
            x_range = max(x_max - x_min, 100)
            y_range = max(y_max - y_min, 100)
            z_range = max(z_max - z_min, 50)
            
            # Create frames
            for i, telemetry_point in enumerate(telemetry_data):
                frame = self._create_frame(
                    telemetry_point, telemetry_data[:i+1], i, template_config,
                    (x_min, x_max, y_min, y_max, z_min, z_max),
                    (x_range, y_range, z_range),
                    drone_config, environment_config, mission_config
                )
                
                if frame is not None:
                    out.write(frame)
                else:
                    logger.warning(f"Failed to create frame {i}")
            
            out.release()
            
            # Convert from AVI to MP4 H.264 for compatibility
            self._convert_avi_to_h264(temp_video_path, video_path)
            
            logger.info(f"Successfully created video: {video_path}")
            return True
            
        except Exception as e:
            logger.error(f"Error creating video frames: {e}")
            return False
    
    def _create_frame(self, telemetry_point: Dict, trajectory_history: List[Dict], 
                     frame_index: int, template_config: Dict, bounds: Tuple,
                     ranges: Tuple, drone_config: Dict = None, 
                     environment_config: Dict = None, mission_config: Dict = None) -> np.ndarray:
        """
        Create a single video frame.
        
        Args:
            telemetry_point: Current telemetry data point
            trajectory_history: Historical trajectory data
            frame_index: Current frame index
            template_config: Template configuration
            bounds: Trajectory bounds (x_min, x_max, y_min, y_max, z_min, z_max)
            ranges: Trajectory ranges (x_range, y_range, z_range)
            drone_config: Drone configuration
            environment_config: Environment configuration
            mission_config: Mission configuration
            
        Returns:
            OpenCV frame as numpy array
        """
        try:
            frame_size = template_config["frame_size"]
            width, height = frame_size
            
            # Create background frame with terrain/location-specific visuals
            frame = self._create_background_frame(height, width, template_config, 
                                                 environment_config, mission_config)
            
            # Get current position
            position = telemetry_point.get("position", {})
            current_x = position.get("x", 0)
            current_y = position.get("y", 0) 
            current_z = position.get("z", 0)
            
            # Calculate screen coordinates
            x_min, x_max, y_min, y_max, z_min, z_max = bounds
            x_range, y_range, z_range = ranges
            
            # Map to screen coordinates (using 70% of screen for flight area)
            flight_area_width = int(width * 0.7)
            flight_area_height = int(height * 0.7)
            flight_area_x = int(width * 0.15)
            flight_area_y = int(height * 0.15)
            
            # Convert world coordinates to screen coordinates
            if x_range > 0:
                screen_x = int(flight_area_x + (current_x - x_min) / x_range * flight_area_width)
            else:
                screen_x = flight_area_x + flight_area_width // 2
                
            if y_range > 0:
                screen_y = int(flight_area_y + flight_area_height - (current_y - y_min) / y_range * flight_area_height)
            else:
                screen_y = flight_area_y + flight_area_height // 2
            
            # Draw environment background - use smart background detection
            background = self._create_background_frame(frame.shape[0], frame.shape[1], template_config, environment_config)
            frame[:] = background
            
            # Draw trajectory history
            if template_config.get("show_trajectory", False) and len(trajectory_history) > 1:
                self._draw_trajectory(frame, trajectory_history, bounds, ranges, 
                                    flight_area_x, flight_area_y, flight_area_width, flight_area_height)
            
            # Draw drone
            self._draw_drone(frame, screen_x, screen_y, current_z, z_max, telemetry_point)
            
            # Draw telemetry overlays
            if template_config.get("show_telemetry", False):
                self._draw_telemetry_overlay(frame, telemetry_point, frame_index, template_config)
            
            # Draw mission info
            if mission_config:
                self._draw_mission_info(frame, telemetry_point, mission_config)
            
            return frame
            
        except Exception as e:
            logger.error(f"Error creating frame: {e}")
            return None
    
    def _draw_environment_background(self, frame: np.ndarray, template_config: Dict, 
                                   environment_config: Dict = None):
        """Draw environment-specific background."""
        height, width = frame.shape[:2]
        
        if environment_config:
            env_name = environment_config.get("name", "earth").lower()
            
            if env_name == "mars":
                # Mars-like reddish background
                color = (40, 60, 120)  # BGR
            elif env_name == "moon":
                # Moon-like gray background
                color = (60, 60, 60)
            else:
                # Earth-like blue background
                color = (120, 80, 40)
            
            # Create gradient background
            for y in range(height):
                intensity = 0.3 + 0.7 * (y / height)
                line_color = tuple(int(c * intensity) for c in color)
                cv2.line(frame, (0, y), (width, y), line_color, 1)
    
    def _draw_trajectory(self, frame: np.ndarray, trajectory_history: List[Dict], 
                        bounds: Tuple, ranges: Tuple, flight_area_x: int, 
                        flight_area_y: int, flight_area_width: int, flight_area_height: int):
        """Draw drone trajectory history."""
        if len(trajectory_history) < 2:
            return
        
        x_min, x_max, y_min, y_max, z_min, z_max = bounds
        x_range, y_range, z_range = ranges
        
        # Convert trajectory to screen coordinates
        screen_points = []
        for point in trajectory_history:
            position = point.get("position", {})
            x = position.get("x", 0)
            y = position.get("y", 0)
            
            if x_range > 0:
                screen_x = int(flight_area_x + (x - x_min) / x_range * flight_area_width)
            else:
                screen_x = flight_area_x + flight_area_width // 2
                
            if y_range > 0:
                screen_y = int(flight_area_y + flight_area_height - (y - y_min) / y_range * flight_area_height)
            else:
                screen_y = flight_area_y + flight_area_height // 2
            
            screen_points.append((screen_x, screen_y))
        
        # Draw trajectory line
        for i in range(1, len(screen_points)):
            alpha = i / len(screen_points)  # Fade older trajectory
            color = (int(255 * alpha), int(180 * alpha), int(50 * alpha))  # Yellow to orange
            cv2.line(frame, screen_points[i-1], screen_points[i], color, 2)
    
    def _draw_drone(self, frame: np.ndarray, screen_x: int, screen_y: int, 
                   altitude: float, max_altitude: float, telemetry_point: Dict):
        """Draw drone representation."""
        # Drone size based on altitude
        base_size = 15
        altitude_factor = max(0.5, min(2.0, 1.0 + altitude / max(max_altitude, 100)))
        drone_size = int(base_size * altitude_factor)
        
        # Draw drone body
        cv2.circle(frame, (screen_x, screen_y), drone_size, (0, 255, 255), -1)  # Yellow body
        cv2.circle(frame, (screen_x, screen_y), drone_size, (0, 200, 200), 2)   # Border
        
        # Draw propellers
        prop_distance = drone_size + 5
        propeller_positions = [
            (screen_x - prop_distance, screen_y - prop_distance),
            (screen_x + prop_distance, screen_y - prop_distance),
            (screen_x - prop_distance, screen_y + prop_distance),
            (screen_x + prop_distance, screen_y + prop_distance)
        ]
        
        for prop_x, prop_y in propeller_positions:
            cv2.circle(frame, (prop_x, prop_y), 4, (255, 255, 255), -1)
        
        # Draw altitude indicator
        altitude_text = f"ALT: {altitude:.1f}m"
        cv2.putText(frame, altitude_text, (screen_x - 30, screen_y - drone_size - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def _draw_telemetry_overlay(self, frame: np.ndarray, telemetry_point: Dict, 
                               frame_index: int, template_config: Dict):
        """Draw telemetry data overlay."""
        height, width = frame.shape[:2]
        
        # Telemetry panel background
        panel_x, panel_y = 10, 10
        panel_width, panel_height = 300, 200
        
        # Semi-transparent background
        overlay = frame.copy()
        cv2.rectangle(overlay, (panel_x, panel_y), 
                     (panel_x + panel_width, panel_y + panel_height), 
                     (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # Draw telemetry text
        y_offset = panel_y + 20
        line_height = 20
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        color = (255, 255, 255)
        
        # Time
        time_val = telemetry_point.get("timestamp", 0)
        cv2.putText(frame, f"Time: {time_val:.2f}s", 
                   (panel_x + 10, y_offset), font, font_scale, color, 1)
        y_offset += line_height
        
        # Position
        position = telemetry_point.get("position", {})
        cv2.putText(frame, f"X: {position.get('x', 0):.1f}m", 
                   (panel_x + 10, y_offset), font, font_scale, color, 1)
        y_offset += line_height
        
        cv2.putText(frame, f"Y: {position.get('y', 0):.1f}m", 
                   (panel_x + 10, y_offset), font, font_scale, color, 1)
        y_offset += line_height
        
        cv2.putText(frame, f"Z: {position.get('z', 0):.1f}m", 
                   (panel_x + 10, y_offset), font, font_scale, color, 1)
        y_offset += line_height
        
        # Velocity
        velocity = telemetry_point.get("velocity", {})
        speed = math.sqrt(velocity.get("x", 0)**2 + velocity.get("y", 0)**2 + velocity.get("z", 0)**2)
        cv2.putText(frame, f"Speed: {speed:.1f}m/s", 
                   (panel_x + 10, y_offset), font, font_scale, color, 1)
        y_offset += line_height
        
        # Mission progress
        progress = telemetry_point.get("mission_progress", 0)
        cv2.putText(frame, f"Progress: {progress:.1f}%", 
                   (panel_x + 10, y_offset), font, font_scale, color, 1)
    
    def _draw_mission_info(self, frame: np.ndarray, telemetry_point: Dict, mission_config: Dict):
        """Draw mission information."""
        height, width = frame.shape[:2]
        
        # Mission info panel
        panel_x = width - 250
        panel_y = 10
        panel_width = 240
        panel_height = 100
        
        # Semi-transparent background
        overlay = frame.copy()
        cv2.rectangle(overlay, (panel_x, panel_y), 
                     (panel_x + panel_width, panel_y + panel_height), 
                     (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # Mission info
        y_offset = panel_y + 20
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        color = (255, 255, 255)
        
        mission_type = mission_config.get("type", "Unknown")
        cv2.putText(frame, f"Mission: {mission_type}", 
                   (panel_x + 10, y_offset), font, font_scale, color, 1)
        y_offset += 20
        
        current_waypoint = telemetry_point.get("current_waypoint", 0)
        waypoints = mission_config.get("waypoints", [])
        total_waypoints = len(waypoints)
        
        cv2.putText(frame, f"Waypoint: {current_waypoint}/{total_waypoints}", 
                   (panel_x + 10, y_offset), font, font_scale, color, 1)
        y_offset += 20
        
        if current_waypoint < len(waypoints):
            current_action = waypoints[current_waypoint].get("action", "Unknown")
            cv2.putText(frame, f"Action: {current_action}", 
                       (panel_x + 10, y_offset), font, font_scale, color, 1)
    
    def _convert_avi_to_h264(self, input_avi_path: str, output_mp4_path: str):
        """Convert AVI video to H.264 MP4 format for better compatibility."""
        try:
            cmd = [
                'ffmpeg', '-y',
                '-i', input_avi_path,
                '-c:v', 'libx264',
                '-preset', 'medium',
                '-crf', '23',
                '-pix_fmt', 'yuv420p',
                '-movflags', '+faststart',
                output_mp4_path
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                # Verify output file was created and is valid
                if os.path.exists(output_mp4_path) and os.path.getsize(output_mp4_path) > 0:
                    # Remove the temporary AVI file
                    if os.path.exists(input_avi_path):
                        os.remove(input_avi_path)
                    logger.info("Successfully converted AVI to H.264 MP4")
                else:
                    logger.error("H.264 conversion produced empty file")
                    return False
            else:
                logger.error(f"FFmpeg conversion failed: {result.stderr}")
                return False
                
            return True
                    
        except Exception as e:
            logger.error(f"Error converting AVI to H.264: {e}")
            return False
    
    def _convert_to_h264(self, video_path: str):
        """Convert video to H.264 format for better compatibility (legacy method)."""
        try:
            temp_path = video_path.replace(".mp4", "_temp.mp4")
            
            cmd = [
                'ffmpeg', '-y',
                '-i', video_path,
                '-c:v', 'libx264',
                '-preset', 'medium',
                '-crf', '23',
                '-pix_fmt', 'yuv420p',
                '-movflags', '+faststart',
                temp_path
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                # Verify temp file was created and is valid
                if os.path.exists(temp_path) and os.path.getsize(temp_path) > 0:
                    os.replace(temp_path, video_path)
                    logger.info("Successfully converted to H.264")
                else:
                    logger.warning("H.264 conversion produced empty file, keeping original")
            else:
                logger.warning(f"H.264 conversion failed: {result.stderr}")
                # Remove temp file if it exists
                if os.path.exists(temp_path):
                    os.remove(temp_path)
                    
        except Exception as e:
            logger.warning(f"Error converting to H.264: {e}")
            # Clean up temp file if it exists
            temp_path = video_path.replace(".mp4", "_temp.mp4")
            if os.path.exists(temp_path):
                os.remove(temp_path)
    
    def _add_scifi_audio(self, video_path: str, output_path: str) -> bool:
        """Add sci-fi audio to video."""
        try:
            # Generate audio track
            audio_file = self._generate_discovery_audio()
            
            if not audio_file:
                return False
            
            cmd = [
                'ffmpeg', '-y',
                '-i', video_path,
                '-i', audio_file,
                '-c:v', 'copy',
                '-c:a', 'aac',
                '-b:a', '128k',
                '-map', '0:v',
                '-map', '1:a',
                '-shortest',
                output_path
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            return result.returncode == 0
            
        except Exception as e:
            logger.error(f"Error adding audio: {e}")
            return False
    
    def _create_background_frame(self, height: int, width: int, template_config: Dict,
                               environment_config: Dict = None, mission_config: Dict = None) -> np.ndarray:
        """Create a background frame with location-specific terrain visuals."""
        try:
            # Check if this is a Real World location-based video
            mission_type = mission_config.get("type", "") if mission_config else ""
            environment_name = environment_config.get("name", "") if environment_config else ""
            location_name = environment_config.get("location", "") if environment_config else ""
            
            # Debug logging (location-specific background detection)
            logger.info(f"Background detection - Mission: '{mission_type}', Env: '{environment_name}', Location: '{location_name}'")
            
            # Try to get real satellite imagery for any Real World location
            # Real World videos have location_name set and environment_name is "earth"
            if (location_name and 
                environment_name.lower() == "earth" and 
                not any(keyword in location_name.lower() for keyword in ["mars", "moon", "canyon"])):
                logger.info(f"Attempting to get satellite imagery for Real World location: {location_name}")
                satellite_background = self._create_satellite_background(location_name, height, width)
                if satellite_background is not None:
                    logger.info(f"✅ Using real Google Maps satellite imagery for {location_name}")
                    return satellite_background
                else:
                    logger.warning(f"⚠️ Failed to get satellite imagery for {location_name}, using synthetic background")
            
            # Fallback to specific synthetic backgrounds
            if ("grand canyon" in environment_name.lower() or 
                "grand canyon" in location_name.lower() or 
                "canyon" in location_name.lower()):
                logger.info("Creating Grand Canyon background")
                return self._create_canyon_background(height, width)
            elif "mars" in environment_name.lower() or "martian" in environment_name.lower():
                return self._create_mars_background(height, width)
            elif "moon" in environment_name.lower() or "lunar" in environment_name.lower():
                return self._create_lunar_background(height, width)
            else:
                # Default Earth background with terrain
                logger.info("Using default Earth terrain background")
                return self._create_earth_terrain_background(height, width)
                
        except Exception as e:
            logger.warning(f"Error creating background frame: {e}")
            # Fallback to basic background
            return np.full((height, width, 3), template_config["background_color"], dtype=np.uint8)
    
    def _create_canyon_background(self, height: int, width: int) -> np.ndarray:
        """Create Grand Canyon-style background."""
        try:
            frame = np.zeros((height, width, 3), dtype=np.uint8)
            
            # Start with blue sky at top
            sky_height = height // 3
            for y in range(sky_height):
                # Blue sky gradient from light to darker blue
                blue_intensity = int(180 + (75 * y / sky_height))
                sky_color = (blue_intensity, 220, 255)  # Light blue to deeper blue
                cv2.line(frame, (0, y), (width, y), sky_color, 1)
            
            # Canyon terrain layers - create distinct red/orange rock layers
            canyon_colors = [
                (45, 45, 139),   # Dark red-brown (BGR format)
                (52, 82, 160),   # Saddle brown lighter  
                (92, 92, 205),   # Indian red
                (135, 184, 222), # Burlywood
                (140, 180, 210), # Tan
                (143, 143, 188)  # Rosy brown
            ]
            
            # Create horizontal canyon layers
            layer_count = 8
            canyon_start = sky_height
            canyon_height = height - canyon_start
            
            for i in range(layer_count):
                layer_height = canyon_height // layer_count
                start_y = canyon_start + (i * layer_height)
                end_y = min(start_y + layer_height, height)
                
                # Select color for this layer
                color = canyon_colors[i % len(canyon_colors)]
                
                # Create the layer with some variation
                for y in range(start_y, end_y):
                    # Add some horizontal variation to make it look more natural
                    for x in range(0, width, 10):
                        variation = random.randint(-5, 5)
                        end_x = min(x + 10 + variation, width)
                        cv2.line(frame, (x, y), (end_x, y), color, 1)
            
            # Add some darker shadows for depth
            for _ in range(20):
                x = random.randint(0, width)
                y = random.randint(canyon_start, height)
                size = random.randint(5, 15)
                shadow_color = (25, 25, 80)  # Dark red-brown
                cv2.circle(frame, (x, y), size, shadow_color, -1)
            
            logger.info(f"Created Grand Canyon background: {width}x{height}, colors used")
            return frame
            
        except Exception as e:
            logger.error(f"Error creating canyon background: {e}")
            # Fallback to simple red-orange background
            frame = np.full((height, width, 3), (45, 82, 160), dtype=np.uint8)  # Reddish color
            return frame
    
    def _create_earth_terrain_background(self, height: int, width: int) -> np.ndarray:
        """Create Earth terrain background with mountains and sky."""
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Sky gradient
        for y in range(height // 2):
            blue_intensity = int(135 + (120 * y / (height // 2)))
            sky_color = (blue_intensity, 206, 250)
            cv2.line(frame, (0, y), (width, y), sky_color, 1)
        
        # Mountain silhouettes
        mountain_color = (34, 139, 34)  # Forest green
        for mountain in range(3):
            peak_x = width // 4 + mountain * width // 3
            peak_y = height // 3 + random.randint(-50, 50)
            base_y = height - 100
            
            # Create mountain triangle
            mountain_points = [
                (peak_x - 150, base_y),
                (peak_x, peak_y),
                (peak_x + 150, base_y)
            ]
            cv2.fillPoly(frame, [np.array(mountain_points)], mountain_color)
        
        # Ground
        ground_color = (101, 67, 33)  # Brown earth
        cv2.rectangle(frame, (0, height - 100), (width, height), ground_color, -1)
        
        return frame
    
    def _create_mars_background(self, height: int, width: int) -> np.ndarray:
        """Create Mars terrain background."""
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Mars sky (butterscotch/orange)
        for y in range(height // 2):
            orange_intensity = int(100 + (100 * y / (height // 2)))
            mars_sky = (30, orange_intensity, 200)  # Orange-ish sky
            cv2.line(frame, (0, y), (width, y), mars_sky, 1)
        
        # Martian terrain (reddish)
        terrain_color = (42, 42, 165)  # Mars red
        cv2.rectangle(frame, (0, height // 2), (width, height), terrain_color, -1)
        
        # Add some craters
        for _ in range(8):
            x = random.randint(50, width - 50)
            y = random.randint(height // 2, height - 50)
            radius = random.randint(15, 40)
            crater_color = (30, 30, 120)  # Darker red
            cv2.circle(frame, (x, y), radius, crater_color, -1)
        
        return frame
    
    def _create_lunar_background(self, height: int, width: int) -> np.ndarray:
        """Create lunar terrain background."""
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Black space
        frame.fill(0)
        
        # Lunar surface (gray)
        surface_color = (169, 169, 169)  # Dark gray
        cv2.rectangle(frame, (0, height // 2), (width, height), surface_color, -1)
        
        # Add craters
        for _ in range(12):
            x = random.randint(30, width - 30)
            y = random.randint(height // 2, height - 30)
            radius = random.randint(10, 35)
            crater_color = (105, 105, 105)  # Darker gray
            cv2.circle(frame, (x, y), radius, crater_color, -1)
        
        # Add some stars
        for _ in range(100):
            x = random.randint(0, width)
            y = random.randint(0, height // 2)
            cv2.circle(frame, (x, y), 1, (255, 255, 255), -1)
        
        return frame
    
    def _create_satellite_background(self, location_name: str, height: int, width: int) -> Optional[np.ndarray]:
        """Create background using real Google Maps satellite imagery."""
        try:
            # Get Google Maps API key
            api_key = os.environ.get('GOOGLE_MAPS_API_KEY')
            if not api_key:
                logger.warning("Google Maps API key not available")
                return None
            
            # First, geocode the location to get coordinates
            geocode_url = "https://maps.googleapis.com/maps/api/geocode/json"
            geocode_params = {
                'address': location_name,
                'key': api_key
            }
            
            geocode_response = requests.get(geocode_url, params=geocode_params, timeout=10)
            geocode_data = geocode_response.json()
            
            if geocode_data.get('status') != 'OK' or not geocode_data.get('results'):
                logger.warning(f"Could not geocode location: {location_name}")
                return None
            
            # Get coordinates
            location = geocode_data['results'][0]['geometry']['location']
            lat, lng = location['lat'], location['lng']
            
            # Get satellite imagery
            static_map_url = "https://maps.googleapis.com/maps/api/staticmap"
            static_map_params = {
                'center': f"{lat},{lng}",
                'zoom': 16,  # Good detail level for drone footage
                'size': "640x640",  # Max free tier size
                'maptype': 'satellite',
                'key': api_key
            }
            
            image_response = requests.get(static_map_url, params=static_map_params, timeout=15)
            image_response.raise_for_status()
            
            # Convert to OpenCV format
            pil_image = Image.open(io.BytesIO(image_response.content))
            
            # Resize to fill the video frame
            pil_image = pil_image.resize((width, height), Image.Resampling.LANCZOS)
            
            # Convert to BGR (OpenCV format)
            rgb_array = np.array(pil_image)
            bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
            
            logger.info(f"Successfully created satellite background for {location_name} at {lat:.4f}, {lng:.4f}")
            return bgr_array
            
        except requests.RequestException as e:
            logger.warning(f"Network error getting satellite imagery for {location_name}: {e}")
            return None
        except Exception as e:
            logger.warning(f"Error creating satellite background for {location_name}: {e}")
            return None
    
    def _generate_discovery_audio(self) -> Optional[str]:
        """Generate sci-fi discovery audio."""
        try:
            # Audio themes
            themes = [
                "discovery_ambient", "space_exploration", "technological_wonder",
                "planetary_survey", "future_expedition"
            ]
            
            # Select random theme
            theme = random.choice(themes)
            audio_filename = f"scifi_{theme}_{random.randint(1000, 9999)}.wav"
            audio_path = os.path.join(self.audio_directory, audio_filename)
            
            # Generate 10 seconds of synthetic audio using FFmpeg
            cmd = [
                'ffmpeg', '-y',
                '-f', 'lavfi',
                '-i', f'sine=frequency=200+200*sin(2*PI*t*0.1):duration=10',
                '-af', f'volume=0.3,tremolo=5:0.9,chorus=0.5:0.9:50:0.4:0.25:2',
                audio_path
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0 and os.path.exists(audio_path):
                return audio_path
            else:
                logger.warning(f"Failed to generate audio: {result.stderr}")
                return None
                
        except Exception as e:
            logger.warning(f"Error generating audio: {e}")
            return None
    
    def get_available_templates(self) -> Dict:
        """Get available video templates."""
        return {name: {
            "name": config["name"],
            "description": config["description"]
        } for name, config in self.templates.items()}
    
    def get_created_videos(self) -> List[Dict]:
        """Get list of created videos."""
        try:
            videos = []
            if os.path.exists(self.video_directory):
                for filename in os.listdir(self.video_directory):
                    if filename.endswith('.mp4'):
                        file_path = os.path.join(self.video_directory, filename)
                        file_size = os.path.getsize(file_path) / (1024 * 1024)  # MB
                        created_time = os.path.getctime(file_path)
                        
                        videos.append({
                            "filename": filename,
                            "size_mb": round(file_size, 2),
                            "created": datetime.fromtimestamp(created_time).strftime("%Y-%m-%d %H:%M:%S")
                        })
            
            # Sort by creation time (newest first)
            videos.sort(key=lambda x: x["created"], reverse=True)
            return videos
            
        except Exception as e:
            logger.error(f"Error getting video list: {e}")
            return []