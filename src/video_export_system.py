"""
Video Export System for Drone Flight Recording

Creates high-quality MP4 video exports of drone simulation flights
with customizable views, overlays, and effects.
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
from PIL import Image, ImageDraw, ImageFont
import io
import base64
from datetime import datetime, timedelta
import threading
import time

logger = logging.getLogger(__name__)

class SciFiAudioSystem:
    """
    System for generating and adding sci-fi discovery audio to drone videos.
    """
    
    def __init__(self):
        """Initialize the sci-fi audio system."""
        self.audio_themes = [
            "discovery_ambient",
            "space_exploration", 
            "technological_wonder",
            "planetary_survey",
            "future_expedition"
        ]
        self.audio_directory = "audio_assets"
        self._ensure_audio_directory()
        
    def _ensure_audio_directory(self):
        """Ensure audio assets directory exists."""
        if not os.path.exists(self.audio_directory):
            os.makedirs(self.audio_directory)
    
    def add_scifi_audio(self, video_path: str, output_path: str) -> bool:
        """
        Add randomly selected sci-fi discovery audio to video.
        
        Args:
            video_path: Path to input video file
            output_path: Path for output video with audio
            
        Returns:
            Boolean indicating success
        """
        try:
            # Generate or select audio track
            audio_file = self._generate_discovery_audio()
            
            if not audio_file:
                logger.warning("No audio file available for video enhancement")
                return False
            
            # Use FFmpeg to combine video with audio
            cmd = [
                'ffmpeg', '-y',  # -y to overwrite output file
                '-i', video_path,  # Input video
                '-i', audio_file,  # Input audio
                '-c:v', 'copy',    # Copy video stream
                '-c:a', 'aac',     # Audio codec
                '-map', '0:v',     # Map video from first input
                '-map', '1:a',     # Map audio from second input
                '-shortest',       # End when shortest stream ends
                output_path
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                logger.info(f"Successfully added sci-fi audio to video: {output_path}")
                return True
            else:
                logger.error(f"FFmpeg error: {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Error adding sci-fi audio: {e}")
            return False
    
    def _generate_discovery_audio(self) -> str:
        """
        Generate or select sci-fi discovery audio track.
        
        Returns:
            Path to audio file or None if unavailable
        """
        try:
            # Randomly select a theme
            theme = random.choice(self.audio_themes)
            
            # Generate procedural sci-fi audio using tone synthesis
            audio_file = os.path.join(self.audio_directory, f"{theme}_{int(time.time())}.wav")
            
            # Create synthetic sci-fi audio with varying tones
            self._create_synthetic_scifi_audio(audio_file, theme)
            
            return audio_file
            
        except Exception as e:
            logger.error(f"Error generating discovery audio: {e}")
            return None
    
    def _create_synthetic_scifi_audio(self, output_file: str, theme: str):
        """
        Create synthetic sci-fi audio using procedural generation.
        
        Args:
            output_file: Path for output audio file
            theme: Audio theme to generate
        """
        try:
            # Audio parameters
            sample_rate = 44100
            duration = 30  # 30 seconds base duration
            
            # Generate time array
            t = np.linspace(0, duration, int(sample_rate * duration))
            
            # Create base audio based on theme
            if theme == "discovery_ambient":
                audio = self._generate_ambient_discovery(t)
            elif theme == "space_exploration":
                audio = self._generate_space_exploration(t)
            elif theme == "technological_wonder":
                audio = self._generate_tech_wonder(t)
            elif theme == "planetary_survey":
                audio = self._generate_planetary_survey(t)
            else:  # future_expedition
                audio = self._generate_future_expedition(t)
            
            # Normalize audio
            audio = audio / np.max(np.abs(audio))
            
            # Convert to 16-bit PCM
            audio_int = (audio * 32767).astype(np.int16)
            
            # Save as WAV file using basic format
            self._save_wav_file(output_file, audio_int, sample_rate)
            
            logger.info(f"Created synthetic sci-fi audio: {output_file}")
            
        except Exception as e:
            logger.error(f"Error creating synthetic audio: {e}")
            raise
    
    def _generate_ambient_discovery(self, t: np.ndarray) -> np.ndarray:
        """Generate ambient discovery audio."""
        # Low-frequency ambient base
        base = 0.3 * np.sin(2 * np.pi * 40 * t)
        
        # Add ethereal high tones
        ethereal = 0.2 * np.sin(2 * np.pi * 220 * t) * np.exp(-t/10)
        
        # Add mysterious warbling
        warble = 0.15 * np.sin(2 * np.pi * 110 * t * (1 + 0.1 * np.sin(2 * np.pi * 0.5 * t)))
        
        return base + ethereal + warble
    
    def _generate_space_exploration(self, t: np.ndarray) -> np.ndarray:
        """Generate space exploration audio."""
        # Deep space resonance
        resonance = 0.4 * np.sin(2 * np.pi * 55 * t)
        
        # Cosmic wind simulation
        wind = 0.1 * np.random.normal(0, 1, len(t))
        
        # Pulsing beacon effect
        beacon = 0.25 * np.sin(2 * np.pi * 880 * t) * (np.sin(2 * np.pi * 1.2 * t) > 0.8)
        
        return resonance + wind + beacon
    
    def _generate_tech_wonder(self, t: np.ndarray) -> np.ndarray:
        """Generate technological wonder audio."""
        # Digital harmony
        harmony = 0.3 * (np.sin(2 * np.pi * 165 * t) + 0.5 * np.sin(2 * np.pi * 330 * t))
        
        # Synthetic arpeggios
        arpeggio = 0.2 * np.sin(2 * np.pi * 440 * t * (1 + 0.05 * np.sin(2 * np.pi * 4 * t)))
        
        # Processing sounds
        processing = 0.1 * np.sin(2 * np.pi * 1760 * t) * np.exp(-5 * (t % 2))
        
        return harmony + arpeggio + processing
    
    def _generate_planetary_survey(self, t: np.ndarray) -> np.ndarray:
        """Generate planetary survey audio."""
        # Scanning tones
        scan = 0.3 * np.sin(2 * np.pi * 220 * t * (1 + 0.5 * np.sin(2 * np.pi * 0.3 * t)))
        
        # Data transmission beeps
        beeps = 0.2 * np.sin(2 * np.pi * 1100 * t) * (np.sin(2 * np.pi * 2 * t) > 0.9)
        
        # Atmospheric hum
        hum = 0.15 * np.sin(2 * np.pi * 80 * t) * (1 + 0.1 * np.sin(2 * np.pi * 0.1 * t))
        
        return scan + beeps + hum
    
    def _generate_future_expedition(self, t: np.ndarray) -> np.ndarray:
        """Generate future expedition audio."""
        # Exploration melody
        melody = 0.3 * np.sin(2 * np.pi * 330 * t * (1 + 0.1 * np.sin(2 * np.pi * 0.7 * t)))
        
        # Discovery chimes
        chimes = 0.2 * np.sin(2 * np.pi * 660 * t) * np.exp(-2 * (t % 5))
        
        # Adventure bass
        bass = 0.25 * np.sin(2 * np.pi * 110 * t)
        
        return melody + chimes + bass
    
    def _save_wav_file(self, filename: str, audio_data: np.ndarray, sample_rate: int):
        """
        Save audio data as WAV file.
        
        Args:
            filename: Output filename
            audio_data: Audio data as numpy array
            sample_rate: Sample rate in Hz
        """
        # Basic WAV file format implementation
        with open(filename, 'wb') as f:
            # WAV header
            f.write(b'RIFF')
            f.write((36 + len(audio_data) * 2).to_bytes(4, 'little'))
            f.write(b'WAVE')
            f.write(b'fmt ')
            f.write((16).to_bytes(4, 'little'))
            f.write((1).to_bytes(2, 'little'))  # PCM
            f.write((1).to_bytes(2, 'little'))  # Mono
            f.write(sample_rate.to_bytes(4, 'little'))
            f.write((sample_rate * 2).to_bytes(4, 'little'))
            f.write((2).to_bytes(2, 'little'))
            f.write((16).to_bytes(2, 'little'))
            f.write(b'data')
            f.write((len(audio_data) * 2).to_bytes(4, 'little'))
            
            # Audio data
            f.write(audio_data.tobytes())

class VideoExportSystem:
    """
    System for exporting drone simulation flights as MP4 videos.
    """
    
    def __init__(self):
        """Initialize video export system."""
        self.video_writer = None
        self.recording = False
        self.frames = []
        self.export_settings = {
            'fps': 30,
            'resolution': (1920, 1080),
            'bitrate': '5M',
            'codec': 'mp4v'
        }
        # Initialize sci-fi audio system
        self.audio_system = SciFiAudioSystem()
    
    def start_recording(self, session_id: str, export_settings: Dict = None) -> bool:
        """
        Start recording a simulation session for video export.
        
        Args:
            session_id: Simulation session ID
            export_settings: Custom export settings
            
        Returns:
            True if recording started successfully
        """
        try:
            if export_settings:
                self.export_settings.update(export_settings)
            
            self.session_id = session_id
            self.recording = True
            self.frames = []
            
            logger.info(f"Started video recording for session {session_id}")
            return True
            
        except Exception as e:
            logger.error(f"Error starting video recording: {e}")
            return False
    
    def add_frame(self, telemetry_data: Dict, view_type: str = "cockpit") -> bool:
        """
        Add a frame to the current recording.
        
        Args:
            telemetry_data: Current telemetry data
            view_type: Type of view (cockpit, chase, overhead, cinematic)
            
        Returns:
            True if frame added successfully
        """
        if not self.recording:
            return False
            
        try:
            frame = self._generate_frame(telemetry_data, view_type)
            self.frames.append(frame)
            
            # Limit memory usage by keeping only recent frames
            if len(self.frames) > 1800:  # 1 minute at 30fps
                self.frames = self.frames[-1800:]
            
            return True
            
        except Exception as e:
            logger.error(f"Error adding frame to recording: {e}")
            return False
    
    def stop_recording(self) -> bool:
        """
        Stop recording and prepare for export.
        
        Returns:
            True if recording stopped successfully
        """
        try:
            self.recording = False
            logger.info(f"Stopped video recording for session {self.session_id}")
            return True
            
        except Exception as e:
            logger.error(f"Error stopping video recording: {e}")
            return False
    
    def export_video(self, output_path: str, export_options: Dict = None) -> Dict[str, Any]:
        """
        Export recorded frames as MP4 video.
        
        Args:
            output_path: Path for output video file
            export_options: Export customization options
            
        Returns:
            Dictionary with export results
        """
        try:
            if not self.frames:
                return {'success': False, 'error': 'No frames to export'}
            
            # Apply export options
            options = {
                'title_screen': True,
                'telemetry_overlay': True,
                'view_transitions': True,
                'background_music': True,
                'quality': 'high'
            }
            if export_options:
                options.update(export_options)
            
            # Initialize video writer
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(
                output_path,
                fourcc,
                self.export_settings['fps'],
                self.export_settings['resolution']
            )
            
            if not video_writer.isOpened():
                return {'success': False, 'error': 'Failed to open video writer'}
            
            frame_count = 0
            
            # Add title screen
            if options['title_screen']:
                title_frames = self._generate_title_screen()
                for frame in title_frames:
                    video_writer.write(frame)
                    frame_count += 1
            
            # Process and write main frames
            for i, frame_data in enumerate(self.frames):
                frame = self._process_frame(frame_data, options)
                video_writer.write(frame)
                frame_count += 1
                
                # Progress update every 30 frames
                if i % 30 == 0:
                    progress = (i / len(self.frames)) * 100
                    logger.info(f"Export progress: {progress:.1f}%")
            
            # Add end screen
            if options['title_screen']:
                end_frames = self._generate_end_screen()
                for frame in end_frames:
                    video_writer.write(frame)
                    frame_count += 1
            
            video_writer.release()
            
            # Add sci-fi discovery audio if enabled
            if options.get('background_music', True):
                enhanced_output_path = output_path.replace('.mp4', '_with_audio.mp4')
                success = self.audio_system.add_scifi_audio(output_path, enhanced_output_path)
                if success:
                    # Replace original with enhanced version
                    os.rename(enhanced_output_path, output_path)
                    logger.info(f"Added sci-fi discovery audio to video: {output_path}")
                else:
                    logger.warning("Failed to add audio, keeping video without audio")
            
            # Get video file size
            file_size = os.path.getsize(output_path)
            duration = frame_count / self.export_settings['fps']
            
            logger.info(f"Video export completed: {output_path}")
            
            return {
                'success': True,
                'output_path': output_path,
                'duration_seconds': duration,
                'frame_count': frame_count,
                'file_size_mb': file_size / (1024 * 1024),
                'resolution': self.export_settings['resolution'],
                'fps': self.export_settings['fps'],
                'audio_enhanced': options.get('background_music', True)
            }
            
        except Exception as e:
            logger.error(f"Error exporting video: {e}")
            return {'success': False, 'error': str(e)}
    
    def create_highlight_reel(self, telemetry_history: List[Dict], 
                            events: List[Dict], duration_seconds: int = 60) -> Dict[str, Any]:
        """
        Create a highlight reel from telemetry and events.
        
        Args:
            telemetry_history: Full telemetry history
            events: Mission events
            duration_seconds: Target duration for highlight reel
            
        Returns:
            Dictionary with highlight reel data
        """
        try:
            highlights = []
            
            # Extract interesting moments
            for event in events:
                if event['event_type'] in ['takeoff', 'waypoint_reached', 'landing', 'mission_complete']:
                    # Find telemetry around this event
                    event_time = event['timestamp']
                    relevant_telemetry = [
                        t for t in telemetry_history 
                        if abs(t['timestamp'] - event_time) <= 5.0  # 5 second window
                    ]
                    
                    if relevant_telemetry:
                        highlights.append({
                            'event': event,
                            'telemetry': relevant_telemetry,
                            'priority': self._calculate_highlight_priority(event)
                        })
            
            # Sort by priority and trim to fit duration
            highlights.sort(key=lambda x: x['priority'], reverse=True)
            
            # Select highlights to fit target duration
            selected_highlights = []
            total_duration = 0
            
            for highlight in highlights:
                highlight_duration = len(highlight['telemetry']) / 10.0  # Assume 10Hz telemetry
                if total_duration + highlight_duration <= duration_seconds:
                    selected_highlights.append(highlight)
                    total_duration += highlight_duration
                else:
                    break
            
            return {
                'highlights': selected_highlights,
                'total_duration': total_duration,
                'frame_count': int(total_duration * self.export_settings['fps'])
            }
            
        except Exception as e:
            logger.error(f"Error creating highlight reel: {e}")
            return {'highlights': [], 'total_duration': 0, 'frame_count': 0}
    
    def _generate_frame(self, telemetry_data: Dict, view_type: str) -> Dict[str, Any]:
        """Generate a single video frame from telemetry data."""
        # Create frame data structure with safe defaults
        safe_telemetry = {
            'timestamp': telemetry_data.get('timestamp', 0.0),
            'position': telemetry_data.get('position', {'x': 0, 'y': 0, 'z': 50}),
            'attitude': telemetry_data.get('attitude', {'pitch': 0, 'yaw': 0, 'roll': 0}),
            'altitude': telemetry_data.get('altitude', 50.0),
            'ground_speed': telemetry_data.get('ground_speed', 5.0),
            'mission_progress': telemetry_data.get('mission_progress', 0.0)
        }
        
        frame_data = {
            'timestamp': safe_telemetry['timestamp'],
            'view_type': view_type,
            'telemetry': safe_telemetry,
            'camera_position': self._calculate_camera_position(safe_telemetry, view_type),
            'scene_elements': self._generate_scene_elements(safe_telemetry)
        }
        
        return frame_data
    
    def _calculate_camera_position(self, telemetry_data: Dict, view_type: str) -> Dict[str, float]:
        """Calculate camera position based on view type and drone position."""
        drone_pos = telemetry_data['position']
        
        if view_type == "cockpit":
            # First-person view from drone
            return {
                'x': drone_pos['x'],
                'y': drone_pos['y'], 
                'z': drone_pos['z'],
                'pitch': telemetry_data['attitude']['pitch'],
                'yaw': telemetry_data['attitude']['yaw'],
                'roll': telemetry_data['attitude']['roll']
            }
        elif view_type == "chase":
            # Follow behind and above drone
            offset_distance = 20  # meters
            return {
                'x': drone_pos['x'] - offset_distance,
                'y': drone_pos['y'],
                'z': drone_pos['z'] + 10,
                'pitch': -15,
                'yaw': telemetry_data['attitude']['yaw'],
                'roll': 0
            }
        elif view_type == "overhead":
            # Top-down view
            return {
                'x': drone_pos['x'],
                'y': drone_pos['y'],
                'z': drone_pos['z'] + 50,
                'pitch': -90,
                'yaw': 0,
                'roll': 0
            }
        elif view_type == "cinematic":
            # Dynamic cinematic camera
            time_factor = telemetry_data['timestamp'] * 0.1
            orbit_radius = 30
            return {
                'x': drone_pos['x'] + orbit_radius * math.cos(time_factor),
                'y': drone_pos['y'] + orbit_radius * math.sin(time_factor),
                'z': drone_pos['z'] + 15,
                'pitch': -20,
                'yaw': math.degrees(time_factor) + 180,
                'roll': 0
            }
        
        return {'x': 0, 'y': 0, 'z': 0, 'pitch': 0, 'yaw': 0, 'roll': 0}
    
    def _generate_scene_elements(self, telemetry_data: Dict) -> List[Dict]:
        """Generate 3D scene elements for rendering."""
        elements = []
        
        # Add drone model
        elements.append({
            'type': 'drone',
            'position': telemetry_data['position'],
            'attitude': telemetry_data['attitude'],
            'scale': 1.0
        })
        
        # Add ground plane
        elements.append({
            'type': 'ground',
            'position': {'x': 0, 'y': 0, 'z': 0},
            'size': 1000,
            'texture': 'terrain'
        })
        
        # Add waypoint markers
        if 'current_waypoint' in telemetry_data:
            elements.append({
                'type': 'waypoint_marker',
                'position': {'x': 100, 'y': 100, 'z': 50},  # Example waypoint
                'active': True
            })
        
        return elements
    
    def _process_frame(self, frame_data: Dict, options: Dict) -> np.ndarray:
        """Process frame data into final video frame with enhanced terrain visualization."""
        height, width = self.export_settings['resolution'][1], self.export_settings['resolution'][0]
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Enhanced sky simulation with realistic gradient
        for y in range(height):
            sky_intensity = 1 - (y / height) * 0.6
            r = int(135 * sky_intensity)  # Light blue sky
            g = int(206 * sky_intensity)
            b = int(235 * sky_intensity)
            frame[y, :] = [b, g, r]  # BGR format for OpenCV
        
        # Add realistic terrain visualization
        frame = self._add_terrain_visualization(frame, frame_data)
        
        # Add telemetry overlay if enabled
        if options.get('telemetry_overlay', True):
            frame = self._add_telemetry_overlay(frame, frame_data)
        
        # Add enhanced drone representation
        frame = self._add_drone_representation(frame, frame_data)
        
        # Add environmental effects
        frame = self._add_environmental_effects(frame, frame_data)
        
        return frame
    
    def _add_terrain_visualization(self, frame: np.ndarray, frame_data: Dict) -> np.ndarray:
        """Add realistic terrain visualization to the frame."""
        height, width = frame.shape[:2]
        horizon_y = height // 2
        
        # Get drone position for terrain context
        telemetry = frame_data.get('telemetry', {})
        drone_pos = telemetry.get('position', {'x': 0, 'y': 0, 'z': 0})
        altitude = telemetry.get('altitude', 50.0)
        
        # Create terrain-based ground visualization
        if altitude > 100:
            # High altitude view - show terrain patches
            self._draw_terrain_patches(frame, horizon_y, width, height, drone_pos)
        else:
            # Low altitude view - show detailed ground
            self._draw_detailed_ground(frame, horizon_y, width, height, drone_pos)
        
        # Add horizon features (mountains, buildings, etc.)
        self._draw_horizon_features(frame, horizon_y, width, drone_pos)
        
        return frame
    
    def _draw_terrain_patches(self, frame: np.ndarray, horizon_y: int, width: int, height: int, drone_pos: Dict):
        """Draw terrain patches for high altitude view."""
        # Create varied terrain patches
        patch_size = 40
        colors = [
            (45, 85, 45),    # Dark green - forests
            (60, 100, 60),   # Medium green - grassland  
            (80, 120, 80),   # Light green - fields
            (70, 70, 40),    # Brown - bare earth
            (90, 90, 70),    # Tan - sandy areas
        ]
        
        for y in range(horizon_y, height, patch_size):
            for x in range(0, width, patch_size):
                # Vary patch colors based on position
                color_index = ((x // patch_size) + (y // patch_size)) % len(colors)
                color = colors[color_index]
                
                # Add some randomness to patch size and color
                patch_w = patch_size + (x % 10) - 5
                patch_h = patch_size + (y % 8) - 4
                
                # Slightly vary the color
                varied_color = (
                    max(0, min(255, color[0] + (x % 20) - 10)),
                    max(0, min(255, color[1] + (y % 20) - 10)),
                    max(0, min(255, color[2] + (x % 15) - 7))
                )
                
                cv2.rectangle(frame, (x, y), (x + patch_w, y + patch_h), varied_color, -1)
    
    def _draw_detailed_ground(self, frame: np.ndarray, horizon_y: int, width: int, height: int, drone_pos: Dict):
        """Draw detailed ground texture for low altitude view."""
        # Create more detailed ground with grass-like texture
        base_color = (45, 85, 45)  # Base green
        
        # Fill ground area
        frame[horizon_y:, :] = base_color
        
        # Add texture details
        for i in range(0, width, 3):
            for j in range(horizon_y, height, 3):
                # Add small color variations for texture
                if (i + j) % 7 == 0:
                    variation = (
                        max(0, min(255, base_color[0] + 15)),
                        max(0, min(255, base_color[1] + 20)),
                        max(0, min(255, base_color[2] + 10))
                    )
                    frame[j:j+2, i:i+2] = variation
    
    def _draw_horizon_features(self, frame: np.ndarray, horizon_y: int, width: int, drone_pos: Dict):
        """Draw horizon features like mountains or buildings."""
        # Add mountain silhouettes
        mountain_points = []
        for x in range(0, width, 20):
            # Create mountain profile
            mountain_height = 30 + (x % 40) + ((x * 7) % 25)
            mountain_points.append([x, horizon_y - mountain_height])
        
        # Add final points to close the shape
        mountain_points.append([width, horizon_y])
        mountain_points.append([0, horizon_y])
        
        mountain_color = (60, 60, 80)  # Bluish mountains
        cv2.fillPoly(frame, [np.array(mountain_points, np.int32)], mountain_color)
    
    def _add_environmental_effects(self, frame: np.ndarray, frame_data: Dict) -> np.ndarray:
        """Add environmental effects like clouds, shadows, etc."""
        height, width = frame.shape[:2]
        
        # Add simple cloud effects
        for i in range(3):
            cloud_x = (width // 4) * i + (frame_data['telemetry']['timestamp'] * 5) % (width // 4)
            cloud_y = 50 + i * 30
            
            # Draw simple cloud shape
            cv2.ellipse(frame, (int(cloud_x), cloud_y), (40, 15), 0, 0, 360, (255, 255, 255), -1)
            cv2.ellipse(frame, (int(cloud_x + 20), cloud_y), (30, 12), 0, 0, 360, (255, 255, 255), -1)
        
        return frame
    
    def _add_telemetry_overlay(self, frame: np.ndarray, telemetry: Dict) -> np.ndarray:
        """Add telemetry data overlay to frame."""
        height, width = frame.shape[:2]
        
        # Create overlay text with safe defaults
        overlay_data = [
            f"Time: {telemetry.get('timestamp', 0.0):.1f}s",
            f"Altitude: {telemetry.get('altitude', 0.0):.1f}m",
            f"Speed: {telemetry.get('ground_speed', 0.0):.1f}m/s",
            f"Progress: {telemetry.get('mission_progress', 0.0):.1f}%"
        ]
        
        # Draw semi-transparent background
        overlay_height = len(overlay_data) * 30 + 20
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (300, overlay_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # Draw text
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (255, 255, 255)
        thickness = 1
        
        for i, text in enumerate(overlay_data):
            y_pos = 35 + i * 25
            cv2.putText(frame, text, (20, y_pos), font, font_scale, color, thickness)
        
        return frame
    
    def _add_drone_representation(self, frame: np.ndarray, frame_data: Dict) -> np.ndarray:
        """Add enhanced visual representation of drone to frame."""
        height, width = frame.shape[:2]
        
        # Position drone based on view type
        view_type = frame_data.get('view_type', 'chase')
        
        if view_type == 'chase':
            # Drone visible in front center
            drone_x, drone_y = width // 2, height // 2 + 20
        elif view_type == 'overhead':
            # Drone smaller, in center for top-down view
            drone_x, drone_y = width // 2, height // 2
        else:  # cockpit view
            # No drone visible in first person view
            return frame
        
        # Draw enhanced drone body
        drone_size = 12 if view_type == 'overhead' else 15
        
        # Main body
        cv2.circle(frame, (drone_x, drone_y), drone_size, (80, 80, 80), -1)
        cv2.circle(frame, (drone_x, drone_y), drone_size, (0, 0, 0), 2)
        
        # Center core
        cv2.circle(frame, (drone_x, drone_y), drone_size // 2, (120, 120, 120), -1)
        
        # Draw rotors with motion blur effect
        rotor_distance = drone_size + 8
        rotor_positions = [
            (drone_x - rotor_distance, drone_y - rotor_distance),
            (drone_x + rotor_distance, drone_y - rotor_distance),
            (drone_x - rotor_distance, drone_y + rotor_distance),
            (drone_x + rotor_distance, drone_y + rotor_distance)
        ]
        
        for pos in rotor_positions:
            # Rotor disc (motion blur)
            cv2.circle(frame, pos, 8, (200, 200, 200, 100), -1)
            cv2.circle(frame, pos, 8, (0, 0, 0), 1)
            # Rotor center
            cv2.circle(frame, pos, 3, (60, 60, 60), -1)
        
        # Add LED lights
        led_positions = [
            (drone_x, drone_y - drone_size - 3),  # Front - white
            (drone_x, drone_y + drone_size + 3),  # Back - red
        ]
        led_colors = [(255, 255, 255), (0, 0, 255)]  # White front, red back
        
        for i, pos in enumerate(led_positions):
            cv2.circle(frame, pos, 2, led_colors[i], -1)
        
        # Add drone shadow on ground (if visible)
        if view_type != 'overhead':
            altitude = frame_data.get('telemetry', {}).get('altitude', 50.0)
            if altitude < 50:  # Only show shadow when low
                shadow_offset = int(altitude / 2)
                shadow_y = height - 100 + shadow_offset
                shadow_size = max(5, drone_size - shadow_offset // 2)
                
                # Shadow ellipse
                cv2.ellipse(frame, (drone_x + shadow_offset, shadow_y), 
                           (shadow_size * 2, shadow_size), 0, 0, 360, (0, 0, 0), -1)
        
        return frame
    
    def _generate_title_screen(self) -> List[np.ndarray]:
        """Generate title screen frames."""
        frames = []
        height, width = self.export_settings['resolution'][1], self.export_settings['resolution'][0]
        duration_frames = self.export_settings['fps'] * 3  # 3 seconds
        
        for i in range(duration_frames):
            frame = np.zeros((height, width, 3), dtype=np.uint8)
            
            # Background gradient
            for y in range(height):
                intensity = int(128 + 127 * math.sin(y * 0.01 + i * 0.1))
                frame[y, :] = [intensity // 4, intensity // 3, intensity // 2]
            
            # Title text
            title = "Drone Simulation Flight Recording"
            subtitle = f"Session: {self.session_id}"
            timestamp = f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"
            
            font = cv2.FONT_HERSHEY_SIMPLEX
            
            # Main title
            title_size = cv2.getTextSize(title, font, 1.5, 3)[0]
            title_x = (width - title_size[0]) // 2
            cv2.putText(frame, title, (title_x, height // 2 - 50), font, 1.5, (255, 255, 255), 3)
            
            # Subtitle
            subtitle_size = cv2.getTextSize(subtitle, font, 1.0, 2)[0]
            subtitle_x = (width - subtitle_size[0]) // 2
            cv2.putText(frame, subtitle, (subtitle_x, height // 2 + 20), font, 1.0, (200, 200, 200), 2)
            
            # Timestamp
            timestamp_size = cv2.getTextSize(timestamp, font, 0.7, 1)[0]
            timestamp_x = (width - timestamp_size[0]) // 2
            cv2.putText(frame, timestamp, (timestamp_x, height // 2 + 80), font, 0.7, (150, 150, 150), 1)
            
            frames.append(frame)
        
        return frames
    
    def _generate_end_screen(self) -> List[np.ndarray]:
        """Generate end screen frames."""
        frames = []
        height, width = self.export_settings['resolution'][1], self.export_settings['resolution'][0]
        duration_frames = self.export_settings['fps'] * 2  # 2 seconds
        
        for i in range(duration_frames):
            frame = np.zeros((height, width, 3), dtype=np.uint8)
            
            # Simple fade to black
            alpha = 1.0 - (i / duration_frames)
            frame.fill(int(50 * alpha))
            
            # End text
            end_text = "Flight Recording Complete"
            font = cv2.FONT_HERSHEY_SIMPLEX
            text_size = cv2.getTextSize(end_text, font, 1.2, 2)[0]
            text_x = (width - text_size[0]) // 2
            text_color = (int(255 * alpha), int(255 * alpha), int(255 * alpha))
            cv2.putText(frame, end_text, (text_x, height // 2), font, 1.2, text_color, 2)
            
            frames.append(frame)
        
        return frames
    
    def _calculate_highlight_priority(self, event: Dict) -> float:
        """Calculate priority score for highlight selection."""
        priority_map = {
            'takeoff': 0.9,
            'mission_complete': 1.0,
            'waypoint_reached': 0.6,
            'landing': 0.8,
            'emergency': 0.95,
            'photo_taken': 0.7
        }
        
        return priority_map.get(event['event_type'], 0.5)
    
    def get_export_presets(self) -> Dict[str, Dict]:
        """Get predefined export presets."""
        return {
            'high_quality': {
                'fps': 60,
                'resolution': (1920, 1080),
                'bitrate': '10M',
                'title_screen': True,
                'telemetry_overlay': True
            },
            'standard': {
                'fps': 30,
                'resolution': (1920, 1080),
                'bitrate': '5M',
                'title_screen': True,
                'telemetry_overlay': True
            },
            'mobile': {
                'fps': 30,
                'resolution': (1280, 720),
                'bitrate': '3M',
                'title_screen': True,
                'telemetry_overlay': True
            },
            'web_share': {
                'fps': 24,
                'resolution': (1280, 720),
                'bitrate': '2M',
                'title_screen': True,
                'telemetry_overlay': False
            }
        }