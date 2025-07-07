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
from typing import Dict, List, Tuple, Optional, Any
import logging
from PIL import Image, ImageDraw, ImageFont
import io
import base64
from datetime import datetime, timedelta
import threading
import time

logger = logging.getLogger(__name__)

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
                'background_music': False,
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
                'fps': self.export_settings['fps']
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
        # Create frame data structure
        frame_data = {
            'timestamp': telemetry_data['timestamp'],
            'view_type': view_type,
            'telemetry': telemetry_data,
            'camera_position': self._calculate_camera_position(telemetry_data, view_type),
            'scene_elements': self._generate_scene_elements(telemetry_data)
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
        """Process frame data into final video frame."""
        # Create base frame (solid color for now, would be 3D rendered scene)
        height, width = self.export_settings['resolution'][1], self.export_settings['resolution'][0]
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Background gradient (sky simulation)
        for y in range(height):
            intensity = int(255 * (1 - y / height))
            frame[y, :] = [intensity // 3, intensity // 2, intensity]  # Blue gradient
        
        # Add horizon line
        horizon_y = height // 2
        cv2.line(frame, (0, horizon_y), (width, horizon_y), (100, 100, 100), 2)
        
        # Add ground texture
        ground_color = (50, 80, 50)  # Dark green
        frame[horizon_y:, :] = ground_color
        
        # Add telemetry overlay if enabled
        if options.get('telemetry_overlay', True):
            frame = self._add_telemetry_overlay(frame, frame_data['telemetry'])
        
        # Add drone representation
        frame = self._add_drone_representation(frame, frame_data)
        
        return frame
    
    def _add_telemetry_overlay(self, frame: np.ndarray, telemetry: Dict) -> np.ndarray:
        """Add telemetry data overlay to frame."""
        height, width = frame.shape[:2]
        
        # Create overlay text
        overlay_data = [
            f"Time: {telemetry['timestamp']:.1f}s",
            f"Altitude: {telemetry['altitude']:.1f}m",
            f"Speed: {telemetry['ground_speed']:.1f}m/s",
            f"Progress: {telemetry['mission_progress']:.1f}%"
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
        """Add visual representation of drone to frame."""
        height, width = frame.shape[:2]
        
        # Simple drone icon in center (would be 3D model in full implementation)
        center_x, center_y = width // 2, height // 2
        
        # Draw drone body
        cv2.circle(frame, (center_x, center_y), 8, (255, 255, 255), -1)
        cv2.circle(frame, (center_x, center_y), 8, (0, 0, 0), 2)
        
        # Draw rotors
        rotor_positions = [
            (center_x - 15, center_y - 15),
            (center_x + 15, center_y - 15),
            (center_x - 15, center_y + 15),
            (center_x + 15, center_y + 15)
        ]
        
        for pos in rotor_positions:
            cv2.circle(frame, pos, 5, (200, 200, 200), -1)
            cv2.circle(frame, pos, 5, (0, 0, 0), 1)
        
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