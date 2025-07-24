"""
Virtual Drone USB/WiFi Game Controller System
Comprehensive Python implementation for controlling virtual drones with standard game controllers
"""

import pygame
import time
import math
import json
import threading
from typing import Dict, Tuple, Optional, Callable
from dataclasses import dataclass

# --- Configuration Classes ---

@dataclass
class ControllerConfig:
    """Configuration for different controller types"""
    name: str
    axis_left_stick_x: int = 0
    axis_left_stick_y: int = 1
    axis_right_stick_x: int = 3
    axis_right_stick_y: int = 4
    axis_left_trigger: int = 2
    axis_right_trigger: int = 5
    button_takeoff: int = 0  # A/X button
    button_land: int = 1     # B/Circle button
    button_emergency: int = 3  # Y/Triangle button
    button_hover: int = 2    # X/Square button
    dead_zone: float = 0.15
    sensitivity: float = 1.0

@dataclass
class DroneState:
    """Complete drone state representation"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    vyaw: float = 0.0
    status: str = "LANDED"
    battery: float = 100.0
    signal_strength: int = 100

# --- Controller Input Manager ---

class GameControllerInput:
    """Advanced game controller input handling with multiple controller support"""
    
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        
        self.controllers = {}
        self.active_controller = None
        self.config = self._load_default_configs()
        self.callbacks = {}
        
        self._detect_controllers()
    
    def _load_default_configs(self) -> Dict[str, ControllerConfig]:
        """Load default configurations for common controllers"""
        return {
            "xbox": ControllerConfig(
                name="Xbox Controller",
                axis_left_stick_x=0, axis_left_stick_y=1,
                axis_right_stick_x=4, axis_right_stick_y=3,
                axis_left_trigger=2, axis_right_trigger=5,
                button_takeoff=0, button_land=1, button_emergency=3, button_hover=2
            ),
            "playstation": ControllerConfig(
                name="PlayStation Controller",
                axis_left_stick_x=0, axis_left_stick_y=1,
                axis_right_stick_x=2, axis_right_stick_y=3,
                axis_left_trigger=4, axis_right_trigger=5,
                button_takeoff=1, button_land=2, button_emergency=0, button_hover=3
            ),
            "generic": ControllerConfig(
                name="Generic Controller",
                axis_left_stick_x=0, axis_left_stick_y=1,
                axis_right_stick_x=3, axis_right_stick_y=4,
                button_takeoff=0, button_land=1, button_emergency=3, button_hover=2
            )
        }
    
    def _detect_controllers(self):
        """Detect and initialize connected controllers"""
        joystick_count = pygame.joystick.get_count()
        
        if joystick_count == 0:
            print("No game controllers detected. Please connect a controller.")
            return False
        
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            controller_name = joystick.get_name().lower()
            
            # Auto-detect controller type
            if "xbox" in controller_name:
                config = self.config["xbox"]
            elif "playstation" in controller_name or "ps" in controller_name:
                config = self.config["playstation"]
            else:
                config = self.config["generic"]
            
            self.controllers[i] = {
                'joystick': joystick,
                'config': config,
                'name': joystick.get_name()
            }
            
            print(f"Controller {i}: {joystick.get_name()} (Axes: {joystick.get_numaxes()}, Buttons: {joystick.get_numbuttons()})")
        
        # Set first controller as active
        self.active_controller = 0
        print(f"Active controller: {self.controllers[0]['name']}")
        return True
    
    def apply_deadzone(self, value: float, deadzone: float = 0.15) -> float:
        """Apply deadzone to prevent stick drift"""
        if abs(value) < deadzone:
            return 0.0
        # Scale the remaining range to -1.0 to 1.0
        if value > 0:
            return (value - deadzone) / (1.0 - deadzone)
        else:
            return (value + deadzone) / (1.0 - deadzone)
    
    def apply_expo_curve(self, value: float, expo: float = 0.7) -> float:
        """Apply exponential curve for smoother control"""
        sign = 1 if value >= 0 else -1
        abs_value = abs(value)
        return sign * (expo * (abs_value ** 3) + (1 - expo) * abs_value)
    
    def get_controller_input(self) -> Optional[Dict]:
        """Get processed input from active controller"""
        if self.active_controller is None or self.active_controller not in self.controllers:
            return None
        
        controller = self.controllers[self.active_controller]
        joystick = controller['joystick']
        config = controller['config']
        
        # Process pygame events
        pygame.event.pump()
        
        # Read analog stick values
        try:
            left_x = self.apply_deadzone(joystick.get_axis(config.axis_left_stick_x), config.dead_zone)
            left_y = -self.apply_deadzone(joystick.get_axis(config.axis_left_stick_y), config.dead_zone)  # Inverted
            right_x = self.apply_deadzone(joystick.get_axis(config.axis_right_stick_x), config.dead_zone)
            right_y = -self.apply_deadzone(joystick.get_axis(config.axis_right_stick_y), config.dead_zone)  # Inverted
            
            # Apply exponential curves for smoother control
            left_x = self.apply_expo_curve(left_x) * config.sensitivity
            left_y = self.apply_expo_curve(left_y) * config.sensitivity
            right_x = self.apply_expo_curve(right_x) * config.sensitivity
            right_y = self.apply_expo_curve(right_y) * config.sensitivity
            
            # Read trigger values (if available)
            left_trigger = 0.0
            right_trigger = 0.0
            if joystick.get_numaxes() > config.axis_left_trigger:
                left_trigger = (joystick.get_axis(config.axis_left_trigger) + 1.0) / 2.0
            if joystick.get_numaxes() > config.axis_right_trigger:
                right_trigger = (joystick.get_axis(config.axis_right_trigger) + 1.0) / 2.0
            
            # Read button states
            buttons = {}
            for i in range(joystick.get_numbuttons()):
                buttons[i] = joystick.get_button(i)
            
            return {
                'left_stick': {'x': left_x, 'y': left_y},
                'right_stick': {'x': right_x, 'y': right_y},
                'triggers': {'left': left_trigger, 'right': right_trigger},
                'buttons': buttons,
                'config': config
            }
            
        except pygame.error as e:
            print(f"Controller input error: {e}")
            return None
    
    def register_callback(self, event_type: str, callback: Callable):
        """Register callback for specific controller events"""
        if event_type not in self.callbacks:
            self.callbacks[event_type] = []
        self.callbacks[event_type].append(callback)
    
    def process_events(self):
        """Process pygame events and trigger callbacks"""
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if 'button_down' in self.callbacks:
                    for callback in self.callbacks['button_down']:
                        callback(event.joy, event.button)
            elif event.type == pygame.JOYBUTTONUP:
                if 'button_up' in self.callbacks:
                    for callback in self.callbacks['button_up']:
                        callback(event.joy, event.button)

# --- Virtual Drone Simulation ---

class VirtualDrone:
    """Advanced virtual drone with realistic physics and control"""
    
    def __init__(self, drone_type: str = "quadcopter"):
        self.state = DroneState()
        self.drone_type = drone_type
        self.max_speeds = self._get_drone_specs(drone_type)
        self.control_smoothing = 0.1  # Control smoothing factor
        self.physics_enabled = True
        self.auto_hover = True
        
        # Control targets (for smooth transitions)
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vz = 0.0
        self.target_vyaw = 0.0
        
        print(f"Virtual {drone_type} initialized. Status: {self.state.status}")
    
    def _get_drone_specs(self, drone_type: str) -> Dict[str, float]:
        """Get performance specifications for different drone types"""
        specs = {
            "quadcopter": {
                "max_horizontal_speed": 15.0,  # m/s
                "max_vertical_speed": 5.0,
                "max_yaw_rate": 180.0,  # degrees/s
                "max_tilt": 30.0  # degrees
            },
            "racing": {
                "max_horizontal_speed": 25.0,
                "max_vertical_speed": 8.0,
                "max_yaw_rate": 360.0,
                "max_tilt": 45.0
            },
            "heavy_lift": {
                "max_horizontal_speed": 8.0,
                "max_vertical_speed": 3.0,
                "max_yaw_rate": 90.0,
                "max_tilt": 20.0
            }
        }
        return specs.get(drone_type, specs["quadcopter"])
    
    def takeoff(self):
        """Initiate takeoff sequence"""
        if self.state.status == "LANDED":
            print("🚁 TAKEOFF - Initiating takeoff sequence")
            self.state.status = "TAKING_OFF"
            self.target_vy = 2.0  # Gentle takeoff
            threading.Timer(2.0, self._complete_takeoff).start()
    
    def _complete_takeoff(self):
        """Complete takeoff sequence"""
        if self.state.status == "TAKING_OFF":
            self.state.status = "HOVERING"
            self.state.y = 3.0  # Default hover altitude
            self.target_vy = 0.0
            print("✅ Takeoff complete - Hovering at 3m")
    
    def land(self):
        """Initiate landing sequence"""
        if self.state.status in ["HOVERING", "FLYING"]:
            print("🛬 LANDING - Initiating landing sequence")
            self.state.status = "LANDING"
            self.target_vx = 0.0
            self.target_vz = 0.0
            self.target_vyaw = 0.0
            self.target_vy = -1.0  # Gentle descent
            threading.Timer(3.0, self._complete_landing).start()
    
    def _complete_landing(self):
        """Complete landing sequence"""
        if self.state.status == "LANDING":
            self.state.status = "LANDED"
            self.state.y = 0.0
            self.state.vx = self.state.vy = self.state.vz = self.state.vyaw = 0.0
            print("✅ Landing complete")
    
    def emergency_stop(self):
        """Emergency stop - immediate shutdown"""
        print("🚨 EMERGENCY STOP - All systems halted")
        self.state.status = "EMERGENCY"
        self.state.vx = self.state.vy = self.state.vz = self.state.vyaw = 0.0
        self.target_vx = self.target_vy = self.target_vz = self.target_vyaw = 0.0
        if self.state.y > 0:
            self.state.y = 0.0  # Emergency landing
    
    def set_control_inputs(self, left_stick: Dict, right_stick: Dict, triggers: Dict):
        """Process controller inputs and set target velocities"""
        if self.state.status not in ["HOVERING", "FLYING"]:
            return
        
        # Update flight status
        if self.state.status == "HOVERING" and (abs(left_stick['x']) > 0.1 or abs(left_stick['y']) > 0.1):
            self.state.status = "FLYING"
        elif self.state.status == "FLYING" and abs(left_stick['x']) < 0.05 and abs(left_stick['y']) < 0.05:
            self.state.status = "HOVERING"
        
        # Map controller inputs to drone movements
        # Left stick: horizontal movement (X/Z plane)
        self.target_vx = left_stick['x'] * self.max_speeds['max_horizontal_speed']
        self.target_vz = left_stick['y'] * self.max_speeds['max_horizontal_speed']
        
        # Right stick: vertical movement and yaw
        self.target_vy = right_stick['y'] * self.max_speeds['max_vertical_speed']
        self.target_vyaw = right_stick['x'] * self.max_speeds['max_yaw_rate']
        
        # Triggers can be used for fine altitude control
        if triggers['left'] > 0.1:
            self.target_vy = -triggers['left'] * self.max_speeds['max_vertical_speed'] * 0.5
        elif triggers['right'] > 0.1:
            self.target_vy = triggers['right'] * self.max_speeds['max_vertical_speed'] * 0.5
    
    def update_physics(self, dt: float):
        """Update drone physics simulation"""
        if self.state.status in ["LANDED", "EMERGENCY"]:
            return
        
        # Smooth control transitions
        self.state.vx += (self.target_vx - self.state.vx) * self.control_smoothing
        self.state.vy += (self.target_vy - self.state.vy) * self.control_smoothing
        self.state.vz += (self.target_vz - self.state.vz) * self.control_smoothing
        self.state.vyaw += (self.target_vyaw - self.state.vyaw) * self.control_smoothing
        
        # Update position
        self.state.x += self.state.vx * dt
        self.state.y += self.state.vy * dt
        self.state.z += self.state.vz * dt
        self.state.yaw = (self.state.yaw + self.state.vyaw * dt) % 360
        
        # Calculate attitude based on movement (simplified physics)
        if self.physics_enabled:
            # Roll based on sideways movement
            self.state.roll = -self.state.vx * 2.0
            self.state.roll = max(-self.max_speeds['max_tilt'], min(self.max_speeds['max_tilt'], self.state.roll))
            
            # Pitch based on forward/backward movement
            self.state.pitch = -self.state.vz * 2.0
            self.state.pitch = max(-self.max_speeds['max_tilt'], min(self.max_speeds['max_tilt'], self.state.pitch))
        
        # Ground collision detection
        if self.state.y < 0:
            self.state.y = 0
            self.state.vy = 0
            if self.state.status not in ["LANDING", "LANDED"]:
                print("⚠️ Ground collision detected")
        
        # Battery simulation (simplified)
        if self.state.status in ["FLYING", "HOVERING"]:
            power_consumption = (abs(self.state.vx) + abs(self.state.vy) + abs(self.state.vz)) * 0.1
            self.state.battery -= power_consumption * dt
            self.state.battery = max(0, self.state.battery)
    
    def get_telemetry(self) -> Dict:
        """Get comprehensive telemetry data"""
        return {
            'position': {'x': self.state.x, 'y': self.state.y, 'z': self.state.z},
            'attitude': {'roll': self.state.roll, 'pitch': self.state.pitch, 'yaw': self.state.yaw},
            'velocity': {'vx': self.state.vx, 'vy': self.state.vy, 'vz': self.state.vz},
            'status': self.state.status,
            'battery': self.state.battery,
            'signal': self.state.signal_strength
        }
    
    def print_telemetry(self):
        """Print formatted telemetry"""
        telemetry = (
            f"Status: {self.state.status:10} | "
            f"Pos: X:{self.state.x:6.2f} Y:{self.state.y:6.2f} Z:{self.state.z:6.2f} | "
            f"Att: R:{self.state.roll:5.1f}° P:{self.state.pitch:5.1f}° Y:{self.state.yaw:5.1f}° | "
            f"Vel: {math.sqrt(self.state.vx**2 + self.state.vz**2):5.2f}m/s | "
            f"Batt: {self.state.battery:5.1f}%"
        )
        print(telemetry, end='\r')

# --- Main Control System ---

class DroneControlSystem:
    """Main system integrating controller input and drone simulation"""
    
    def __init__(self, drone_type: str = "quadcopter"):
        self.controller = GameControllerInput()
        self.drone = VirtualDrone(drone_type)
        self.running = False
        self.telemetry_rate = 10  # Hz
        
        # Register controller event callbacks
        self.controller.register_callback('button_down', self._handle_button_press)
        
        print("🎮 Drone Control System initialized")
        print("Controls:")
        print("  Left Stick: Horizontal movement (strafe/forward-back)")
        print("  Right Stick: Vertical movement & yaw rotation")
        print("  A/X Button: Takeoff")
        print("  B/Circle Button: Land")
        print("  Y/Triangle Button: Emergency Stop")
    
    def _handle_button_press(self, controller_id: int, button: int):
        """Handle controller button presses"""
        if controller_id != self.controller.active_controller:
            return
        
        config = self.controller.controllers[controller_id]['config']
        
        if button == config.button_takeoff:
            self.drone.takeoff()
        elif button == config.button_land:
            self.drone.land()
        elif button == config.button_emergency:
            self.drone.emergency_stop()
            self.running = False
        elif button == config.button_hover:
            if self.drone.state.status == "FLYING":
                self.drone.state.status = "HOVERING"
                self.drone.target_vx = self.drone.target_vz = 0.0
    
    def run(self):
        """Main control loop"""
        if not self.controller.controllers:
            print("❌ No controllers detected. Cannot start control system.")
            return
        
        self.running = True
        clock = pygame.time.Clock()
        last_time = time.time()
        telemetry_counter = 0
        
        print("🚀 Drone Control System started. Press Y/Triangle for emergency stop.")
        
        try:
            while self.running:
                # Process controller events
                self.controller.process_events()
                
                # Get controller input
                input_data = self.controller.get_controller_input()
                if input_data:
                    # Send control inputs to drone
                    self.drone.set_control_inputs(
                        input_data['left_stick'],
                        input_data['right_stick'],
                        input_data['triggers']
                    )
                
                # Update drone physics
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time
                self.drone.update_physics(dt)
                
                # Print telemetry at specified rate
                telemetry_counter += 1
                if telemetry_counter >= (60 // self.telemetry_rate):
                    self.drone.print_telemetry()
                    telemetry_counter = 0
                
                # Check for low battery
                if self.drone.state.battery < 10 and self.drone.state.status != "LANDING":
                    print(f"\n⚠️  LOW BATTERY: {self.drone.state.battery:.1f}% - Auto-landing initiated")
                    self.drone.land()
                
                # Maintain 60 FPS
                clock.tick(60)
                
        except KeyboardInterrupt:
            print("\n⌨️  Keyboard interrupt detected")
        
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Clean shutdown"""
        print(f"\n🛑 Shutting down drone control system")
        self.running = False
        if self.drone.state.status in ["FLYING", "HOVERING"]:
            print("⚠️  Drone still airborne - Emergency landing")
            self.drone.emergency_stop()
        pygame.quit()

# --- Configuration and Testing ---

def test_controller_detection():
    """Test function to detect and display controller information"""
    controller = GameControllerInput()
    if not controller.controllers:
        return False
    
    print("\n📋 Controller Detection Results:")
    for i, ctrl in controller.controllers.items():
        joystick = ctrl['joystick']
        print(f"  Controller {i}: {ctrl['name']}")
        print(f"    Axes: {joystick.get_numaxes()}")
        print(f"    Buttons: {joystick.get_numbuttons()}")
        print(f"    Config: {ctrl['config'].name}")
    
    return True

def main():
    """Main entry point"""
    print("🎮 Virtual Drone USB/WiFi Controller System")
    print("=" * 50)
    
    # Test controller detection
    if not test_controller_detection():
        print("❌ No controllers detected. Please connect a game controller and try again.")
        return
    
    # Create and run control system
    try:
        control_system = DroneControlSystem("quadcopter")
        control_system.run()
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        print("👋 System terminated")

if __name__ == "__main__":
    main()