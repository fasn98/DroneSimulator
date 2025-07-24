"""
Controller Integration Module
Integrates the game controller system with the existing drone simulation platform
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.drone_controller import DroneControlSystem, GameControllerInput, VirtualDrone
from src.simulator import Simulator
from src.drone_model import DroneModel
from src.environment import Environment
from src.mission_manager import MissionManager
import json
import threading
import time

class IntegratedControlSystem:
    """Integration bridge between game controller and existing simulation system"""
    
    def __init__(self, config_file: str = None):
        self.controller = GameControllerInput()
        self.simulation_active = False
        self.bridge_active = False
        
        # Load simulation configuration
        if config_file and os.path.exists(config_file):
            with open(config_file, 'r') as f:
                self.sim_config = json.load(f)
        else:
            self.sim_config = self._default_config()
        
        # Initialize simulation components
        self.simulator = None
        self.control_thread = None
        
        print("🔗 Integrated Control System initialized")
    
    def _default_config(self):
        """Default simulation configuration"""
        return {
            "drone_model": "professional_quadcopter_optimized",
            "environment": "earth",
            "mission_type": "test_flight",
            "controller_enabled": True,
            "telemetry_rate": 10
        }
    
    def setup_simulation(self):
        """Setup the main simulation system"""
        try:
            # Create drone model
            drone_model = DroneModel()
            drone_model.load_from_config(f"config/drone_models.json", self.sim_config["drone_model"])
            
            # Create environment
            environment = Environment()
            environment.load_from_config(f"config/environments.json", self.sim_config["environment"])
            
            # Create mission manager
            mission_manager = MissionManager()
            mission_manager.load_mission(f"config/missions.json", self.sim_config["mission_type"])
            
            # Create simulator
            self.simulator = Simulator(drone_model, environment, mission_manager)
            
            print(f"✅ Simulation setup complete:")
            print(f"   Drone: {self.sim_config['drone_model']}")
            print(f"   Environment: {self.sim_config['environment']}")
            print(f"   Mission: {self.sim_config['mission_type']}")
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to setup simulation: {e}")
            return False
    
    def start_controller_bridge(self):
        """Start the controller-to-simulation bridge"""
        if not self.controller.controllers:
            print("❌ No controllers detected")
            return False
        
        if not self.simulator:
            print("❌ Simulation not initialized")
            return False
        
        self.bridge_active = True
        self.control_thread = threading.Thread(target=self._controller_loop, daemon=True)
        self.control_thread.start()
        
        print("🎮 Controller bridge started")
        return True
    
    def _controller_loop(self):
        """Main controller processing loop"""
        import pygame
        
        clock = pygame.time.Clock()
        last_control_time = time.time()
        
        # Register controller callbacks
        self.controller.register_callback('button_down', self._handle_controller_button)
        
        while self.bridge_active:
            try:
                # Process controller events
                self.controller.process_events()
                
                # Get controller input
                input_data = self.controller.get_controller_input()
                
                if input_data and self.simulation_active:
                    self._apply_controller_to_simulation(input_data)
                
                # Control rate limiting
                current_time = time.time()
                if current_time - last_control_time >= 1.0 / 60:  # 60 Hz
                    last_control_time = current_time
                
                clock.tick(60)
                
            except Exception as e:
                print(f"Controller loop error: {e}")
                break
    
    def _handle_controller_button(self, controller_id: int, button: int):
        """Handle controller button presses for simulation control"""
        if controller_id != self.controller.active_controller:
            return
        
        config = self.controller.controllers[controller_id]['config']
        
        if button == config.button_takeoff:
            if not self.simulation_active:
                self.start_simulation()
            else:
                print("🚁 Manual takeoff command")
                # Could trigger specific takeoff behavior
        
        elif button == config.button_land:
            if self.simulation_active:
                print("🛬 Manual landing command")
                # Could trigger landing sequence
        
        elif button == config.button_emergency:
            print("🚨 Emergency stop - Stopping simulation")
            self.stop_simulation()
            self.bridge_active = False
        
        elif button == config.button_hover:
            if self.simulation_active:
                print("⏸️ Hover mode activated")
                # Reset control inputs to zero
                self._send_hover_command()
    
    def _apply_controller_to_simulation(self, input_data):
        """Apply controller inputs to the simulation"""
        if not self.simulator or not self.simulation_active:
            return
        
        # Extract controller inputs
        left_stick = input_data['left_stick']
        right_stick = input_data['right_stick']
        triggers = input_data['triggers']
        
        # Map controller inputs to simulation control commands
        # This is where you'd translate stick movements to drone physics
        
        # Example: Direct velocity control
        control_commands = {
            'velocity_x': left_stick['x'] * 10.0,  # Strafe left/right
            'velocity_y': right_stick['y'] * 5.0,  # Up/down
            'velocity_z': left_stick['y'] * 10.0,  # Forward/back
            'yaw_rate': right_stick['x'] * 90.0    # Rotation
        }
        
        # Apply to simulation (you'd need to implement this in your simulator)
        if hasattr(self.simulator, 'set_manual_control'):
            self.simulator.set_manual_control(control_commands)
    
    def _send_hover_command(self):
        """Send hover command to simulation"""
        if self.simulator and hasattr(self.simulator, 'set_manual_control'):
            hover_commands = {
                'velocity_x': 0.0,
                'velocity_y': 0.0,
                'velocity_z': 0.0,
                'yaw_rate': 0.0
            }
            self.simulator.set_manual_control(hover_commands)
    
    def start_simulation(self):
        """Start the main simulation"""
        if not self.simulator:
            if not self.setup_simulation():
                return False
        
        try:
            # Start simulation in a separate thread
            sim_thread = threading.Thread(
                target=self.simulator.run_simulation, 
                kwargs={'duration': None, 'manual_control': True},
                daemon=True
            )
            sim_thread.start()
            
            self.simulation_active = True
            print("🚀 Simulation started with controller integration")
            return True
            
        except Exception as e:
            print(f"❌ Failed to start simulation: {e}")
            return False
    
    def stop_simulation(self):
        """Stop the simulation"""
        self.simulation_active = False
        if self.simulator:
            self.simulator.stop()
        print("🛑 Simulation stopped")
    
    def get_telemetry(self):
        """Get current simulation telemetry"""
        if self.simulator and self.simulation_active:
            return {
                'position': self.simulator.drone_model.position,
                'velocity': self.simulator.drone_model.velocity,
                'attitude': self.simulator.drone_model.attitude,
                'status': 'FLYING' if self.simulation_active else 'LANDED'
            }
        return None
    
    def run_integrated_system(self):
        """Run the complete integrated system"""
        print("🚀 Starting Integrated Drone Control System")
        print("=" * 50)
        
        # Test controller detection
        if not self.controller.controllers:
            print("❌ No controllers detected. Please connect a game controller.")
            return
        
        # Setup simulation
        if not self.setup_simulation():
            print("❌ Failed to setup simulation system")
            return
        
        # Start controller bridge
        if not self.start_controller_bridge():
            print("❌ Failed to start controller bridge")
            return
        
        print("\n🎮 Controls:")
        print("  A/X Button: Start simulation")
        print("  B/Circle Button: Landing mode")
        print("  Y/Triangle Button: Emergency stop")
        print("  X/Square Button: Hover mode")
        print("  Left Stick: Horizontal movement")
        print("  Right Stick: Vertical movement & rotation")
        print("\nPress Ctrl+C to exit")
        
        try:
            # Main monitoring loop
            while self.bridge_active:
                time.sleep(0.1)
                
                # Optional: Print telemetry
                telemetry = self.get_telemetry()
                if telemetry:
                    pos = telemetry['position']
                    vel = telemetry['velocity']
                    print(f"Pos: X:{pos[0]:.2f} Y:{pos[1]:.2f} Z:{pos[2]:.2f} | "
                          f"Vel: {vel[0]:.2f} {vel[1]:.2f} {vel[2]:.2f} | "
                          f"Status: {telemetry['status']}", end='\r')
        
        except KeyboardInterrupt:
            print("\n⌨️ Keyboard interrupt received")
        
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Clean shutdown of all systems"""
        print("\n🛑 Shutting down integrated control system")
        self.bridge_active = False
        self.stop_simulation()
        
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        
        try:
            import pygame
            pygame.quit()
        except:
            pass
        
        print("👋 System shutdown complete")

def main():
    """Main entry point for integrated system"""
    try:
        system = IntegratedControlSystem()
        system.run_integrated_system()
    except Exception as e:
        print(f"❌ System error: {e}")

if __name__ == "__main__":
    main()