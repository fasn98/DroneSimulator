"""
Virtual Controller Simulator
Simulates game controller input for testing in environments without physical controllers
"""

import time
import math
import threading
import random
from typing import Dict, Optional
from dataclasses import dataclass

@dataclass
class SimulatedInput:
    """Simulated controller input state"""
    left_stick_x: float = 0.0
    left_stick_y: float = 0.0
    right_stick_x: float = 0.0
    right_stick_y: float = 0.0
    left_trigger: float = 0.0
    right_trigger: float = 0.0
    buttons: Dict[int, bool] = None
    
    def __post_init__(self):
        if self.buttons is None:
            self.buttons = {i: False for i in range(12)}

class ControllerSimulator:
    """Simulates game controller input patterns for testing"""
    
    def __init__(self):
        self.running = False
        self.current_input = SimulatedInput()
        self.pattern_mode = "manual"  # manual, circular, random, takeoff_landing
        self.start_time = 0
        self.simulation_thread = None
        
    def start_simulation(self, pattern: str = "circular"):
        """Start simulating controller input"""
        self.pattern_mode = pattern
        self.running = True
        self.start_time = time.time()
        
        self.simulation_thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self.simulation_thread.start()
        
        print(f"Controller simulation started - Pattern: {pattern}")
    
    def stop_simulation(self):
        """Stop the simulation"""
        self.running = False
        if self.simulation_thread:
            self.simulation_thread.join(timeout=1.0)
        print("Controller simulation stopped")
    
    def _simulation_loop(self):
        """Main simulation loop"""
        while self.running:
            elapsed = time.time() - self.start_time
            
            if self.pattern_mode == "circular":
                self._simulate_circular_pattern(elapsed)
            elif self.pattern_mode == "random":
                self._simulate_random_pattern(elapsed)
            elif self.pattern_mode == "takeoff_landing":
                self._simulate_takeoff_landing_pattern(elapsed)
            elif self.pattern_mode == "figure8":
                self._simulate_figure8_pattern(elapsed)
            
            time.sleep(0.05)  # 20 Hz update rate
    
    def _simulate_circular_pattern(self, elapsed: float):
        """Simulate circular flight pattern"""
        radius = 0.7
        speed = 0.5
        
        # Circular horizontal movement
        self.current_input.left_stick_x = radius * math.cos(elapsed * speed)
        self.current_input.left_stick_y = radius * math.sin(elapsed * speed)
        
        # Gentle altitude variation
        self.current_input.right_stick_y = 0.2 * math.sin(elapsed * 0.3)
        
        # Slow yaw rotation
        self.current_input.right_stick_x = 0.3 * math.sin(elapsed * 0.2)
    
    def _simulate_random_pattern(self, elapsed: float):
        """Simulate random controller movements"""
        # Add some randomness with smooth changes
        noise_scale = 0.1
        time_scale = 0.5
        
        self.current_input.left_stick_x += random.uniform(-noise_scale, noise_scale)
        self.current_input.left_stick_y += random.uniform(-noise_scale, noise_scale)
        self.current_input.right_stick_x += random.uniform(-noise_scale, noise_scale)
        self.current_input.right_stick_y += random.uniform(-noise_scale, noise_scale)
        
        # Clamp values
        self.current_input.left_stick_x = max(-1.0, min(1.0, self.current_input.left_stick_x))
        self.current_input.left_stick_y = max(-1.0, min(1.0, self.current_input.left_stick_y))
        self.current_input.right_stick_x = max(-1.0, min(1.0, self.current_input.right_stick_x))
        self.current_input.right_stick_y = max(-1.0, min(1.0, self.current_input.right_stick_y))
        
        # Occasional button presses
        if random.random() < 0.01:  # 1% chance per update
            button = random.randint(0, 3)
            self.current_input.buttons[button] = True
            threading.Timer(0.5, lambda: setattr(self.current_input.buttons, str(button), False)).start()
    
    def _simulate_takeoff_landing_pattern(self, elapsed: float):
        """Simulate takeoff, hover, and landing sequence"""
        cycle_time = 20.0  # 20 second cycle
        phase = (elapsed % cycle_time) / cycle_time
        
        if phase < 0.2:  # Takeoff phase (0-4 seconds)
            self.current_input.right_stick_y = 0.8  # Ascend
            self.current_input.buttons[0] = True  # Takeoff button
        elif phase < 0.7:  # Flight phase (4-14 seconds)
            self.current_input.buttons[0] = False
            # Gentle movements during flight
            self.current_input.left_stick_x = 0.3 * math.sin(elapsed * 0.5)
            self.current_input.left_stick_y = 0.3 * math.cos(elapsed * 0.3)
            self.current_input.right_stick_y = 0.1 * math.sin(elapsed * 0.2)
        else:  # Landing phase (14-20 seconds)
            self.current_input.left_stick_x = 0.0
            self.current_input.left_stick_y = 0.0
            self.current_input.right_stick_y = -0.5  # Descend
            self.current_input.buttons[1] = True  # Land button
    
    def _simulate_figure8_pattern(self, elapsed: float):
        """Simulate figure-8 flight pattern"""
        t = elapsed * 0.3  # Slow down the pattern
        
        # Figure-8 mathematical pattern
        self.current_input.left_stick_x = 0.8 * math.sin(t)
        self.current_input.left_stick_y = 0.8 * math.sin(2 * t)
        
        # Slight altitude changes
        self.current_input.right_stick_y = 0.2 * math.sin(t * 0.5)
    
    def get_simulated_input(self) -> Dict:
        """Get current simulated input in the format expected by the drone controller"""
        return {
            'left_stick': {
                'x': self.current_input.left_stick_x,
                'y': self.current_input.left_stick_y
            },
            'right_stick': {
                'x': self.current_input.right_stick_x,
                'y': self.current_input.right_stick_y
            },
            'triggers': {
                'left': self.current_input.left_trigger,
                'right': self.current_input.right_trigger
            },
            'buttons': self.current_input.buttons.copy()
        }
    
    def set_manual_input(self, left_x=0, left_y=0, right_x=0, right_y=0, buttons=None):
        """Manually set controller input values"""
        self.current_input.left_stick_x = max(-1.0, min(1.0, left_x))
        self.current_input.left_stick_y = max(-1.0, min(1.0, left_y))
        self.current_input.right_stick_x = max(-1.0, min(1.0, right_x))
        self.current_input.right_stick_y = max(-1.0, min(1.0, right_y))
        
        if buttons:
            for button_id, pressed in buttons.items():
                if button_id in self.current_input.buttons:
                    self.current_input.buttons[button_id] = pressed

def test_controller_simulation():
    """Test the controller simulator with different patterns"""
    from src.drone_controller import VirtualDrone
    
    print("Testing Virtual Controller Simulator")
    print("=" * 40)
    
    # Create simulator and drone
    simulator = ControllerSimulator()
    drone = VirtualDrone("quadcopter")
    
    patterns = ["circular", "random", "takeoff_landing", "figure8"]
    
    for pattern in patterns:
        print(f"\nTesting {pattern} pattern for 10 seconds...")
        
        simulator.start_simulation(pattern)
        start_time = time.time()
        last_time = time.time()
        
        while time.time() - start_time < 10:
            # Get simulated input
            input_data = simulator.get_simulated_input()
            
            # Apply to drone
            drone.set_control_inputs(
                input_data['left_stick'],
                input_data['right_stick'],
                input_data['triggers']
            )
            
            # Update drone physics
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            drone.update_physics(dt)
            
            # Display telemetry
            if int(time.time() * 2) % 2 == 0:  # Update display at 2 Hz
                drone.print_telemetry()
            
            time.sleep(0.05)
        
        simulator.stop_simulation()
        print(f"\n{pattern} pattern test complete")
        time.sleep(1)
    
    print("\nAll simulation patterns tested successfully!")

def interactive_controller_demo():
    """Interactive demo showing controller simulation capabilities"""
    simulator = ControllerSimulator()
    
    print("Interactive Controller Simulator Demo")
    print("=" * 40)
    print("Available patterns:")
    print("1. Circular - Smooth circular flight")
    print("2. Random - Random movements")
    print("3. Takeoff/Landing - Complete flight cycle")
    print("4. Figure8 - Figure-8 pattern")
    print("5. Manual - Set custom values")
    print("6. Exit")
    
    while True:
        try:
            choice = input("\nSelect pattern (1-6): ").strip()
            
            if choice == "1":
                simulator.start_simulation("circular")
                input("Press Enter to stop...")
                simulator.stop_simulation()
            
            elif choice == "2":
                simulator.start_simulation("random")
                input("Press Enter to stop...")
                simulator.stop_simulation()
            
            elif choice == "3":
                simulator.start_simulation("takeoff_landing")
                input("Press Enter to stop...")
                simulator.stop_simulation()
            
            elif choice == "4":
                simulator.start_simulation("figure8")
                input("Press Enter to stop...")
                simulator.stop_simulation()
            
            elif choice == "5":
                print("Manual input mode - Enter values between -1.0 and 1.0")
                try:
                    left_x = float(input("Left stick X: ") or "0")
                    left_y = float(input("Left stick Y: ") or "0")
                    right_x = float(input("Right stick X: ") or "0")
                    right_y = float(input("Right stick Y: ") or "0")
                    
                    simulator.set_manual_input(left_x, left_y, right_x, right_y)
                    input_data = simulator.get_simulated_input()
                    
                    print("Simulated input:")
                    print(f"  Left stick: ({input_data['left_stick']['x']:.2f}, {input_data['left_stick']['y']:.2f})")
                    print(f"  Right stick: ({input_data['right_stick']['x']:.2f}, {input_data['right_stick']['y']:.2f})")
                    
                except ValueError:
                    print("Invalid input values")
            
            elif choice == "6":
                break
            
            else:
                print("Invalid choice")
                
        except KeyboardInterrupt:
            print("\nDemo interrupted")
            break
        except Exception as e:
            print(f"Error: {e}")
    
    simulator.stop_simulation()
    print("Demo ended")

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "test":
        test_controller_simulation()
    else:
        interactive_controller_demo()