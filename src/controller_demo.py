"""
Game Controller Demo and Testing Script
Demonstrates all controller capabilities and provides testing interface
"""

import pygame
import time
import math
from src.drone_controller import GameControllerInput, VirtualDrone, DroneControlSystem

class ControllerDemo:
    """Demo class for testing controller functionality"""
    
    def __init__(self):
        self.controller = GameControllerInput()
        self.running = False
        self.demo_mode = "input_test"  # input_test, calibration, flight_demo
    
    def run_input_test(self):
        """Test and display all controller inputs"""
        print("🎮 Controller Input Test Mode")
        print("Move sticks and press buttons to see input values")
        print("Press Start/Menu to exit")
        
        clock = pygame.time.Clock()
        self.running = True
        
        while self.running:
            # Process events
            self.controller.process_events()
            
            # Get input data
            input_data = self.controller.get_controller_input()
            
            if input_data:
                # Display input values
                left = input_data['left_stick']
                right = input_data['right_stick']
                triggers = input_data['triggers']
                buttons = input_data['buttons']
                
                # Clear screen and display inputs
                print("\033[2J\033[H", end="")  # Clear screen
                print("🎮 CONTROLLER INPUT TEST")
                print("=" * 40)
                print(f"Left Stick:  X: {left['x']:6.3f}  Y: {left['y']:6.3f}")
                print(f"Right Stick: X: {right['x']:6.3f}  Y: {right['y']:6.3f}")
                print(f"Triggers:    L: {triggers['left']:6.3f}  R: {triggers['right']:6.3f}")
                print("\nButtons:")
                
                # Display button states
                for i, pressed in buttons.items():
                    if pressed:
                        print(f"  Button {i}: PRESSED")
                
                # Check for exit condition (button 7 is typically Start/Menu)
                if buttons.get(7, False) or buttons.get(9, False):
                    self.running = False
            
            clock.tick(30)
    
    def run_calibration(self):
        """Interactive controller calibration"""
        print("🔧 Controller Calibration Mode")
        print("Follow the prompts to calibrate your controller")
        
        calibration_data = {}
        
        # Test each input
        inputs_to_test = [
            ("left_stick_right", "Move LEFT STICK fully RIGHT and press A"),
            ("left_stick_left", "Move LEFT STICK fully LEFT and press A"),
            ("left_stick_up", "Move LEFT STICK fully UP and press A"),
            ("left_stick_down", "Move LEFT STICK fully DOWN and press A"),
            ("right_stick_right", "Move RIGHT STICK fully RIGHT and press A"),
            ("right_stick_left", "Move RIGHT STICK fully LEFT and press A"),
            ("right_stick_up", "Move RIGHT STICK fully UP and press A"),
            ("right_stick_down", "Move RIGHT STICK fully DOWN and press A"),
        ]
        
        clock = pygame.time.Clock()
        
        for test_name, instruction in inputs_to_test:
            print(f"\n{instruction}")
            waiting = True
            
            while waiting:
                self.controller.process_events()
                input_data = self.controller.get_controller_input()
                
                if input_data and input_data['buttons'].get(0, False):  # A button
                    calibration_data[test_name] = {
                        'left_stick': input_data['left_stick'],
                        'right_stick': input_data['right_stick']
                    }
                    waiting = False
                    time.sleep(0.5)  # Debounce
                
                clock.tick(30)
        
        # Display calibration results
        print("\n📊 Calibration Results:")
        for test_name, data in calibration_data.items():
            print(f"{test_name}: {data}")
        
        # Save calibration data
        import json
        with open('controller_calibration.json', 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        print("✅ Calibration saved to controller_calibration.json")
    
    def run_flight_demo(self):
        """Full flight demonstration"""
        print("🚁 Flight Demo Mode")
        
        # Create virtual drone
        drone = VirtualDrone("quadcopter")
        clock = pygame.time.Clock()
        last_time = time.time()
        
        self.running = True
        
        # Register button callbacks
        self.controller.register_callback('button_down', 
            lambda controller_id, button: self._handle_demo_button(drone, controller_id, button))
        
        print("🎮 Flight Demo Controls:")
        print("  A/X: Takeoff")
        print("  B/Circle: Land")
        print("  Y/Triangle: Emergency Stop")
        print("  Left Stick: Horizontal movement")
        print("  Right Stick: Vertical & yaw")
        
        while self.running:
            # Process controller
            self.controller.process_events()
            input_data = self.controller.get_controller_input()
            
            if input_data:
                # Apply controls to drone
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
            drone.print_telemetry()
            
            clock.tick(60)
    
    def _handle_demo_button(self, drone, controller_id, button):
        """Handle button presses in demo mode"""
        if controller_id != self.controller.active_controller:
            return
        
        config = self.controller.controllers[controller_id]['config']
        
        if button == config.button_takeoff:
            drone.takeoff()
        elif button == config.button_land:
            drone.land()
        elif button == config.button_emergency:
            drone.emergency_stop()
            self.running = False
    
    def run_latency_test(self):
        """Test controller latency and responsiveness"""
        print("⚡ Controller Latency Test")
        print("Press buttons rapidly to test response time")
        
        button_times = {}
        clock = pygame.time.Clock()
        self.running = True
        start_time = time.time()
        
        while self.running and (time.time() - start_time) < 30:  # 30 second test
            self.controller.process_events()
            input_data = self.controller.get_controller_input()
            
            if input_data:
                current_time = time.time()
                
                # Track button press timing
                for button_id, pressed in input_data['buttons'].items():
                    if pressed and button_id not in button_times:
                        button_times[button_id] = current_time
                        print(f"Button {button_id} pressed at {current_time:.3f}s")
                    elif not pressed and button_id in button_times:
                        duration = current_time - button_times[button_id]
                        print(f"Button {button_id} held for {duration:.3f}s")
                        del button_times[button_id]
                
                # Exit on Start button
                if input_data['buttons'].get(7, False):
                    self.running = False
            
            clock.tick(120)  # High frequency for latency testing
        
        print("✅ Latency test complete")
    
    def run_menu(self):
        """Main demo menu"""
        while True:
            print("\n🎮 CONTROLLER DEMO MENU")
            print("=" * 30)
            print("1. Input Test - View all controller inputs")
            print("2. Calibration - Calibrate controller ranges")
            print("3. Flight Demo - Full drone control demo")
            print("4. Latency Test - Test response timing")
            print("5. Exit")
            
            try:
                choice = input("\nSelect option (1-5): ")
                
                if choice == "1":
                    self.run_input_test()
                elif choice == "2":
                    self.run_calibration()
                elif choice == "3":
                    self.run_flight_demo()
                elif choice == "4":
                    self.run_latency_test()
                elif choice == "5":
                    break
                else:
                    print("❌ Invalid choice")
                    
            except KeyboardInterrupt:
                print("\n👋 Exiting demo")
                break
            except Exception as e:
                print(f"❌ Error: {e}")

def advanced_controller_info():
    """Display detailed controller information"""
    controller = GameControllerInput()
    
    if not controller.controllers:
        print("❌ No controllers detected")
        return
    
    print("🎮 DETAILED CONTROLLER INFORMATION")
    print("=" * 50)
    
    for i, ctrl in controller.controllers.items():
        joystick = ctrl['joystick']
        config = ctrl['config']
        
        print(f"\nController {i}: {ctrl['name']}")
        print(f"  GUID: {joystick.get_guid()}")
        print(f"  Axes: {joystick.get_numaxes()}")
        print(f"  Buttons: {joystick.get_numbuttons()}")
        print(f"  Hats: {joystick.get_numhats()}")
        print(f"  Configuration: {config.name}")
        print(f"  Dead Zone: {config.dead_zone}")
        print(f"  Sensitivity: {config.sensitivity}")
        
        # Test all axes
        print("  Axis Values:")
        for axis in range(joystick.get_numaxes()):
            value = joystick.get_axis(axis)
            print(f"    Axis {axis}: {value:6.3f}")
        
        # Test all buttons
        print("  Button States:")
        pressed_buttons = []
        for button in range(joystick.get_numbuttons()):
            if joystick.get_button(button):
                pressed_buttons.append(str(button))
        
        if pressed_buttons:
            print(f"    Pressed: {', '.join(pressed_buttons)}")
        else:
            print("    None pressed")

def main():
    """Main demo entry point"""
    print("🎮 Virtual Drone Controller Demo & Testing Suite")
    print("=" * 55)
    
    # Check for controllers first
    controller = GameControllerInput()
    if not controller.controllers:
        print("❌ No game controllers detected!")
        print("Please connect a controller and try again.")
        return
    
    print("✅ Controllers detected:")
    for i, ctrl in controller.controllers.items():
        print(f"  {i}: {ctrl['name']}")
    
    # Show detailed info
    advanced_controller_info()
    
    # Run demo menu
    demo = ControllerDemo()
    demo.run_menu()

if __name__ == "__main__":
    main()