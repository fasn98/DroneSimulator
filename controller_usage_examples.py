#!/usr/bin/env python3
"""
Virtual Drone Controller - Usage Examples and Demonstrations
Complete examples showing all controller system capabilities
"""

import time
import threading
import sys
from src.drone_controller import DroneControlSystem, GameControllerInput, VirtualDrone

def example_1_basic_controller_detection():
    """Example 1: Basic controller detection and information display"""
    print("🎮 Example 1: Controller Detection")
    print("=" * 40)
    
    controller = GameControllerInput()
    
    if not controller.controllers:
        print("❌ No controllers detected. Connect a USB/WiFi controller to test.")
        print("Supported controllers: Xbox, PlayStation, generic USB controllers")
        return False
    
    print("✅ Controllers detected:")
    for i, ctrl in controller.controllers.items():
        joystick = ctrl['joystick']
        config = ctrl['config']
        
        print(f"  Controller {i}: {ctrl['name']}")
        print(f"    Axes: {joystick.get_numaxes()}")
        print(f"    Buttons: {joystick.get_numbuttons()}")
        print(f"    Configuration: {config.name}")
        print(f"    Dead Zone: {config.dead_zone}")
        print(f"    Button Mapping:")
        print(f"      Takeoff: {config.button_takeoff}")
        print(f"      Land: {config.button_land}")
        print(f"      Emergency: {config.button_emergency}")
    
    return True

def example_2_input_reading():
    """Example 2: Real-time controller input reading"""
    print("\n🕹️ Example 2: Real-time Input Reading")
    print("=" * 40)
    print("Move sticks and press buttons for 10 seconds...")
    print("Left Stick: Horizontal movement | Right Stick: Vertical & yaw")
    
    controller = GameControllerInput()
    
    if not controller.controllers:
        print("❌ No controllers available for testing")
        return
    
    start_time = time.time()
    max_duration = 10.0  # 10 seconds
    
    while time.time() - start_time < max_duration:
        input_data = controller.get_controller_input()
        
        if input_data:
            left = input_data['left_stick']
            right = input_data['right_stick']
            triggers = input_data['triggers']
            
            # Display current input values
            print(f"\rLeft: ({left['x']:5.2f}, {left['y']:5.2f}) | "
                  f"Right: ({right['x']:5.2f}, {right['y']:5.2f}) | "
                  f"Triggers: L{triggers['left']:4.2f} R{triggers['right']:4.2f}", end="")
            
            # Show pressed buttons
            pressed = [str(i) for i, pressed in input_data['buttons'].items() if pressed]
            if pressed:
                print(f" | Buttons: {','.join(pressed)}", end="")
        
        time.sleep(0.05)  # 20 Hz update rate
    
    print("\n✅ Input reading test complete")

def example_3_virtual_drone_control():
    """Example 3: Complete virtual drone control demonstration"""
    print("\n🚁 Example 3: Virtual Drone Control")
    print("=" * 40)
    print("Controls:")
    print("  A/X Button: Takeoff")
    print("  B/Circle Button: Land")
    print("  Y/Triangle Button: Emergency Stop (exits demo)")
    print("  Left Stick: Horizontal movement")
    print("  Right Stick: Vertical movement & yaw")
    print("\nStarting 30-second flight demo...")
    
    # Create drone and controller
    drone = VirtualDrone("quadcopter")
    controller = GameControllerInput()
    
    if not controller.controllers:
        print("❌ No controllers available - simulating with random movements")
        return simulate_flight_without_controller(drone)
    
    # Setup
    start_time = time.time()
    last_time = time.time()
    demo_duration = 30.0
    
    # Register button handler
    def handle_buttons(controller_id, button):
        if controller_id != controller.active_controller:
            return
        
        config = controller.controllers[controller_id]['config']
        
        if button == config.button_takeoff:
            drone.takeoff()
        elif button == config.button_land:
            drone.land()
        elif button == config.button_emergency:
            drone.emergency_stop()
            return True  # Signal to exit
        
        return False
    
    controller.register_callback('button_down', 
        lambda cid, btn: handle_buttons(cid, btn))
    
    print("Demo running... Press Y/Triangle to stop early")
    
    try:
        while time.time() - start_time < demo_duration:
            # Process controller
            controller.process_events()
            input_data = controller.get_controller_input()
            
            if input_data:
                # Apply controller inputs to drone
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
            
            time.sleep(0.016)  # ~60 FPS
            
    except KeyboardInterrupt:
        print("\n⌨️ Demo interrupted by user")
    
    print(f"\n✅ Virtual drone demo complete")
    print(f"Final position: X:{drone.state.x:.1f}m Y:{drone.state.y:.1f}m Z:{drone.state.z:.1f}m")
    print(f"Battery remaining: {drone.state.battery:.1f}%")

def simulate_flight_without_controller(drone):
    """Simulate flight patterns without physical controller"""
    import math
    
    print("🤖 Simulating flight patterns without controller...")
    
    # Auto takeoff
    drone.takeoff()
    time.sleep(2)
    
    start_time = time.time()
    
    while time.time() - start_time < 15:  # 15 second demo
        current_time = time.time() - start_time
        
        # Create circular flight pattern
        radius = 5.0
        speed = 0.5
        
        # Simulate controller inputs for circular movement
        fake_left_stick = {
            'x': math.cos(current_time * speed),
            'y': math.sin(current_time * speed)
        }
        fake_right_stick = {
            'x': speed * 0.5,  # Slow yaw rotation
            'y': 0.1 * math.sin(current_time * 2)  # Gentle altitude variation
        }
        fake_triggers = {'left': 0.0, 'right': 0.0}
        
        # Apply to drone
        drone.set_control_inputs(fake_left_stick, fake_right_stick, fake_triggers)
        
        # Update physics
        dt = 0.05
        drone.update_physics(dt)
        
        # Display telemetry
        drone.print_telemetry()
        
        time.sleep(dt)
    
    # Auto land
    drone.land()
    time.sleep(2)
    
    print(f"\n✅ Simulated flight complete")

def example_4_controller_configuration():
    """Example 4: Controller configuration and customization"""
    print("\n⚙️ Example 4: Controller Configuration")
    print("=" * 40)
    
    controller = GameControllerInput()
    
    if not controller.controllers:
        print("❌ No controllers to configure")
        return
    
    # Show current configuration
    active_config = controller.controllers[0]['config']
    print("Current configuration:")
    print(f"  Dead zone: {active_config.dead_zone}")
    print(f"  Sensitivity: {active_config.sensitivity}")
    print(f"  Controller type: {active_config.name}")
    
    # Demonstrate configuration modification
    print("\nTesting different sensitivity settings...")
    
    sensitivities = [0.5, 1.0, 1.5, 2.0]
    
    for sensitivity in sensitivities:
        print(f"\nTesting sensitivity: {sensitivity}")
        active_config.sensitivity = sensitivity
        
        # Test for 3 seconds
        start_time = time.time()
        while time.time() - start_time < 3:
            input_data = controller.get_controller_input()
            
            if input_data:
                left = input_data['left_stick']
                right = input_data['right_stick']
                
                print(f"\rSensitivity {sensitivity}: "
                      f"Left({left['x']:5.2f},{left['y']:5.2f}) "
                      f"Right({right['x']:5.2f},{right['y']:5.2f})", end="")
            
            time.sleep(0.1)
    
    # Reset to default
    active_config.sensitivity = 1.0
    print(f"\n✅ Configuration test complete")

def example_5_integration_demo():
    """Example 5: Integration with existing simulation system"""
    print("\n🔗 Example 5: Integration Demo")
    print("=" * 40)
    
    try:
        from src.controller_integration import IntegratedControlSystem
        
        print("Creating integrated control system...")
        system = IntegratedControlSystem()
        
        # Setup simulation
        if system.setup_simulation():
            print("✅ Simulation system integrated successfully")
            print(f"Configuration: {system.sim_config}")
        else:
            print("❌ Failed to setup simulation integration")
            return
        
        # Test controller bridge
        if system.start_controller_bridge():
            print("✅ Controller bridge started")
        else:
            print("❌ Controller bridge failed to start")
            return
        
        print("Integrated system ready for use")
        print("Note: Full integration requires physical controller for testing")
        
        time.sleep(2)
        system.shutdown()
        
    except ImportError as e:
        print(f"❌ Integration modules not available: {e}")
    except Exception as e:
        print(f"❌ Integration error: {e}")

def example_6_performance_test():
    """Example 6: Performance and latency testing"""
    print("\n⚡ Example 6: Performance Testing")
    print("=" * 40)
    
    controller = GameControllerInput()
    
    if not controller.controllers:
        print("❌ No controllers for performance testing")
        return
    
    print("Testing controller input latency and processing speed...")
    
    # Test input processing speed
    iterations = 1000
    start_time = time.time()
    
    for i in range(iterations):
        input_data = controller.get_controller_input()
        
        if input_data:
            # Simulate processing
            left = input_data['left_stick']
            right = input_data['right_stick']
            
            # Simple calculations
            magnitude = (left['x']**2 + left['y']**2)**0.5
            angle = math.atan2(right['y'], right['x'])
    
    end_time = time.time()
    total_time = end_time - start_time
    avg_time = (total_time / iterations) * 1000  # ms
    
    print(f"Performance Results:")
    print(f"  Total iterations: {iterations}")
    print(f"  Total time: {total_time:.3f}s")
    print(f"  Average processing time: {avg_time:.3f}ms per cycle")
    print(f"  Theoretical max frequency: {1000/avg_time:.1f} Hz")
    
    if avg_time < 16.67:  # 60 FPS
        print("✅ Performance excellent for 60+ FPS control")
    elif avg_time < 33.33:  # 30 FPS
        print("⚠️ Performance adequate for 30+ FPS control")
    else:
        print("❌ Performance may be insufficient for real-time control")

def main():
    """Run all examples in sequence"""
    print("🎮 Virtual Drone Controller - Complete Examples")
    print("=" * 60)
    print("This demonstration shows all controller system capabilities")
    print()
    
    # Run examples
    examples = [
        ("Controller Detection", example_1_basic_controller_detection),
        ("Input Reading", example_2_input_reading),
        ("Virtual Drone Control", example_3_virtual_drone_control),
        ("Configuration", example_4_controller_configuration),
        ("Integration Demo", example_5_integration_demo),
        ("Performance Test", example_6_performance_test)
    ]
    
    for name, func in examples:
        try:
            print(f"\n{'='*20} {name} {'='*20}")
            func()
        except KeyboardInterrupt:
            print(f"\n⌨️ Skipping {name} due to user interrupt")
            continue
        except Exception as e:
            print(f"❌ Error in {name}: {e}")
            continue
        
        # Pause between examples
        time.sleep(1)
    
    print("\n" + "="*60)
    print("🎉 All examples completed!")
    print("For interactive testing, run: python src/controller_demo.py")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n👋 Examples terminated by user")
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        sys.exit(1)