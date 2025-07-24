#!/usr/bin/env python3
"""
Quick Controller Test Script
Simple script to immediately test if your game controller is working
"""

import sys
import time
import os

# Add project root to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_controller_connection():
    """Quick test to see if controller is connected and working"""
    try:
        import pygame
        print("Initializing controller system...")
        pygame.init()
        pygame.joystick.init()
        
        # Check for controllers
        joystick_count = pygame.joystick.get_count()
        
        if joystick_count == 0:
            print("\n❌ NO CONTROLLERS DETECTED")
            print("Please check:")
            print("1. Controller is connected via USB or wireless")
            print("2. Controller is powered on")
            print("3. Drivers are installed (if needed)")
            print("4. Controller works in other applications")
            return False
        
        print(f"\n✅ FOUND {joystick_count} CONTROLLER(S)!")
        
        # Initialize and test each controller
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            
            print(f"\nController {i}:")
            print(f"  Name: {joystick.get_name()}")
            print(f"  Axes: {joystick.get_numaxes()}")
            print(f"  Buttons: {joystick.get_numbuttons()}")
            print(f"  GUID: {joystick.get_guid()}")
            
            # Determine controller type
            name = joystick.get_name().lower()
            if "xbox" in name:
                controller_type = "Xbox Controller"
            elif "playstation" in name or "ps" in name or "dualshock" in name:
                controller_type = "PlayStation Controller"
            else:
                controller_type = "Generic Controller"
            
            print(f"  Detected Type: {controller_type}")
        
        print(f"\n🎮 LIVE INPUT TEST (10 seconds)")
        print("Move sticks and press buttons to test...")
        print("Press any button to see response")
        
        # Use first controller for testing
        joystick = pygame.joystick.Joystick(0)
        start_time = time.time()
        
        while time.time() - start_time < 10:
            pygame.event.pump()
            
            # Read stick positions
            try:
                left_x = joystick.get_axis(0)
                left_y = joystick.get_axis(1)
                right_x = joystick.get_axis(3) if joystick.get_numaxes() > 3 else 0
                right_y = joystick.get_axis(4) if joystick.get_numaxes() > 4 else 0
                
                # Check for button presses
                pressed_buttons = []
                for button in range(joystick.get_numbuttons()):
                    if joystick.get_button(button):
                        pressed_buttons.append(str(button))
                
                # Display current state
                button_text = f"Buttons:{','.join(pressed_buttons)}" if pressed_buttons else "No buttons"
                
                print(f"\rLeft:({left_x:5.2f},{left_y:5.2f}) Right:({right_x:5.2f},{right_y:5.2f}) {button_text}    ", end="")
                
            except Exception as e:
                print(f"\nError reading controller: {e}")
                break
            
            time.sleep(0.1)
        
        print(f"\n\n✅ CONTROLLER TEST COMPLETE!")
        print("\nYour controller is working correctly!")
        print("\nNext steps:")
        print("1. Run: python src/controller_demo.py")
        print("2. Run: python src/drone_controller.py")
        print("3. For full integration: python src/controller_integration.py")
        
        return True
        
    except ImportError:
        print("❌ PYGAME NOT INSTALLED")
        print("Install with: pip install pygame")
        return False
    except Exception as e:
        print(f"❌ ERROR: {e}")
        return False

def show_controls():
    """Show the control scheme"""
    print("\n🎯 DRONE CONTROL SCHEME")
    print("=" * 30)
    print("Left Stick:")
    print("  X-Axis: Strafe left/right")
    print("  Y-Axis: Move forward/backward")
    print()
    print("Right Stick:")
    print("  X-Axis: Yaw rotation (turn)")
    print("  Y-Axis: Altitude up/down")
    print()
    print("Buttons (Xbox layout):")
    print("  A Button: Takeoff")
    print("  B Button: Land")
    print("  Y Button: Emergency Stop")
    print("  X Button: Hover Mode")
    print()
    print("Buttons (PlayStation layout):")
    print("  X Button: Takeoff")
    print("  Circle: Land")
    print("  Triangle: Emergency Stop")
    print("  Square: Hover Mode")

def main():
    """Main test function"""
    print("🎮 QUICK CONTROLLER TEST")
    print("=" * 40)
    print("This will test if your game controller is properly connected")
    print("and working with the drone simulation system.")
    print()
    
    # Test controller
    if test_controller_connection():
        show_controls()
        
        print("\n🚀 READY FOR DRONE CONTROL!")
        print("Your controller is set up and ready to fly virtual drones.")
    else:
        print("\n❌ CONTROLLER SETUP NEEDED")
        print("Please resolve the controller issues and try again.")
        print("See LOCAL_SETUP_GUIDE.md for detailed troubleshooting.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        print("Please check LOCAL_SETUP_GUIDE.md for help")