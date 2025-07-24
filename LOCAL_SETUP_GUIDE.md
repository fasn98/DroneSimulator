# Local Setup Guide for Game Controller Testing

## Quick Start - Testing Your Controller

### 1. Download the Project
```bash
git clone [your-project-repository]
cd drone-simulation
```

### 2. Install Dependencies
```bash
pip install pygame numpy scipy matplotlib flask flask-socketio
```

### 3. Connect Your Controller
- Connect your Xbox, PlayStation, or generic USB controller
- Ensure it's recognized by your system (check device manager/system preferences)

### 4. Test Controller Detection
```bash
python src/controller_demo.py
```
This will show a menu with options to:
- View controller information
- Test input readings
- Calibrate controller
- Run flight demonstrations

### 5. Basic Drone Control
```bash
python src/drone_controller.py
```
This starts the main control system with these controls:
- **Left Stick**: Horizontal movement (strafe left/right, forward/back)
- **Right Stick**: Vertical movement (up/down) & yaw rotation
- **A/X Button**: Takeoff
- **B/Circle Button**: Land
- **Y/Triangle Button**: Emergency Stop

### 6. Full Integration with Simulation
```bash
python src/controller_integration.py
```
This connects your controller to the complete simulation platform.

## Troubleshooting

### Controller Not Detected
1. **Check Connection**: Ensure USB cable is properly connected
2. **Check Drivers**: Install controller drivers if needed
3. **Test in Other Apps**: Verify controller works in games/other software
4. **Try Different USB Port**: Some ports may not provide enough power

### Button Mapping Issues
1. **Run Calibration**: Use the demo menu's calibration option
2. **Check Controller Type**: The system auto-detects Xbox/PlayStation/generic
3. **Manual Configuration**: Edit controller configs if needed

### Poor Response/Lag
1. **Close Other Applications**: Free up system resources
2. **Check USB Cable**: Use a high-quality USB cable
3. **Adjust Settings**: Reduce dead zone or increase sensitivity

## Advanced Features

### Custom Configuration
Edit `src/drone_controller.py` to modify:
- Dead zone settings (default: 0.15)
- Sensitivity curves (default: 1.0)
- Button mappings
- Control smoothing (default: 0.1)

### Multiple Controllers
The system supports up to 4 controllers simultaneously:
```python
# Controllers are automatically numbered 0, 1, 2, 3
# First detected controller becomes active
```

### Integration with Web Interface
1. Start the web server: `python run.py`
2. Start controller system: `python src/controller_integration.py`
3. Open browser to `http://localhost:5000`
4. Use controller for manual control while viewing telemetry

## Performance Optimization

### For Best Performance
- **Close unnecessary applications**
- **Use wired connection** (USB cable)
- **High refresh rate**: System runs at 60 FPS
- **Low latency**: Typical response time <16ms

### System Requirements
- **OS**: Windows 10+, macOS 10.14+, or Linux
- **Python**: 3.7 or higher
- **RAM**: 4GB minimum, 8GB recommended
- **Controller**: Any DirectInput/XInput compatible controller

## Testing Checklist

✅ **Controller Detection**
- [ ] Controller appears in demo menu
- [ ] Correct controller type identified
- [ ] All axes and buttons responsive

✅ **Basic Controls**
- [ ] Left stick controls horizontal movement
- [ ] Right stick controls vertical/yaw
- [ ] Takeoff button works
- [ ] Landing button works
- [ ] Emergency stop works

✅ **Advanced Features**
- [ ] Smooth control response
- [ ] Dead zone eliminates drift
- [ ] Real-time telemetry display
- [ ] Integration with simulation works

## Common Controller Types

### Xbox Controllers
- **Xbox One/Series X**: Full compatibility
- **Xbox 360**: Full compatibility
- **Button Layout**: A=Takeoff, B=Land, Y=Emergency, X=Hover

### PlayStation Controllers
- **PS4/PS5**: Full compatibility (may need DS4Windows on PC)
- **PS3**: Limited compatibility
- **Button Layout**: X=Takeoff, Circle=Land, Triangle=Emergency, Square=Hover

### Generic USB Controllers
- **Logitech F310/F710**: Excellent compatibility
- **Generic brands**: Usually work with auto-detection
- **Configuration**: Automatic mapping with fallback to generic config

## Next Steps

Once your controller is working:
1. **Practice basic flight** with the standalone controller system
2. **Try different drone types** (quadcopter, racing, heavy-lift)
3. **Integrate with missions** using the full simulation platform
4. **Record flights** using the video export system
5. **Analyze performance** with the analytics dashboard

For support, check the `CONTROLLER_DOCUMENTATION.md` file for detailed technical information.