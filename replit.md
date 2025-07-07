# Exploration Drone Simulation Application

## Overview

This is a comprehensive Python-based simulation application for exploration drones designed to model UAV behavior across different planetary environments (Earth, Mars, Moon). The application provides both command-line and web-based interfaces for controlling and monitoring drone simulations with realistic physics modeling, mission planning, and data visualization capabilities.

## System Architecture

### Core Architecture
- **Modular Design**: Clean separation of concerns across physics, environment, mission management, and visualization
- **Multi-Interface Support**: Both CLI (`main.py`) and web interface (`web_server.py`) for different use cases
- **Real-time Simulation**: WebSocket-based real-time data streaming for web interface
- **Configuration-Driven**: JSON-based configuration system for drones, environments, and missions

### Technology Stack
- **Backend**: Python 3.x with Flask web framework
- **Real-time Communication**: Flask-SocketIO for WebSocket connections
- **Physics Simulation**: NumPy, SciPy for mathematical computations
- **Data Visualization**: Matplotlib, Seaborn, Plotly for charts and plots
- **Frontend**: HTML5, JavaScript, Bootstrap 5, Font Awesome icons
- **Data Storage**: JSON/CSV file-based storage (no database currently)

## Key Components

### 1. Simulation Engine (`src/simulator.py`)
- **Purpose**: Main orchestrator that coordinates all simulation components
- **Features**: Manages simulation lifecycle, timestep control, component integration
- **Architecture**: Uses composition pattern to integrate drone model, physics engine, environment, and mission manager

### 2. Physics Engine (`src/physics_engine.py`)
- **Purpose**: Implements 6-DoF (6 Degrees of Freedom) flight dynamics
- **Features**: Gravitational forces, aerodynamic calculations, propulsion modeling
- **Integration**: Uses RK4 or Euler methods for numerical integration

### 3. Drone Model (`src/drone_model.py`)
- **Purpose**: Manages drone specifications and physical parameters
- **Configuration**: Loads from `config/drone_models.json` with detailed specifications
- **Features**: Mass properties, dimensions, propulsion, aerodynamics, flight envelope

### 4. Environment (`src/environment.py`)
- **Purpose**: Models planetary environments with atmospheric conditions
- **Supported Environments**: Earth, Mars, Moon with different gravity and atmospheric properties
- **Features**: Atmospheric models, wind simulation, temperature variations

### 5. Mission Manager (`src/mission_manager.py`)
- **Purpose**: Handles mission planning and execution
- **Mission Types**: Reconnaissance, sample transport, monitoring
- **Features**: Waypoint navigation, success criteria, constraint management

### 6. Data Logger (`src/data_logger.py`)
- **Purpose**: Comprehensive telemetry data collection and storage
- **Formats**: CSV and JSON export capabilities
- **Features**: Real-time logging, performance metrics, mission event tracking

### 7. Visualization (`src/visualization.py`)
- **Purpose**: Data plotting and analysis tools
- **Features**: 3D trajectory plots, performance metrics, mission monitoring charts

### 8. Web Interface (`web/`)
- **Purpose**: Browser-based simulation control and monitoring
- **Features**: Real-time telemetry display, interactive controls, live charts
- **Technology**: Bootstrap 5 UI, Plotly charts, WebSocket communication

## Data Flow

### Simulation Loop
1. **Initialization**: Load configurations for drone, environment, and mission
2. **Physics Step**: Calculate forces, moments, and state updates using physics engine
3. **Mission Update**: Check waypoints, objectives, and mission status
4. **Data Logging**: Record telemetry data and mission events
5. **Real-time Output**: Stream data to web interface via WebSocket
6. **Visualization**: Update charts and plots with new data

### Web Interface Flow
1. **Configuration Loading**: Fetch available drones, environments, and missions
2. **Simulation Control**: Start/stop/pause simulation through REST API
3. **Real-time Updates**: Receive telemetry data via WebSocket
4. **Data Visualization**: Update charts and display current status
5. **Mission Monitoring**: Track mission progress and objectives

## External Dependencies

### Python Packages
- **Flask**: Web framework for REST API and static file serving
- **Flask-SocketIO**: WebSocket support for real-time communication
- **NumPy**: Numerical computations and array operations
- **SciPy**: Scientific computing and numerical integration
- **Matplotlib**: 2D plotting and visualization
- **Seaborn**: Statistical data visualization
- **Pandas**: Data manipulation and analysis (implied usage)

### Frontend Dependencies
- **Bootstrap 5**: UI framework for responsive design
- **Font Awesome**: Icon library for UI elements
- **Plotly.js**: Interactive charting library
- **WebSocket API**: Browser-native WebSocket support

### Configuration Files
- `config/drone_models.json`: Drone specifications and parameters
- `config/environments.json`: Planetary environment definitions
- `config/missions.json`: Mission types and objectives

## Deployment Strategy

### Development Setup
- **Local Development**: Direct Python execution with Flask development server
- **Configuration**: Environment-specific JSON configuration files
- **Data Storage**: Local file system for logs and output data

### Production Considerations
- **Web Server**: Flask development server (suitable for development/testing)
- **File Storage**: Local file system for simulation data and logs
- **Real-time Communication**: WebSocket connections for live data streaming
- **Static Assets**: Served directly by Flask

### Scaling Considerations
- **Multi-User Support**: Currently single-user focused
- **Data Persistence**: File-based storage may need database upgrade for production
- **Performance**: Simulation timestep and complexity affect real-time performance

## Changelog
- July 07, 2025. Initial setup
- July 07, 2025. Fixed telemetry graph layout and waypoint display issues:
  - Resolved telemetry graph overlapping by restructuring Power & Energy and Mission Progress plots
  - Added realistic power consumption calculations based on speed and altitude
  - Fixed waypoint list display to show all mission waypoints regardless of simulation state
  - Added fallback logic for mission selection and improved debug logging
  - Enhanced real-time data visualization with proper graph spacing
- July 07, 2025. Added comprehensive Analytics interface with database integration:
  - Implemented PostgreSQL database with smart data models for sessions, telemetry, and analytics
  - Created Analytics tab in web interface with session history, performance metrics, and interactive charts
  - Added API endpoints for historical data access and performance analysis
  - Integrated real-time database logging during simulations for complete data persistence
  - Enhanced system with smart analytics features for trend analysis and performance comparison
- July 07, 2025. Fixed critical session lifecycle management issues:
  - Resolved sessions getting stuck in "running" state after completion
  - Added proper database session completion when missions finish or are stopped
  - Fixed Analytics interface JavaScript errors for session details display
  - Updated performance summary to handle actual API response format
  - Cleaned up existing stuck sessions in database for accurate analytics display
- July 07, 2025. Completed comprehensive session lifecycle management fixes:
  - Fixed database service method references and Flask application context issues
  - Implemented proper session completion with `_complete_simulation()` method
  - Added Flask app context to telemetry logging and session completion operations
  - Verified manual session stop functionality works correctly
  - Analytics interface now displays accurate session history with proper completion status
  - Performance metrics show correct duration, distance, and energy consumption data

## User Preferences

Preferred communication style: Simple, everyday language.