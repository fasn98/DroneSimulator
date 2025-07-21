/**
 * Drone Simulation Web Interface - Functional Backup Version
 * Restored to last working state with dropdown functionality
 */

class DroneSimulationController {
    constructor() {
        this.telemetryData = [];
        this.missionData = [];
        this.simulationStatus = {
            running: false,
            progress: 0,
            currentWaypoint: 0,
            position: [0, 0, 0],
            velocity: [0, 0, 0],
            attitude: [0, 0, 0]
        };
        this.socket = null;
        this.statusPolling = null;
    }

    async init() {
        console.log('Initializing drone simulation controller...');
        
        try {
            // Load configurations
            await this.loadConfigurations();
            
            // Setup event listeners
            this.setupEventListeners();
            
            // Initialize charts
            this.initializeCharts();
            
            // Connect WebSocket
            this.connectWebSocket();
            
            // Start status polling
            this.startStatusPolling();
            
            // Initialize Analytics functionality
            this.initAnalytics();
            
            console.log('Drone simulation controller initialized successfully');
        
        // Set up custom environment listeners
        this.setupCustomEnvironmentListeners();
        } catch (error) {
            console.error('Error initializing controller:', error);
        }
    }

    async loadConfigurations() {
        try {
            // Load drone models
            const dronesResponse = await fetch('/api/configurations/drones');
            if (dronesResponse.ok) {
                const drones = await dronesResponse.json();
                this.populateSelect('droneModel', Object.keys(drones));
                console.log('Loaded drone models:', Object.keys(drones));
            }

            // Load environments
            const envsResponse = await fetch('/api/configurations/environments');
            if (envsResponse.ok) {
                const environments = await envsResponse.json();
                this.populateSelect('environment', Object.keys(environments));
                console.log('Loaded environments:', Object.keys(environments));
            }

            // Load missions
            const missionsResponse = await fetch('/api/configurations/missions');
            if (missionsResponse.ok) {
                const missions = await missionsResponse.json();
                this.populateSelect('missionType', Object.keys(missions));
                console.log('Loaded missions:', Object.keys(missions));
            }
            
        } catch (error) {
            console.error('Error loading configurations:', error);
        }
    }

    populateSelect(selectId, options) {
        const select = document.getElementById(selectId);
        if (select) {
            // Clear existing options
            select.innerHTML = '';
            
            // Add new options
            options.forEach(option => {
                const optionElement = document.createElement('option');
                optionElement.value = option;
                optionElement.textContent = option;
                select.appendChild(optionElement);
            });
            
            console.log(`Populated ${selectId} with ${options.length} options`);
        } else {
            console.error(`Select element ${selectId} not found`);
        }
    }

    setupEventListeners() {
        // Start simulation button
        const startBtn = document.getElementById('startBtn');
        if (startBtn) {
            startBtn.addEventListener('click', async () => {
                await this.startSimulation();
            });
        }

        // Stop simulation button
        const stopBtn = document.getElementById('stopBtn');
        if (stopBtn) {
            stopBtn.addEventListener('click', async () => {
                await this.stopSimulation();
            });
        }

        // Pause simulation button
        const pauseBtn = document.getElementById('pauseBtn');
        if (pauseBtn) {
            pauseBtn.addEventListener('click', async () => {
                await this.pauseSimulation();
            });
        }
    }

    getSimulationConfig() {
        return {
            drone_model: document.getElementById('droneModel')?.value || 'default_quadrotor',
            environment: document.getElementById('environment')?.value || 'earth',
            mission_type: document.getElementById('missionType')?.value || 'reconnaissance'
        };
    }

    async startSimulation() {
        const config = this.getSimulationConfig();
        console.log('Starting simulation with config:', config);
        
        try {
            const response = await fetch('/api/simulation/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(config)
            });

            if (response.ok) {
                const result = await response.json();
                this.clearSimulationData();
                this.showMessage('Simulation started successfully', 'success');
                this.updateButtonStates(true);
                console.log('Simulation started:', result);
            } else {
                const errorData = await response.json();
                this.showMessage(`Failed to start: ${errorData.error}`, 'error');
                console.error('Start failed:', errorData);
            }
        } catch (error) {
            console.error('Error starting simulation:', error);
            this.showMessage('Error starting simulation', 'error');
        }
    }

    async stopSimulation() {
        try {
            const response = await fetch('/api/simulation/stop', {
                method: 'POST'
            });

            if (response.ok) {
                this.showMessage('Simulation stopped', 'info');
                this.updateButtonStates(false);
            } else {
                this.showMessage('Failed to stop simulation', 'error');
            }
        } catch (error) {
            console.error('Error stopping simulation:', error);
        }
    }

    async pauseSimulation() {
        try {
            const response = await fetch('/api/simulation/pause', {
                method: 'POST'
            });

            if (response.ok) {
                this.showMessage('Simulation paused/resumed', 'info');
            }
        } catch (error) {
            console.error('Error pausing simulation:', error);
        }
    }

    clearSimulationData() {
        this.telemetryData = [];
        this.missionData = [];
        this.simulationStatus = {
            running: false,
            progress: 0,
            currentWaypoint: 0,
            position: [0, 0, 0],
            velocity: [0, 0, 0],
            attitude: [0, 0, 0]
        };
        
        console.log('Cleared simulation data');
    }

    connectWebSocket() {
        this.socket = io();
        
        this.socket.on('connect', () => {
            console.log('WebSocket connected');
            this.updateConnectionStatus(true);
        });

        this.socket.on('disconnect', () => {
            console.log('WebSocket disconnected');
            this.updateConnectionStatus(false);
        });

        this.socket.on('telemetry_update', (data) => {
            console.log('WebSocket telemetry received:', data);
            this.handleTelemetryUpdate(data);
        });

        this.socket.on('status_update', (data) => {
            console.log('WebSocket status received:', data);
            this.handleStatusUpdate(data);
        });
        
        this.socket.on('mission_event', (data) => {
            console.log('WebSocket mission event received:', data);
            this.missionData.push(data);
        });
    }

    handleTelemetryUpdate(telemetry) {
        // Extract payload if wrapped
        const data = telemetry.payload || telemetry;
        
        // Fix timestamp issues - ensure we use simulation time (should be < 1000 seconds for typical simulation)
        let timestamp = data.timestamp;
        if (!timestamp || timestamp > 10000) {
            // Use a simulation-relative timestamp
            timestamp = this.telemetryData.length * 0.1; // Approximate simulation time
        }
        
        // Normalize the telemetry data format with proper data validation
        const normalizedData = {
            timestamp: timestamp,
            position_x: parseFloat(data.position?.x || data.position_x || 0),
            position_y: parseFloat(data.position?.y || data.position_y || 0),
            position_z: parseFloat(data.position?.z || data.position_z || 0),
            velocity_x: parseFloat(data.velocity?.x || data.velocity_x || 0),
            velocity_y: parseFloat(data.velocity?.y || data.velocity_y || 0),
            velocity_z: parseFloat(data.velocity?.z || data.velocity_z || 0),
            ground_speed: parseFloat(data.ground_speed || Math.sqrt((data.velocity?.x || 0)**2 + (data.velocity?.y || 0)**2)),
            altitude: parseFloat(data.altitude || data.position?.z || data.position_z || 0),
            roll: parseFloat(data.attitude?.roll || data.roll || 0),
            pitch: parseFloat(data.attitude?.pitch || data.pitch || 0),
            yaw: parseFloat(data.attitude?.yaw || data.yaw || 0),
            mission_progress: parseFloat(data.mission_progress || 0),
            current_waypoint: parseInt(data.current_waypoint || 0)
        };
        
        // Only add valid data with finite numbers
        const isValidData = Object.values(normalizedData).every(val => 
            typeof val === 'number' && isFinite(val) && !isNaN(val)
        );
        
        if (isValidData) {
            this.telemetryData.push(normalizedData);
            
            // Keep more points to show complete trajectory (up to 2000 points for full flight)
            if (this.telemetryData.length > 2000) {
                this.telemetryData = this.telemetryData.slice(-2000);
            }
            
            this.updatePlots();
            this.updateRealtimeDisplay(normalizedData);
        } else {
            console.log('Skipping invalid telemetry data:', normalizedData);
        }
        
        // Update environment display if available
        if (data.environment) {
            this.updateEnvironmentDisplay(data.environment);
        }
    }

    handleStatusUpdate(status) {
        this.simulationStatus = { ...this.simulationStatus, ...status };
        this.updateStatusDisplay();
        
        // Update mission status and distance in real-time display
        if (status.payload) {
            const statusElement = document.getElementById('missionStatus');
            if (statusElement && status.payload.mission_status) {
                const missionStatus = status.payload.mission_status;
                statusElement.textContent = missionStatus.charAt(0).toUpperCase() + missionStatus.slice(1);
            }

            const distanceElement = document.getElementById('missionDistance');
            if (distanceElement && status.payload.total_distance !== undefined) {
                distanceElement.textContent = `${status.payload.total_distance.toFixed(1)} m`;
            }
            
            // Update environment display
            if (status.payload.environment) {
                this.updateEnvironmentDisplay(status.payload.environment);
            }
        }
    }
    
    updateEnvironmentDisplay(envData) {
        // Update gravity
        const gravityElement = document.getElementById('envGravity');
        if (gravityElement && envData.gravity !== undefined) {
            gravityElement.textContent = `${envData.gravity.toFixed(2)} m/s²`;
        }
        
        // Update air density
        const densityElement = document.getElementById('envDensity');
        if (densityElement && envData.air_density !== undefined) {
            densityElement.textContent = `${envData.air_density.toFixed(3)} kg/m³`;
        }
        
        // Update temperature
        const tempElement = document.getElementById('envTemperature');
        if (tempElement && envData.temperature !== undefined) {
            tempElement.textContent = `${envData.temperature.toFixed(1)}°C`;
        }
        
        // Update wind speed
        const windElement = document.getElementById('envWind');
        if (windElement && envData.wind_speed !== undefined) {
            windElement.textContent = `${envData.wind_speed.toFixed(1)} m/s`;
        }
    }
    
    setupCustomEnvironmentListeners() {
        // Load environment preset
        document.getElementById('loadPresetEnv').addEventListener('click', () => {
            const selectEl = document.getElementById('envPresetSelect');
            if (selectEl.style.display === 'none') {
                selectEl.style.display = 'block';
            } else {
                const selectedEnv = selectEl.value;
                this.loadEnvironmentPreset(selectedEnv);
                selectEl.style.display = 'none';
            }
        });
        
        // Environment preset selection
        document.getElementById('envPresetSelect').addEventListener('change', (e) => {
            this.loadEnvironmentPreset(e.target.value);
            e.target.style.display = 'none';
        });
        
        // Reset to Earth defaults
        document.getElementById('resetCustomEnv').addEventListener('click', () => {
            this.loadEnvironmentPreset('earth');
        });
        
        // Apply custom environment
        document.getElementById('applyCustomEnv').addEventListener('click', () => {
            this.applyCustomEnvironment();
        });
    }
    
    async loadEnvironmentPreset(envName) {
        try {
            const response = await fetch('/api/configurations/environments');
            if (response.ok) {
                const environments = await response.json();
                const envConfig = environments[envName];
                
                if (envConfig) {
                    // Update input fields
                    document.getElementById('customGravity').value = envConfig.gravity || 9.81;
                    document.getElementById('customAirDensity').value = envConfig.atmosphere?.sea_level_density || 1.225;
                    document.getElementById('customTemperature').value = 
                        (envConfig.atmosphere?.sea_level_temperature || 288.15) - 273.15; // Convert K to C
                    document.getElementById('customWindSpeed').value = envConfig.wind?.base_speed || 0;
                    document.getElementById('enableWind').checked = envConfig.wind?.enabled || false;
                    document.getElementById('customEnvName').value = envName.charAt(0).toUpperCase() + envName.slice(1);
                    
                    this.showToast(`Loaded ${envName} environment preset`, 'success');
                }
            }
        } catch (error) {
            console.error('Error loading environment preset:', error);
            this.showToast('Error loading environment preset', 'error');
        }
    }
    
    async applyCustomEnvironment() {
        const customConfig = {
            gravity: parseFloat(document.getElementById('customGravity').value),
            atmosphere: {
                sea_level_density: parseFloat(document.getElementById('customAirDensity').value),
                sea_level_temperature: parseFloat(document.getElementById('customTemperature').value) + 273.15, // Convert C to K
                sea_level_pressure: 101325.0,
                temperature_lapse_rate: -0.0065,
                gas_constant: 287.0
            },
            wind: {
                enabled: document.getElementById('enableWind').checked,
                base_speed: parseFloat(document.getElementById('customWindSpeed').value),
                direction: parseFloat(document.getElementById('customWindDirection').value),
                gust_factor: 1.5,
                direction_variability: 30.0
            }
        };
        
        try {
            const response = await fetch('/api/environment/custom', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(customConfig)
            });
            
            if (response.ok) {
                const envName = document.getElementById('customEnvName').value || 'Custom';
                this.showToast(`Applied custom environment: ${envName}`, 'success');
            } else {
                this.showToast('Error applying custom environment', 'error');
            }
        } catch (error) {
            console.error('Error applying custom environment:', error);
            this.showToast('Error applying custom environment', 'error');
        }
    }
    
    showToast(message, type = 'info') {
        // Simple toast notification
        const toast = document.createElement('div');
        toast.className = `alert alert-${type === 'error' ? 'danger' : 'success'} position-fixed`;
        toast.style.cssText = 'top: 20px; right: 20px; z-index: 9999; max-width: 300px;';
        toast.innerHTML = `${message} <button type="button" class="btn-close" onclick="this.parentElement.remove()"></button>`;
        document.body.appendChild(toast);
        
        setTimeout(() => {
            if (toast.parentElement) {
                toast.remove();
            }
        }, 5000);
    }

    updatePlots() {
        this.updateTrajectoryPlot();
        this.updateRealtimePlots();
    }

    updateTrajectoryPlot() {
        console.log('Updating trajectory plot with', this.telemetryData.length, 'points');
        
        if (this.telemetryData.length === 0) {
            console.log('No telemetry data available for plotting');
            return;
        }
        
        try {
            // Extract position data - already normalized
            const positions = this.telemetryData.map(d => ({
                x: d.position_x,
                y: d.position_y,
                z: d.position_z,
                time: d.timestamp
            }));

            console.log('Sample positions:', positions.slice(-3));

            const trace = {
                x: positions.map(p => p.x),
                y: positions.map(p => p.y),
                z: positions.map(p => p.z),
                type: 'scatter3d',
                mode: 'lines+markers',
                marker: { size: 4, color: 'blue' },
                line: { color: 'blue', width: 3 },
                name: 'Flight Path'
            };

            // Add time information to hover text
            const hoverText = positions.map(p => 
                `Time: ${p.time.toFixed(1)}s<br>Position: (${p.x.toFixed(1)}, ${p.y.toFixed(1)}, ${p.z.toFixed(1)})`
            );

            // Calculate optimal ranges based on actual trajectory data
            const xValues = positions.map(p => p.x);
            const yValues = positions.map(p => p.y);
            const zValues = positions.map(p => p.z);
            
            const xMin = Math.min(...xValues, 0);
            const xMax = Math.max(...xValues, 0);
            const yMin = Math.min(...yValues, 0);
            const yMax = Math.max(...yValues, 0);
            const zMin = Math.min(...zValues, 0);
            const zMax = Math.max(...zValues, 30);
            
            // Add 10% padding around the trajectory
            const xPadding = (xMax - xMin) * 0.1 || 10;
            const yPadding = (yMax - yMin) * 0.1 || 10;
            const zPadding = (zMax - zMin) * 0.1 || 5;

            const layout = {
                title: `3D Flight Trajectory (${this.telemetryData.length} points) - Time: ${positions[positions.length-1]?.time.toFixed(1)}s`,
                scene: {
                    xaxis: { 
                        title: 'X (m)', 
                        range: [xMin - xPadding, xMax + xPadding] 
                    },
                    yaxis: { 
                        title: 'Y (m)', 
                        range: [yMin - yPadding, yMax + yPadding] 
                    },
                    zaxis: { 
                        title: 'Z (m)', 
                        range: [Math.max(0, zMin - zPadding), zMax + zPadding] 
                    },
                    camera: {
                        eye: { x: 1.5, y: 1.5, z: 1.5 }
                    }
                },
                margin: { l: 0, r: 0, b: 0, t: 50 }
            };

            // Add hover text to trace
            trace.hovertext = hoverText;
            trace.hoverinfo = 'text';

            const trajectoryElement = document.getElementById('trajectoryPlot');
            if (trajectoryElement) {
                Plotly.react('trajectoryPlot', [trace], layout);
                console.log('Trajectory plot updated successfully');
            } else {
                console.error('trajectoryPlot element not found');
            }
        } catch (error) {
            console.error('Error updating trajectory plot:', error);
        }
    }

    async startStatusPolling() {
        this.statusPolling = setInterval(async () => {
            try {
                const response = await fetch('/api/simulation/status');
                if (response.ok) {
                    const status = await response.json();
                    
                    // Map API response to our format
                    const mappedStatus = {
                        running: status.simulation_running || false,
                        progress: status.mission_progress || 0,
                        currentWaypoint: status.current_waypoint || 0,
                        position: status.position || [0, 0, 0],
                        velocity: status.velocity || [0, 0, 0],
                        attitude: status.attitude || [0, 0, 0]
                    };
                    
                    this.simulationStatus = mappedStatus;
                    this.updateStatusDisplay();
                    this.updateButtonStates(mappedStatus.running);
                    
                    // Remove fake telemetry creation that was causing periodic zero speeds
                    // Real telemetry data comes through WebSocket, no need for fake data
                }
            } catch (error) {
                // Silent fail for polling
            }
        }, 2000);
    }

    updateStatusDisplay() {
        // Update status badge
        const statusElement = document.getElementById('status');
        if (statusElement) {
            statusElement.textContent = this.simulationStatus.running ? 'Running' : 'Stopped';
            statusElement.className = `badge ${this.simulationStatus.running ? 'bg-success' : 'bg-secondary'}`;
        }

        // Update position display
        const posElement = document.getElementById('dronePosition');
        if (posElement && this.simulationStatus.position) {
            const pos = this.simulationStatus.position;
            posElement.textContent = `${pos[0].toFixed(1)}, ${pos[1].toFixed(1)}, ${pos[2].toFixed(1)}`;
        }

        // Update velocity display
        const velElement = document.getElementById('droneVelocity');
        if (velElement && this.simulationStatus.velocity) {
            const vel = this.simulationStatus.velocity;
            const speed = Math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2);
            velElement.textContent = `${speed.toFixed(1)} m/s`;
        }

        // Update progress
        const progressElement = document.getElementById('missionProgressPercentage');
        if (progressElement) {
            progressElement.textContent = `${Math.round(this.simulationStatus.progress || 0)}%`;
        }

        // Update waypoint
        const waypointElement = document.getElementById('missionWaypoints');
        if (waypointElement) {
            waypointElement.textContent = `${this.simulationStatus.currentWaypoint}/6`;
        }

        // Update attitude display
        const attElement = document.getElementById('droneAttitude');
        if (attElement && this.simulationStatus.attitude) {
            const att = this.simulationStatus.attitude;
            attElement.textContent = `R:${(att[0] * 180/Math.PI).toFixed(1)}° P:${(att[1] * 180/Math.PI).toFixed(1)}° Y:${(att[2] * 180/Math.PI).toFixed(1)}°`;
        }
    }

    updateButtonStates(running) {
        const startBtn = document.getElementById('startBtn');
        const stopBtn = document.getElementById('stopBtn');
        const pauseBtn = document.getElementById('pauseBtn');

        if (startBtn) startBtn.disabled = running;
        if (stopBtn) stopBtn.disabled = !running;
        if (pauseBtn) pauseBtn.disabled = !running;
    }

    updateConnectionStatus(connected) {
        const connectionElement = document.getElementById('connectionStatus');
        if (connectionElement) {
            connectionElement.textContent = connected ? 'Connected' : 'Disconnected';
            connectionElement.className = `badge ${connected ? 'bg-success' : 'bg-danger'}`;
        }
    }

    initializeCharts() {
        setTimeout(() => {
            // Initialize 3D trajectory plot
            const trajectoryElement = document.getElementById('trajectoryPlot');
            if (trajectoryElement) {
                Plotly.newPlot('trajectoryPlot', [], { 
                    title: 'Flight Trajectory - Start simulation to see data',
                    scene: {
                        xaxis: { title: 'X (m)' },
                        yaxis: { title: 'Y (m)' },
                        zaxis: { title: 'Z (m)' },
                        aspectmode: 'cube'
                    }
                });
            }

            // Initialize real-time plots with visible sample data to ensure rendering
            const altElement = document.getElementById('altitudePlot');
            if (altElement) {
                const sampleTrace = {
                    x: [0, 10, 20],
                    y: [0, 15, 25],
                    type: 'scatter',
                    mode: 'lines+markers',
                    name: 'Altitude',
                    line: { color: 'green', width: 3 },
                    marker: { size: 6 }
                };
                
                Plotly.newPlot('altitudePlot', [sampleTrace], { 
                    title: { text: 'Altitude vs Time (Ready for Data)', font: { size: 16 } },
                    xaxis: { title: 'Time (s)', showgrid: true, gridcolor: '#e6e6e6' },
                    yaxis: { title: 'Altitude (m)', showgrid: true, gridcolor: '#e6e6e6' },
                    margin: { l: 60, r: 30, t: 50, b: 50 },
                    plot_bgcolor: 'white',
                    paper_bgcolor: 'white'
                }, {
                    responsive: true,
                    displayModeBar: false
                });
                console.log('Initialized altitude plot with visible sample data');
            }

            const speedElement = document.getElementById('speedPlot');
            if (speedElement) {
                const sampleTrace = {
                    x: [0, 10, 20],
                    y: [0, 3, 5],
                    type: 'scatter',
                    mode: 'lines+markers',
                    name: 'Ground Speed',
                    line: { color: 'orange', width: 3 },
                    marker: { size: 6 }
                };
                
                Plotly.newPlot('speedPlot', [sampleTrace], { 
                    title: { text: 'Speed vs Time (Ready for Data)', font: { size: 16 } },
                    xaxis: { title: 'Time (s)', showgrid: true, gridcolor: '#e6e6e6' },
                    yaxis: { title: 'Speed (m/s)', showgrid: true, gridcolor: '#e6e6e6' },
                    margin: { l: 60, r: 30, t: 50, b: 50 },
                    plot_bgcolor: 'white',
                    paper_bgcolor: 'white'
                }, {
                    responsive: true,
                    displayModeBar: false
                });
                console.log('Initialized speed plot with visible sample data');
            }

            const attElement = document.getElementById('attitudePlot');
            if (attElement) {
                const rollTrace = {
                    x: [0, 10, 20],
                    y: [0, 5, -3],
                    type: 'scatter',
                    mode: 'lines',
                    name: 'Roll',
                    line: { color: 'red', width: 2 }
                };
                
                const pitchTrace = {
                    x: [0, 10, 20],
                    y: [0, -2, 4],
                    type: 'scatter',
                    mode: 'lines',
                    name: 'Pitch',
                    line: { color: 'blue', width: 2 }
                };
                
                Plotly.newPlot('attitudePlot', [rollTrace, pitchTrace], { 
                    title: { text: 'Attitude vs Time (Ready for Data)', font: { size: 16 } },
                    xaxis: { title: 'Time (s)', showgrid: true, gridcolor: '#e6e6e6' },
                    yaxis: { title: 'Angle (degrees)', showgrid: true, gridcolor: '#e6e6e6' },
                    margin: { l: 60, r: 30, t: 50, b: 50 },
                    legend: { x: 0, y: 1 },
                    plot_bgcolor: 'white',
                    paper_bgcolor: 'white'
                }, {
                    responsive: true,
                    displayModeBar: false
                });
                console.log('Initialized attitude plot with visible sample data');
            }
        }, 100);
    }

    updateRealtimeDisplay(data) {
        // Update real-time attitude information
        const attElement = document.getElementById('droneAttitude');
        if (attElement) {
            attElement.textContent = `R:${(data.roll * 180/Math.PI).toFixed(1)}° P:${(data.pitch * 180/Math.PI).toFixed(1)}° Y:${(data.yaw * 180/Math.PI).toFixed(1)}°`;
        }

        // Update current velocity
        const velElement = document.getElementById('droneVelocity');
        if (velElement) {
            velElement.textContent = `${data.ground_speed.toFixed(1)} m/s`;
        }

        // Update current altitude
        const altElement = document.getElementById('droneAltitude');
        if (altElement) {
            altElement.textContent = `${data.altitude.toFixed(1)} m`;
        }

        // Update mission status and distance
        const statusElement = document.getElementById('missionStatus');
        if (statusElement) {
            const status = data.mission_status || 'executing';
            statusElement.textContent = status.charAt(0).toUpperCase() + status.slice(1);
        }

        const distanceElement = document.getElementById('missionDistance');
        if (distanceElement) {
            const distance = data.total_distance || 0;
            distanceElement.textContent = `${distance.toFixed(1)} m`;
        }
    }

    updateRealtimePlots() {
        if (this.telemetryData.length < 2) {
            console.log('Not enough telemetry data for real-time plots');
            return;
        }

        try {
            // Get last 50 points for real-time plotting
            const recentData = this.telemetryData.slice(-50);
            const times = recentData.map(d => d.timestamp);
            
            console.log('Updating real-time plots with', recentData.length, 'points');
            console.log('Sample times:', times.slice(0, 3));
            console.log('Sample altitudes:', recentData.slice(0, 3).map(d => d.altitude));
            console.log('Sample speeds:', recentData.slice(0, 3).map(d => d.ground_speed));
            
            // Altitude vs Time plot
            this.updateAltitudePlot(times, recentData.map(d => d.altitude));
            
            // Speed vs Time plot  
            this.updateSpeedPlot(times, recentData.map(d => d.ground_speed));
            
            // Attitude plots
            this.updateAttitudePlots(times, recentData);
            
        } catch (error) {
            console.error('Error updating real-time plots:', error);
        }
    }

    updateAltitudePlot(times, altitudes) {
        const element = document.getElementById('altitudePlot');
        if (!element) {
            console.log('altitudePlot element not found');
            return;
        }

        // Filter out invalid data - handle both null and NaN values
        const validData = times.map((time, i) => ({
            time: time,
            altitude: altitudes[i]
        })).filter(d => d.time != null && d.altitude != null && !isNaN(d.time) && !isNaN(d.altitude) && isFinite(d.time) && isFinite(d.altitude));

        if (validData.length === 0) {
            console.log('No valid altitude data to plot');
            return;
        }

        const trace = {
            x: validData.map(d => d.time),
            y: validData.map(d => d.altitude),
            type: 'scatter',
            mode: 'lines+markers',
            name: 'Altitude',
            line: { color: 'green', width: 3 },
            marker: { size: 6 }
        };

        const layout = {
            title: { text: 'Altitude vs Time', font: { size: 16 } },
            xaxis: { 
                title: 'Time (s)',
                showgrid: true,
                gridcolor: '#e6e6e6'
            },
            yaxis: { 
                title: 'Altitude (m)',
                showgrid: true,
                gridcolor: '#e6e6e6'
            },
            margin: { l: 60, r: 30, t: 50, b: 50 },
            plot_bgcolor: 'white',
            paper_bgcolor: 'white'
        };

        // Force redraw with proper configuration
        Plotly.newPlot(element, [trace], layout, {
            responsive: true,
            displayModeBar: false
        });

        console.log(`Updated altitude plot with ${validData.length} valid points`);
    }

    updateSpeedPlot(times, speeds) {
        const element = document.getElementById('speedPlot');
        if (!element) {
            console.log('speedPlot element not found');
            return;
        }

        // Filter out invalid data - handle both null and NaN values  
        const validData = times.map((time, i) => ({
            time: time,
            speed: speeds[i]
        })).filter(d => d.time != null && d.speed != null && !isNaN(d.time) && !isNaN(d.speed) && isFinite(d.time) && isFinite(d.speed));

        if (validData.length === 0) {
            console.log('No valid speed data to plot');
            return;
        }

        const trace = {
            x: validData.map(d => d.time),
            y: validData.map(d => d.speed),
            type: 'scatter',
            mode: 'lines+markers',
            name: 'Ground Speed',
            line: { color: 'orange', width: 2 },
            marker: { size: 4 }
        };

        const layout = {
            title: { text: 'Speed vs Time', font: { size: 16 } },
            xaxis: { 
                title: 'Time (s)',
                showgrid: true,
                gridcolor: '#e6e6e6'
            },
            yaxis: { 
                title: 'Speed (m/s)',
                showgrid: true,
                gridcolor: '#e6e6e6'
            },
            margin: { l: 60, r: 30, t: 50, b: 50 },
            plot_bgcolor: 'white',
            paper_bgcolor: 'white'
        };

        // Force redraw with proper configuration
        Plotly.newPlot(element, [trace], layout, {
            responsive: true,
            displayModeBar: false
        });
        console.log('Updated speed plot with', validData.length, 'valid points');
    }

    updateAttitudePlots(times, data) {
        const element = document.getElementById('attitudePlot');
        if (!element) {
            console.log('attitudePlot element not found');
            return;
        }

        // Filter out invalid data and convert to degrees
        const validData = times.map((time, i) => ({
            time: time,
            roll: data[i].roll * 180/Math.PI,
            pitch: data[i].pitch * 180/Math.PI,
            yaw: data[i].yaw * 180/Math.PI
        })).filter(d => d.time != null && d.roll != null && d.pitch != null && d.yaw != null && 
                      !isNaN(d.time) && !isNaN(d.roll) && !isNaN(d.pitch) && !isNaN(d.yaw) &&
                      isFinite(d.time) && isFinite(d.roll) && isFinite(d.pitch) && isFinite(d.yaw));

        if (validData.length === 0) {
            console.log('No valid attitude data to plot');
            return;
        }

        const rollTrace = {
            x: validData.map(d => d.time),
            y: validData.map(d => d.roll),
            type: 'scatter',
            mode: 'lines',
            name: 'Roll',
            line: { color: 'red', width: 2 }
        };

        const pitchTrace = {
            x: validData.map(d => d.time),
            y: validData.map(d => d.pitch),
            type: 'scatter',
            mode: 'lines',
            name: 'Pitch',
            line: { color: 'blue', width: 2 }
        };

        const yawTrace = {
            x: validData.map(d => d.time),
            y: validData.map(d => d.yaw),
            type: 'scatter',
            mode: 'lines',
            name: 'Yaw',
            line: { color: 'purple', width: 2 }
        };

        const layout = {
            title: { text: 'Attitude vs Time', font: { size: 16 } },
            xaxis: { 
                title: 'Time (s)',
                showgrid: true,
                gridcolor: '#e6e6e6'
            },
            yaxis: { 
                title: 'Angle (degrees)',
                showgrid: true,
                gridcolor: '#e6e6e6'
            },
            margin: { l: 60, r: 30, t: 50, b: 50 },
            legend: { x: 0, y: 1 },
            plot_bgcolor: 'white',
            paper_bgcolor: 'white'
        };

        // Force redraw with proper configuration
        Plotly.newPlot(element, [rollTrace, pitchTrace, yawTrace], layout, {
            responsive: true,
            displayModeBar: false
        });
        console.log('Updated attitude plot with', validData.length, 'valid points');
    }

    // Analytics functionality
    initAnalytics() {
        console.log('Initializing Analytics functionality');
        
        // Bind Analytics tab activation
        const analyticsTab = document.querySelector('[href="#analytics"]');
        if (analyticsTab) {
            analyticsTab.addEventListener('click', () => {
                setTimeout(() => this.loadSessionHistory(), 100);
            });
        }
        
        // Bind refresh button
        const refreshBtn = document.getElementById('refreshHistoryBtn');
        if (refreshBtn) {
            refreshBtn.addEventListener('click', () => {
                this.loadSessionHistory();
            });
        }
        
        // Bind analytics update button
        const updateAnalyticsBtn = document.getElementById('updateAnalyticsBtn');
        if (updateAnalyticsBtn) {
            updateAnalyticsBtn.addEventListener('click', () => {
                this.updateAnalyticsCharts();
            });
        }
    }

    async loadSessionHistory() {
        console.log('Loading session history for Analytics');
        
        try {
            const response = await fetch('/api/analytics/sessions?limit=50');
            
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            
            const data = await response.json();
            console.log('Session history data:', data);
            
            this.displaySessionHistory(data.sessions);
            this.loadPerformanceSummary(data.sessions);
            
            // Update session count display
            const sessionCountElement = document.getElementById('sessionCount');
            if (sessionCountElement) {
                sessionCountElement.textContent = `${data.sessions.length} sessions found`;
            }
            
        } catch (error) {
            console.error('Error loading session history:', error);
            this.showMessage('Failed to load session history', 'error');
            
            // Show fallback message
            const sessionHistoryElement = document.getElementById('sessionHistory');
            if (sessionHistoryElement) {
                sessionHistoryElement.innerHTML = `
                    <div class="alert alert-warning">
                        <i class="fas fa-exclamation-triangle"></i>
                        Unable to load session history. ${error.message}
                    </div>
                `;
            }
        }
    }

    displaySessionHistory(sessions) {
        const sessionHistoryElement = document.getElementById('sessionHistory');
        if (!sessionHistoryElement) return;
        
        if (!sessions || sessions.length === 0) {
            sessionHistoryElement.innerHTML = `
                <div class="alert alert-info">
                    <i class="fas fa-info-circle"></i>
                    No simulation sessions found. Run a simulation to see history here.
                </div>
            `;
            return;
        }
        
        let html = '';
        sessions.forEach(session => {
            const startTime = new Date(session.start_time).toLocaleString();
            const statusIcon = session.status === 'completed' ? 'check-circle text-success' : 
                             session.status === 'running' ? 'play-circle text-primary' : 'exclamation-circle text-warning';
            
            html += `
                <div class="card mb-2 session-card" data-session-id="${session.id}">
                    <div class="card-body p-3">
                        <div class="row align-items-center">
                            <div class="col-8">
                                <h6 class="mb-1">
                                    <i class="fas fa-${statusIcon}"></i>
                                    ${session.drone_model} - ${session.environment}
                                </h6>
                                <small class="text-muted">
                                    ${session.mission_type} | ${startTime}
                                </small>
                            </div>
                            <div class="col-4 text-end">
                                <div class="small">
                                    <div>Progress: ${session.mission_progress || 0}%</div>
                                    <div>Duration: ${session.duration ? session.duration.toFixed(1) + 's' : 'N/A'}</div>
                                </div>
                                <button class="btn btn-outline-primary btn-sm mt-1 view-details-btn" 
                                        data-session-id="${session.id}">
                                    <i class="fas fa-eye"></i> View Details
                                </button>
                            </div>
                        </div>
                    </div>
                </div>
            `;
        });
        
        sessionHistoryElement.innerHTML = html;
        
        // Add click handlers for view details buttons
        sessionHistoryElement.querySelectorAll('.view-details-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                e.stopPropagation();
                const sessionId = btn.getAttribute('data-session-id');
                this.viewSessionDetails(sessionId);
            });
        });
    }

    loadPerformanceSummary(sessions) {
        const summaryElement = document.getElementById('performanceSummary');
        if (!summaryElement || !sessions) return;
        
        const totalSessions = sessions.length;
        const completedSessions = sessions.filter(s => s.status === 'completed').length;
        const successRate = totalSessions > 0 ? (completedSessions / totalSessions * 100).toFixed(1) : 0;
        
        const avgDuration = sessions.filter(s => s.duration).reduce((sum, s) => sum + s.duration, 0) / 
                           sessions.filter(s => s.duration).length || 0;
        const avgProgress = sessions.reduce((sum, s) => sum + (s.mission_progress || 0), 0) / totalSessions || 0;
        
        summaryElement.innerHTML = `
            <div class="row">
                <div class="col-6">
                    <div class="metric-card">
                        <div class="metric-value">${totalSessions}</div>
                        <div class="metric-label">Total Sessions</div>
                    </div>
                </div>
                <div class="col-6">
                    <div class="metric-card">
                        <div class="metric-value">${successRate}%</div>
                        <div class="metric-label">Success Rate</div>
                    </div>
                </div>
                <div class="col-6">
                    <div class="metric-card">
                        <div class="metric-value">${avgDuration.toFixed(1)}s</div>
                        <div class="metric-label">Avg Duration</div>
                    </div>
                </div>
                <div class="col-6">
                    <div class="metric-card">
                        <div class="metric-value">${avgProgress.toFixed(1)}%</div>
                        <div class="metric-label">Avg Progress</div>
                    </div>
                </div>
            </div>
        `;
    }

    async viewSessionDetails(sessionId) {
        console.log('Loading details for session:', sessionId);
        // This would show detailed session information
        this.showMessage(`Loading details for session ${sessionId}...`, 'info');
    }

    async updateAnalyticsCharts() {
        console.log('Updating analytics charts');
        // This would update the performance and trends charts
        this.showMessage('Analytics charts updated', 'success');
    }

    showMessage(message, type) {
        console.log(`${type.toUpperCase()}: ${message}`);
        
        // Create simple alert
        const alertClass = type === 'success' ? 'alert-success' : 
                          type === 'error' ? 'alert-danger' : 'alert-info';
        
        const alertDiv = document.createElement('div');
        alertDiv.className = `alert ${alertClass} alert-dismissible fade show`;
        alertDiv.innerHTML = `
            ${message}
            <button type="button" class="btn-close" data-bs-dismiss="alert"></button>
        `;
        
        // Insert at top of main content
        const mainContent = document.querySelector('.container-fluid') || document.body;
        mainContent.insertBefore(alertDiv, mainContent.firstChild);
        
        // Auto-dismiss after 3 seconds
        setTimeout(() => {
            if (alertDiv.parentNode) {
                alertDiv.remove();
            }
        }, 3000);
    }
}

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    const controller = new DroneSimulationController();
    controller.init();
});