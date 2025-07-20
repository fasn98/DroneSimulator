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
        
        // Normalize the telemetry data format
        const normalizedData = {
            timestamp: data.timestamp || Date.now() / 1000,
            position_x: data.position?.x || data.position_x || 0,
            position_y: data.position?.y || data.position_y || 0,
            position_z: data.position?.z || data.position_z || 0,
            velocity_x: data.velocity?.x || data.velocity_x || 0,
            velocity_y: data.velocity?.y || data.velocity_y || 0,
            velocity_z: data.velocity?.z || data.velocity_z || 0,
            ground_speed: data.ground_speed || Math.sqrt((data.velocity?.x || 0)**2 + (data.velocity?.y || 0)**2),
            altitude: data.altitude || data.position?.z || data.position_z || 0,
            roll: data.attitude?.roll || data.roll || 0,
            pitch: data.attitude?.pitch || data.pitch || 0,
            yaw: data.attitude?.yaw || data.yaw || 0,
            mission_progress: data.mission_progress || 0,
            current_waypoint: data.current_waypoint || 0
        };
        
        this.telemetryData.push(normalizedData);
        
        // Keep more points to show complete trajectory (up to 2000 points for full flight)
        if (this.telemetryData.length > 2000) {
            this.telemetryData = this.telemetryData.slice(-2000);
        }
        
        this.updatePlots();
        this.updateRealtimeDisplay(normalizedData);
        
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
                    
                    // Also create fake telemetry from status for trajectory plotting
                    if (mappedStatus.running && status.position) {
                        const fakeTelemetry = {
                            position_x: status.position[0],
                            position_y: status.position[1], 
                            position_z: status.position[2],
                            timestamp: Date.now() / 1000
                        };
                        
                        // Add to telemetry data for plotting
                        this.telemetryData.push(fakeTelemetry);
                        if (this.telemetryData.length > 2000) {
                            this.telemetryData = this.telemetryData.slice(-2000);
                        }
                        this.updatePlots();
                    }
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

            // Initialize real-time plots with proper layouts
            const altElement = document.getElementById('altitudePlot');
            if (altElement) {
                Plotly.newPlot('altitudePlot', [], { 
                    title: 'Altitude vs Time',
                    xaxis: { title: 'Time (s)' },
                    yaxis: { title: 'Altitude (m)' },
                    margin: { l: 50, r: 20, t: 40, b: 50 }
                });
                console.log('Initialized altitude plot');
            }

            const speedElement = document.getElementById('speedPlot');
            if (speedElement) {
                Plotly.newPlot('speedPlot', [], { 
                    title: 'Speed vs Time',
                    xaxis: { title: 'Time (s)' },
                    yaxis: { title: 'Speed (m/s)' },
                    margin: { l: 50, r: 20, t: 40, b: 50 }
                });
                console.log('Initialized speed plot');
            }

            const attElement = document.getElementById('attitudePlot');
            if (attElement) {
                Plotly.newPlot('attitudePlot', [], { 
                    title: 'Attitude vs Time',
                    xaxis: { title: 'Time (s)' },
                    yaxis: { title: 'Angle (degrees)' },
                    margin: { l: 50, r: 20, t: 40, b: 50 }
                });
                console.log('Initialized attitude plot');
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

        // Filter out invalid data
        const validData = times.map((time, i) => ({
            time: time,
            altitude: altitudes[i]
        })).filter(d => !isNaN(d.time) && !isNaN(d.altitude));

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
            line: { color: 'green', width: 2 },
            marker: { size: 4 }
        };

        const layout = {
            title: 'Altitude vs Time',
            xaxis: { 
                title: 'Time (s)',
                range: [Math.min(...validData.map(d => d.time)), Math.max(...validData.map(d => d.time))]
            },
            yaxis: { 
                title: 'Altitude (m)',
                range: [Math.min(...validData.map(d => d.altitude)) - 5, Math.max(...validData.map(d => d.altitude)) + 5]
            },
            margin: { l: 50, r: 20, t: 40, b: 50 },
            showlegend: false,
            autosize: true
        };

        Plotly.react('altitudePlot', [trace], layout);
        console.log('Updated altitude plot with', validData.length, 'valid points');
    }

    updateSpeedPlot(times, speeds) {
        const element = document.getElementById('speedPlot');
        if (!element) {
            console.log('speedPlot element not found');
            return;
        }

        // Filter out invalid data
        const validData = times.map((time, i) => ({
            time: time,
            speed: speeds[i]
        })).filter(d => !isNaN(d.time) && !isNaN(d.speed));

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
            title: 'Speed vs Time',
            xaxis: { 
                title: 'Time (s)',
                range: [Math.min(...validData.map(d => d.time)), Math.max(...validData.map(d => d.time))]
            },
            yaxis: { 
                title: 'Speed (m/s)',
                range: [0, Math.max(...validData.map(d => d.speed)) * 1.1]
            },
            margin: { l: 50, r: 20, t: 40, b: 50 },
            showlegend: false,
            autosize: true
        };

        Plotly.react('speedPlot', [trace], layout);
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
        })).filter(d => !isNaN(d.time) && !isNaN(d.roll) && !isNaN(d.pitch) && !isNaN(d.yaw));

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
            title: 'Attitude vs Time',
            xaxis: { 
                title: 'Time (s)',
                range: [Math.min(...validData.map(d => d.time)), Math.max(...validData.map(d => d.time))]
            },
            yaxis: { 
                title: 'Angle (degrees)',
                range: [-180, 180]
            },
            margin: { l: 50, r: 20, t: 40, b: 50 },
            legend: { x: 0, y: 1 },
            autosize: true
        };

        Plotly.react('attitudePlot', [rollTrace, pitchTrace, yawTrace], layout);
        console.log('Updated attitude plot with', validData.length, 'valid points');
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