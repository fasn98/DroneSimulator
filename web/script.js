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
            
            // Load Analytics immediately for testing
            setTimeout(() => {
                console.log('Force loading Analytics for debugging');
                this.loadSessionHistory();
            }, 1000);
            
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

        // Session 87 test button
        const session87Btn = document.getElementById('loadSession87Btn');
        if (session87Btn) {
            session87Btn.addEventListener('click', () => {
                console.log('Loading session 87 for vertical speed testing');
                this.loadSessionTelemetry(87);
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
        
        // Calculate ground speed from velocity components if not provided
        const velocity_x = parseFloat(data.velocity?.x || data.velocity_x || 0);
        const velocity_y = parseFloat(data.velocity?.y || data.velocity_y || 0);
        const velocity_z = parseFloat(data.velocity?.z || data.velocity_z || 0);
        const ground_speed = parseFloat(data.ground_speed || Math.sqrt(velocity_x**2 + velocity_y**2));
        const pitch = parseFloat(data.attitude?.pitch || data.pitch || 0);
        
        // Calculate vertical speed and vector speed based on horizontal speed and pitch angle
        // Vertical speed = horizontal_speed * sin(pitch) + velocity_z
        // Vector speed = sqrt(horizontal_speed^2 + vertical_speed^2)
        const vertical_speed = ground_speed * Math.sin(pitch) + velocity_z;
        const vector_speed = Math.sqrt(ground_speed**2 + vertical_speed**2);
        
        // Normalize the telemetry data format with proper data validation
        const normalizedData = {
            timestamp: timestamp,
            position_x: parseFloat(data.position?.x || data.position_x || 0),
            position_y: parseFloat(data.position?.y || data.position_y || 0),
            position_z: parseFloat(data.position?.z || data.position_z || 0),
            velocity_x: velocity_x,
            velocity_y: velocity_y,
            velocity_z: velocity_z,
            ground_speed: ground_speed,
            vertical_speed: vertical_speed,
            vector_speed: vector_speed,
            altitude: parseFloat(data.altitude || data.position?.z || data.position_z || 0),
            roll: parseFloat(data.attitude?.roll || data.roll || 0),
            pitch: pitch,
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
        
        // Update compass on dashboard during real-time monitoring
        if (this.telemetryData.length > 0) {
            const times = this.telemetryData.map(d => d.timestamp);
            this.updateCompassYaw(times, this.telemetryData);
        }
        
        // Store telemetry data in localStorage for persistence
        if (this.telemetryData.length > 0) {
            try {
                const recentData = this.telemetryData.slice(-500); // Keep last 500 points
                localStorage.setItem('telemetryData', JSON.stringify(recentData));
            } catch (error) {
                console.log('Could not save telemetry data to localStorage:', error);
            }
        }
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

        // Update progress display - both percentage text and visual elements
        const progressPercentage = Math.round(this.simulationStatus.progress || 0);
        
        // Update percentage text
        const progressElement = document.getElementById('missionProgressPercentage');
        if (progressElement) {
            progressElement.textContent = `${progressPercentage}%`;
        }
        
        // Update visual progress bar fill
        const progressFill = document.getElementById('missionProgressFill');
        if (progressFill) {
            progressFill.style.width = `${progressPercentage}%`;
        }
        
        // Update drone icon position
        const droneIcon = document.getElementById('droneIconContainer');
        if (droneIcon) {
            droneIcon.style.left = `${progressPercentage}%`;
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

            // Initialize real-time plots as empty - no sample data
            const altElement = document.getElementById('altitudePlot');
            if (altElement) {
                Plotly.newPlot('altitudePlot', [], { 
                    title: { text: 'Altitude vs Time - Start Simulation to See Data', font: { size: 16 } },
                    xaxis: { title: 'Time (s)', showgrid: true, gridcolor: '#e6e6e6', range: [0, 10] },
                    yaxis: { title: 'Altitude (m)', showgrid: true, gridcolor: '#e6e6e6', range: [0, 10] },
                    margin: { l: 60, r: 30, t: 50, b: 50 },
                    plot_bgcolor: '#f8f9fa',
                    paper_bgcolor: 'white',
                    annotations: [{
                        text: 'No simulation data available<br>Start a simulation to see live telemetry',
                        x: 0.5,
                        y: 0.5,
                        xref: 'paper',
                        yref: 'paper',
                        showarrow: false,
                        font: { size: 14, color: '#6c757d' },
                        align: 'center'
                    }]
                }, {
                    responsive: true,
                    displayModeBar: false
                });
                console.log('Initialized empty altitude plot');
            }

            const speedElement = document.getElementById('speedPlot');
            if (speedElement) {
                Plotly.newPlot('speedPlot', [], { 
                    title: { text: 'Speed vs Time - Start Simulation to See Data', font: { size: 16 } },
                    xaxis: { title: 'Time (s)', showgrid: true, gridcolor: '#e6e6e6', range: [0, 10] },
                    yaxis: { title: 'Speed (m/s)', showgrid: true, gridcolor: '#e6e6e6', range: [0, 5] },
                    margin: { l: 60, r: 30, t: 50, b: 50 },
                    plot_bgcolor: '#f8f9fa',
                    paper_bgcolor: 'white',
                    legend: { x: 0, y: 1 },
                    annotations: [{
                        text: 'No simulation data available<br>Start a simulation to see Horizontal, Vertical & Vector speeds',
                        x: 0.5,
                        y: 0.5,
                        xref: 'paper',
                        yref: 'paper',
                        showarrow: false,
                        font: { size: 14, color: '#6c757d' },
                        align: 'center'
                    }]
                }, {
                    responsive: true,
                    displayModeBar: false
                });
                console.log('Initialized empty speed plot');
            }

            const attElement = document.getElementById('attitudePlot');
            if (attElement) {
                Plotly.newPlot('attitudePlot', [], { 
                    title: { text: 'Attitude vs Time - Start Simulation to See Data', font: { size: 16 } },
                    xaxis: { title: 'Time (s)', showgrid: true, gridcolor: '#e6e6e6', range: [0, 10] },
                    yaxis: { title: 'Angle (degrees)', showgrid: true, gridcolor: '#e6e6e6', range: [-10, 10] },
                    margin: { l: 60, r: 30, t: 50, b: 50 },
                    plot_bgcolor: '#f8f9fa',
                    paper_bgcolor: 'white',
                    annotations: [{
                        text: 'No simulation data available<br>Start a simulation to see live telemetry',
                        x: 0.5,
                        y: 0.5,
                        xref: 'paper',
                        yref: 'paper',
                        showarrow: false,
                        font: { size: 14, color: '#6c757d' },
                        align: 'center'
                    }]
                }, {
                    responsive: true,
                    displayModeBar: false
                });
                console.log('Initialized empty attitude plot');
            }
        }, 100);
    }

    updateRealtimeDisplay(data) {
        // Update real-time attitude information
        const attElement = document.getElementById('droneAttitude');
        if (attElement) {
            attElement.textContent = `R:${(data.roll * 180/Math.PI).toFixed(1)}° P:${(data.pitch * 180/Math.PI).toFixed(1)}° Y:${(data.yaw * 180/Math.PI).toFixed(1)}°`;
        }

        // Update current velocity - show vector speed as main velocity
        const velElement = document.getElementById('droneVelocity');
        if (velElement) {
            velElement.textContent = `${data.vector_speed.toFixed(1)} m/s`;
        }

        // Update current altitude
        const altElement = document.getElementById('droneAltitude');
        if (altElement) {
            altElement.textContent = `${data.altitude.toFixed(1)} m`;
        }

        // Update horizontal speed
        const horizontalSpeedElement = document.getElementById('droneHorizontalSpeed');
        if (horizontalSpeedElement) {
            horizontalSpeedElement.textContent = `${data.ground_speed.toFixed(1)} m/s`;
        }

        // Update vertical speed
        const verticalSpeedElement = document.getElementById('droneVerticalSpeed');
        if (verticalSpeedElement) {
            verticalSpeedElement.textContent = `${data.vertical_speed.toFixed(1)} m/s`;
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
        if (this.telemetryData.length === 0) {
            console.log('No telemetry data for real-time plots');
            return;
        }

        try {
            // For historical data, use more points to show the complete flight
            const dataPoints = this.telemetryData.length > 100 ? 200 : Math.min(50, this.telemetryData.length);
            const recentData = this.telemetryData.slice(-dataPoints);
            const times = recentData.map(d => d.timestamp);
            
            console.log('Updating real-time plots with', recentData.length, 'points');
            console.log('Sample times:', times.slice(0, 3));
            console.log('Sample altitudes:', recentData.slice(0, 3).map(d => d.altitude));
            console.log('Sample speeds:', recentData.slice(0, 3).map(d => d.ground_speed));
            
            // Altitude vs Time plot
            this.updateAltitudePlot(times, recentData.map(d => d.altitude));
            
            // Speed vs Time plot with multiple speed types
            this.updateSpeedPlot(times, recentData);
            
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
            paper_bgcolor: 'white',
            annotations: [] // Clear any "no data" annotations when showing real data
        };

        // Debug plot element before creation
        console.log('Altitude plot element:', element);
        console.log('Element dimensions:', element.offsetWidth, 'x', element.offsetHeight);
        console.log('Element visibility:', window.getComputedStyle(element).display);
        
        // Force complete plot recreation to ensure visibility
        Plotly.newPlot(element, [trace], layout, {
            responsive: true,
            displayModeBar: false
        }).then(() => {
            console.log(`Successfully created altitude plot with ${validData.length} valid points`);
            console.log('Plot element after creation:', element.offsetWidth, 'x', element.offsetHeight);
            // Force resize to ensure plot is visible
            Plotly.Plots.resize(element);
        }).catch(error => {
            console.error('Error creating altitude plot:', error);
        });
    }

    updateSpeedPlot(times, speedData) {
        const element = document.getElementById('speedPlot');
        if (!element) {
            console.log('speedPlot element not found');
            return;
        }

        // Filter out invalid data - handle both null and NaN values  
        const validData = times.map((time, i) => ({
            time: time,
            ground_speed: speedData[i].ground_speed,
            vertical_speed: speedData[i].vertical_speed,
            vector_speed: speedData[i].vector_speed
        })).filter(d => d.time != null && d.ground_speed != null && d.vertical_speed != null && d.vector_speed != null &&
                      !isNaN(d.time) && !isNaN(d.ground_speed) && !isNaN(d.vertical_speed) && !isNaN(d.vector_speed) && 
                      isFinite(d.time) && isFinite(d.ground_speed) && isFinite(d.vertical_speed) && isFinite(d.vector_speed));

        if (validData.length === 0) {
            console.log('No valid speed data to plot');
            return;
        }

        // Ground/Horizontal Speed trace
        const groundTrace = {
            x: validData.map(d => d.time),
            y: validData.map(d => d.ground_speed),
            type: 'scatter',
            mode: 'lines',
            name: 'Horizontal Speed',
            line: { color: 'orange', width: 2 }
        };

        // Vertical Speed trace
        const verticalTrace = {
            x: validData.map(d => d.time),
            y: validData.map(d => d.vertical_speed),
            type: 'scatter',
            mode: 'lines',
            name: 'Vertical Speed',
            line: { color: 'green', width: 2 }
        };

        // Vector Speed trace
        const vectorTrace = {
            x: validData.map(d => d.time),
            y: validData.map(d => d.vector_speed),
            type: 'scatter',
            mode: 'lines',
            name: 'Vector Speed',
            line: { color: 'purple', width: 2 }
        };

        const layout = {
            title: { text: 'Speed vs Time (Horizontal, Vertical, Vector)', font: { size: 16 } },
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
            legend: { x: 0, y: 1 },
            plot_bgcolor: 'white',
            paper_bgcolor: 'white',
            annotations: [] // Clear any "no data" annotations when showing real data
        };

        // Force complete plot recreation to ensure visibility
        Plotly.newPlot(element, [groundTrace, verticalTrace, vectorTrace], layout, {
            responsive: true,
            displayModeBar: false
        }).then(() => {
            console.log(`Successfully created speed plot with ${validData.length} valid points`);
            // Force resize to ensure plot is visible
            Plotly.Plots.resize(element);
        }).catch(error => {
            console.error('Error creating speed plot:', error);
        });
    }

    updateAttitudePlots(times, data) {
        // Update attitude plot (roll and pitch only)
        this.updateAttitudePlotOnly(times, data);
        
        // Update speed plot separately
        this.updateDashboardSpeedPlot(times, data);
    }

    updateAttitudePlotOnly(times, data) {
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
            paper_bgcolor: 'white',
            annotations: []
        };

        // Force complete plot recreation - Roll and Pitch only
        Plotly.newPlot(element, [rollTrace, pitchTrace], layout, {
            responsive: true,
            displayModeBar: false
        }).then(() => {
            console.log(`Successfully created attitude plot with ${validData.length} valid points`);
            // Force resize to ensure plot is visible
            Plotly.Plots.resize(element);
        }).catch(error => {
            console.error('Error creating attitude plot:', error);
        });
    }

    updateDashboardSpeedPlot(times, data) {
        const element = document.getElementById('dashboardSpeedPlot');
        if (!element) {
            console.log('dashboardSpeedPlot element not found');
            return;
        }

        // Filter out invalid data
        const validData = times.map((time, i) => ({
            time: time,
            vertical_speed: data[i].vertical_speed || 0,
            ground_speed: data[i].ground_speed || 0
        })).filter(d => d.time != null && !isNaN(d.time) && isFinite(d.time));

        if (validData.length === 0) {
            console.log('No valid speed data to plot');
            return;
        }

        const verticalSpeedTrace = {
            x: validData.map(d => d.time),
            y: validData.map(d => d.vertical_speed),
            type: 'scatter',
            mode: 'lines',
            name: 'Vertical Speed',
            line: { color: 'orange', width: 2 }
        };

        const horizontalSpeedTrace = {
            x: validData.map(d => d.time),
            y: validData.map(d => d.ground_speed),
            type: 'scatter',
            mode: 'lines',
            name: 'Horizontal Speed',
            line: { color: 'green', width: 2 }
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
            legend: { x: 0, y: 1 },
            plot_bgcolor: 'white',
            paper_bgcolor: 'white',
            annotations: []
        };

        // Force complete plot recreation with speed data
        Plotly.newPlot(element, [verticalSpeedTrace, horizontalSpeedTrace], layout, {
            responsive: true,
            displayModeBar: false
        }).then(() => {
            console.log(`Successfully created dashboard speed plot with ${validData.length} valid points`);
            // Force resize to ensure plot is visible
            Plotly.Plots.resize(element);
        }).catch(error => {
            console.error('Error creating dashboard speed plot:', error);
        });
    }

    // Analytics functionality
    initAnalytics() {
        console.log('Initializing Analytics functionality');
        
        // Bind Analytics tab activation
        const analyticsTab = document.querySelector('[href="#analytics"]');
        if (analyticsTab) {
            analyticsTab.addEventListener('click', () => {
                console.log('Analytics tab clicked, loading session history');
                setTimeout(() => this.loadSessionHistory(), 100);
            });
        }
        
        // Also bind to bootstrap tab events for more reliable detection
        const analyticsTabElement = document.getElementById('analytics-tab');
        if (analyticsTabElement) {
            analyticsTabElement.addEventListener('shown.bs.tab', () => {
                console.log('Analytics tab shown, loading session history');
                this.loadSessionHistory();
            });
        }
        
        console.log('Analytics tab element found:', !!analyticsTabElement);
        console.log('Analytics link found:', !!analyticsTab);
        
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
        
        // Bind apply filters button
        const applyFiltersBtn = document.getElementById('applyFiltersBtn');
        if (applyFiltersBtn) {
            applyFiltersBtn.addEventListener('click', () => {
                this.applySessionFilters();
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
                                    Session ${session.id}: ${session.drone_model} - ${session.environment}
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
                                <div class="btn-group mt-1" role="group">
                                    <button class="btn btn-outline-primary btn-sm view-details-btn" 
                                            data-session-id="${session.id}">
                                        <i class="fas fa-eye"></i> View Details
                                    </button>
                                    <button class="btn btn-outline-success btn-sm load-telemetry-btn" 
                                            data-session-id="${session.id}"
                                            title="Load telemetry data into Telemetry tab">
                                        <i class="fas fa-chart-line"></i> Load to Telemetry
                                    </button>
                                    <button class="btn btn-outline-info btn-sm trajectory-3d-btn" 
                                            data-session-id="${session.id}"
                                            title="Generate 3D trajectory visualization">
                                        <i class="fas fa-cube"></i> 3D Trajectory
                                    </button>
                                </div>
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
        
        // Add click handlers for load telemetry buttons
        sessionHistoryElement.querySelectorAll('.load-telemetry-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                e.stopPropagation();
                const sessionId = btn.getAttribute('data-session-id');
                this.loadSessionTelemetry(sessionId);
            });
        });
        
        // Add click handlers for 3D trajectory buttons
        sessionHistoryElement.querySelectorAll('.trajectory-3d-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                e.stopPropagation();
                const sessionId = btn.getAttribute('data-session-id');
                this.generateTrajectory3D(sessionId);
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
    
    async generateTrajectory3D(sessionId) {
        console.log('Loading 3D trajectory for session:', sessionId);
        
        try {
            // Show loading message
            this.showMessage(`Loading trajectory data for session ${sessionId}...`, 'info');
            
            // Fetch telemetry data from API
            const response = await fetch(`/api/history/sessions/${sessionId}/telemetry?limit=2000`);
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            
            const data = await response.json();
            console.log(`Loaded ${data.telemetry.length} trajectory points for session ${sessionId}`);
            
            if (!data.telemetry || data.telemetry.length === 0) {
                this.showMessage('No trajectory data found for this session', 'warning');
                return;
            }
            
            // Clear current trajectory data and load session data
            this.clearTrajectoryData();
            this.loadHistoricalTrajectory(data.telemetry, sessionId);
            
            // Switch to trajectory tab
            this.switchToTrajectoryTab();
            
            this.showMessage(`Loaded 3D trajectory with ${data.telemetry.length} points from session ${sessionId}`, 'success');
            
        } catch (error) {
            console.error('Error loading 3D trajectory:', error);
            this.showMessage(`Error loading trajectory: ${error.message}`, 'error');
        }
    }
    
    clearTrajectoryData() {
        console.log('Clearing current trajectory data');
        // Clear any existing trajectory plot
        const element = document.getElementById('trajectoryPlot');
        if (element) {
            Plotly.purge(element);
        }
    }
    
    loadHistoricalTrajectory(telemetryPoints, sessionId) {
        console.log(`Loading ${telemetryPoints.length} historical trajectory points`);
        
        // Convert database telemetry format to trajectory data
        const trajectoryData = telemetryPoints.map(point => {
            const position = point.position || {};
            return {
                x: parseFloat(position.x || 0),
                y: parseFloat(position.y || 0), 
                z: parseFloat(position.z || point.altitude || 0),
                timestamp: point.timestamp || 0
            };
        }).filter(p => !isNaN(p.x) && !isNaN(p.y) && !isNaN(p.z) && isFinite(p.x) && isFinite(p.y) && isFinite(p.z));
        
        if (trajectoryData.length === 0) {
            console.log('No valid trajectory points found');
            return;
        }
        
        console.log('Sample trajectory positions:', trajectoryData.slice(0, 3));
        
        // Update the existing trajectory plot using the main controller's method
        if (window.droneController && window.droneController.updateTrajectoryPlot) {
            window.droneController.updateTrajectoryPlot(trajectoryData);
        } else {
            // Fallback: create the plot directly
            this.createTrajectoryPlot(trajectoryData);
        }
    }
    
    switchToTrajectoryTab() {
        // Find and activate the trajectory tab
        const trajectoryTab = document.querySelector('[data-bs-target="#trajectory"]');
        if (trajectoryTab) {
            const tabTrigger = new bootstrap.Tab(trajectoryTab);
            tabTrigger.show();
            console.log('Switched to Trajectory tab for 3D visualization');
        }
    }
    
    createTrajectoryPlot(trajectoryData) {
        const element = document.getElementById('trajectoryPlot');
        if (!element) {
            console.log('trajectoryPlot element not found');
            return;
        }

        console.log('Creating 3D trajectory plot with', trajectoryData.length, 'points');

        // Create 3D line plot
        const trace = {
            x: trajectoryData.map(p => p.x),
            y: trajectoryData.map(p => p.y),
            z: trajectoryData.map(p => p.z),
            type: 'scatter3d',
            mode: 'lines+markers',
            marker: {
                size: 2,
                color: trajectoryData.map((p, i) => i),
                colorscale: 'Viridis',
                showscale: true,
                colorbar: {
                    title: 'Time Progression'
                }
            },
            line: {
                color: 'blue',
                width: 4
            },
            name: 'Flight Path'
        };

        // Add start and end markers
        const startTrace = {
            x: [trajectoryData[0].x],
            y: [trajectoryData[0].y],
            z: [trajectoryData[0].z],
            type: 'scatter3d',
            mode: 'markers',
            marker: {
                size: 10,
                color: 'green'
            },
            name: 'Start'
        };

        const endTrace = {
            x: [trajectoryData[trajectoryData.length - 1].x],
            y: [trajectoryData[trajectoryData.length - 1].y],
            z: [trajectoryData[trajectoryData.length - 1].z],
            type: 'scatter3d',
            mode: 'markers',
            marker: {
                size: 10,
                color: 'red'
            },
            name: 'End'
        };

        const layout = {
            title: '3D Flight Trajectory',
            scene: {
                xaxis: { title: 'X Position (m)', range: [-50, 200] },
                yaxis: { title: 'Y Position (m)', range: [-50, 250] },
                zaxis: { title: 'Z Altitude (m)', range: [0, 100] },
                aspectmode: 'cube'
            },
            margin: { l: 0, r: 0, t: 50, b: 0 }
        };

        Plotly.newPlot(element, [trace, startTrace, endTrace], layout, {
            responsive: true,
            displayModeBar: true
        }).then(() => {
            console.log(`Successfully created 3D trajectory plot with ${trajectoryData.length} points`);
            Plotly.Plots.resize(element);
        }).catch(error => {
            console.error('Error creating 3D trajectory plot:', error);
        });
    }
    
    async loadSessionTelemetry(sessionId) {
        console.log('Loading telemetry for session:', sessionId);
        
        try {
            // Show loading message
            this.showMessage(`Loading telemetry data for session ${sessionId}...`, 'info');
            
            // Fetch telemetry data from API
            const response = await fetch(`/api/history/sessions/${sessionId}/telemetry?limit=2000`);
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            
            const data = await response.json();
            console.log(`Loaded ${data.telemetry.length} telemetry points for session ${sessionId}`);
            
            if (!data.telemetry || data.telemetry.length === 0) {
                this.showMessage('No telemetry data found for this session', 'warning');
                return;
            }
            
            // Clear current telemetry data and load session data
            this.clearTelemetryData();
            this.loadHistoricalTelemetry(data.telemetry, sessionId);
            
            // Switch to telemetry tab
            this.switchToTelemetryTab();
            
            this.showMessage(`Loaded ${data.telemetry.length} telemetry points from session ${sessionId}`, 'success');
            
        } catch (error) {
            console.error('Error loading session telemetry:', error);
            this.showMessage(`Error loading telemetry: ${error.message}`, 'error');
        }
    }
    
    clearTelemetryData() {
        console.log('Clearing current telemetry data');
        this.telemetryData = [];
        this.missionData = [];
        
        // Clear any localStorage data
        localStorage.removeItem('telemetryData');
        localStorage.removeItem('missionData');
    }
    
    loadHistoricalTelemetry(telemetryPoints, sessionId) {
        console.log(`Loading ${telemetryPoints.length} historical telemetry points`);
        
        // Convert database telemetry format to internal format
        this.telemetryData = telemetryPoints.map(point => {
            // Database stores nested objects for position, velocity, attitude
            const position = point.position || {};
            const velocity = point.velocity || {};
            const attitude = point.attitude || {};
            
            return {
                timestamp: point.timestamp || 0,
                position_x: parseFloat(position.x || 0),
                position_y: parseFloat(position.y || 0),
                position_z: parseFloat(position.z || 0),
                velocity_x: parseFloat(velocity.x || 0),
                velocity_y: parseFloat(velocity.y || 0),
                velocity_z: parseFloat(velocity.z || 0),
                ground_speed: parseFloat(point.ground_speed || 0),
                vertical_speed: parseFloat(point.vertical_speed || velocity.z || 0),
                vector_speed: parseFloat(point.vector_speed || 0),
                altitude: parseFloat(point.altitude || position.z || 0),
                roll: parseFloat(attitude.roll || 0),
                pitch: parseFloat(attitude.pitch || 0),
                yaw: parseFloat(attitude.yaw || 0),
                mission_progress: parseFloat(point.mission_progress || 0),
                current_waypoint: parseInt(point.current_waypoint || 0)
            };
        });
        
        console.log('Loading', telemetryPoints.length, 'historical telemetry points');
        
        // Update trajectory plot first
        this.updateTrajectoryPlot();
        console.log('Trajectory plot updated successfully');
        
        // Clear and recreate real-time plots sequentially to avoid Canvas2D conflicts
        console.log('Updating real-time plots with', Math.min(1000, telemetryPoints.length), 'points');
        this.recreateRealtimePlots();
        
        // Update status display to show it's historical data
        this.updateHistoricalDisplay(sessionId);
    }
    
    switchToTelemetryTab() {
        // Activate telemetry tab
        const telemetryTab = document.getElementById('telemetry-tab');
        if (telemetryTab) {
            const tabTrigger = new bootstrap.Tab(telemetryTab);
            tabTrigger.show();
        }
    }
    
    recreateRealtimePlots() {
        // Clear plots sequentially to avoid Canvas2D conflicts
        const plots = ['altitudeSpeedPlot', 'telemetryAttitudePlot', 'powerPlot', 'missionProgressPlot'];
        
        // Clear all plots first
        plots.forEach(plotId => {
            const element = document.getElementById(plotId);
            if (element) {
                Plotly.purge(element);
                // Ensure element has proper dimensions
                element.style.width = '100%';
                element.style.height = '300px';
                element.style.display = 'block';
            }
        });
        
        // Recreate plots after clearing is complete
        setTimeout(() => {
            this.updateTelemetryTabPlots();
        }, 300);
    }
    
    updateTelemetryTabPlots() {
        if (this.telemetryData.length === 0) return;
        
        // Get last 1000 data points for complete mission display
        const recentData = this.telemetryData.slice(-1000);
        const times = recentData.map(d => d.timestamp);
        const data = recentData;
        
        console.log('Sample times:', times.slice(-3));
        console.log('Sample altitudes:', data.slice(-3).map(d => d.altitude));
        console.log('Sample ground speeds:', data.slice(-3).map(d => d.ground_speed));
        console.log('Sample vertical speeds:', data.slice(-3).map(d => d.vertical_speed));
        console.log('Sample vector speeds:', data.slice(-3).map(d => d.vector_speed));
        
        // Update telemetry tab specific plots
        this.updateAltitudeSpeedCombined(times, data);
        this.updateTelemetryAttitude(times, data);
        this.updatePowerEnergy(times, data);
        this.updateMissionProgress(times, data);
    }
    
    updateAltitudeSpeedCombined(times, data) {
        const element = document.getElementById('altitudeSpeedPlot');
        if (!element) {
            console.log('altitudeSpeedPlot element not found');
            return;
        }

        const validData = times.map((time, i) => ({
            time: time,
            altitude: data[i].altitude,
            ground_speed: data[i].ground_speed || 0,
            vertical_speed: data[i].vertical_speed || 0,
            vector_speed: data[i].vector_speed || Math.sqrt((data[i].ground_speed || 0)**2 + (data[i].vertical_speed || 0)**2)
        })).filter(d => d.time != null && d.altitude != null && 
                      !isNaN(d.time) && !isNaN(d.altitude) &&
                      isFinite(d.time) && isFinite(d.altitude));

        if (validData.length === 0) return;

        const altTrace = {
            x: validData.map(d => d.time),
            y: validData.map(d => d.altitude),
            type: 'scatter',
            mode: 'lines',
            name: 'Altitude (m)',
            line: { color: 'blue', width: 2 },
            yaxis: 'y1'
        };

        const verticalSpeedTrace = {
            x: validData.map(d => d.time),
            y: validData.map(d => d.vertical_speed),
            type: 'scatter',
            mode: 'lines',
            name: 'Vertical Speed (m/s)',
            line: { color: 'green', width: 2 },
            yaxis: 'y2'
        };

        const totalSpeedTrace = {
            x: validData.map(d => d.time),
            y: validData.map(d => d.vector_speed),
            type: 'scatter',
            mode: 'lines',
            name: 'Total Speed (m/s)',
            line: { color: 'red', width: 2 },
            yaxis: 'y2'
        };

        const layout = {
            title: 'Altitude & Speed vs Time',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Altitude (m)', titlefont: { color: 'blue' }, side: 'left' },
            yaxis2: { title: 'Speed (m/s)', titlefont: { color: 'darkred' }, overlaying: 'y', side: 'right' },
            margin: { l: 60, r: 100, t: 50, b: 50 },
            legend: { 
                x: 1.02, 
                y: 1, 
                xanchor: 'left',
                bgcolor: 'rgba(255,255,255,0.8)',
                bordercolor: '#ccc',
                borderwidth: 1
            }
        };

        console.log('Creating altitude/speed plot, element dimensions:', element.offsetWidth, 'x', element.offsetHeight);
        
        Plotly.newPlot(element, [altTrace, verticalSpeedTrace, totalSpeedTrace], layout, {
            responsive: true,
            displayModeBar: false
        }).then(() => {
            console.log(`Successfully created altitude/speed plot with ${validData.length} points`);
            Plotly.Plots.resize(element);
        }).catch(error => {
            console.error('Error creating altitude/speed plot:', error);
        });
    }
    
    updateTelemetryAttitude(times, data) {
        const element = document.getElementById('telemetryAttitudePlot');
        if (!element) {
            console.log('telemetryAttitudePlot element not found');
            return;
        }

        const validData = times.map((time, i) => ({
            time: time,
            roll: data[i].roll * 180/Math.PI,
            pitch: data[i].pitch * 180/Math.PI,
            yaw: data[i].yaw * 180/Math.PI
        })).filter(d => d.time != null && !isNaN(d.time) && isFinite(d.time));

        if (validData.length === 0) return;

        const traces = [
            { x: validData.map(d => d.time), y: validData.map(d => d.roll), name: 'Roll', line: { color: 'red', width: 2 }, type: 'scatter', mode: 'lines' },
            { x: validData.map(d => d.time), y: validData.map(d => d.pitch), name: 'Pitch', line: { color: 'blue', width: 2 }, type: 'scatter', mode: 'lines' },
            { x: validData.map(d => d.time), y: validData.map(d => d.yaw), name: 'Yaw', line: { color: 'green', width: 2 }, type: 'scatter', mode: 'lines' }
        ];

        const layout = {
            title: 'Attitude vs Time',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Angle (degrees)' },
            margin: { l: 60, r: 30, t: 50, b: 50 }
        };

        console.log('Creating attitude plot, element dimensions:', element.offsetWidth, 'x', element.offsetHeight);
        
        Plotly.newPlot(element, traces, layout, {
            responsive: true,
            displayModeBar: false
        }).then(() => {
            console.log(`Successfully created attitude plot with ${validData.length} points`);
            Plotly.Plots.resize(element);
        }).catch(error => {
            console.error('Error creating attitude plot:', error);
        });
    }
    
    updatePowerEnergy(times, data) {
        const element = document.getElementById('powerPlot');
        if (!element) return;

        const validData = times.map((time, i) => {
            const speed = data[i].ground_speed || 0;
            const altitude = data[i].altitude || 0;
            const power = 50 + (speed * 2) + (altitude * 0.1);
            return { time: time, power: power, energy: power * time / 3600 };
        }).filter(d => !isNaN(d.time) && isFinite(d.time));

        if (validData.length === 0) return;

        const traces = [
            { x: validData.map(d => d.time), y: validData.map(d => d.power), name: 'Power (W)', line: { color: 'orange' }, yaxis: 'y1' },
            { x: validData.map(d => d.time), y: validData.map(d => d.energy), name: 'Energy (Wh)', line: { color: 'green' }, yaxis: 'y2' }
        ];

        const layout = {
            title: 'Power & Energy vs Time',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Power (W)', titlefont: { color: 'orange' }, side: 'left' },
            yaxis2: { title: 'Energy (Wh)', titlefont: { color: 'green' }, overlaying: 'y', side: 'right' },
            margin: { l: 60, r: 60, t: 50, b: 50 }
        };

        Plotly.newPlot(element, traces, layout, { responsive: true, displayModeBar: false })
            .then(() => console.log(`Successfully created power plot with ${validData.length} points`))
            .catch(error => console.error('Error creating power plot:', error));
    }
    
    updateCompassYaw(times, data) {
        const element = document.getElementById('compassPlot');
        if (!element) return;

        const validData = times.map((time, i) => ({
            time: time,
            yaw: data[i].yaw * 180/Math.PI
        })).filter(d => d.time != null && !isNaN(d.time) && isFinite(d.time));

        if (validData.length === 0) return;

        // Get the latest yaw value for compass display
        const currentYaw = validData[validData.length - 1].yaw;
        const normalizedYaw = ((currentYaw % 360) + 360) % 360; // Normalize to 0-360

        // Create compass-style plot
        const compassTrace = {
            r: [1], // Distance from center
            theta: [normalizedYaw], // Angle
            type: 'scatterpolar',
            mode: 'markers+text',
            marker: {
                size: 20,
                color: 'red',
                symbol: 'arrow-up'
            },
            text: [`${normalizedYaw.toFixed(1)}°`],
            textposition: 'middle center',
            name: 'Current Heading'
        };

        // Add cardinal directions
        const cardinalTrace = {
            r: [0.8, 0.8, 0.8, 0.8],
            theta: [0, 90, 180, 270],
            type: 'scatterpolar',
            mode: 'text',
            text: ['N', 'E', 'S', 'W'],
            textfont: { size: 16, color: 'black' },
            showlegend: false
        };

        const layout = {
            title: 'Compass - Current Heading',
            polar: {
                radialaxis: {
                    visible: false,
                    range: [0, 1]
                },
                angularaxis: {
                    direction: 'clockwise',
                    rotation: 90,
                    tickmode: 'array',
                    tickvals: [0, 45, 90, 135, 180, 225, 270, 315],
                    ticktext: ['0°', '45°', '90°', '135°', '180°', '225°', '270°', '315°']
                }
            },
            margin: { l: 40, r: 40, t: 50, b: 40 },
            showlegend: false
        };

        Plotly.newPlot(element, [compassTrace, cardinalTrace], layout, { responsive: true, displayModeBar: false })
            .then(() => console.log(`Successfully created compass plot showing ${normalizedYaw.toFixed(1)}° heading`))
            .catch(error => console.error('Error creating compass plot:', error));
    }
    
    updateMissionProgress(times, data) {
        const element = document.getElementById('missionProgressPlot');
        if (!element) return;

        const validData = times.map((time, i) => ({
            time: time,
            progress: data[i].mission_progress || 0,
            waypoint: data[i].current_waypoint || 0
        })).filter(d => !isNaN(d.time) && isFinite(d.time));

        if (validData.length === 0) return;

        const traces = [
            { 
                x: validData.map(d => d.time), 
                y: validData.map(d => d.progress), 
                name: 'Progress (%)', 
                line: { color: 'green', width: 2 }, 
                type: 'scatter',
                mode: 'lines',
                yaxis: 'y1' 
            },
            { 
                x: validData.map(d => d.time), 
                y: validData.map(d => d.waypoint), 
                name: 'Waypoint', 
                line: { color: 'blue', width: 2 }, 
                type: 'scatter',
                mode: 'lines',
                yaxis: 'y2' 
            }
        ];

        const layout = {
            title: 'Mission Progress vs Time',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Progress (%)', titlefont: { color: 'green' }, side: 'left', range: [0, 100] },
            yaxis2: { title: 'Waypoint', titlefont: { color: 'blue' }, overlaying: 'y', side: 'right' },
            margin: { l: 60, r: 60, t: 50, b: 50 }
        };

        Plotly.newPlot(element, traces, layout, { responsive: true, displayModeBar: false })
            .then(() => console.log(`Successfully created mission progress plot with ${validData.length} points`))
            .catch(error => console.error('Error creating mission progress plot:', error));
    }
    
    updateHistoricalDisplay(sessionId) {
        // Update the status to show this is historical data
        const statusElement = document.getElementById('simulationStatus');
        if (statusElement) {
            statusElement.innerHTML = `
                <span class="badge bg-info">Historical Data</span>
                Session #${sessionId}
            `;
        }
        
        // Update mission status
        const missionStatusElement = document.getElementById('missionStatus');
        if (missionStatusElement) {
            missionStatusElement.textContent = `Historical Session ${sessionId}`;
        }
    }
    
    async applySessionFilters() {
        console.log('Applying session filters');
        
        try {
            // Get filter values
            const statusFilter = document.getElementById('statusFilter')?.value || '';
            const environmentFilter = document.getElementById('environmentFilter')?.value || '';
            const missionTypeFilter = document.getElementById('missionTypeFilter')?.value || '';
            const droneModelFilter = document.getElementById('droneModelFilter')?.value || '';
            const sessionLimit = document.getElementById('sessionLimitFilter')?.value || '50';
            
            console.log('Filters:', { statusFilter, environmentFilter, missionTypeFilter, droneModelFilter, sessionLimit });
            
            // Build query parameters
            const params = new URLSearchParams();
            if (sessionLimit && sessionLimit !== '0') params.append('limit', sessionLimit);
            if (statusFilter) params.append('status', statusFilter);
            if (environmentFilter) params.append('environment', environmentFilter);
            if (missionTypeFilter) params.append('mission_type', missionTypeFilter);
            if (droneModelFilter) params.append('drone_model', droneModelFilter);
            
            // Fetch filtered sessions
            const response = await fetch(`/api/analytics/sessions?${params.toString()}`);
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            
            const data = await response.json();
            console.log(`Filtered sessions: ${data.sessions.length} sessions found`);
            
            // Apply client-side filtering for more precise results
            let filteredSessions = data.sessions;
            
            if (statusFilter) {
                filteredSessions = filteredSessions.filter(session => {
                    if (statusFilter === 'completed') {
                        return session.status === 'completed' || session.status === 'completed_timeout';
                    }
                    return session.status === statusFilter;
                });
            }
            
            if (environmentFilter) {
                filteredSessions = filteredSessions.filter(session => session.environment === environmentFilter);
            }
            
            if (missionTypeFilter) {
                filteredSessions = filteredSessions.filter(session => session.mission_type === missionTypeFilter);
            }
            
            if (droneModelFilter) {
                filteredSessions = filteredSessions.filter(session => session.drone_model === droneModelFilter);
            }
            
            // Update display
            this.displaySessionHistory(filteredSessions);
            this.loadPerformanceSummary(filteredSessions);
            
            // Update counts
            const sessionCountElement = document.getElementById('sessionCount');
            if (sessionCountElement) {
                sessionCountElement.textContent = `${data.sessions.length} total sessions`;
            }
            
            const filteredCountElement = document.getElementById('filteredCount');
            if (filteredCountElement) {
                filteredCountElement.textContent = `${filteredSessions.length} sessions shown`;
            }
            
            this.showMessage(`Applied filters: ${filteredSessions.length} sessions found`, 'success');
            
        } catch (error) {
            console.error('Error applying filters:', error);
            this.showMessage(`Error applying filters: ${error.message}`, 'error');
        }
    }

    async updateAnalyticsCharts() {
        console.log('Updating analytics charts');
        
        // Get selected session filter
        const sessionFilter = document.getElementById('analyticsFilter');
        const timeFilter = document.getElementById('analyticsTimeframe');
        
        if (!sessionFilter || !timeFilter) {
            console.error('Analytics filter elements not found');
            return;
        }
        
        const selectedFilter = sessionFilter.value;
        const selectedTimeframe = timeFilter.value;
        
        try {
            // Get session data based on filters
            let url = '/api/analytics/sessions?';
            if (selectedTimeframe !== 'All Sessions') {
                const limit = selectedTimeframe.replace(' Sessions', '');
                url += `limit=${limit}`;
            }
            
            const response = await fetch(url);
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            
            const data = await response.json();
            let sessions = data.sessions || [];
            
            // Filter sessions based on selection
            if (selectedFilter !== 'All Sessions') {
                // If a specific session is selected, filter to just that one
                const sessionId = parseInt(selectedFilter);
                if (!isNaN(sessionId)) {
                    sessions = sessions.filter(s => s.id === sessionId);
                }
            }
            
            console.log(`Creating charts for ${sessions.length} sessions`);
            
            // Create performance charts
            this.createPerformanceCharts(sessions);
            
            this.showMessage(`Analytics updated for ${sessions.length} session(s)`, 'success');
            
        } catch (error) {
            console.error('Error updating analytics charts:', error);
            this.showMessage('Error updating analytics charts', 'error');
        }
    }
    
    createPerformanceCharts(sessions) {
        if (!sessions || sessions.length === 0) {
            console.log('No sessions to chart');
            return;
        }
        
        // Create altitude vs time chart
        this.createAltitudeChart(sessions);
        
        // Create mission progress chart  
        this.createMissionProgressChart(sessions);
        
        // Create session details table instead of third chart for now
        this.createSessionDetailsTable(sessions);
    }
    
    createAltitudeChart(sessions) {
        const chartContainer = document.getElementById('performanceChart');
        if (!chartContainer) {
            console.error('Performance chart container not found');
            return;
        }
        
        const traces = sessions.map(session => ({
            x: [0, session.duration || 100],
            y: [0, session.max_altitude || 0],
            type: 'scatter',
            mode: 'lines+markers',
            name: `Session ${session.id} (${session.environment})`,
            line: { width: 2 }
        }));
        
        const layout = {
            title: 'Altitude Performance',
            xaxis: { title: 'Time (seconds)' },
            yaxis: { title: 'Altitude (meters)' },
            margin: { t: 40, r: 20, b: 40, l: 60 }
        };
        
        Plotly.newPlot(chartContainer, traces, layout, {responsive: true});
    }
    
    createMissionProgressChart(sessions) {
        const chartContainer = document.getElementById('trendsChart');
        if (!chartContainer) {
            console.error('Trends chart container not found');
            return;
        }
        
        const traces = [{
            x: sessions.map(s => `Session ${s.id}`),
            y: sessions.map(s => s.mission_progress || 0),
            type: 'bar',
            name: 'Mission Progress',
            marker: { color: sessions.map(s => s.mission_progress >= 100 ? '#28a745' : '#007bff') }
        }];
        
        const layout = {
            title: 'Mission Progress Comparison',
            xaxis: { title: 'Sessions' },
            yaxis: { title: 'Progress (%)' },
            margin: { t: 40, r: 20, b: 80, l: 60 }
        };
        
        Plotly.newPlot(chartContainer, traces, layout, {responsive: true});
    }
    
    createSessionDetailsTable(sessions) {
        const tableContainer = document.getElementById('sessionDetailsTable');
        if (!tableContainer) {
            console.error('Session details table container not found');
            return;
        }
        
        if (!sessions || sessions.length === 0) {
            tableContainer.innerHTML = '<div class="alert alert-info">No sessions selected</div>';
            return;
        }
        
        let tableHtml = `
            <div class="table-responsive">
                <table class="table table-striped table-sm">
                    <thead>
                        <tr>
                            <th>Session ID</th>
                            <th>Drone</th>
                            <th>Environment</th>
                            <th>Mission</th>
                            <th>Progress</th>
                            <th>Duration</th>
                            <th>Max Speed</th>
                            <th>Max Altitude</th>
                            <th>Status</th>
                        </tr>
                    </thead>
                    <tbody>
        `;
        
        sessions.forEach(session => {
            const statusBadge = session.status === 'completed' ? 'badge bg-success' : 
                               session.status === 'running' ? 'badge bg-primary' : 'badge bg-warning';
            
            tableHtml += `
                <tr>
                    <td>#${session.id}</td>
                    <td>${session.drone_model}</td>
                    <td>${session.environment}</td>
                    <td>${session.mission_type}</td>
                    <td>${(session.mission_progress || 0).toFixed(1)}%</td>
                    <td>${session.duration ? session.duration.toFixed(1) + 's' : 'N/A'}</td>
                    <td>${(session.max_speed || 0).toFixed(2)} m/s</td>
                    <td>${(session.max_altitude || 0).toFixed(1)} m</td>
                    <td><span class="${statusBadge}">${session.status}</span></td>
                </tr>
            `;
        });
        
        tableHtml += `
                    </tbody>
                </table>
            </div>
        `;
        
        tableContainer.innerHTML = tableHtml;
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