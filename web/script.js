/**
 * Drone Simulation Web Interface
 * Client-side JavaScript for real-time simulation control and monitoring
 */

class DroneSimulationController {
    constructor() {
        this.websocket = null;
        this.simulationRunning = false;
        this.simulationPaused = false;
        this.telemetryData = [];
        this.missionData = [];
        this.currentStatus = {};
        
        // Chart instances
        this.charts = {
            realtime: null,
            trajectory: null,
            altitude: null,
            attitude: null,
            power: null
        };
        
        // Initialize the interface
        this.init();
    }
    
    async init() {
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
        
        console.log('Drone Simulation Controller initialized');
    }
    
    async loadConfigurations() {
        try {
            // Load drone models
            const dronesResponse = await fetch('/api/configurations/drones');
            const drones = await dronesResponse.json();
            this.populateSelect('droneModel', drones);
            
            // Load environments
            const envsResponse = await fetch('/api/configurations/environments');
            const environments = await envsResponse.json();
            this.populateSelect('environment', environments);
            
            // Load missions
            const missionsResponse = await fetch('/api/configurations/missions');
            const missions = await missionsResponse.json();
            this.populateSelect('missionType', missions);
            
            // Store missions globally for waypoint list
            window.missionConfigurations = missions;
            
        } catch (error) {
            console.error('Error loading configurations:', error);
            this.showError('Failed to load configurations');
        }
    }
    
    populateSelect(selectId, options) {
        const select = document.getElementById(selectId);
        select.innerHTML = '<option value="">Select...</option>';
        
        for (const [key, value] of Object.entries(options)) {
            const option = document.createElement('option');
            option.value = key;
            option.textContent = `${key} - ${value.description || 'No description'}`;
            select.appendChild(option);
        }
    }
    
    setupEventListeners() {
        // Start simulation button
        document.getElementById('startBtn').addEventListener('click', () => {
            this.startSimulation();
        });
        
        // Pause simulation button
        document.getElementById('pauseBtn').addEventListener('click', () => {
            this.pauseSimulation();
        });
        
        // Stop simulation button
        document.getElementById('stopBtn').addEventListener('click', () => {
            this.stopSimulation();
        });
        
        // Tab change events
        document.querySelectorAll('[data-bs-toggle="tab"]').forEach(tab => {
            tab.addEventListener('shown.bs.tab', (event) => {
                this.onTabChange(event.target.id);
            });
        });
    }
    
    async startSimulation() {
        const config = this.getSimulationConfig();
        
        if (!this.validateConfig(config)) {
            this.showError('Please fill in all required configuration fields');
            return;
        }
        
        try {
            this.setLoadingState(true);
            
            const response = await fetch('/api/simulation/start', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(config)
            });
            
            if (response.ok) {
                const result = await response.json();
                this.simulationRunning = true;
                this.updateControlButtons();
                this.showSuccess('Simulation started successfully');
                this.logMessage('info', 'Simulation started');
            } else {
                const error = await response.json();
                this.showError(error.message || 'Failed to start simulation');
            }
        } catch (error) {
            console.error('Error starting simulation:', error);
            this.showError('Network error while starting simulation');
        } finally {
            this.setLoadingState(false);
        }
    }
    
    async pauseSimulation() {
        try {
            const response = await fetch('/api/simulation/pause', {
                method: 'POST'
            });
            
            if (response.ok) {
                this.simulationPaused = !this.simulationPaused;
                this.updateControlButtons();
                this.showInfo(this.simulationPaused ? 'Simulation paused' : 'Simulation resumed');
            }
        } catch (error) {
            console.error('Error pausing simulation:', error);
            this.showError('Failed to pause simulation');
        }
    }
    
    async stopSimulation() {
        try {
            const response = await fetch('/api/simulation/stop', {
                method: 'POST'
            });
            
            if (response.ok) {
                this.simulationRunning = false;
                this.simulationPaused = false;
                this.updateControlButtons();
                this.showInfo('Simulation stopped');
                this.logMessage('info', 'Simulation stopped');
            }
        } catch (error) {
            console.error('Error stopping simulation:', error);
            this.showError('Failed to stop simulation');
        }
    }
    
    getSimulationConfig() {
        return {
            drone_model: document.getElementById('droneModel').value,
            environment: document.getElementById('environment').value,
            mission_type: document.getElementById('missionType').value,
            duration: parseFloat(document.getElementById('duration').value),
            timestep: parseFloat(document.getElementById('timestep').value),
            realtime: document.getElementById('realtime').checked
        };
    }
    
    validateConfig(config) {
        return config.drone_model && config.environment && config.mission_type &&
               config.duration > 0 && config.timestep > 0;
    }
    
    updateControlButtons() {
        const startBtn = document.getElementById('startBtn');
        const pauseBtn = document.getElementById('pauseBtn');
        const stopBtn = document.getElementById('stopBtn');
        
        if (this.simulationRunning) {
            startBtn.disabled = true;
            pauseBtn.disabled = false;
            stopBtn.disabled = false;
            pauseBtn.innerHTML = this.simulationPaused ? 
                '<i class="fas fa-play"></i> Resume' : 
                '<i class="fas fa-pause"></i> Pause';
        } else {
            startBtn.disabled = false;
            pauseBtn.disabled = true;
            stopBtn.disabled = true;
            pauseBtn.innerHTML = '<i class="fas fa-pause"></i> Pause';
        }
    }
    
    connectWebSocket() {
        // Use Socket.IO client instead of raw WebSocket
        this.websocket = io();
        
        this.websocket.on('connect', () => {
            console.log('SocketIO connected');
            this.updateConnectionStatus(true);
        });
        
        this.websocket.on('disconnect', () => {
            console.log('SocketIO disconnected');
            this.updateConnectionStatus(false);
        });
        
        this.websocket.on('telemetry_update', (data) => {
            this.handleWebSocketMessage(data);
        });
        
        this.websocket.on('mission_event', (data) => {
            this.handleWebSocketMessage(data);
        });
        
        this.websocket.on('status_update', (data) => {
            this.handleWebSocketMessage(data);
        });
        
        this.websocket.on('mission_complete', (data) => {
            this.handleMissionComplete(data);
        });
        
        this.websocket.on('log', (data) => {
            this.handleWebSocketMessage(data);
        });
        
        this.websocket.on('connect_error', (error) => {
            console.error('SocketIO connection error:', error);
            this.updateConnectionStatus(false);
        });
    }
    
    handleWebSocketMessage(data) {
        switch (data.type) {
            case 'telemetry':
                this.handleTelemetryUpdate(data.payload);
                break;
            case 'mission_event':
                this.handleMissionEvent(data.payload);
                break;
            case 'status':
                this.handleStatusUpdate(data.payload);
                break;
            case 'log':
                this.handleLogMessage(data.payload);
                break;
            default:
                console.log('Unknown message type:', data.type);
        }
    }
    
    handleTelemetryUpdate(telemetry) {
        this.telemetryData.push(telemetry);
        
        // Keep only last 1000 points for performance
        if (this.telemetryData.length > 1000) {
            this.telemetryData.shift();
        }
        
        // Update real-time displays
        this.updateDroneStatus(telemetry);
        this.updateEnvironmentStatus(telemetry);
        this.updateRealtimePlots();
        
        // Update progress
        this.updateSimulationProgress(telemetry.simulation_time);
    }
    
    handleMissionEvent(event) {
        this.missionData.push(event);
        this.updateMissionDisplay();
        this.logMessage('info', `Mission event: ${event.description}`);
    }
    
    handleStatusUpdate(status) {
        this.currentStatus = status;
        this.updateStatusDisplay();
    }
    
    handleLogMessage(logData) {
        this.logMessage(logData.level, logData.message);
    }
    
    handleMissionComplete(data) {
        this.simulationRunning = false;
        this.simulationPaused = false;
        this.updateControlButtons();
        this.showInfo(`Mission completed! Total time: ${data.total_time.toFixed(1)}s`);
        this.logMessage('success', `Mission completed in ${data.total_time.toFixed(1)} seconds`);
    }
    
    updateDroneStatus(telemetry) {
        try {
            // Position
            const position = telemetry.position || {};
            document.getElementById('dronePosition').textContent = 
                `${(position.x || 0).toFixed(1)}, ${(position.y || 0).toFixed(1)}, ${(position.z || 0).toFixed(1)}`;
            
            // Velocity
            const velocity = telemetry.velocity || {};
            const speed = Math.sqrt((velocity.x || 0)**2 + (velocity.y || 0)**2 + (velocity.z || 0)**2);
            document.getElementById('droneVelocity').textContent = 
                `${speed.toFixed(1)} m/s`;
            
            // Attitude (convert to degrees)
            const attitude = telemetry.attitude || {};
            document.getElementById('droneAttitude').textContent = 
                `${((attitude.roll || 0) * 180 / Math.PI).toFixed(1)}°, ${((attitude.pitch || 0) * 180 / Math.PI).toFixed(1)}°, ${((attitude.yaw || 0) * 180 / Math.PI).toFixed(1)}°`;
            
            // Battery (placeholder - will be 100% for now)
            document.getElementById('droneBattery').textContent = `100%`;
            
            // Additional telemetry displays (if elements exist)
            const altElement = document.getElementById('droneAltitude');
            if (altElement) altElement.textContent = `${(telemetry.altitude || 0).toFixed(1)} m`;
            
            const speedElement = document.getElementById('droneSpeed');
            if (speedElement) speedElement.textContent = `${(telemetry.ground_speed || 0).toFixed(1)} m/s`;
            
            const vSpeedElement = document.getElementById('droneVerticalSpeed');
            if (vSpeedElement) vSpeedElement.textContent = `${(telemetry.vertical_speed || 0).toFixed(1)} m/s`;
        } catch (error) {
            console.error('Error updating drone status:', error);
        }
    }
    
    updateEnvironmentStatus(telemetry) {
        document.getElementById('envGravity').textContent = 
            `${this.currentStatus.environment?.gravity || 9.81} m/s²`;
        
        document.getElementById('envDensity').textContent = 
            `${telemetry.air_density || 1.225} kg/m³`;
        
        document.getElementById('envTemperature').textContent = 
            `${telemetry.temperature || 15}°C`;
        
        document.getElementById('envWind').textContent = 
            `${telemetry.wind_speed || 0} m/s`;
    }
    
    updateStatusDisplay() {
        const statusElement = document.getElementById('statusDisplay');
        const status = this.currentStatus.simulation_status || 'ready';
        
        let badgeClass = 'bg-secondary';
        let statusText = 'Ready';
        
        switch (status) {
            case 'running':
                badgeClass = 'bg-success';
                statusText = 'Running';
                break;
            case 'paused':
                badgeClass = 'bg-warning';
                statusText = 'Paused';
                break;
            case 'completed':
                badgeClass = 'bg-info';
                statusText = 'Completed';
                break;
            case 'failed':
                badgeClass = 'bg-danger';
                statusText = 'Failed';
                break;
        }
        
        statusElement.innerHTML = `<span class="badge ${badgeClass}">${statusText}</span>`;
    }
    
    updateSimulationProgress(currentTime) {
        const duration = parseFloat(document.getElementById('duration').value) || 600;
        const progress = Math.min((currentTime / duration) * 100, 100);
        
        document.getElementById('progressBar').style.width = `${progress}%`;
        document.getElementById('currentTime').textContent = currentTime.toFixed(2);
    }
    
    async updateMissionDisplay() {
        try {
            // Get mission data from server
            const response = await fetch('/api/simulation/mission');
            if (response.ok) {
                const missionData = await response.json();
                
                if (Object.keys(missionData).length > 0) {
                    document.getElementById('missionStatus').textContent = 
                        this.simulationRunning ? 'In Progress' : 'Ready';
                    document.getElementById('missionWaypoints').textContent = 
                        `${missionData.current_waypoint || 0}/${missionData.total_waypoints || 0}`;
                    document.getElementById('missionDistance').textContent = 
                        `${this.calculateMissionDistance().toFixed(1)} m`;
                    
                    const progress = missionData.progress || 0;
                    const progressBar = document.getElementById('missionProgressBar');
                    if (progressBar) {
                        progressBar.style.width = `${progress}%`;
                        progressBar.textContent = `${progress.toFixed(1)}%`;
                    }
                    
                    this.updateWaypointList(missionData);
                }
            }
        } catch (error) {
            console.error('Error updating mission display:', error);
        }
    }
    
    calculateMissionDistance() {
        if (this.telemetryData.length < 2) return 0;
        
        let totalDistance = 0;
        for (let i = 1; i < this.telemetryData.length; i++) {
            const prev = this.telemetryData[i-1].position || {};
            const curr = this.telemetryData[i].position || {};
            
            const dx = (curr.x || 0) - (prev.x || 0);
            const dy = (curr.y || 0) - (prev.y || 0);
            const dz = (curr.z || 0) - (prev.z || 0);
            
            totalDistance += Math.sqrt(dx*dx + dy*dy + dz*dz);
        }
        
        return totalDistance;
    }
    
    updateWaypointList(missionData) {
        const waypointList = document.getElementById('waypointList');
        
        if (!waypointList) return;
        
        // Get waypoints from loaded mission configuration
        const missionConfigs = window.missionConfigurations || {};
        const currentMissionType = document.getElementById('missionSelect')?.value;
        const currentMission = missionConfigs[currentMissionType];
        
        if (!currentMission || !currentMission.waypoints) {
            waypointList.innerHTML = '<p class="text-muted">No waypoints available</p>';
            return;
        }
        
        const currentWaypoint = missionData?.current_waypoint || 0;
        
        let html = '';
        currentMission.waypoints.forEach((waypoint, index) => {
            const isActive = index === currentWaypoint;
            const isCompleted = index < currentWaypoint;
            
            let itemClass = 'list-group-item d-flex align-items-center';
            if (isActive) itemClass += ' list-group-item-primary';
            if (isCompleted) itemClass += ' list-group-item-success';
            
            const icon = this.getWaypointIcon(waypoint.action);
            
            html += `
                <div class="${itemClass}">
                    <span class="me-3">${icon}</span>
                    <div class="flex-grow-1">
                        <strong>${waypoint.action.charAt(0).toUpperCase() + waypoint.action.slice(1)}</strong><br>
                        <small class="text-muted">
                            Position: ${waypoint.x}, ${waypoint.y}, ${waypoint.z} | 
                            Duration: ${waypoint.duration}s
                        </small>
                    </div>
                    <div class="ms-2">
                        ${isCompleted ? '✓' : isActive ? '⏵' : '○'}
                    </div>
                </div>
            `;
        });
        
        waypointList.innerHTML = html;
    }
    
    getWaypointIcon(action) {
        const icons = {
            'takeoff': '<i class="fas fa-plane-departure"></i>',
            'land': '<i class="fas fa-plane-arrival"></i>',
            'navigate': '<i class="fas fa-location-arrow"></i>',
            'photo': '<i class="fas fa-camera"></i>',
            'pickup': '<i class="fas fa-hand-paper"></i>',
            'delivery': '<i class="fas fa-shipping-fast"></i>',
            'monitor': '<i class="fas fa-eye"></i>',
            'hover': '<i class="fas fa-pause-circle"></i>'
        };
        
        return icons[action] || '<i class="fas fa-circle"></i>';
    }
    
    initializeCharts() {
        // Initialize real-time plot
        this.charts.realtime = this.createRealtimePlot();
        
        // Initialize trajectory plot
        this.charts.trajectory = this.createTrajectoryPlot();
        
        // Initialize other plots
        this.charts.altitude = this.createAltitudePlot();
        this.charts.attitude = this.createAttitudePlot();
        this.charts.power = this.createPowerPlot();
    }
    
    createRealtimePlot() {
        const layout = {
            title: 'Real-time Telemetry',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Value' },
            legend: { x: 0, y: 1 },
            margin: { l: 50, r: 50, t: 50, b: 50 }
        };
        
        const config = {
            responsive: true,
            displayModeBar: false
        };
        
        Plotly.newPlot('realtimePlots', [], layout, config);
        return true;
    }
    
    createTrajectoryPlot() {
        const layout = {
            title: '3D Flight Trajectory',
            scene: {
                xaxis: { title: 'X (m)' },
                yaxis: { title: 'Y (m)' },
                zaxis: { title: 'Z (m)' }
            },
            margin: { l: 0, r: 0, t: 50, b: 0 }
        };
        
        const config = {
            responsive: true,
            displayModeBar: true
        };
        
        Plotly.newPlot('trajectoryPlot', [], layout, config);
        return true;
    }
    
    createAltitudePlot() {
        const layout = {
            title: 'Altitude & Speed',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Altitude (m)' },
            yaxis2: {
                title: 'Speed (m/s)',
                overlaying: 'y',
                side: 'right'
            },
            margin: { l: 50, r: 50, t: 50, b: 50 }
        };
        
        const config = {
            responsive: true,
            displayModeBar: false
        };
        
        Plotly.newPlot('altitudeSpeedPlot', [], layout, config);
        return true;
    }
    
    createAttitudePlot() {
        const layout = {
            title: 'Attitude Angles',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Angle (degrees)' },
            margin: { l: 50, r: 50, t: 50, b: 50 }
        };
        
        const config = {
            responsive: true,
            displayModeBar: false
        };
        
        Plotly.newPlot('attitudePlot', [], layout, config);
        return true;
    }
    
    createPowerPlot() {
        const layout = {
            title: 'Power & Energy',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Power (W)' },
            yaxis2: {
                title: 'Battery Level (%)',
                overlaying: 'y',
                side: 'right'
            },
            margin: { l: 50, r: 50, t: 50, b: 50 }
        };
        
        const config = {
            responsive: true,
            displayModeBar: false
        };
        
        Plotly.newPlot('powerPlot', [], layout, config);
        return true;
    }
    
    updateRealtimePlots() {
        if (this.telemetryData.length === 0) return;
        
        try {
            const times = this.telemetryData.map(d => d.timestamp || 0);
            const altitudes = this.telemetryData.map(d => d.altitude || 0);
            const speeds = this.telemetryData.map(d => d.ground_speed || 0);
            const verticalSpeeds = this.telemetryData.map(d => d.vertical_speed || 0);
            
            // Update real-time plot
            const realtimeData = [
                {
                    x: times,
                    y: altitudes,
                    type: 'scatter',
                    mode: 'lines',
                    name: 'Altitude (m)',
                    line: { color: 'blue' }
                },
                {
                    x: times,
                    y: speeds,
                    type: 'scatter',
                    mode: 'lines',
                    name: 'Ground Speed (m/s)',
                    yaxis: 'y2',
                    line: { color: 'red' }
                },
                {
                    x: times,
                    y: verticalSpeeds,
                    type: 'scatter',
                    mode: 'lines',
                    name: 'Vertical Speed (m/s)',
                    yaxis: 'y2',
                    line: { color: 'green' }
                }
            ];
            
            const layout = {
                title: 'Real-time Telemetry',
                xaxis: { title: 'Time (s)' },
                yaxis: { title: 'Altitude (m)', side: 'left' },
                yaxis2: { title: 'Speed (m/s)', side: 'right', overlaying: 'y' },
                showlegend: true
            };
            
            Plotly.react('realtimePlots', realtimeData, layout);
            
            // Update other plots on visible tabs
            this.updateVisiblePlots();
        } catch (error) {
            console.error('Error updating real-time plots:', error);
        }
    }
    
    updateVisiblePlots() {
        // Only update plots that are currently visible
        const activeTab = document.querySelector('.nav-link.active').id;
        
        switch (activeTab) {
            case 'trajectory-tab':
                this.updateTrajectoryPlot();
                break;
            case 'telemetry-tab':
                this.updateTelemetryPlots();
                break;
        }
    }
    
    updateTrajectoryPlot() {
        if (this.telemetryData.length === 0) return;
        
        try {
            const positions = this.telemetryData.map(d => d.position || {x: 0, y: 0, z: 0});
            const x = positions.map(p => p.x || 0);
            const y = positions.map(p => p.y || 0);
            const z = positions.map(p => p.z || 0);
            const times = this.telemetryData.map(d => d.timestamp || 0);
        
        const trajectoryData = [{
            x: x,
            y: y,
            z: z,
            type: 'scatter3d',
            mode: 'lines+markers',
            marker: {
                size: 3,
                color: times,
                colorscale: 'Viridis',
                showscale: true
            },
            line: {
                width: 4,
                color: times,
                colorscale: 'Viridis'
            },
            name: 'Flight Path'
        }];
        
        // Add start and end markers
        if (positions.length > 0) {
            trajectoryData.push({
                x: [x[0]],
                y: [y[0]],
                z: [z[0]],
                type: 'scatter3d',
                mode: 'markers',
                marker: { size: 8, color: 'green' },
                name: 'Start'
            });
            
            trajectoryData.push({
                x: [x[x.length - 1]],
                y: [y[y.length - 1]],
                z: [z[z.length - 1]],
                type: 'scatter3d',
                mode: 'markers',
                marker: { size: 8, color: 'red' },
                name: 'Current'
            });
        }
        
        const layout = {
            title: '3D Flight Trajectory',
            scene: {
                xaxis: { title: 'X (m)' },
                yaxis: { title: 'Y (m)' },
                zaxis: { title: 'Z (m)' }
            }
        };
        
        Plotly.react('trajectoryPlot', trajectoryData, layout);
        } catch (error) {
            console.error('Error updating trajectory plot:', error);
        }
    }
    
    updateTelemetryPlots() {
        if (this.telemetryData.length === 0) return;
        
        const times = this.telemetryData.map(d => d.simulation_time);
        const altitudes = this.telemetryData.map(d => d.position[2]);
        const speeds = this.telemetryData.map(d => d.airspeed);
        const attitudes = this.telemetryData.map(d => d.attitude.map(a => a * 180 / Math.PI));
        const power = this.telemetryData.map(d => d.power_consumption);
        const battery = this.telemetryData.map(d => d.battery_level * 100);
        
        // Altitude & Speed plot
        const altSpeedData = [
            {
                x: times,
                y: altitudes,
                type: 'scatter',
                mode: 'lines',
                name: 'Altitude',
                line: { color: 'blue' }
            },
            {
                x: times,
                y: speeds,
                type: 'scatter',
                mode: 'lines',
                name: 'Speed',
                yaxis: 'y2',
                line: { color: 'red' }
            }
        ];
        
        Plotly.react('altitudeSpeedPlot', altSpeedData);
        
        // Attitude plot
        const attitudeData = [
            {
                x: times,
                y: attitudes.map(a => a[0]),
                type: 'scatter',
                mode: 'lines',
                name: 'Roll',
                line: { color: 'red' }
            },
            {
                x: times,
                y: attitudes.map(a => a[1]),
                type: 'scatter',
                mode: 'lines',
                name: 'Pitch',
                line: { color: 'green' }
            },
            {
                x: times,
                y: attitudes.map(a => a[2]),
                type: 'scatter',
                mode: 'lines',
                name: 'Yaw',
                line: { color: 'blue' }
            }
        ];
        
        Plotly.react('attitudePlot', attitudeData);
        
        // Power plot
        const powerData = [
            {
                x: times,
                y: power,
                type: 'scatter',
                mode: 'lines',
                name: 'Power',
                line: { color: 'orange' }
            },
            {
                x: times,
                y: battery,
                type: 'scatter',
                mode: 'lines',
                name: 'Battery',
                yaxis: 'y2',
                line: { color: 'green' }
            }
        ];
        
        Plotly.react('powerPlot', powerData);
    }
    
    onTabChange(tabId) {
        // Update plots when tabs are switched
        setTimeout(() => {
            switch (tabId) {
                case 'trajectory-tab':
                    this.updateTrajectoryPlot();
                    break;
                case 'telemetry-tab':
                    this.updateTelemetryPlots();
                    break;
            }
        }, 100);
    }
    
    async startStatusPolling() {
        setInterval(async () => {
            try {
                const response = await fetch('/api/simulation/status');
                if (response.ok) {
                    const status = await response.json();
                    this.handleStatusUpdate(status);
                }
            } catch (error) {
                // Silently handle polling errors
            }
        }, 2000);
    }
    
    setLoadingState(loading) {
        const startBtn = document.getElementById('startBtn');
        if (loading) {
            startBtn.disabled = true;
            startBtn.innerHTML = '<i class="fas fa-spinner fa-spin"></i> Starting...';
        } else {
            startBtn.disabled = false;
            startBtn.innerHTML = '<i class="fas fa-play"></i> Start Simulation';
        }
    }
    
    updateConnectionStatus(connected) {
        // Visual indicator for WebSocket connection status
        const statusElement = document.getElementById('statusDisplay');
        if (connected) {
            statusElement.classList.remove('text-danger');
            statusElement.classList.add('text-success');
        } else {
            statusElement.classList.remove('text-success');
            statusElement.classList.add('text-danger');
        }
    }
    
    logMessage(level, message) {
        const logDisplay = document.getElementById('logDisplay');
        const timestamp = new Date().toLocaleTimeString();
        const logEntry = document.createElement('div');
        logEntry.className = `log-entry ${level}`;
        logEntry.textContent = `[${timestamp}] ${message}`;
        
        logDisplay.appendChild(logEntry);
        logDisplay.scrollTop = logDisplay.scrollHeight;
        
        // Keep only last 100 log entries
        while (logDisplay.children.length > 100) {
            logDisplay.removeChild(logDisplay.firstChild);
        }
    }
    
    showError(message) {
        this.showNotification(message, 'danger');
        this.logMessage('error', message);
    }
    
    showSuccess(message) {
        this.showNotification(message, 'success');
    }
    
    showInfo(message) {
        this.showNotification(message, 'info');
    }
    
    showNotification(message, type) {
        // Create a toast notification
        const toast = document.createElement('div');
        toast.className = `alert alert-${type} alert-dismissible fade show position-fixed`;
        toast.style.cssText = 'top: 20px; right: 20px; z-index: 9999; min-width: 300px;';
        toast.innerHTML = `
            ${message}
            <button type="button" class="btn-close" data-bs-dismiss="alert"></button>
        `;
        
        document.body.appendChild(toast);
        
        // Auto-remove after 5 seconds
        setTimeout(() => {
            if (toast.parentNode) {
                toast.parentNode.removeChild(toast);
            }
        }, 5000);
    }
}

// Initialize the application when the page loads
document.addEventListener('DOMContentLoaded', () => {
    window.droneController = new DroneSimulationController();
});
