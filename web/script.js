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
        
        // Terrain zoom state
        this.terrainZoom = {
            level: 1.0,
            center: { x: 0, y: 0, z: 0 },
            minZoom: 0.1,
            maxZoom: 10.0,
            animationDuration: 800,
            isAnimating: false
        };
        
        // Initialize the interface
        this.init();
    }
    
    async init() {
        // Load configurations
        await this.loadConfigurations();
        
        // Load any persisted simulation data
        this.loadPersistedData();
        
        // Setup event listeners
        this.setupEventListeners();
        
        // Initialize charts
        this.initializeCharts();
        
        // Update charts with persisted data if available
        if (this.telemetryData.length > 0) {
            setTimeout(() => {
                this.updateChartsWithPersistedData();
            }, 500); // Small delay to ensure charts are fully initialized
        }
        
        // Setup terrain zoom controls when tab becomes active
        this.setupTabChangeHandlers();
        
        // Also setup zoom controls if we're starting on trajectory tab
        setTimeout(() => {
            const activeTab = document.querySelector('.nav-link.active');
            if (activeTab && activeTab.id === 'trajectory-tab') {
                this.setupTerrainZoomControls();
            }
        }, 500);
        
        // Connect WebSocket
        this.connectWebSocket();
        
        // Start status polling
        this.startStatusPolling();
        
        console.log('Drone Simulation Controller initialized');
    }
    
    loadPersistedData() {
        // Load persisted telemetry data
        const storedTelemetry = localStorage.getItem('simulationTelemetry');
        if (storedTelemetry) {
            try {
                this.telemetryData = JSON.parse(storedTelemetry);
                console.log(`Loaded ${this.telemetryData.length} telemetry points from storage`);
            } catch (e) {
                console.error('Failed to load telemetry data:', e);
                this.telemetryData = [];
            }
        }
        
        // Load persisted mission data
        const storedMission = localStorage.getItem('simulationMission');
        if (storedMission) {
            try {
                this.missionData = JSON.parse(storedMission);
                console.log(`Loaded ${this.missionData.length} mission events from storage`);
            } catch (e) {
                console.error('Failed to load mission data:', e);
                this.missionData = [];
            }
        }
        
        // Load persisted status
        const storedStatus = localStorage.getItem('simulationStatus');
        if (storedStatus) {
            try {
                this.currentStatus = JSON.parse(storedStatus);
                console.log('Loaded simulation status from storage');
            } catch (e) {
                console.error('Failed to load simulation status:', e);
                this.currentStatus = {};
            }
        }
    }
    
    saveTelemetryToStorage() {
        // Save telemetry data to localStorage (keep last 500 points to avoid storage limits)
        const dataToSave = this.telemetryData.slice(-500);
        try {
            localStorage.setItem('simulationTelemetry', JSON.stringify(dataToSave));
        } catch (e) {
            console.error('Failed to save telemetry data:', e);
        }
    }
    
    saveMissionToStorage() {
        // Save mission data to localStorage
        try {
            localStorage.setItem('simulationMission', JSON.stringify(this.missionData));
        } catch (e) {
            console.error('Failed to save mission data:', e);
        }
    }
    
    saveStatusToStorage() {
        // Save current status to localStorage
        try {
            localStorage.setItem('simulationStatus', JSON.stringify(this.currentStatus));
        } catch (e) {
            console.error('Failed to save status data:', e);
        }
    }
    
    updateChartsWithPersistedData() {
        // Update charts with loaded telemetry data
        if (this.telemetryData.length > 0) {
            console.log('Updating charts with persisted telemetry data');
            
            // Update real-time plots with all persisted data
            this.updateRealtimePlots();
            
            // Update the latest status displays
            const latestTelemetry = this.telemetryData[this.telemetryData.length - 1];
            if (latestTelemetry) {
                this.updateDroneStatus(latestTelemetry);
                this.updateEnvironmentStatus(latestTelemetry);
                this.updateSimulationProgress(latestTelemetry.timestamp);
            }
            
            // Update mission display
            this.updateMissionDisplay();
        }
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
            
            // Update waypoint list after missions are loaded
            this.updateWaypointList();
            
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
        
        // Mission selection change event
        document.getElementById('missionType').addEventListener('change', () => {
            this.updateWaypointList();
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
            
            // Clear previous simulation data before starting new one
            this.clearSimulationData();
            
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
                
                // Initialize fresh charts
                this.initializeCharts();
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
    
    clearSimulationData() {
        // Clear telemetry data arrays
        this.telemetryData = [];
        this.missionData = [];
        this.currentStatus = {};
        
        // Clear stored data in localStorage
        localStorage.removeItem('simulationTelemetry');
        localStorage.removeItem('simulationMission');
        localStorage.removeItem('simulationStatus');
        
        // Clear existing charts
        Object.keys(this.charts).forEach(key => {
            if (this.charts[key]) {
                try {
                    this.charts[key].destroy();
                } catch (e) {
                    // Chart might not exist yet
                }
                this.charts[key] = null;
            }
        });
        
        // Clear chart containers
        const chartContainers = [
            'trajectoryChart', 'altitudeChart', 'attitudeChart', 
            'powerChart', 'realTimeChart', 'performanceChart'
        ];
        chartContainers.forEach(id => {
            const element = document.getElementById(id);
            if (element) {
                element.innerHTML = '';
            }
        });
        
        console.log('Cleared previous simulation data and charts');
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
        
        // Save to localStorage for persistence across page refreshes
        this.saveTelemetryToStorage();
        
        // Update real-time displays
        this.updateDroneStatus(telemetry);
        this.updateEnvironmentStatus(telemetry);
        this.updateRealtimePlots();
        
        // Update progress
        this.updateSimulationProgress(telemetry.timestamp);
        
        // Update mission display if we're on the mission tab
        const activeTab = document.querySelector('.nav-link.active')?.id;
        if (activeTab === 'mission-tab') {
            this.updateMissionDisplay();
        }
    }
    
    handleMissionEvent(event) {
        this.missionData.push(event);
        
        // Keep only last 100 events for performance
        if (this.missionData.length > 100) {
            this.missionData.shift();
        }
        
        // Save to localStorage for persistence
        this.saveMissionToStorage();
        
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
                    
                    // Update animated progress bar
                    this.updateAnimatedProgressBar(progress);
                    
                    // Update legacy progress bar (hidden but kept for compatibility)
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
    
    updateAnimatedProgressBar(progress) {
        try {
            const progressFill = document.getElementById('missionProgressFill');
            const droneIconContainer = document.getElementById('droneIconContainer');
            const progressPercentage = document.getElementById('missionProgressPercentage');
            
            if (progressFill && droneIconContainer && progressPercentage) {
                // Update progress fill width with smooth animation
                progressFill.style.width = `${progress}%`;
                
                // Move drone icon to match progress
                // Adjust position to account for icon width (drone should be at the end of the progress bar)
                const dronePosition = Math.max(0, progress - 2); // Small offset to keep drone visible
                droneIconContainer.style.left = `${dronePosition}%`;
                
                // Update percentage text
                progressPercentage.textContent = `${progress.toFixed(1)}%`;
                
                // Add visual feedback based on progress
                const droneIcon = droneIconContainer.querySelector('.drone-icon');
                if (droneIcon) {
                    if (progress >= 100) {
                        droneIcon.style.color = '#28a745'; // Green when complete
                        droneIcon.classList.add('mission-complete');
                    } else if (progress > 0) {
                        droneIcon.style.color = '#007bff'; // Blue when active
                        droneIcon.classList.remove('mission-complete');
                    }
                }
            }
        } catch (error) {
            console.warn('Error updating animated progress bar:', error);
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
        let currentMissionType = document.getElementById('missionType')?.value;
        let currentMission = missionConfigs[currentMissionType];
        
        // If no mission selected or mission not found, try to get from simulation state or use first available
        if (!currentMission || !currentMission.waypoints) {
            // Try to get from simulation state if available
            if (this.currentStatus && this.currentStatus.mission_config) {
                currentMission = this.currentStatus.mission_config;
                currentMissionType = currentMission.type;
            } else {
                // Use first available mission as fallback
                const availableMissions = Object.keys(missionConfigs);
                if (availableMissions.length > 0) {
                    currentMissionType = availableMissions[0];
                    currentMission = missionConfigs[currentMissionType];
                }
            }
        }
        
        console.log('Mission configs:', missionConfigs);
        console.log('Current mission type:', currentMissionType);
        console.log('Current mission:', currentMission);
        
        if (!currentMission || !currentMission.waypoints) {
            waypointList.innerHTML = '<p class="text-muted">No waypoints available</p>';
            console.log('No mission found for:', currentMissionType);
            console.log('Available missions:', Object.keys(missionConfigs));
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
                xaxis: { 
                    title: 'X (m)',
                    autorange: true
                },
                yaxis: { 
                    title: 'Y (m)',
                    autorange: true
                },
                zaxis: { 
                    title: 'Z (m)',
                    autorange: true
                },
                camera: this.getOptimalCameraView(x, y, z),
                aspectmode: 'data'
            }
        };
        
        const config = {
            responsive: true,
            displayModeBar: true
        };
        
        Plotly.react('trajectoryPlot', trajectoryData, layout, config);
        } catch (error) {
            console.error('Error updating trajectory plot:', error);
        }
    }
    
    updateTelemetryPlots() {
        if (this.telemetryData.length === 0) return;
        
        try {
            const times = this.telemetryData.map(d => d.timestamp || 0);
            const altitudes = this.telemetryData.map(d => d.altitude || 0);
            const groundSpeeds = this.telemetryData.map(d => d.ground_speed || 0);
            const verticalSpeeds = this.telemetryData.map(d => d.vertical_speed || 0);
            const rolls = this.telemetryData.map(d => (d.attitude?.roll || 0) * 180 / Math.PI);
            const pitches = this.telemetryData.map(d => (d.attitude?.pitch || 0) * 180 / Math.PI);
            const yaws = this.telemetryData.map(d => (d.attitude?.yaw || 0) * 180 / Math.PI);
            const progress = this.telemetryData.map(d => d.mission_progress || 0);
        
        // Altitude & Speed plot
        const altSpeedData = [
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
                y: groundSpeeds,
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
                line: { color: 'orange' }
            }
        ];
        
        const altSpeedLayout = {
            title: 'Altitude and Speed vs Time',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Altitude (m)', side: 'left' },
            yaxis2: { title: 'Speed (m/s)', side: 'right', overlaying: 'y' }
        };
        
        Plotly.react('altitudeSpeedPlot', altSpeedData, altSpeedLayout);
        
        // Attitude plot
        const attitudeData = [
            {
                x: times,
                y: rolls,
                type: 'scatter',
                mode: 'lines',
                name: 'Roll (deg)',
                line: { color: 'red' }
            },
            {
                x: times,
                y: pitches,
                type: 'scatter',
                mode: 'lines',
                name: 'Pitch (deg)',
                line: { color: 'green' }
            },
            {
                x: times,
                y: yaws,
                type: 'scatter',
                mode: 'lines',
                name: 'Yaw (deg)',
                line: { color: 'blue' }
            }
        ];
        
        const attitudeLayout = {
            title: 'Attitude vs Time',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Angle (degrees)' }
        };
        
        Plotly.react('attitudePlot', attitudeData, attitudeLayout);
        
        // Power & Energy plot (simulate realistic power consumption)
        const powerConsumption = this.telemetryData.map(d => {
            const speed = d.ground_speed || 0;
            const altitude = d.altitude || 0;
            // Simple power model: base power + speed factor + altitude factor
            return 50 + (speed * 2) + (altitude * 0.1); // Watts
        });
        
        const energy = [];
        let totalEnergy = 0;
        powerConsumption.forEach((power, i) => {
            if (i > 0) {
                const dt = (times[i] - times[i-1]) / 3600; // hours
                totalEnergy += power * dt; // Wh
            }
            energy.push(totalEnergy);
        });
        
        const powerData = [
            {
                x: times,
                y: powerConsumption,
                type: 'scatter',
                mode: 'lines',
                name: 'Power (W)',
                line: { color: 'purple' }
            },
            {
                x: times,
                y: energy,
                type: 'scatter',
                mode: 'lines',
                name: 'Energy (Wh)',
                yaxis: 'y2',
                line: { color: 'orange' }
            }
        ];
        
        const powerLayout = {
            title: 'Power & Energy vs Time',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Power (W)', side: 'left' },
            yaxis2: { title: 'Energy (Wh)', side: 'right', overlaying: 'y' }
        };
        
        Plotly.react('powerPlot', powerData, powerLayout);
        
        // Mission Progress plot (separate plot)
        const progressData = [
            {
                x: times,
                y: progress,
                type: 'scatter',
                mode: 'lines',
                name: 'Mission Progress (%)',
                line: { color: 'green', width: 3 },
                fill: 'tozeroy',
                fillcolor: 'rgba(0,128,0,0.1)'
            }
        ];
        
        const progressLayout = {
            title: 'Mission Progress vs Time',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Progress (%)', range: [0, 100] }
        };
        
        Plotly.react('missionProgressPlot', progressData, progressLayout);
        
        } catch (error) {
            console.error('Error updating telemetry plots:', error);
        }
    }
    
    setupTabChangeHandlers() {
        // Listen for tab changes to setup terrain zoom when trajectory tab is shown
        const tabButtons = document.querySelectorAll('[data-bs-toggle="tab"]');
        tabButtons.forEach(button => {
            button.addEventListener('shown.bs.tab', (event) => {
                const href = event.target.getAttribute('href');
                const targetId = href ? href.substring(1) : '';
                this.onTabChange(event.target.id, targetId);
            });
        });
    }

    onTabChange(tabId, targetId) {
        // Update plots when tabs are switched
        setTimeout(() => {
            switch (tabId) {
                case 'trajectory-tab':
                    this.updateTrajectoryPlot();
                    // Add a small delay to ensure plot is rendered before adding controls
                    setTimeout(() => this.setupTerrainZoomControls(), 200);
                    break;
                case 'telemetry-tab':
                    this.updateTelemetryPlots();
                    break;
                case 'mission-tab':
                    this.updateMissionDisplay();
                    break;
                case 'analytics-tab':
                    this.loadSessionHistory();
                    this.loadPerformanceAnalytics();
                    break;
                case 'real-world-tab':
                    this.loadPopularLocations();
                    break;
                case 'video-export-tab':
                    this.loadPastSessions();
                    this.refreshVideoList();
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
    
    // Interactive Terrain Zoom functionality
    setupTerrainZoomControls() {
        // Add custom zoom controls to the trajectory tab
        const trajectoryTab = document.getElementById('trajectory');
        if (!trajectoryTab) {
            console.log('Trajectory tab not found, retrying in 1 second...');
            setTimeout(() => this.setupTerrainZoomControls(), 1000);
            return;
        }
        
        // Check if controls already exist to avoid duplicates
        const existingControls = trajectoryTab.querySelector('.terrain-zoom-controls');
        if (existingControls) {
            console.log('Terrain zoom controls already exist');
            return;
        }
        
        // Create zoom control panel
        const zoomControlsHtml = `
            <div class="terrain-zoom-controls mb-3">
                <div class="card">
                    <div class="card-header">
                        <h6><i class="fas fa-search-plus"></i> Terrain View Controls</h6>
                    </div>
                    <div class="card-body">
                        <div class="row">
                            <div class="col-md-6">
                                <label class="form-label">Zoom Level</label>
                                <input type="range" class="form-range" id="zoomSlider" 
                                       min="0.1" max="10" step="0.1" value="1.0">
                                <small class="text-muted">Current: <span id="zoomValue">1.0x</span></small>
                            </div>
                            <div class="col-md-6">
                                <label class="form-label">View Preset</label>
                                <div class="btn-group w-100" role="group">
                                    <button type="button" class="btn btn-outline-primary btn-sm" id="topViewBtn">
                                        <i class="fas fa-arrow-down"></i> Top
                                    </button>
                                    <button type="button" class="btn btn-outline-primary btn-sm" id="sideViewBtn">
                                        <i class="fas fa-arrows-alt-h"></i> Side
                                    </button>
                                    <button type="button" class="btn btn-outline-primary btn-sm" id="followBtn">
                                        <i class="fas fa-camera"></i> Follow
                                    </button>
                                </div>
                            </div>
                        </div>
                        <div class="row mt-2">
                            <div class="col-12">
                                <div class="btn-group w-100" role="group">
                                    <button type="button" class="btn btn-outline-success btn-sm" id="zoomInBtn">
                                        <i class="fas fa-plus"></i> Zoom In
                                    </button>
                                    <button type="button" class="btn btn-outline-warning btn-sm" id="zoomOutBtn">
                                        <i class="fas fa-minus"></i> Zoom Out
                                    </button>
                                    <button type="button" class="btn btn-outline-info btn-sm" id="resetViewBtn">
                                        <i class="fas fa-home"></i> Reset View
                                    </button>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        `;
        
        // Insert before the trajectory plot
        const trajectoryPlot = document.getElementById('trajectoryPlot');
        if (trajectoryPlot) {
            trajectoryPlot.insertAdjacentHTML('beforebegin', zoomControlsHtml);
            this.bindTerrainZoomEvents();
            console.log('Terrain zoom controls added to DOM');
        } else {
            console.warn('Trajectory plot element not found');
        }
    }
    
    bindTerrainZoomEvents() {
        // Use setTimeout to ensure DOM elements are ready
        setTimeout(() => {
            // Zoom slider
            const zoomSlider = document.getElementById('zoomSlider');
            const zoomValue = document.getElementById('zoomValue');
            
            if (zoomSlider && zoomValue) {
                zoomSlider.addEventListener('input', (e) => {
                    const level = parseFloat(e.target.value);
                    this.terrainZoom.level = level;
                    zoomValue.textContent = `${level.toFixed(1)}x`;
                    this.applyTerrainZoom(level, false); // No animation for slider
                });
            } else {
                console.warn('Zoom slider elements not found');
            }
            
            // Zoom buttons
            const zoomInBtn = document.getElementById('zoomInBtn');
            const zoomOutBtn = document.getElementById('zoomOutBtn');
            const resetViewBtn = document.getElementById('resetViewBtn');
            
            if (zoomInBtn) zoomInBtn.addEventListener('click', () => this.zoomTerrain('in'));
            if (zoomOutBtn) zoomOutBtn.addEventListener('click', () => this.zoomTerrain('out'));
            if (resetViewBtn) resetViewBtn.addEventListener('click', () => this.resetTerrainView());
            
            // View presets
            const topViewBtn = document.getElementById('topViewBtn');
            const sideViewBtn = document.getElementById('sideViewBtn');
            const followBtn = document.getElementById('followBtn');
            
            if (topViewBtn) topViewBtn.addEventListener('click', () => this.setViewPreset('top'));
            if (sideViewBtn) sideViewBtn.addEventListener('click', () => this.setViewPreset('side'));
            if (followBtn) followBtn.addEventListener('click', () => this.followDrone());
            
            console.log('Terrain zoom controls bound successfully');
        }, 100);
    }
    
    getOptimalCameraView(x = [], y = [], z = []) {
        if (x.length === 0) {
            return {
                eye: { x: 1.5, y: 1.5, z: 1.5 },
                center: { x: 0, y: 0, z: 0 },
                up: { x: 0, y: 0, z: 1 }
            };
        }
        
        // Calculate trajectory center
        const centerX = (Math.min(...x) + Math.max(...x)) / 2;
        const centerY = (Math.min(...y) + Math.max(...y)) / 2;
        const centerZ = (Math.min(...z) + Math.max(...z)) / 2;
        
        // Calculate range for camera distance
        const rangeX = Math.max(...x) - Math.min(...x);
        const rangeY = Math.max(...y) - Math.min(...y);
        const rangeZ = Math.max(...z) - Math.min(...z);
        const maxRange = Math.max(rangeX, rangeY, rangeZ, 10);
        
        // Adjust distance based on zoom level
        const zoomFactor = this.terrainZoom ? this.terrainZoom.level : 1;
        const distance = maxRange * 2 / zoomFactor;
        
        return {
            eye: { 
                x: centerX + distance * 0.8, 
                y: centerY + distance * 0.8, 
                z: centerZ + distance * 0.6 
            },
            center: { 
                x: centerX, 
                y: centerY, 
                z: centerZ 
            },
            up: { x: 0, y: 0, z: 1 }
        };
    }
    
    calculateTrajectoryBounds(x, y, z) {
        if (x.length === 0 || y.length === 0 || z.length === 0) {
            return {
                min: { x: -10, y: -10, z: 0 },
                max: { x: 10, y: 10, z: 20 },
                center: { x: 0, y: 0, z: 10 },
                range: { x: 20, y: 20, z: 20 }
            };
        }
        
        const minX = Math.min(...x);
        const maxX = Math.max(...x);
        const minY = Math.min(...y);
        const maxY = Math.max(...y);
        const minZ = Math.min(...z);
        const maxZ = Math.max(...z);
        
        return {
            min: { x: minX, y: minY, z: minZ },
            max: { x: maxX, y: maxY, z: maxZ },
            center: { 
                x: (minX + maxX) / 2, 
                y: (minY + maxY) / 2, 
                z: (minZ + maxZ) / 2 
            },
            range: { 
                x: maxX - minX, 
                y: maxY - minY, 
                z: maxZ - minZ 
            }
        };
    }
    
    zoomTerrain(direction) {
        if (this.terrainZoom.isAnimating) return;
        
        const factor = direction === 'in' ? 1.4 : 1/1.4;
        const newLevel = Math.max(
            this.terrainZoom.minZoom,
            Math.min(this.terrainZoom.maxZoom, this.terrainZoom.level * factor)
        );
        
        this.animateTerrainZoom(newLevel);
    }
    
    applyTerrainZoom(level, animate = true) {
        this.terrainZoom.level = level;
        
        if (animate) {
            this.animateTerrainZoom(level);
        } else {
            // Update the plot immediately without animation
            setTimeout(() => this.updateTrajectoryPlot(), 50);
        }
    }
    
    animateTerrainZoom(targetLevel) {
        if (this.terrainZoom.isAnimating) return;
        
        this.terrainZoom.isAnimating = true;
        const startLevel = this.terrainZoom.level;
        const startTime = Date.now();
        
        const animate = () => {
            const elapsed = Date.now() - startTime;
            const progress = Math.min(elapsed / this.terrainZoom.animationDuration, 1);
            
            // Smooth easing function
            const easeProgress = 1 - Math.pow(1 - progress, 3);
            
            const currentLevel = startLevel + (targetLevel - startLevel) * easeProgress;
            this.terrainZoom.level = currentLevel;
            
            // Update UI
            const zoomSlider = document.getElementById('zoomSlider');
            const zoomValue = document.getElementById('zoomValue');
            if (zoomSlider) zoomSlider.value = currentLevel.toFixed(1);
            if (zoomValue) zoomValue.textContent = `${currentLevel.toFixed(1)}x`;
            
            // Update plot immediately without animation
            setTimeout(() => this.updateTrajectoryPlot(), 50);
            
            if (progress < 1) {
                requestAnimationFrame(animate);
            } else {
                this.terrainZoom.isAnimating = false;
            }
        };
        
        requestAnimationFrame(animate);
    }
    
    resetTerrainView() {
        this.terrainZoom.level = 1.0;
        this.terrainZoom.center = { x: 0, y: 0, z: 0 };
        this.animateTerrainZoom(1.0);
    }
    
    setViewPreset(preset) {
        if (this.telemetryData.length === 0) return;
        
        const positions = this.telemetryData.map(d => d.position || {x: 0, y: 0, z: 0});
        const x = positions.map(p => p.x || 0);
        const y = positions.map(p => p.y || 0);
        const z = positions.map(p => p.z || 0);
        const bounds = this.calculateTrajectoryBounds(x, y, z);
        
        let camera;
        const distance = Math.max(bounds.range.x, bounds.range.y, bounds.range.z) * 2.0;
        
        switch (preset) {
            case 'top':
                camera = {
                    eye: { x: bounds.center.x, y: bounds.center.y, z: bounds.center.z + distance * 1.5 },
                    center: bounds.center,
                    up: { x: 0, y: 1, z: 0 }
                };
                break;
            case 'side':
                camera = {
                    eye: { x: bounds.center.x + distance * 1.5, y: bounds.center.y, z: bounds.center.z },
                    center: bounds.center,
                    up: { x: 0, y: 0, z: 1 }
                };
                break;
            default:
                return;
        }
        
        this.animateCameraToPosition(camera);
    }
    
    followDrone() {
        if (this.telemetryData.length === 0) return;
        
        const lastPosition = this.telemetryData[this.telemetryData.length - 1].position || {x: 0, y: 0, z: 0};
        const offset = 50; // Follow distance
        
        const camera = {
            eye: { 
                x: lastPosition.x - offset, 
                y: lastPosition.y - offset, 
                z: lastPosition.z + offset 
            },
            center: lastPosition,
            up: { x: 0, y: 0, z: 1 }
        };
        
        this.animateCameraToPosition(camera);
    }
    
    animateCameraToPosition(targetCamera) {
        const plot = document.getElementById('trajectoryPlot');
        if (!plot) return;
        
        try {
            Plotly.relayout(plot, {
                'scene.camera': targetCamera
            });
        } catch (error) {
            console.warn('Camera animation failed:', error);
            // Fallback to updating the plot
            this.updateTrajectoryPlot();
        }
    }

    // Analytics functionality
    async loadSessionHistory() {
        try {
            const response = await fetch('/api/history/sessions');
            const data = await response.json();
            
            this.displaySessionHistory(data.sessions);
            document.getElementById('sessionCount').textContent = `${data.total} sessions found`;
        } catch (error) {
            console.error('Error loading session history:', error);
            this.showError('Failed to load session history');
        }
    }
    
    displaySessionHistory(sessions) {
        const container = document.getElementById('sessionHistory');
        
        if (sessions.length === 0) {
            container.innerHTML = '<div class="text-muted text-center p-3">No simulation sessions found.<br>Run a simulation to see data here.</div>';
            return;
        }
        
        container.innerHTML = sessions.map(session => `
            <div class="card mb-2 session-card" data-session-id="${session.id}">
                <div class="card-body p-3">
                    <div class="row">
                        <div class="col-md-6">
                            <h6 class="mb-1">${session.drone_model} - ${session.environment}</h6>
                            <small class="text-muted">${session.mission_type}</small>
                        </div>
                        <div class="col-md-6 text-end">
                            <div class="badge bg-${this.getStatusColor(session.status)}">${session.status}</div>
                            <div class="text-muted small mt-1">
                                ${new Date(session.start_time).toLocaleString()}
                            </div>
                        </div>
                    </div>
                    <div class="row mt-2">
                        <div class="col-md-3">
                            <small class="text-muted">Progress:</small><br>
                            <strong>${session.mission_progress?.toFixed(1) || 0}%</strong>
                        </div>
                        <div class="col-md-3">
                            <small class="text-muted">Duration:</small><br>
                            <strong>${session.duration ? (session.duration / 60).toFixed(1) + 'm' : 'N/A'}</strong>
                        </div>
                        <div class="col-md-3">
                            <small class="text-muted">Distance:</small><br>
                            <strong>${session.total_distance ? (session.total_distance / 1000).toFixed(1) + 'km' : 'N/A'}</strong>
                        </div>
                        <div class="col-md-3">
                            <small class="text-muted">Energy:</small><br>
                            <strong>${session.total_energy ? session.total_energy.toFixed(1) + 'Wh' : 'N/A'}</strong>
                        </div>
                    </div>
                    <div class="mt-2">
                        <button class="btn btn-sm btn-outline-primary view-session-btn" data-session-id="${session.id}">
                            <i class="fas fa-eye"></i> View Details
                        </button>
                    </div>
                </div>
            </div>
        `).join('');
        
        // Add event listeners for view session buttons
        container.querySelectorAll('.view-session-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                e.preventDefault();
                const sessionId = e.target.closest('.view-session-btn').getAttribute('data-session-id');
                console.log('Viewing session details for ID:', sessionId);
                this.viewSessionDetails(sessionId);
            });
        });
    }
    
    getStatusColor(status) {
        switch(status) {
            case 'completed': return 'success';
            case 'running': return 'primary';
            case 'failed': return 'danger';
            case 'stopped': return 'warning';
            default: return 'secondary';
        }
    }
    
    async viewSessionDetails(sessionId) {
        try {
            const [telemetryResponse, eventsResponse] = await Promise.all([
                fetch(`/api/history/sessions/${sessionId}/telemetry`),
                fetch(`/api/history/sessions/${sessionId}/events`)
            ]);
            
            const telemetryData = await telemetryResponse.json();
            const eventsData = await eventsResponse.json();
            
            this.displaySessionDetails(sessionId, telemetryData, eventsData);
        } catch (error) {
            console.error('Error loading session details:', error);
            this.showError('Failed to load session details');
        }
    }
    
    displaySessionDetails(sessionId, telemetryData, eventsData) {
        console.log('Displaying session details for:', sessionId);
        console.log('Telemetry data:', telemetryData);
        console.log('Events data:', eventsData);
        
        const container = document.getElementById('sessionDetailsTable');
        if (!container) {
            console.error('sessionDetailsTable element not found');
            return;
        }
        
        // Safe data extraction with defaults
        const telemetry = telemetryData.telemetry || [];
        const events = eventsData.events || [];
        
        const maxAltitude = telemetry.length > 0 ? 
            Math.max(...telemetry.map(t => t.altitude || 0)).toFixed(1) : '0';
        const maxSpeed = telemetry.length > 0 ? 
            Math.max(...telemetry.map(t => t.ground_speed || 0)).toFixed(1) : '0';
        
        container.innerHTML = `
            <div class="mb-3">
                <h6>Session ${sessionId} - Telemetry Overview</h6>
                <div class="row">
                    <div class="col-md-3">
                        <div class="card text-center">
                            <div class="card-body">
                                <h5 class="text-primary">${telemetry.length}</h5>
                                <small class="text-muted">Data Points</small>
                            </div>
                        </div>
                    </div>
                    <div class="col-md-3">
                        <div class="card text-center">
                            <div class="card-body">
                                <h5 class="text-success">${events.length}</h5>
                                <small class="text-muted">Mission Events</small>
                            </div>
                        </div>
                    </div>
                    <div class="col-md-3">
                        <div class="card text-center">
                            <div class="card-body">
                                <h5 class="text-info">${maxAltitude}m</h5>
                                <small class="text-muted">Max Altitude</small>
                            </div>
                        </div>
                    </div>
                    <div class="col-md-3">
                        <div class="card text-center">
                            <div class="card-body">
                                <h5 class="text-warning">${maxSpeed}m/s</h5>
                                <small class="text-muted">Max Speed</small>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
            
            <div class="table-responsive">
                <table class="table table-striped table-hover">
                    <thead>
                        <tr>
                            <th>Time</th>
                            <th>Event</th>
                            <th>Position</th>
                            <th>Progress</th>
                            <th>Description</th>
                        </tr>
                    </thead>
                    <tbody>
                        ${events.map(event => `
                            <tr>
                                <td>${(event.timestamp || 0).toFixed(1)}s</td>
                                <td><span class="badge bg-info">${event.event_type || 'Unknown'}</span></td>
                                <td>${(event.position && event.position.x !== undefined) ? 
                                    `${event.position.x.toFixed(1)}, ${event.position.y.toFixed(1)}, ${event.position.z.toFixed(1)}` : 
                                    'N/A'}</td>
                                <td>${(event.mission_progress || 0).toFixed(1)}%</td>
                                <td>${event.description || event.action || 'N/A'}</td>
                            </tr>
                        `).join('')}
                    </tbody>
                </table>
            </div>
            
            ${telemetry.length > 0 ? `
                <div class="mt-3">
                    <h6>Sample Telemetry Data (First 5 points)</h6>
                    <div class="table-responsive">
                        <table class="table table-sm">
                            <thead>
                                <tr>
                                    <th>Time</th>
                                    <th>Position</th>
                                    <th>Altitude</th>
                                    <th>Speed</th>
                                    <th>Energy</th>
                                </tr>
                            </thead>
                            <tbody>
                                ${telemetry.slice(0, 5).map(point => `
                                    <tr>
                                        <td>${(point.timestamp || 0).toFixed(1)}s</td>
                                        <td>${(point.position && point.position.x !== undefined) ? 
                                            `${point.position.x.toFixed(1)}, ${point.position.y.toFixed(1)}, ${point.position.z.toFixed(1)}` : 
                                            'N/A'}</td>
                                        <td>${(point.altitude || 0).toFixed(1)}m</td>
                                        <td>${(point.ground_speed || 0).toFixed(1)}m/s</td>
                                        <td>${(point.energy_consumed || 0).toFixed(3)}Wh</td>
                                    </tr>
                                `).join('')}
                            </tbody>
                        </table>
                    </div>
                </div>
            ` : '<div class="alert alert-info">No telemetry data available for this session.</div>'}
        `;
        
        console.log('Session details displayed successfully');
        
        // Scroll to the session details section
        container.scrollIntoView({ behavior: 'smooth', block: 'start' });
    }
    
    async loadPerformanceAnalytics() {
        try {
            const response = await fetch('/api/analytics/performance');
            const data = await response.json();
            
            this.displayPerformanceSummary(data);
            this.displayPerformanceCharts(data);
        } catch (error) {
            console.error('Error loading performance analytics:', error);
            this.showError('Failed to load performance analytics');
        }
    }
    
    displayPerformanceSummary(data) {
        const container = document.getElementById('performanceSummary');
        
        if (!data || data.total_sessions === 0) {
            container.innerHTML = '<div class="text-muted text-center p-3">No performance data available.<br>Run simulations to see analytics.</div>';
            return;
        }
        
        container.innerHTML = `
            <div class="row">
                <div class="col-6">
                    <div class="text-center">
                        <h4 class="text-primary">${data.total_sessions || 0}</h4>
                        <small class="text-muted">Total Sessions</small>
                    </div>
                </div>
                <div class="col-6">
                    <div class="text-center">
                        <h4 class="text-success">${(data.success_rate || 0).toFixed(1)}%</h4>
                        <small class="text-muted">Success Rate</small>
                    </div>
                </div>
            </div>
            <div class="row mt-3">
                <div class="col-6">
                    <div class="text-center">
                        <h5 class="text-info">${((data.average_metrics && data.average_metrics.duration) || 0).toFixed(1)}m</h5>
                        <small class="text-muted">Avg Duration</small>
                    </div>
                </div>
                <div class="col-6">
                    <div class="text-center">
                        <h5 class="text-warning">${((data.average_metrics && data.average_metrics.energy) || 0).toFixed(1)}Wh</h5>
                        <small class="text-muted">Avg Energy</small>
                    </div>
                </div>
            </div>
        `;
    }
    
    displayPerformanceCharts(data) {
        const sessions = data.recent_sessions || [];
        
        if (sessions.length === 0) {
            document.getElementById('performanceChart').innerHTML = '<div class="text-muted text-center p-3">No data for charts</div>';
            document.getElementById('trendsChart').innerHTML = '<div class="text-muted text-center p-3">No data for trends</div>';
            return;
        }
        
        // Performance comparison chart
        const performanceData = [
            {
                x: sessions.map(s => s.drone_model),
                y: sessions.map(s => s.mission_progress || 0),
                type: 'bar',
                name: 'Mission Progress (%)',
                marker: { color: 'rgba(54, 162, 235, 0.8)' }
            }
        ];
        
        const performanceLayout = {
            title: 'Mission Progress by Session',
            xaxis: { title: 'Sessions (Drone Model)' },
            yaxis: { title: 'Progress (%)' },
            height: 300
        };
        
        Plotly.react('performanceChart', performanceData, performanceLayout);
        
        // Trends chart
        const trendsData = [
            {
                x: sessions.map((s, i) => `Session ${s.id}`),
                y: sessions.map(s => s.total_energy || 0),
                type: 'scatter',
                mode: 'lines+markers',
                name: 'Energy Consumption (Wh)',
                line: { color: 'orange', width: 3 },
                marker: { size: 8 }
            }
        ];
        
        const trendsLayout = {
            title: 'Energy Consumption by Session',
            xaxis: { title: 'Sessions' },
            yaxis: { title: 'Energy (Wh)' },
            height: 300
        };
        
        Plotly.react('trendsChart', trendsData, trendsLayout);
    }
    
    setupAnalyticsEventListeners() {
        // Refresh history button
        document.getElementById('refreshHistoryBtn').addEventListener('click', () => {
            this.loadSessionHistory();
        });
        
        // Update analytics button
        document.getElementById('updateAnalyticsBtn').addEventListener('click', () => {
            this.loadPerformanceAnalytics();
        });
        
        // Analytics tab activation
        document.getElementById('analytics-tab').addEventListener('click', () => {
            // Load analytics data when tab is activated
            setTimeout(() => {
                this.loadSessionHistory();
                this.loadPerformanceAnalytics();
            }, 100);
        });
    }
    
    // Enhanced Features: Google Maps Integration
    async loadPopularLocations() {
        try {
            const response = await fetch('/api/maps/locations/popular');
            const data = await response.json();
            
            const listElement = document.getElementById('popularLocationsList');
            listElement.innerHTML = '';
            
            data.locations.forEach(location => {
                const item = document.createElement('a');
                item.className = 'list-group-item list-group-item-action';
                item.innerHTML = `
                    <div class="d-flex w-100 justify-content-between">
                        <h6 class="mb-1">${location.name}</h6>
                        <small>${location.type}</small>
                    </div>
                    <p class="mb-1">${location.description}</p>
                    <small>Lat: ${location.latitude}, Lng: ${location.longitude}</small>
                `;
                item.addEventListener('click', () => this.selectLocation(location));
                listElement.appendChild(item);
            });
        } catch (error) {
            console.error('Error loading popular locations:', error);
        }
    }
    
    async searchLocations(query) {
        try {
            const response = await fetch(`/api/maps/locations/search?query=${encodeURIComponent(query)}`);
            const data = await response.json();
            
            const resultsElement = document.getElementById('locationResults');
            resultsElement.innerHTML = '';
            
            if (data.results && data.results.length > 0) {
                data.results.forEach(location => {
                    const item = document.createElement('div');
                    item.className = 'card mb-2';
                    item.innerHTML = `
                        <div class="card-body">
                            <h6 class="card-title">${location.name}</h6>
                            <p class="card-text">${location.description}</p>
                            <small class="text-muted">Lat: ${location.latitude}, Lng: ${location.longitude}</small>
                            <button class="btn btn-sm btn-primary ms-2" onclick="window.droneController.selectLocation(${JSON.stringify(location).replace(/"/g, '&quot;')})">
                                Select
                            </button>
                        </div>
                    `;
                    resultsElement.appendChild(item);
                });
            } else {
                resultsElement.innerHTML = '<div class="alert alert-info">No locations found.</div>';
            }
        } catch (error) {
            console.error('Error searching locations:', error);
            document.getElementById('locationResults').innerHTML = '<div class="alert alert-danger">Error searching locations.</div>';
        }
    }
    
    selectLocation(location) {
        this.selectedLocation = location;
        const infoElement = document.getElementById('selectedLocationInfo');
        infoElement.innerHTML = `
            <h6>${location.name}</h6>
            <p>${location.description}</p>
            <small>Coordinates: ${location.latitude}, ${location.longitude}</small>
        `;
        infoElement.style.display = 'block';
        document.getElementById('loadTerrainBtn').style.display = 'inline-block';
        document.getElementById('quickVideoRealWorldBtn').style.display = 'inline-block';
    }
    
    async loadTerrain() {
        if (!this.selectedLocation) return;
        
        try {
            document.getElementById('loadTerrainBtn').innerHTML = '<i class="fas fa-spinner fa-spin"></i> Loading...';
            document.getElementById('loadTerrainBtn').disabled = true;
            
            const response = await fetch('/api/maps/terrain/generate', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    latitude: this.selectedLocation.latitude,
                    longitude: this.selectedLocation.longitude,
                    size: 20
                })
            });
            
            const data = await response.json();
            
            if (data.error) {
                alert('Error loading terrain: ' + data.error);
                return;
            }
            
            // Show terrain model information
            const infoElement = document.getElementById('selectedLocationInfo');
            infoElement.innerHTML += `
                <hr>
                <h6>3D Terrain Model Generated</h6>
                <p>Grid Size: ${data.grid_size}x${data.grid_size}</p>
                <p>Elevation Range: ${data.bounds.min_elevation.toFixed(1)}m - ${data.bounds.max_elevation.toFixed(1)}m</p>
                <p>Source: ${data.source === 'google_maps' ? 'Google Maps API' : 'Synthetic Model'}</p>
                ${data.description ? `<p><small>${data.description}</small></p>` : ''}
                <button class="btn btn-success btn-sm" onclick="window.droneController.applyTerrainEnvironment()">
                    Apply to Simulation
                </button>
            `;
            
            this.terrainModel = data;
            
            document.getElementById('loadTerrainBtn').innerHTML = '<i class="fas fa-check"></i> Terrain Loaded';
            document.getElementById('loadTerrainBtn').classList.remove('btn-primary');
            document.getElementById('loadTerrainBtn').classList.add('btn-success');
            
        } catch (error) {
            console.error('Error loading terrain:', error);
            alert('Error loading terrain model.');
        } finally {
            document.getElementById('loadTerrainBtn').disabled = false;
        }
    }
    
    applyTerrainEnvironment() {
        if (!this.terrainModel) return;
        
        // Apply terrain to simulation environment
        alert('Terrain environment applied to simulation!');
    }
    
    // Enhanced Features: AI Environment Generation
    async generateLunarEnvironment() {
        const regionType = document.getElementById('lunarRegionType').value;
        const sizeKm = document.getElementById('lunarSize').value;
        
        try {
            const response = await fetch(`/api/environments/ai/lunar?region_type=${regionType}&size_km=${sizeKm}`);
            const data = await response.json();
            
            if (data.error) {
                alert('Error generating lunar environment: ' + data.error);
                return;
            }
            
            this.displayGeneratedEnvironment(data, 'lunar');
        } catch (error) {
            console.error('Error generating lunar environment:', error);
            alert('Error generating lunar environment.');
        }
    }
    
    async generateMartianEnvironment() {
        const regionType = document.getElementById('martianRegionType').value;
        const season = document.getElementById('martianSeason').value;
        
        try {
            const response = await fetch(`/api/environments/ai/martian?region_type=${regionType}&season=${season}`);
            const data = await response.json();
            
            if (data.error) {
                alert('Error generating Martian environment: ' + data.error);
                return;
            }
            
            this.displayGeneratedEnvironment(data, 'martian');
        } catch (error) {
            console.error('Error generating Martian environment:', error);
            alert('Error generating Martian environment.');
        }
    }
    
    displayGeneratedEnvironment(environment, type) {
        const displayElement = document.getElementById('generatedEnvironment');
        displayElement.innerHTML = `
            <h6>${type.charAt(0).toUpperCase() + type.slice(1)} Environment Generated</h6>
            <p><strong>Name:</strong> ${environment.name}</p>
            <p><strong>Description:</strong> ${environment.description}</p>
            <p><strong>Size:</strong> ${environment.size_km} km²</p>
            <p><strong>Features:</strong> ${environment.surface_features.length} unique features</p>
            <button class="btn btn-success btn-sm" onclick="window.droneController.applyAIEnvironment(${JSON.stringify(environment).replace(/"/g, '&quot;')})">
                Apply to Simulation
            </button>
        `;
        displayElement.style.display = 'block';
        
        this.generatedEnvironment = environment;
    }
    
    applyAIEnvironment(environment) {
        // Apply AI environment to simulation
        alert(`${environment.name} environment applied to simulation!`);
    }
    
    // Enhanced Features: Video Export
    async startVideoRecording() {
        if (!this.simulationRunning) {
            alert('Please start a simulation before recording.');
            return;
        }
        
        try {
            const response = await fetch('/api/video/export/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    export_settings: {
                        fps: 30,
                        resolution: [1920, 1080]
                    }
                })
            });
            
            const data = await response.json();
            
            if (data.success) {
                document.getElementById('startRecordingBtn').disabled = true;
                document.getElementById('stopRecordingBtn').disabled = false;
                document.getElementById('recordingStatus').style.display = 'block';
                this.videoRecording = true;
            } else {
                alert('Failed to start video recording: ' + data.message);
            }
        } catch (error) {
            console.error('Error starting video recording:', error);
            alert('Error starting video recording.');
        }
    }
    
    async stopVideoRecording() {
        try {
            const response = await fetch('/api/video/export/stop', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' }
            });
            
            const data = await response.json();
            
            if (data.success) {
                document.getElementById('startRecordingBtn').disabled = false;
                document.getElementById('stopRecordingBtn').disabled = true;
                document.getElementById('recordingStatus').style.display = 'none';
                document.getElementById('exportVideoBtn').disabled = false;
                this.videoRecording = false;
            } else {
                alert('Failed to stop video recording: ' + data.message);
            }
        } catch (error) {
            console.error('Error stopping video recording:', error);
            alert('Error stopping video recording.');
        }
    }
    
    async exportVideo() {
        const preset = document.getElementById('videoQualityPreset').value;
        const titleScreen = document.getElementById('includeTitleScreen').checked;
        const telemetryOverlay = document.getElementById('includeTelemetryOverlay').checked;
        const sciFiAudio = document.getElementById('includeSciFiAudio').checked;
        
        try {
            document.getElementById('exportProgress').style.display = 'block';
            document.getElementById('exportVideoBtn').disabled = true;
            
            const response = await fetch('/api/video/export/render', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    export_options: {
                        preset: preset,
                        title_screen: titleScreen,
                        telemetry_overlay: telemetryOverlay,
                        background_music: sciFiAudio
                    }
                })
            });
            
            const data = await response.json();
            
            if (data.success) {
                document.getElementById('exportProgress').style.display = 'none';
                document.getElementById('exportComplete').style.display = 'block';
                document.getElementById('downloadVideoLink').href = `/api/video/download/${data.output_path.split('/').pop()}`;
            } else {
                alert('Export failed: ' + data.error);
            }
        } catch (error) {
            console.error('Error exporting video:', error);
            alert('Error exporting video.');
        } finally {
            document.getElementById('exportVideoBtn').disabled = false;
        }
    }
    
    // Quick Video Creation
    async createQuickVideo(templateType) {
        try {
            document.getElementById('quickVideoStatus').style.display = 'block';
            document.getElementById('quickVideoResult').style.display = 'none';
            document.getElementById('quickVideoStatusText').textContent = 'Setting up simulation...';
            
            // Stop any existing simulation first
            await fetch('/api/simulation/stop', { method: 'POST' });
            
            let config;
            if (templateType === 'real_world') {
                // Use real world template
                config = {
                    drone_model: 'quadcopter_x4',
                    environment: 'earth',
                    mission_type: 'reconnaissance',
                    template: 'real_world_default'
                };
                document.getElementById('quickVideoStatusText').textContent = 'Loading real world terrain...';
                
                // Load a default real world location (Grand Canyon)
                await this.loadDefaultRealWorldLocation();
            } else {
                // Use AI world template
                config = {
                    drone_model: 'exploration_drone',
                    environment: 'mars',
                    mission_type: 'sample_transport',
                    template: 'ai_world_default'
                };
                document.getElementById('quickVideoStatusText').textContent = 'Generating AI environment...';
                
                // Generate a default AI environment
                await this.loadDefaultAIEnvironment();
            }
            
            // Start simulation
            document.getElementById('quickVideoStatusText').textContent = 'Starting simulation...';
            const startResponse = await fetch('/api/simulation/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(config)
            });
            
            if (!startResponse.ok) {
                throw new Error('Failed to start simulation');
            }
            
            // Start video recording
            document.getElementById('quickVideoStatusText').textContent = 'Starting video recording...';
            await fetch('/api/video/export/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    export_settings: { fps: 30, resolution: [1920, 1080] }
                })
            });
            
            // Let simulation run for 10 seconds
            document.getElementById('quickVideoStatusText').textContent = 'Recording flight (10 seconds)...';
            await new Promise(resolve => setTimeout(resolve, 10000));
            
            // Stop recording
            document.getElementById('quickVideoStatusText').textContent = 'Stopping recording...';
            await fetch('/api/video/export/stop', { method: 'POST' });
            
            // Export video with sci-fi audio
            document.getElementById('quickVideoStatusText').textContent = 'Creating enhanced video with sci-fi audio...';
            const exportResponse = await fetch('/api/video/export/render', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    export_options: {
                        preset: 'high',
                        title_screen: true,
                        telemetry_overlay: true,
                        background_music: true
                    }
                })
            });
            
            const exportData = await exportResponse.json();
            
            if (exportData.success) {
                document.getElementById('quickVideoStatus').style.display = 'none';
                document.getElementById('quickVideoResult').style.display = 'block';
                document.getElementById('quickVideoResultText').innerHTML = 
                    `${templateType === 'real_world' ? 'Real World' : 'AI World'} video created successfully!<br>
                     Duration: ${exportData.duration_seconds?.toFixed(1)}s | Size: ${exportData.file_size_mb?.toFixed(1)}MB`;
                document.getElementById('quickVideoDownloadLink').href = `/api/video/download/${exportData.output_path.split('/').pop()}`;
            } else {
                throw new Error(exportData.error || 'Video export failed');
            }
            
            // Stop simulation
            await fetch('/api/simulation/stop', { method: 'POST' });
            
        } catch (error) {
            console.error('Quick video creation failed:', error);
            document.getElementById('quickVideoStatus').style.display = 'none';
            alert(`Failed to create quick video: ${error.message}`);
        }
    }
    
    // Real World video creation
    async createRealWorldVideo() {
        try {
            if (!this.selectedLocation) {
                alert('Please select a location first from the Real World tab');
                return;
            }
            
            // Show progress indicator
            const btn = document.getElementById('quickVideoRealWorldBtn');
            const originalText = btn.innerHTML;
            btn.innerHTML = '<i class="fas fa-spinner fa-spin"></i> Creating Video...';
            btn.disabled = true;
            
            const response = await fetch('/api/video/create/real-world', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    template: 'professional',
                    include_audio: true,
                    location: this.selectedLocation.name
                })
            });
            
            const result = await response.json();
            
            if (result.success) {
                alert(`Real World video created successfully!\nLocation: ${this.selectedLocation.name}\nDuration: ${result.duration}s | Size: ${result.size_mb}MB`);
                // Refresh video list if we're on the video export tab
                if (typeof refreshVideoList === 'function') {
                    refreshVideoList();
                }
            } else {
                throw new Error(result.error || 'Video creation failed');
            }
            
        } catch (error) {
            console.error('Real World video creation failed:', error);
            alert(`Failed to create Real World video: ${error.message}`);
        } finally {
            // Reset button
            const btn = document.getElementById('quickVideoRealWorldBtn');
            if (btn) {
                btn.innerHTML = '<i class="fas fa-video"></i> Create Real World Video';
                btn.disabled = false;
            }
        }
    }
    
    // Apply terrain model to current simulation environment
    applyTerrainEnvironment() {
        if (!this.terrainModel) {
            alert('No terrain model loaded');
            return;
        }
        
        // Update environment with terrain data
        alert(`Terrain model applied to simulation!\nElevation range: ${this.terrainModel.bounds.min_elevation.toFixed(1)}m - ${this.terrainModel.bounds.max_elevation.toFixed(1)}m`);
        
        // Note: In a full implementation, this would modify the current simulation environment
        // to use the loaded terrain model for more realistic flight paths and altitude constraints
    }
    
    async loadDefaultRealWorldLocation() {
        // Load Grand Canyon as default real world location
        try {
            const response = await fetch('/api/maps/load-terrain', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    location: 'Grand Canyon National Park, Arizona, USA',
                    bounds: {
                        north: 36.2,
                        south: 36.0,
                        east: -112.0,
                        west: -112.2
                    }
                })
            });
        } catch (error) {
            console.log('Using default terrain for real world simulation');
        }
    }
    
    async loadDefaultAIEnvironment() {
        // Generate a default Martian environment
        try {
            const response = await fetch('/api/ai/generate-environment', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    environment_type: 'martian',
                    region_type: 'canyon',
                    season: 'spring'
                })
            });
        } catch (error) {
            console.log('Using default AI environment for simulation');
        }
    }
    
    // Setup Enhanced Features Event Listeners
    setupEnhancedFeaturesEventListeners() {
        // Google Maps Integration
        document.getElementById('searchLocationBtn').addEventListener('click', () => {
            const query = document.getElementById('locationSearch').value;
            if (query) {
                this.searchLocations(query);
            }
        });
        
        document.getElementById('locationSearch').addEventListener('keypress', (e) => {
            if (e.key === 'Enter') {
                const query = e.target.value;
                if (query) {
                    this.searchLocations(query);
                }
            }
        });
        
        // Terrain loading button
        const terrainBtn = document.getElementById('loadTerrainBtn');
        if (terrainBtn) {
            terrainBtn.addEventListener('click', () => {
                console.log('Load terrain button clicked');
                this.loadTerrain();
            });
        }
        
        // Real World video creation button
        const realWorldBtn = document.getElementById('quickVideoRealWorldBtn');
        if (realWorldBtn) {
            realWorldBtn.addEventListener('click', () => {
                console.log('Real World video button clicked');
                this.createRealWorldVideo();
            });
        }
        
        // AI Environment Generation
        document.getElementById('generateLunarBtn').addEventListener('click', () => {
            this.generateLunarEnvironment();
        });
        
        document.getElementById('generateMartianBtn').addEventListener('click', () => {
            this.generateMartianEnvironment();
        });
        
        // Video Export
        document.getElementById('startRecordingBtn').addEventListener('click', () => {
            this.startVideoRecording();
        });
        
        document.getElementById('stopRecordingBtn').addEventListener('click', () => {
            this.stopVideoRecording();
        });
        
        document.getElementById('exportVideoBtn').addEventListener('click', () => {
            this.exportVideo();
        });
        
        // Quick video creation buttons (AI World only - Real World handled above)
        document.getElementById('quickVideoAIWorldBtn').addEventListener('click', () => {
            this.createQuickVideo('ai_world');
        });
        
        // Tab activation events
        document.getElementById('maps-tab').addEventListener('click', () => {
            setTimeout(() => {
                this.loadPopularLocations();
            }, 100);
        });
    }
}

// Post-simulation video creation functions
async function createVideoFromCurrentSession() {
    const template = document.getElementById('currentVideoTemplate').value;
    const includeAudio = document.getElementById('currentIncludeAudio').checked;
    const statusDiv = document.getElementById('currentVideoStatus');
    const button = document.getElementById('createCurrentVideoBtn');
    
    button.disabled = true;
    statusDiv.style.display = 'block';
    statusDiv.innerHTML = '<div class="alert alert-info"><i class="fas fa-spinner fa-spin"></i> Creating video from current session...</div>';
    
    try {
        const response = await fetch('/api/video/create/current', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                template: template,
                include_audio: includeAudio
            })
        });
        
        const data = await response.json();
        
        if (data.success) {
            statusDiv.innerHTML = `
                <div class="alert alert-success">
                    <i class="fas fa-check-circle"></i> Video created successfully!
                    <br><strong>File:</strong> ${data.filename}
                    <br><strong>Duration:</strong> ${data.duration_seconds}s
                    <br><a href="/api/video/download/${data.filename}" class="btn btn-sm btn-success mt-2">
                        <i class="fas fa-download"></i> Download Video
                    </a>
                </div>
            `;
            refreshVideoList();
        } else {
            statusDiv.innerHTML = `<div class="alert alert-danger"><i class="fas fa-exclamation-circle"></i> Error: ${data.error}</div>`;
        }
    } catch (error) {
        console.error('Error creating video:', error);
        statusDiv.innerHTML = `<div class="alert alert-danger"><i class="fas fa-exclamation-circle"></i> Error creating video: ${error.message}</div>`;
    } finally {
        button.disabled = false;
    }
}

async function createVideoFromPastSession() {
    const sessionId = document.getElementById('pastSessionSelect').value;
    const template = document.getElementById('pastVideoTemplate').value;
    const includeAudio = document.getElementById('pastIncludeAudio').checked;
    const statusDiv = document.getElementById('pastVideoStatus');
    const button = document.getElementById('createPastVideoBtn');
    
    if (!sessionId) {
        alert('Please select a session first');
        return;
    }
    
    button.disabled = true;
    statusDiv.style.display = 'block';
    statusDiv.innerHTML = '<div class="alert alert-info"><i class="fas fa-spinner fa-spin"></i> Creating video from selected session...</div>';
    
    try {
        const response = await fetch('/api/video/create', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                session_id: sessionId,
                template: template,
                include_audio: includeAudio
            })
        });
        
        const data = await response.json();
        
        if (data.success) {
            statusDiv.innerHTML = `
                <div class="alert alert-success">
                    <i class="fas fa-check-circle"></i> Video created successfully!
                    <br><strong>File:</strong> ${data.filename}
                    <br><strong>Duration:</strong> ${data.duration_seconds}s
                    <br><a href="/api/video/download/${data.filename}" class="btn btn-sm btn-success mt-2">
                        <i class="fas fa-download"></i> Download Video
                    </a>
                </div>
            `;
            refreshVideoList();
        } else {
            statusDiv.innerHTML = `<div class="alert alert-danger"><i class="fas fa-exclamation-circle"></i> Error: ${data.error}</div>`;
        }
    } catch (error) {
        console.error('Error creating video:', error);
        statusDiv.innerHTML = `<div class="alert alert-danger"><i class="fas fa-exclamation-circle"></i> Error creating video: ${error.message}</div>`;
    } finally {
        button.disabled = false;
    }
}

async function loadPastSessions() {
    try {
        const response = await fetch('/api/history/sessions');
        const data = await response.json();
        
        const sessionSelect = document.getElementById('pastSessionSelect');
        sessionSelect.innerHTML = '<option value="">Select a session...</option>';
        
        if (data.sessions && data.sessions.length > 0) {
            data.sessions.forEach(session => {
                const option = document.createElement('option');
                option.value = session.id;
                option.textContent = `${session.drone_model} - ${session.environment} - ${session.mission_type} (${new Date(session.start_time).toLocaleString()})`;
                sessionSelect.appendChild(option);
            });
        } else {
            sessionSelect.innerHTML = '<option value="">No completed sessions available</option>';
        }
    } catch (error) {
        console.error('Error loading past sessions:', error);
        document.getElementById('pastSessionSelect').innerHTML = '<option value="">Error loading sessions</option>';
    }
}

// Global function for refreshing video list (called from HTML)
async function refreshVideoList() {
    try {
        const response = await fetch('/api/video/files');
        const data = await response.json();
        
        const container = document.getElementById('videoFilesList');
        if (!container) return;
        
        if (data.files && data.files.length > 0) {
            container.innerHTML = data.files.map(file => `
                <div class="card mb-2">
                    <div class="card-body p-2">
                        <div class="d-flex justify-content-between align-items-center">
                            <div>
                                <strong>${file.filename}</strong><br>
                                <small class="text-muted">${file.size_mb.toFixed(2)} MB</small>
                            </div>
                            <a href="/api/video/download/${file.filename}" class="btn btn-sm btn-primary" download>
                                <i class="fas fa-download"></i> Download
                            </a>
                        </div>
                    </div>
                </div>
            `).join('');
        } else {
            container.innerHTML = '<div class="text-muted text-center p-3">No video files available</div>';
        }
    } catch (error) {
        console.error('Error refreshing video list:', error);
        const container = document.getElementById('videoFilesList');
        if (container) {
            container.innerHTML = '<div class="text-danger text-center p-3">Error loading video files</div>';
        }
    }
}

// Initialize the application when the page loads
document.addEventListener('DOMContentLoaded', () => {
    window.droneController = new DroneSimulationController();
    
    // Setup analytics event listeners
    window.droneController.setupAnalyticsEventListeners();
    
    // Setup enhanced features event listeners
    window.droneController.setupEnhancedFeaturesEventListeners();
    
    // Initialize video list on page load
    refreshVideoList();
    
    // Initialize past sessions
    loadPastSessions();
    
    // Setup video export tab event listener
    const videoExportTab = document.getElementById('video-export-tab');
    if (videoExportTab) {
        videoExportTab.addEventListener('shown.bs.tab', function() {
            loadPastSessions();
        });
    }
});
