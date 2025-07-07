/**
 * Simplified Drone Simulation Web Interface - Fixed Axis Scaling
 * Removed all terrain zoom controls for clean, stable 3D trajectory visualization
 */

class DroneSimulationController {
    constructor() {
        this.telemetryData = [];
        this.missionData = [];
        this.simulationStatus = {
            running: false,
            progress: 0,
            currentWaypoint: 0,
            totalWaypoints: 0
        };
        this.socket = null;
        this.charts = {};
        this.statusPolling = null;
    }

    async init() {
        console.log('Initializing simplified drone simulation controller...');
        
        // Load configurations
        await this.loadConfigurations();
        
        // Setup event listeners
        this.setupEventListeners();
        
        // Load persisted data
        this.loadPersistedData();
        
        // Setup tab change handlers
        this.setupTabChangeHandlers();
        
        // Connect WebSocket
        this.connectWebSocket();
        
        // Initialize charts
        this.initializeCharts();
        
        // Start status polling
        this.startStatusPolling();
        
        console.log('Drone Simulation Controller initialized');
    }

    // Simple trajectory plot with FIXED axis scaling
    updateTrajectoryPlot() {
        if (this.telemetryData.length === 0) return;
        
        try {
            const x = this.telemetryData.map(d => d.position?.x || 0);
            const y = this.telemetryData.map(d => d.position?.y || 0);
            const z = this.telemetryData.map(d => d.position?.z || 0);
            
            const trajectoryData = [{
                x: x,
                y: y,
                z: z,
                type: 'scatter3d',
                mode: 'lines+markers',
                line: { color: 'blue', width: 4 },
                marker: { size: 3, color: z, colorscale: 'Viridis' },
                name: 'Flight Path'
            }];
            
            // Add current position marker
            if (x.length > 0) {
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
            
            // FIXED axis ranges - no dynamic scaling issues
            const layout = {
                title: '3D Flight Trajectory',
                scene: {
                    xaxis: { 
                        title: 'X (m)',
                        range: [-50, 200]
                    },
                    yaxis: { 
                        title: 'Y (m)',
                        range: [-50, 250]
                    },
                    zaxis: { 
                        title: 'Z (m)',
                        range: [0, 100]
                    },
                    camera: {
                        eye: { x: 1.5, y: 1.5, z: 1.5 },
                        center: { x: 0, y: 0, z: 0 },
                        up: { x: 0, y: 0, z: 1 }
                    },
                    aspectmode: 'cube'
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

    // Load configurations
    async loadConfigurations() {
        try {
            const [dronesResponse, environmentsResponse, missionsResponse] = await Promise.all([
                fetch('/api/configurations/drones'),
                fetch('/api/configurations/environments'),
                fetch('/api/configurations/missions')
            ]);

            const drones = await dronesResponse.json();
            const environments = await environmentsResponse.json();
            const missions = await missionsResponse.json();

            this.populateSelect('droneModel', Object.keys(drones));
            this.populateSelect('environment', Object.keys(environments));
            this.populateSelect('missionType', Object.keys(missions));
            
            console.log('Mission configs:', missions);
        } catch (error) {
            console.error('Error loading configurations:', error);
        }
    }

    populateSelect(selectId, options) {
        const select = document.getElementById(selectId);
        if (!select) return;
        
        select.innerHTML = '';
        options.forEach(option => {
            const optionElement = document.createElement('option');
            optionElement.value = option;
            optionElement.textContent = option;
            select.appendChild(optionElement);
        });
    }

    setupEventListeners() {
        const startBtn = document.getElementById('startBtn');
        const stopBtn = document.getElementById('stopBtn');
        const pauseBtn = document.getElementById('pauseBtn');

        if (startBtn) startBtn.addEventListener('click', () => this.startSimulation());
        if (stopBtn) stopBtn.addEventListener('click', () => this.stopSimulation());
        if (pauseBtn) pauseBtn.addEventListener('click', () => this.pauseSimulation());
    }

    setupTabChangeHandlers() {
        const tabButtons = document.querySelectorAll('[data-bs-toggle="tab"]');
        tabButtons.forEach(button => {
            button.addEventListener('shown.bs.tab', (event) => {
                const targetId = event.target.getAttribute('href')?.substring(1);
                setTimeout(() => {
                    switch (event.target.id) {
                        case 'trajectory-tab':
                            this.updateTrajectoryPlot();
                            break;
                        case 'telemetry-tab':
                            this.updateTelemetryPlots();
                            break;
                    }
                }, 100);
            });
        });
    }

    async startSimulation() {
        const config = this.getSimulationConfig();
        
        try {
            const response = await fetch('/api/simulation/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(config)
            });

            if (response.ok) {
                this.clearSimulationData();
                this.showSuccess('Simulation started successfully');
                
                // Update button states immediately
                const startBtn = document.getElementById('startBtn');
                const stopBtn = document.getElementById('stopBtn');
                const pauseBtn = document.getElementById('pauseBtn');
                
                if (startBtn) startBtn.disabled = true;
                if (stopBtn) stopBtn.disabled = false;
                if (pauseBtn) pauseBtn.disabled = false;
                
            } else {
                const errorData = await response.json();
                this.showError(`Failed to start simulation: ${errorData.error || 'Unknown error'}`);
            }
        } catch (error) {
            console.error('Error starting simulation:', error);
            this.showError('Error starting simulation');
        }
    }

    async stopSimulation() {
        try {
            const response = await fetch('/api/simulation/stop', { method: 'POST' });
            if (response.ok) {
                this.showInfo('Simulation stopped');
            }
        } catch (error) {
            console.error('Error stopping simulation:', error);
        }
    }

    async pauseSimulation() {
        try {
            const response = await fetch('/api/simulation/pause', { method: 'POST' });
            if (response.ok) {
                this.showInfo('Simulation paused/resumed');
            }
        } catch (error) {
            console.error('Error pausing simulation:', error);
        }
    }

    getSimulationConfig() {
        return {
            drone_model: document.getElementById('droneModel')?.value || 'quadcopter_x4',
            environment: document.getElementById('environment')?.value || 'earth',
            mission_type: document.getElementById('missionType')?.value || 'reconnaissance',
            duration: parseInt(document.getElementById('duration')?.value) || 600,
            timestep: parseFloat(document.getElementById('timestep')?.value) || 0.01,
            realtime: document.getElementById('realtime')?.checked || false
        };
    }

    clearSimulationData() {
        this.telemetryData = [];
        this.missionData = [];
        this.simulationStatus = { running: false, progress: 0, currentWaypoint: 0, totalWaypoints: 0 };
        
        // Clear localStorage
        localStorage.removeItem('telemetryData');
        localStorage.removeItem('missionData');
        localStorage.removeItem('simulationStatus');
        
        console.log('Cleared previous simulation data and charts');
    }

    connectWebSocket() {
        this.socket = io();
        
        this.socket.on('connect', () => {
            console.log('SocketIO connected');
            this.updateConnectionStatus(true);
        });

        this.socket.on('disconnect', () => {
            console.log('SocketIO disconnected');
            this.updateConnectionStatus(false);
        });

        this.socket.on('telemetry_update', (data) => {
            this.handleTelemetryUpdate(data);
        });

        this.socket.on('mission_event', (data) => {
            this.handleMissionEvent(data);
        });

        this.socket.on('status_update', (data) => {
            this.handleStatusUpdate(data);
        });
    }

    handleTelemetryUpdate(telemetry) {
        this.telemetryData.push(telemetry);
        
        // Keep only last 500 points
        if (this.telemetryData.length > 500) {
            this.telemetryData = this.telemetryData.slice(-500);
        }
        
        this.saveTelemetryToStorage();
        
        // Update status display
        this.updateStatusDisplay();
        
        // Update plots (with error handling)
        try {
            this.updateRealtimePlots();
        } catch (error) {
            console.error('Error in updateRealtimePlots:', error);
        }
    }

    handleMissionEvent(event) {
        this.missionData.push(event);
        this.saveMissionToStorage();
    }

    handleStatusUpdate(status) {
        this.simulationStatus = { ...this.simulationStatus, ...status };
        this.saveStatusToStorage();
        this.updateStatusDisplay();
        
        // Update real-time status display
        if (status.position) {
            const posElement = document.getElementById('dronePosition');
            if (posElement) {
                posElement.textContent = `${status.position[0].toFixed(1)}, ${status.position[1].toFixed(1)}, ${status.position[2].toFixed(1)}`;
            }
        }
        
        if (status.velocity) {
            const velElement = document.getElementById('droneVelocity');
            if (velElement) {
                velElement.textContent = `${status.velocity[0].toFixed(1)}, ${status.velocity[1].toFixed(1)}, ${status.velocity[2].toFixed(1)}`;
            }
        }
        
        if (status.attitude) {
            const attElement = document.getElementById('droneAttitude');
            if (attElement) {
                attElement.textContent = `${(status.attitude[0] * 180/Math.PI).toFixed(1)}°, ${(status.attitude[1] * 180/Math.PI).toFixed(1)}°, ${(status.attitude[2] * 180/Math.PI).toFixed(1)}°`;
            }
        }
    }

    updateRealtimePlots() {
        this.updateTrajectoryPlot();
        this.updateTelemetryPlots();
    }

    updateTelemetryPlots() {
        if (this.telemetryData.length === 0) return;
        
        try {
            const times = this.telemetryData.map(d => d.timestamp || 0);
            const altitudes = this.telemetryData.map(d => d.altitude || 0);
            const speeds = this.telemetryData.map(d => d.ground_speed || 0);
            const power = this.telemetryData.map(d => d.power_consumption || 0);
            const progress = this.telemetryData.map(d => d.mission_progress || 0);
            
            // Update plots that match the HTML structure
            const altitudeSpeedElement = document.getElementById('altitudeSpeedPlot');
            if (altitudeSpeedElement) {
                Plotly.react('altitudeSpeedPlot', [{
                    x: times,
                    y: altitudes,
                    type: 'scatter',
                    mode: 'lines',
                    name: 'Altitude',
                    line: { color: 'blue' },
                    yaxis: 'y'
                }, {
                    x: times,
                    y: speeds,
                    type: 'scatter',
                    mode: 'lines',
                    name: 'Speed',
                    line: { color: 'green' },
                    yaxis: 'y2'
                }], {
                    title: 'Altitude & Speed Over Time',
                    xaxis: { title: 'Time (s)' },
                    yaxis: { title: 'Altitude (m)', side: 'left' },
                    yaxis2: { title: 'Speed (m/s)', side: 'right', overlaying: 'y' }
                });
            }
            
            const powerElement = document.getElementById('powerPlot');
            if (powerElement) {
                Plotly.react('powerPlot', [{
                    x: times,
                    y: power,
                    type: 'scatter',
                    mode: 'lines',
                    name: 'Power Consumption',
                    line: { color: 'red' }
                }], {
                    title: 'Power Consumption Over Time',
                    xaxis: { title: 'Time (s)' },
                    yaxis: { title: 'Power (W)' }
                });
            }
            
            const missionElement = document.getElementById('missionProgressPlot');
            if (missionElement) {
                Plotly.react('missionProgressPlot', [{
                    x: times,
                    y: progress,
                    type: 'scatter',
                    mode: 'lines',
                    name: 'Mission Progress',
                    line: { color: 'purple' }
                }], {
                    title: 'Mission Progress Over Time',
                    xaxis: { title: 'Time (s)' },
                    yaxis: { title: 'Progress (%)' }
                });
            }
            
        } catch (error) {
            console.error('Error updating telemetry plots:', error);
        }
    }

    initializeCharts() {
        // Initialize empty charts only if elements exist
        const emptyLayout = { title: 'No Data Available' };
        
        setTimeout(() => {
            const trajectoryElement = document.getElementById('trajectoryPlot');
            if (trajectoryElement) {
                Plotly.newPlot('trajectoryPlot', [], emptyLayout);
            }
            
            const altitudeSpeedElement = document.getElementById('altitudeSpeedPlot');
            if (altitudeSpeedElement) {
                Plotly.newPlot('altitudeSpeedPlot', [], emptyLayout);
            }
            
            const powerElement = document.getElementById('powerPlot');
            if (powerElement) {
                Plotly.newPlot('powerPlot', [], emptyLayout);
            }
            
            const missionElement = document.getElementById('missionProgressPlot');
            if (missionElement) {
                Plotly.newPlot('missionProgressPlot', [], emptyLayout);
            }
            
            const realtimeElement = document.getElementById('realtimePlots');
            if (realtimeElement) {
                Plotly.newPlot('realtimePlots', [], emptyLayout);
            }
        }, 100);
    }

    async startStatusPolling() {
        this.statusPolling = setInterval(async () => {
            try {
                const response = await fetch('/api/simulation/status');
                const status = await response.json();
                this.handleStatusUpdate(status);
            } catch (error) {
                // Silent fail for polling
            }
        }, 2000);
    }

    updateStatusDisplay() {
        // Update simulation status
        const statusElement = document.getElementById('status');
        if (statusElement) {
            statusElement.textContent = this.simulationStatus.running ? 'Running' : 'Stopped';
            statusElement.className = `badge ${this.simulationStatus.running ? 'bg-success' : 'bg-secondary'}`;
        }

        // Update progress bar
        const progressElement = document.getElementById('progressBar');
        if (progressElement) {
            progressElement.style.width = `${this.simulationStatus.progress || 0}%`;
            progressElement.setAttribute('aria-valuenow', this.simulationStatus.progress || 0);
        }

        // Update button states
        const startBtn = document.getElementById('startBtn');
        const stopBtn = document.getElementById('stopBtn');
        const pauseBtn = document.getElementById('pauseBtn');

        if (this.simulationStatus.running) {
            if (startBtn) startBtn.disabled = true;
            if (stopBtn) stopBtn.disabled = false;
            if (pauseBtn) pauseBtn.disabled = false;
        } else {
            if (startBtn) startBtn.disabled = false;
            if (stopBtn) stopBtn.disabled = true;
            if (pauseBtn) pauseBtn.disabled = true;
        }
    }

    updateConnectionStatus(connected) {
        const statusElement = document.getElementById('connectionStatus');
        if (statusElement) {
            statusElement.textContent = connected ? 'Connected' : 'Disconnected';
            statusElement.className = `badge ${connected ? 'bg-success' : 'bg-danger'}`;
        }
        
        // Also update in header if exists
        const headerStatus = document.querySelector('.connection-status');
        if (headerStatus) {
            headerStatus.textContent = connected ? 'Connected' : 'Disconnected';
            headerStatus.className = `badge connection-status ${connected ? 'bg-success' : 'bg-danger'}`;
        }
    }

    // Persistence functions
    loadPersistedData() {
        try {
            const savedTelemetry = localStorage.getItem('telemetryData');
            const savedMission = localStorage.getItem('missionData');
            const savedStatus = localStorage.getItem('simulationStatus');

            if (savedTelemetry) {
                this.telemetryData = JSON.parse(savedTelemetry);
                console.log(`Loaded ${this.telemetryData.length} telemetry points from storage`);
            }

            if (savedMission) {
                this.missionData = JSON.parse(savedMission);
            }

            if (savedStatus) {
                this.simulationStatus = { ...this.simulationStatus, ...JSON.parse(savedStatus) };
            }

            // Update charts with persisted data
            setTimeout(() => {
                this.updateRealtimePlots();
            }, 500);
        } catch (error) {
            console.error('Error loading persisted data:', error);
        }
    }

    saveTelemetryToStorage() {
        try {
            localStorage.setItem('telemetryData', JSON.stringify(this.telemetryData.slice(-500)));
        } catch (error) {
            console.error('Error saving telemetry data:', error);
        }
    }

    saveMissionToStorage() {
        try {
            localStorage.setItem('missionData', JSON.stringify(this.missionData));
        } catch (error) {
            console.error('Error saving mission data:', error);
        }
    }

    saveStatusToStorage() {
        try {
            localStorage.setItem('simulationStatus', JSON.stringify(this.simulationStatus));
        } catch (error) {
            console.error('Error saving status data:', error);
        }
    }

    // Utility functions
    showError(message) {
        this.showNotification(message, 'danger');
    }

    showSuccess(message) {
        this.showNotification(message, 'success');
    }

    showInfo(message) {
        this.showNotification(message, 'info');
    }

    showNotification(message, type) {
        const alertHtml = `
            <div class="alert alert-${type} alert-dismissible fade show" role="alert">
                ${message}
                <button type="button" class="btn-close" data-bs-dismiss="alert"></button>
            </div>
        `;
        
        const container = document.getElementById('notifications') || document.body;
        container.insertAdjacentHTML('afterbegin', alertHtml);
        
        // Auto dismiss after 5 seconds
        setTimeout(() => {
            const alert = container.querySelector('.alert');
            if (alert) alert.remove();
        }, 5000);
    }
}

// Initialize when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    const controller = new DroneSimulationController();
    controller.init();
});