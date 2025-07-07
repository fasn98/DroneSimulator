/**
 * Simple Working Drone Simulation Interface
 * Focused on reliability and basic functionality
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
        console.log('Initializing working drone simulation controller...');
        
        // Load configurations first
        await this.loadConfigurations();
        
        // Setup event listeners
        this.setupEventListeners();
        
        // Initialize empty charts
        this.initializeCharts();
        
        // Connect WebSocket
        this.connectWebSocket();
        
        // Start status polling
        this.startStatusPolling();
        
        console.log('Working drone simulation controller initialized');
    }

    async loadConfigurations() {
        try {
            // Load drone models
            const dronesResponse = await fetch('/api/configurations/drones');
            const drones = await dronesResponse.json();
            this.populateSelect('droneModel', Object.keys(drones));

            // Load environments
            const envsResponse = await fetch('/api/configurations/environments');
            const environments = await envsResponse.json();
            this.populateSelect('environment', Object.keys(environments));

            // Load missions
            const missionsResponse = await fetch('/api/configurations/missions');
            const missions = await missionsResponse.json();
            this.populateSelect('missionType', Object.keys(missions));
            
        } catch (error) {
            console.error('Error loading configurations:', error);
        }
    }

    populateSelect(selectId, options) {
        const select = document.getElementById(selectId);
        if (select) {
            select.innerHTML = '';
            options.forEach(option => {
                const optionElement = document.createElement('option');
                optionElement.value = option;
                optionElement.textContent = option;
                select.appendChild(optionElement);
            });
        }
    }

    setupEventListeners() {
        // Start button
        const startBtn = document.getElementById('startBtn');
        if (startBtn) {
            startBtn.addEventListener('click', () => this.startSimulation());
        }

        // Stop button
        const stopBtn = document.getElementById('stopBtn');
        if (stopBtn) {
            stopBtn.addEventListener('click', () => this.stopSimulation());
        }

        // Pause button
        const pauseBtn = document.getElementById('pauseBtn');
        if (pauseBtn) {
            pauseBtn.addEventListener('click', () => this.pauseSimulation());
        }
    }

    async startSimulation() {
        const config = {
            drone_model: document.getElementById('droneModel')?.value || 'default_quadrotor',
            environment: document.getElementById('environment')?.value || 'earth',
            mission_type: document.getElementById('missionType')?.value || 'reconnaissance'
        };
        
        try {
            const response = await fetch('/api/simulation/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(config)
            });

            if (response.ok) {
                this.clearSimulationData();
                this.showMessage('Simulation started successfully', 'success');
                this.updateButtonStates(true);
            } else {
                const errorData = await response.json();
                this.showMessage(`Failed to start: ${errorData.error}`, 'error');
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
        
        // Clear all plots
        try {
            if (document.getElementById('trajectoryPlot')) {
                Plotly.purge('trajectoryPlot');
                Plotly.newPlot('trajectoryPlot', [], { title: 'No Data Available' });
            }
        } catch (error) {
            console.error('Error clearing plots:', error);
        }
        
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
            this.handleTelemetryUpdate(data);
        });

        this.socket.on('status_update', (data) => {
            this.handleStatusUpdate(data);
        });
    }

    handleTelemetryUpdate(telemetry) {
        this.telemetryData.push(telemetry);
        
        // Keep only last 200 points for performance
        if (this.telemetryData.length > 200) {
            this.telemetryData = this.telemetryData.slice(-200);
        }
        
        this.updatePlots();
    }

    handleStatusUpdate(status) {
        this.simulationStatus = { ...this.simulationStatus, ...status };
        this.updateStatusDisplay();
    }

    updatePlots() {
        this.updateTrajectoryPlot();
    }

    updateTrajectoryPlot() {
        if (this.telemetryData.length === 0) return;
        
        try {
            const positions = this.telemetryData.map(d => ({
                x: d.position_x || 0,
                y: d.position_y || 0,
                z: d.position_z || 0
            }));

            const trace = {
                x: positions.map(p => p.x),
                y: positions.map(p => p.y),
                z: positions.map(p => p.z),
                type: 'scatter3d',
                mode: 'lines+markers',
                marker: { size: 3, color: 'blue' },
                line: { color: 'blue', width: 2 },
                name: 'Flight Path'
            };

            const layout = {
                title: '3D Flight Trajectory',
                scene: {
                    xaxis: { title: 'X (m)', range: [-50, 200] },
                    yaxis: { title: 'Y (m)', range: [-50, 250] },
                    zaxis: { title: 'Z (m)', range: [0, 100] },
                    camera: {
                        eye: { x: 1.5, y: 1.5, z: 1.5 }
                    }
                },
                margin: { l: 0, r: 0, b: 0, t: 50 }
            };

            Plotly.react('trajectoryPlot', [trace], layout);
        } catch (error) {
            console.error('Error updating trajectory plot:', error);
        }
    }

    async startStatusPolling() {
        this.statusPolling = setInterval(async () => {
            try {
                const response = await fetch('/api/simulation/status');
                const status = await response.json();
                
                // Update our simulation status
                this.simulationStatus.running = status.simulation_running || false;
                this.simulationStatus.progress = status.mission_progress || 0;
                this.simulationStatus.currentWaypoint = status.current_waypoint || 0;
                this.simulationStatus.position = status.position || [0, 0, 0];
                this.simulationStatus.velocity = status.velocity || [0, 0, 0];
                this.simulationStatus.attitude = status.attitude || [0, 0, 0];
                
                this.updateStatusDisplay();
                this.updateButtonStates(this.simulationStatus.running);
                
            } catch (error) {
                // Silent fail for polling
            }
        }, 1000);
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
            const trajectoryElement = document.getElementById('trajectoryPlot');
            if (trajectoryElement) {
                Plotly.newPlot('trajectoryPlot', [], { title: 'No Data Available' });
            }
        }, 100);
    }

    showMessage(message, type) {
        console.log(`${type.toUpperCase()}: ${message}`);
        
        // Try to show in UI if possible
        const alertClass = type === 'success' ? 'alert-success' : 
                          type === 'error' ? 'alert-danger' : 'alert-info';
        
        // Create simple alert
        const alertDiv = document.createElement('div');
        alertDiv.className = `alert ${alertClass} alert-dismissible fade show`;
        alertDiv.innerHTML = `
            ${message}
            <button type="button" class="btn-close" data-bs-dismiss="alert"></button>
        `;
        
        // Try to insert at top of main content
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