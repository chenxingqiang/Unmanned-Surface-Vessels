/**
 * Main Application JavaScript
 * Handles socket.io connection and coordinates all components
 */

// Global variables
let socket;
let simulationRunning = false;
let simulationPaused = false;
let simulationSpeed = 1.0;
let obstacleAvoidanceEnabled = true;  // New variable for obstacle avoidance state
let simulationResults = {
    data: []
};

/**
 * Main application initialization
 */
document.addEventListener('DOMContentLoaded', function() {
    // Initialize charts
    if (typeof initCharts === 'function') {
        initCharts();
    } else {
        console.error('Charts initialization function not found');
    }
    
    // Initialize 3D visualization
    if (typeof initVisualization === 'function') {
        initVisualization();
    } else {
        console.error('Visualization initialization function not found');
    }
    
    // Initialize UI
    initUI();
    
    // Initialize socket connection
    initSocket();

    // Add a debug message
    console.log('Page initialization complete. Ready to simulate.');
    
    // Automatically start simulation after a short delay to ensure everything is loaded
    setTimeout(function() {
        // Auto-start simulation if not already running
        if (!simulationRunning && document.getElementById('start-button') && !document.getElementById('start-button').disabled) {
            console.log('Auto-starting simulation...');
            startSimulation();
        }
    }, 1500);
});

/**
 * Initialize Socket.IO connection
 */
function initSocket() {
    // Get the current page URL and port
    const protocol = window.location.protocol;
    const host = window.location.hostname || 'localhost';
    const port = window.location.port || '5000';
    
    // Create URL for Socket.IO connection
    const socketUrl = `${protocol}//${host}:${port}`;
    console.log(`Initializing Socket.IO connection to ${socketUrl}`);
    
    // Initialize socket with robust error handling
    socket = io(socketUrl, {
        transports: ['websocket', 'polling'],
        reconnectionAttempts: 5,
        reconnectionDelay: 1000,
        timeout: 10000
    });
    
    // Connection event handlers
    socket.on('connect', function() {
        console.log('Socket.IO connected successfully');
        console.log('Socket ID:', socket.id);
        
        // Update connection status
        updateConnectionStatus('connected');
        
        // Request initial state on connection
        socket.emit('get_simulation_state');
    });
    
    socket.on('disconnect', function() {
        console.log('Socket.IO disconnected');
        updateConnectionStatus('disconnected');
    });
    
    socket.on('connect_error', function(error) {
        console.error('Socket.IO connection error:', error);
        updateStatusUI('error');
        showError(`Connection error: ${error.message || 'Unknown error'}`);
        
        // Try to reconnect manually after a delay
        setTimeout(function() {
            console.log('Attempting to reconnect manually...');
            socket.connect();
        }, 2000);
    });
    
    // Server event handlers
    socket.on('connection_response', function(data) {
        console.log('Server connection response:', data);
    });
    
    // Handle simulation status updates
    socket.on('simulation_status', function(data) {
        console.log('Received simulation status:', data);
        updateSimulationStatus(data);
    });
    
    // Handle simulation data with validation and error handling
    socket.on('simulation_data', function(data) {
        try {
            // Simple sanity check on data
            if (!data || typeof data !== 'object') {
                console.error('Invalid data received:', data);
                return;
            }
            
            // Basic log to monitor simulation data flow
            console.log(`Simulation data received - time: ${data.time}`);
            
            // Update visualization
            if (typeof updateVisualization === 'function') {
                updateVisualization(data);
            }
            
            // Update charts
            if (typeof updateCharts === 'function') {
                updateCharts(data);
            }
            
            // Update UI elements
            updateSimulationUI(data);
            
        } catch (error) {
            console.error('Error processing simulation data:', error);
        }
    });
    
    // Handle simulation state
    socket.on('simulation_state', function(data) {
        console.log('Received simulation state:', data);
        if (data.status === 'initialized') {
            updateSimulationStateUI(data);
            
            // Set obstacle avoidance enabled status from server
            if (data.obstacle_avoidance !== undefined) {
                obstacleAvoidanceEnabled = data.obstacle_avoidance;
                updateObstacleAvoidanceUI(obstacleAvoidanceEnabled);
            }
        }
    });
    
    // Handle obstacle avoidance status updates
    socket.on('obstacle_avoidance_status', function(data) {
        console.log('Received obstacle avoidance status:', data);
        obstacleAvoidanceEnabled = data.enabled;
        updateObstacleAvoidanceUI(obstacleAvoidanceEnabled);
    });
    
    // Return socket instance for potential external use
    return socket;
}

/**
 * Initialize UI event handlers
 */
function initUI() {
    // Start button
    document.getElementById('start-button').addEventListener('click', function() {
        startSimulation();
    });
    
    // Pause button
    document.getElementById('pause-button').addEventListener('click', function() {
        if (simulationPaused) {
            resumeSimulation();
        } else {
            pauseSimulation();
        }
    });
    
    // Stop button
    document.getElementById('stop-button').addEventListener('click', function() {
        stopSimulation();
    });
    
    // Simulation speed slider
    document.getElementById('speed-slider').addEventListener('input', function() {
        simulationSpeed = parseFloat(this.value);
        document.getElementById('speed-value').textContent = simulationSpeed.toFixed(1) + 'x';
    });
    
    // View mode buttons
    document.getElementById('view-top').addEventListener('click', function() {
        setViewMode('top');
        setActiveViewButton(this);
    });
    
    document.getElementById('view-follow').addEventListener('click', function() {
        setViewMode('follow');
        setActiveViewButton(this);
    });
    
    document.getElementById('view-free').addEventListener('click', function() {
        setViewMode('free');
        setActiveViewButton(this);
    });
    
    // Add obstacle avoidance toggle to the UI
    addObstacleAvoidanceToggle();
    
    // Add download results button to the UI
    addDownloadResultsButton();
}

/**
 * Add obstacle avoidance toggle button to the UI
 */
function addObstacleAvoidanceToggle() {
    // Check if we already have the control panel
    let controlPanel = document.querySelector('.card-body form');
    if (!controlPanel) return;
    
    // Create toggle switch for obstacle avoidance
    const toggleDiv = document.createElement('div');
    toggleDiv.className = 'mb-3 form-check form-switch';
    toggleDiv.innerHTML = `
        <input class="form-check-input" type="checkbox" id="obstacle-avoidance-toggle" ${obstacleAvoidanceEnabled ? 'checked' : ''}>
        <label class="form-check-label" for="obstacle-avoidance-toggle">Obstacle Avoidance</label>
    `;
    
    // Add toggle before the button group
    const buttonGroup = document.querySelector('.d-grid.gap-2');
    if (buttonGroup && controlPanel.contains(buttonGroup)) {
        controlPanel.insertBefore(toggleDiv, buttonGroup);
    } else {
        // Fallback: just append to the form
        controlPanel.appendChild(toggleDiv);
    }
    
    // Add event listener
    document.getElementById('obstacle-avoidance-toggle').addEventListener('change', function() {
        obstacleAvoidanceEnabled = this.checked;
        socket.emit('toggle_obstacle_avoidance', { enabled: obstacleAvoidanceEnabled });
    });
}

/**
 * Update obstacle avoidance UI based on status
 */
function updateObstacleAvoidanceUI(enabled) {
    const toggle = document.getElementById('obstacle-avoidance-toggle');
    if (toggle) {
        toggle.checked = enabled;
    }
}

/**
 * Set the active view button
 */
function setActiveViewButton(button) {
    // Remove active class from all buttons
    document.querySelectorAll('.btn-group .btn').forEach(function(btn) {
        btn.classList.remove('active');
    });
    
    // Add active class to the clicked button
    button.classList.add('active');
}

/**
 * Load available scenarios from the server
 */
function loadScenarios() {
    fetch('/api/scenarios')
        .then(response => response.json())
        .then(data => {
            const scenarioSelect = document.getElementById('scenario-select');
            scenarioSelect.innerHTML = '';
            
            data.scenarios.forEach(scenario => {
                const option = document.createElement('option');
                option.value = scenario;
                option.textContent = scenario.charAt(0).toUpperCase() + scenario.slice(1).replace(/_/g, ' ');
                scenarioSelect.appendChild(option);
            });
            
            // Set default selection to 'obstacle' if available
            if (data.scenarios.includes('obstacle')) {
                scenarioSelect.value = 'obstacle';
            }
        })
        .catch(error => console.error('Error loading scenarios:', error));
}

/**
 * Load available controllers from the server
 */
function loadControllers() {
    fetch('/api/controllers')
        .then(response => response.json())
        .then(data => {
            const controllerSelect = document.getElementById('controller-select');
            controllerSelect.innerHTML = '';
            
            data.controllers.forEach(controller => {
                const option = document.createElement('option');
                option.value = controller;
                option.textContent = controller.toUpperCase() + ' Controller';
                controllerSelect.appendChild(option);
            });
        })
        .catch(error => console.error('Error loading controllers:', error));
}

/**
 * Start the simulation
 */
function startSimulation() {
    // Get selected scenario and controller
    const scenario = document.getElementById('scenario-select').value;
    const controller = document.getElementById('controller-select').value;
    
    // Reset visualization and charts for new simulation
    resetVisualization();
    resetCharts();
    
    // Initialize simulation results collection
    simulationResults = {
        scenario: scenario,
        controller: controller,
        obstacle_avoidance: obstacleAvoidanceEnabled,
        timestamp: new Date().toISOString(),
        data: []
    };
    
    // Reset download button
    const downloadButton = document.getElementById('download-results-button');
    if (downloadButton) {
        downloadButton.disabled = true;
    }
    
    // Emit start simulation event with obstacle avoidance setting
    socket.emit('start_simulation', {
        scenario: scenario,
        controller: controller,
        use_obstacle_avoidance: obstacleAvoidanceEnabled
    });
    
    // Update UI
    simulationRunning = true;
    simulationPaused = false;
    updateButtonStates();
}

/**
 * Pause the simulation
 */
function pauseSimulation() {
    socket.emit('pause_simulation');
    simulationPaused = true;
    document.getElementById('pause-button').innerHTML = '<i class="fas fa-play me-2"></i>Resume';
}

/**
 * Resume the simulation
 */
function resumeSimulation() {
    socket.emit('resume_simulation');
    simulationPaused = false;
    document.getElementById('pause-button').innerHTML = '<i class="fas fa-pause me-2"></i>Pause';
}

/**
 * Stop the simulation
 */
function stopSimulation() {
    socket.emit('stop_simulation');
    simulationRunning = false;
    simulationPaused = false;
    updateButtonStates();
}

/**
 * Update simulation status based on server response
 */
function updateSimulationStatus(data) {
    switch (data.status) {
        case 'started':
            simulationRunning = true;
            simulationPaused = false;
            break;
        case 'paused':
            simulationPaused = true;
            break;
        case 'resumed':
            simulationPaused = false;
            break;
        case 'stopped':
        case 'completed':
            simulationRunning = false;
            simulationPaused = false;
            // Enable download button if we have data
            const downloadButton = document.getElementById('download-results-button');
            if (downloadButton && simulationResults.data.length > 0) {
                downloadButton.disabled = false;
            }
            break;
        case 'error':
            simulationRunning = false;
            simulationPaused = false;
            showError('Simulation error: ' + data.message);
            break;
    }
    
    updateButtonStates();
}

/**
 * Update button states based on simulation status
 */
function updateButtonStates() {
    const startButton = document.getElementById('start-button');
    const pauseButton = document.getElementById('pause-button');
    const stopButton = document.getElementById('stop-button');
    
    if (simulationRunning) {
        startButton.disabled = true;
        pauseButton.disabled = false;
        stopButton.disabled = false;
        document.getElementById('scenario-select').disabled = true;
        document.getElementById('controller-select').disabled = true;
    } else {
        startButton.disabled = false;
        pauseButton.disabled = true;
        stopButton.disabled = true;
        document.getElementById('scenario-select').disabled = false;
        document.getElementById('controller-select').disabled = false;
        
        // Reset pause button text
        pauseButton.innerHTML = '<i class="fas fa-pause me-2"></i>Pause';
    }
}

/**
 * Update UI with simulation data
 */
function updateSimulationUI(data) {
    // Store simulation data for export
    if (simulationResults && simulationResults.data) {
        // Only keep essential data to avoid extremely large files
        const essentialData = {
            time: data.time,
            state: { ...data.state },
            reference: { ...data.reference },
            control: { ...data.control },
            metrics: { ...data.metrics },
            progress: { ...data.progress }
        };
        
        // If obstacles exist, store them in a condensed format
        if (data.obstacles) {
            essentialData.obstacles = {
                static_count: data.obstacles.static ? data.obstacles.static.length : 0,
                dynamic_count: data.obstacles.dynamic ? data.obstacles.dynamic.length : 0
            };
        }
        
        // Add to results
        simulationResults.data.push(essentialData);
    }
    
    // Update progress bar
    if (data.progress) {
        const progressBar = document.getElementById('progress-bar');
        const progress = data.progress.percentage.toFixed(1);
        progressBar.style.width = progress + '%';
        progressBar.textContent = progress + '%';
    }
    
    // Update simulation time
    if (data.time !== undefined) {
        document.getElementById('simulation-time').textContent = data.time.toFixed(1) + ' s';
    }
    
    // Update metrics
    if (data.metrics) {
        document.getElementById('path-error').textContent = data.metrics.path_error.toFixed(2) + ' m';
        document.getElementById('heading-error').textContent = (data.metrics.heading_error * (180/Math.PI)).toFixed(2) + ' deg';
    }
    
    // Update obstacle information if present
    if (data.obstacles) {
        updateObstacleDisplay(data.obstacles);
    }
}

/**
 * Update obstacle information display
 */
function updateObstacleDisplay(obstacles) {
    // Check if we need to create the obstacle info section
    let obstacleInfoDiv = document.getElementById('obstacle-info');
    
    if (!obstacleInfoDiv) {
        // Create obstacle info section
        const statusCard = document.querySelector('.card-header.bg-info');
        if (!statusCard) return;
        
        const cardBody = statusCard.closest('.card').querySelector('.card-body');
        if (!cardBody) return;
        
        obstacleInfoDiv = document.createElement('div');
        obstacleInfoDiv.id = 'obstacle-info';
        obstacleInfoDiv.className = 'mb-3';
        obstacleInfoDiv.innerHTML = `
            <label class="form-label">Obstacles</label>
            <div class="form-control" style="height: auto;">
                <div id="static-obstacles"><b>Static:</b> 0</div>
                <div id="dynamic-obstacles"><b>Dynamic:</b> 0</div>
                <div id="collision-detection" class="mt-2"></div>
            </div>
        `;
        
        // Append to card body
        cardBody.appendChild(obstacleInfoDiv);
    }
    
    // Update obstacle counts
    const staticCount = obstacles.static ? obstacles.static.length : 0;
    const dynamicCount = obstacles.dynamic ? obstacles.dynamic.length : 0;
    
    document.getElementById('static-obstacles').innerHTML = `<b>Static:</b> ${staticCount}`;
    document.getElementById('dynamic-obstacles').innerHTML = `<b>Dynamic:</b> ${dynamicCount}`;
    
    // Check for collision detection
    const collisionDetectionDiv = document.getElementById('collision-detection');
    if (collisionDetectionDiv) {
        // Check for possible collisions
        let nearbyObstacles = 0;
        let collisionDetected = false;
        
        // Check static obstacles (if visualization data is available)
        if (obstacles.visualization && obstacles.visualization.collision_detection) {
            const collisionData = obstacles.visualization.collision_detection;
            nearbyObstacles = collisionData.nearby_obstacles || 0;
            collisionDetected = collisionData.collision_detected || false;
        } else {
            // Simple distance-based check (if no collision data is available)
            if (obstacles.static) {
                for (const obstacle of obstacles.static) {
                    // For demonstration - just check if USV is within 10m of any obstacle
                    if (obstacle.distance !== undefined && obstacle.distance < 10) {
                        nearbyObstacles++;
                        if (obstacle.distance < 3) {
                            collisionDetected = true;
                        }
                    }
                }
            }
            
            if (obstacles.dynamic) {
                for (const obstacle of obstacles.dynamic) {
                    if (obstacle.distance !== undefined && obstacle.distance < 10) {
                        nearbyObstacles++;
                        if (obstacle.distance < 3) {
                            collisionDetected = true;
                        }
                    }
                }
            }
        }
        
        // Update collision detection display
        if (collisionDetected) {
            collisionDetectionDiv.innerHTML = `
                <div class="alert alert-danger p-1 m-0">
                    <i class="fas fa-exclamation-triangle me-1"></i>
                    <b>Collision Risk Detected!</b>
                </div>
            `;
        } else if (nearbyObstacles > 0) {
            collisionDetectionDiv.innerHTML = `
                <div class="alert alert-warning p-1 m-0">
                    <i class="fas fa-exclamation-circle me-1"></i>
                    <b>Nearby Obstacles:</b> ${nearbyObstacles}
                </div>
            `;
        } else {
            collisionDetectionDiv.innerHTML = `
                <div class="alert alert-success p-1 m-0">
                    <i class="fas fa-check-circle me-1"></i>
                    <b>Path Clear</b>
                </div>
            `;
        }
    }
}

/**
 * Update UI with simulation state
 */
function updateSimulationStateUI(data) {
    // Update selected scenario and controller
    document.getElementById('scenario-select').value = data.scenario;
    document.getElementById('controller-select').value = data.controller;
    
    // Update simulation status
    simulationRunning = data.running;
    simulationPaused = data.paused;
    updateButtonStates();
}

/**
 * Update UI status based on connection state
 */
function updateStatusUI(status) {
    console.log('Connection status:', status);
    
    // Create status indicator if it doesn't exist
    let statusIndicator = document.getElementById('connection-status');
    if (!statusIndicator) {
        statusIndicator = document.createElement('div');
        statusIndicator.id = 'connection-status';
        statusIndicator.style.position = 'fixed';
        statusIndicator.style.bottom = '10px';
        statusIndicator.style.right = '10px';
        statusIndicator.style.padding = '5px 10px';
        statusIndicator.style.borderRadius = '5px';
        statusIndicator.style.fontSize = '12px';
        statusIndicator.style.fontWeight = 'bold';
        statusIndicator.style.zIndex = '9999';
        document.body.appendChild(statusIndicator);
    }
    
    // Update status indicator appearance
    switch (status) {
        case 'connected':
            statusIndicator.textContent = 'Connected';
            statusIndicator.style.backgroundColor = '#28a745';
            statusIndicator.style.color = 'white';
            break;
        case 'disconnected':
            statusIndicator.textContent = 'Disconnected';
            statusIndicator.style.backgroundColor = '#dc3545';
            statusIndicator.style.color = 'white';
            break;
        case 'connecting':
            statusIndicator.textContent = 'Connecting...';
            statusIndicator.style.backgroundColor = '#ffc107';
            statusIndicator.style.color = 'black';
            break;
        case 'error':
            statusIndicator.textContent = 'Connection Error';
            statusIndicator.style.backgroundColor = '#dc3545';
            statusIndicator.style.color = 'white';
            break;
        default:
            statusIndicator.textContent = status;
            statusIndicator.style.backgroundColor = '#6c757d';
            statusIndicator.style.color = 'white';
    }
}

/**
 * Show error message to the user
 */
function showError(message) {
    console.error(message);
    
    // Create alert if it doesn't exist
    if (!document.getElementById('error-alert')) {
        const alertDiv = document.createElement('div');
        alertDiv.id = 'error-alert';
        alertDiv.className = 'alert alert-danger alert-dismissible fade show';
        alertDiv.role = 'alert';
        alertDiv.style.position = 'fixed';
        alertDiv.style.top = '10px';
        alertDiv.style.left = '50%';
        alertDiv.style.transform = 'translateX(-50%)';
        alertDiv.style.zIndex = '9999';
        
        const closeButton = document.createElement('button');
        closeButton.type = 'button';
        closeButton.className = 'btn-close';
        closeButton.setAttribute('data-bs-dismiss', 'alert');
        closeButton.setAttribute('aria-label', 'Close');
        
        alertDiv.appendChild(document.createTextNode(message));
        alertDiv.appendChild(closeButton);
        
        document.body.appendChild(alertDiv);
        
        // Auto-hide after 5 seconds
        setTimeout(() => {
            if (alertDiv.parentNode) {
                alertDiv.parentNode.removeChild(alertDiv);
            }
        }, 5000);
    } else {
        // Update existing alert
        const alertDiv = document.getElementById('error-alert');
        alertDiv.textContent = message;
        
        // Append close button again
        const closeButton = document.createElement('button');
        closeButton.type = 'button';
        closeButton.className = 'btn-close';
        closeButton.setAttribute('data-bs-dismiss', 'alert');
        closeButton.setAttribute('aria-label', 'Close');
        alertDiv.appendChild(closeButton);
    }
}

/**
 * Observe DOM changes using modern MutationObserver API
 * This is the proper way to watch for DOM changes instead of using deprecated DOMNodeInserted events
 * 
 * @param {Element} targetNode - The DOM element to observe
 * @param {Function} callback - Callback function when changes are detected
 * @param {Object} options - Configuration options for the observer
 * @returns {MutationObserver} - The observer instance
 */
function observeDomChanges(targetNode, callback, options = {}) {
    // Default configuration watches for new nodes and attribute changes
    const defaultOptions = {
        childList: true,     // Watch for changes to child elements
        subtree: true,       // Watch the entire subtree
        attributes: true     // Watch for attribute changes
    };
    
    // Create a MutationObserver instance
    const observer = new MutationObserver((mutationsList) => {
        // Call the callback with the mutations
        callback(mutationsList);
    });
    
    // Start observing with merged options
    observer.observe(targetNode, {...defaultOptions, ...options});
    
    // Return the observer so it can be disconnected later if needed
    return observer;
}

/**
 * Add download results button to the UI 
 */
function addDownloadResultsButton() {
    // Check if we already have the button group
    const buttonGroup = document.querySelector('.d-grid.gap-2');
    if (!buttonGroup) return;
    
    // Create download button
    const downloadButton = document.createElement('button');
    downloadButton.type = 'button';
    downloadButton.className = 'btn btn-secondary';
    downloadButton.id = 'download-results-button';
    downloadButton.innerHTML = '<i class="fas fa-download me-2"></i>Download Results';
    downloadButton.disabled = true; // Initially disabled until simulation is run
    
    // Add button to the button group
    buttonGroup.appendChild(downloadButton);
    
    // Add event listener
    downloadButton.addEventListener('click', function() {
        downloadSimulationResults();
    });
}

/**
 * Collect and download simulation results
 */
function downloadSimulationResults() {
    // If no data, show error
    if (simulationResults.data.length === 0) {
        showError('No simulation data available to download');
        return;
    }
    
    // Prepare filename with timestamp
    const timestamp = simulationResults.timestamp || new Date().toISOString().replace(/:/g, '-');
    const scenario = simulationResults.scenario || 'unknown';
    const controller = simulationResults.controller || 'unknown';
    const filename = `usv_simulation_${scenario}_${controller}_${timestamp}.json`;
    
    // Convert data to JSON
    const jsonData = JSON.stringify(simulationResults, null, 2);
    
    // Create download link
    const blob = new Blob([jsonData], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    
    // Trigger download
    document.body.appendChild(a);
    a.click();
    
    // Cleanup
    setTimeout(function() {
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    }, 0);
    
    console.log(`Downloaded simulation results as ${filename}`);
}

/**
 * Update connection status in UI
 */
function updateConnectionStatus(status) {
    const statusElement = document.getElementById('connection-status');
    if (!statusElement) {
        // Create status element if it doesn't exist
        const navbar = document.querySelector('.navbar-nav');
        if (navbar) {
            const statusItem = document.createElement('li');
            statusItem.className = 'nav-item ms-3';
            statusItem.innerHTML = '<span id="connection-status" class="badge rounded-pill"></span>';
            navbar.appendChild(statusItem);
        }
    }
    
    // Get the status element (now it should exist)
    const connectionStatus = document.getElementById('connection-status');
    if (connectionStatus) {
        // Update the status indicator
        switch (status) {
            case 'connected':
                connectionStatus.className = 'badge rounded-pill bg-success';
                connectionStatus.textContent = 'Connected';
                console.log('Connection status updated: connected');
                break;
            case 'disconnected':
                connectionStatus.className = 'badge rounded-pill bg-danger';
                connectionStatus.textContent = 'Disconnected';
                break;
            case 'connecting':
                connectionStatus.className = 'badge rounded-pill bg-warning';
                connectionStatus.textContent = 'Connecting...';
                break;
            default:
                connectionStatus.className = 'badge rounded-pill bg-secondary';
                connectionStatus.textContent = 'Unknown';
        }
    }
    
    // Update global connection status
    document.querySelectorAll('.connection-dependent').forEach(element => {
        if (status === 'connected') {
            element.classList.remove('disabled');
        } else {
            element.classList.add('disabled');
        }
    });
} 