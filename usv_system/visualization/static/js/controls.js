/**
 * Controls.js - UI control handling
 */

// Event handlers specific to controls not covered in the main app.js

/**
 * Add event listeners for keyboard shortcuts
 */
document.addEventListener('keydown', function(event) {
    // Space key to toggle simulation pause/resume
    if (event.code === 'Space' && simulationRunning) {
        event.preventDefault();
        if (simulationPaused) {
            resumeSimulation();
        } else {
            pauseSimulation();
        }
    }
    
    // Escape key to stop simulation
    if (event.code === 'Escape' && simulationRunning) {
        event.preventDefault();
        stopSimulation();
    }
    
    // '1' key for top view
    if (event.code === 'Digit1') {
        event.preventDefault();
        setViewMode('top');
        setActiveViewButton(document.getElementById('view-top'));
    }
    
    // '2' key for follow view
    if (event.code === 'Digit2') {
        event.preventDefault();
        setViewMode('follow');
        setActiveViewButton(document.getElementById('view-follow'));
    }
    
    // '3' key for free view
    if (event.code === 'Digit3') {
        event.preventDefault();
        setViewMode('free');
        setActiveViewButton(document.getElementById('view-free'));
    }
});

/**
 * Add event listeners for speed slider
 */
document.addEventListener('DOMContentLoaded', function() {
    // Speed slider input event - continuous update while sliding
    const speedSlider = document.getElementById('speed-slider');
    const speedValue = document.getElementById('speed-value');
    
    speedSlider.addEventListener('input', function() {
        const value = parseFloat(this.value).toFixed(1);
        speedValue.textContent = value + 'x';
        
        // Update global simulation speed
        simulationSpeed = parseFloat(value);
        
        // Send speed update to server when implemented
        // socket.emit('update_simulation_speed', { speed: simulationSpeed });
    });
    
    // Reset charts when simulation starts
    document.getElementById('start-button').addEventListener('click', function() {
        // Reset charts data only if the resetCharts function exists
        if (typeof resetCharts === 'function') {
            resetCharts();
        }
    });
    
    // Add scenario details expander if it exists
    const scenarioDetails = document.getElementById('scenario-details-btn');
    if (scenarioDetails) {
        scenarioDetails.addEventListener('click', function() {
            const detailsContainer = document.getElementById('scenario-details-container');
            if (detailsContainer.style.display === 'none') {
                detailsContainer.style.display = 'block';
                this.innerHTML = '<i class="fas fa-chevron-up me-2"></i>Hide Details';
            } else {
                detailsContainer.style.display = 'none';
                this.innerHTML = '<i class="fas fa-chevron-down me-2"></i>Show Details';
            }
        });
    }
    
    // Add tooltips to all elements with data-tooltip attribute
    document.querySelectorAll('[data-tooltip]').forEach(element => {
        const tooltip = document.createElement('div');
        tooltip.className = 'tooltip';
        tooltip.textContent = element.getAttribute('data-tooltip');
        tooltip.style.display = 'none';
        document.body.appendChild(tooltip);
        
        element.addEventListener('mouseenter', function(e) {
            const rect = element.getBoundingClientRect();
            tooltip.style.display = 'block';
            tooltip.style.left = rect.left + window.scrollX + 'px';
            tooltip.style.top = rect.bottom + window.scrollY + 5 + 'px';
        });
        
        element.addEventListener('mouseleave', function() {
            tooltip.style.display = 'none';
        });
    });
    
    // Handle scenario selection changes
    const scenarioSelect = document.getElementById('scenario-select');
    scenarioSelect.addEventListener('change', function() {
        // Here you could load scenario-specific details if needed
        updateScenarioInfo(this.value);
    });
    
    // Handle controller selection changes
    const controllerSelect = document.getElementById('controller-select');
    controllerSelect.addEventListener('change', function() {
        // Here you could load controller-specific details if needed
        updateControllerInfo(this.value);
    });
});

/**
 * Update scenario information in the UI
 */
function updateScenarioInfo(scenario) {
    // This function could be used to display additional info about the selected scenario
    console.log('Selected scenario:', scenario);
}

/**
 * Update controller information in the UI
 */
function updateControllerInfo(controller) {
    // This function could be used to display additional info about the selected controller
    console.log('Selected controller:', controller);
}

/**
 * Show loading indicator
 */
function showLoading() {
    const container = document.getElementById('visualization-container');
    
    // Create loading overlay if it doesn't exist
    if (!document.getElementById('loading-overlay')) {
        const loadingOverlay = document.createElement('div');
        loadingOverlay.id = 'loading-overlay';
        loadingOverlay.className = 'loading-overlay';
        
        const spinner = document.createElement('div');
        spinner.className = 'spinner-border text-primary';
        spinner.setAttribute('role', 'status');
        
        const spinnerText = document.createElement('span');
        spinnerText.className = 'visually-hidden';
        spinnerText.textContent = 'Loading...';
        
        spinner.appendChild(spinnerText);
        loadingOverlay.appendChild(spinner);
        container.appendChild(loadingOverlay);
    }
    
    // Show the loading overlay
    document.getElementById('loading-overlay').style.display = 'flex';
}

/**
 * Hide loading indicator
 */
function hideLoading() {
    const loadingOverlay = document.getElementById('loading-overlay');
    if (loadingOverlay) {
        loadingOverlay.style.display = 'none';
    }
}

/**
 * Show error message
 */
function showError(message) {
    // Create alert container if it doesn't exist
    if (!document.getElementById('alert-container')) {
        const alertContainer = document.createElement('div');
        alertContainer.id = 'alert-container';
        alertContainer.className = 'alert alert-danger alert-dismissible fade show';
        alertContainer.setAttribute('role', 'alert');
        
        const closeButton = document.createElement('button');
        closeButton.type = 'button';
        closeButton.className = 'btn-close';
        closeButton.setAttribute('data-bs-dismiss', 'alert');
        closeButton.setAttribute('aria-label', 'Close');
        
        alertContainer.appendChild(document.createTextNode(message));
        alertContainer.appendChild(closeButton);
        
        document.body.insertBefore(alertContainer, document.body.firstChild);
    } else {
        document.getElementById('alert-container').textContent = message;
    }
}

/**
 * Export simulation data
 */
function exportData() {
    // Create data object
    const data = {
        time: timeData,
        position: {
            x: xData,
            y: yData,
            heading: headingData
        },
        velocity: {
            surge: surgeData,
            sway: swayData,
            yaw_rate: yawRateData
        },
        control: {
            thrust: thrustData,
            moment: momentData
        },
        trajectory: {
            usv: usvTrajectoryData,
            reference: referenceTrajectoryData
        }
    };
    
    // Create JSON string
    const jsonData = JSON.stringify(data, null, 2);
    
    // Create download link
    const blob = new Blob([jsonData], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    
    const a = document.createElement('a');
    a.style.display = 'none';
    a.href = url;
    a.download = 'usv_simulation_data.json';
    
    document.body.appendChild(a);
    a.click();
    
    // Clean up
    window.URL.revokeObjectURL(url);
    document.body.removeChild(a);
} 