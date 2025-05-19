/**
 * Charts.js - Data visualization using Chart.js
 */

// Global chart objects
let positionChart, velocityChart, controlChart, trajectoryChart;

// Data arrays for charts
const timeData = [];
const xData = [];
const yData = [];
const headingData = [];
const surgeData = [];
const swayData = [];
const yawRateData = [];
const thrustData = [];
const momentData = [];
const usvTrajectoryData = [];
const referenceTrajectoryData = [];

// Maximum data points to keep in memory
const MAX_DATA_POINTS = 500;

/**
 * Initialize all charts
 */
function initCharts() {
    initPositionChart();
    initVelocityChart();
    initControlChart();
    initTrajectoryChart();
}

/**
 * Initialize position chart (x, y, heading)
 */
function initPositionChart() {
    const ctx = document.getElementById('position-chart').getContext('2d');
    
    positionChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: timeData,
            datasets: [
                {
                    label: 'X Position (m)',
                    data: xData,
                    borderColor: 'rgb(255, 99, 132)',
                    backgroundColor: 'rgba(255, 99, 132, 0.1)',
                    borderWidth: 2,
                    tension: 0.1
                },
                {
                    label: 'Y Position (m)',
                    data: yData,
                    borderColor: 'rgb(54, 162, 235)',
                    backgroundColor: 'rgba(54, 162, 235, 0.1)',
                    borderWidth: 2,
                    tension: 0.1
                },
                {
                    label: 'Heading (deg)',
                    data: headingData,
                    borderColor: 'rgb(255, 206, 86)',
                    backgroundColor: 'rgba(255, 206, 86, 0.1)',
                    borderWidth: 2,
                    tension: 0.1,
                    yAxisID: 'y1'
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            scales: {
                x: {
                    title: {
                        display: true,
                        text: 'Time (s)'
                    }
                },
                y: {
                    title: {
                        display: true,
                        text: 'Position (m)'
                    }
                },
                y1: {
                    position: 'right',
                    title: {
                        display: true,
                        text: 'Heading (deg)'
                    },
                    grid: {
                        drawOnChartArea: false
                    }
                }
            },
            plugins: {
                legend: {
                    position: 'top',
                },
                tooltip: {
                    enabled: true
                }
            }
        }
    });
}

/**
 * Initialize velocity chart (surge, sway, yaw rate)
 */
function initVelocityChart() {
    const ctx = document.getElementById('velocity-chart').getContext('2d');
    
    velocityChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: timeData,
            datasets: [
                {
                    label: 'Surge (m/s)',
                    data: surgeData,
                    borderColor: 'rgb(75, 192, 192)',
                    backgroundColor: 'rgba(75, 192, 192, 0.1)',
                    borderWidth: 2,
                    tension: 0.1
                },
                {
                    label: 'Sway (m/s)',
                    data: swayData,
                    borderColor: 'rgb(153, 102, 255)',
                    backgroundColor: 'rgba(153, 102, 255, 0.1)',
                    borderWidth: 2,
                    tension: 0.1
                },
                {
                    label: 'Yaw Rate (rad/s)',
                    data: yawRateData,
                    borderColor: 'rgb(255, 159, 64)',
                    backgroundColor: 'rgba(255, 159, 64, 0.1)',
                    borderWidth: 2,
                    tension: 0.1
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            scales: {
                x: {
                    title: {
                        display: true,
                        text: 'Time (s)'
                    }
                },
                y: {
                    title: {
                        display: true,
                        text: 'Velocity'
                    }
                }
            },
            plugins: {
                legend: {
                    position: 'top',
                },
                tooltip: {
                    enabled: true
                }
            }
        }
    });
}

/**
 * Initialize control chart (thrust, moment)
 */
function initControlChart() {
    const ctx = document.getElementById('control-chart').getContext('2d');
    
    controlChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: timeData,
            datasets: [
                {
                    label: 'Thrust (N)',
                    data: thrustData,
                    borderColor: 'rgb(255, 99, 132)',
                    backgroundColor: 'rgba(255, 99, 132, 0.1)',
                    borderWidth: 2,
                    tension: 0.1,
                    yAxisID: 'y'
                },
                {
                    label: 'Moment (N.m)',
                    data: momentData,
                    borderColor: 'rgb(54, 162, 235)',
                    backgroundColor: 'rgba(54, 162, 235, 0.1)',
                    borderWidth: 2,
                    tension: 0.1,
                    yAxisID: 'y1'
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            scales: {
                x: {
                    title: {
                        display: true,
                        text: 'Time (s)'
                    }
                },
                y: {
                    position: 'left',
                    title: {
                        display: true,
                        text: 'Thrust (N)'
                    }
                },
                y1: {
                    position: 'right',
                    title: {
                        display: true,
                        text: 'Moment (N.m)'
                    },
                    grid: {
                        drawOnChartArea: false
                    }
                }
            },
            plugins: {
                legend: {
                    position: 'top',
                },
                tooltip: {
                    enabled: true
                }
            }
        }
    });
}

/**
 * Initialize trajectory chart (x-y plane)
 */
function initTrajectoryChart() {
    const ctx = document.getElementById('trajectory-canvas').getContext('2d');
    
    trajectoryChart = new Chart(ctx, {
        type: 'scatter',
        data: {
            datasets: [
                {
                    label: 'USV Trajectory',
                    data: usvTrajectoryData,
                    borderColor: 'rgb(75, 192, 192)',
                    backgroundColor: 'rgba(75, 192, 192, 0.5)',
                    pointRadius: 3,
                    showLine: true,
                    borderWidth: 2
                },
                {
                    label: 'Reference Trajectory',
                    data: referenceTrajectoryData,
                    borderColor: 'rgb(255, 99, 132)',
                    backgroundColor: 'rgba(255, 99, 132, 0.5)',
                    pointRadius: 3,
                    showLine: true,
                    borderDash: [5, 5],
                    borderWidth: 2
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            aspectRatio: 1,
            scales: {
                x: {
                    type: 'linear',
                    position: 'bottom',
                    title: {
                        display: true,
                        text: 'X Position (m)'
                    }
                },
                y: {
                    title: {
                        display: true,
                        text: 'Y Position (m)'
                    }
                }
            },
            plugins: {
                legend: {
                    position: 'top',
                },
                tooltip: {
                    callbacks: {
                        label: function(context) {
                            const label = context.dataset.label || '';
                            const x = context.parsed.x.toFixed(2);
                            const y = context.parsed.y.toFixed(2);
                            return label + ': (' + x + ', ' + y + ')';
                        }
                    }
                }
            }
        }
    });
}

/**
 * Update charts with new simulation data
 */
function updateCharts(data) {
    try {
        // Validate input data
        if (!data) {
            console.error('No data provided to updateCharts');
            return;
        }

        // Debug log (simplified)
        console.log('Updating charts with data - time:', data.time);

        // Validate required data structures
        if (!data.state || !data.control || !data.reference) {
            console.error('Missing required data fields');
            return;
        }

        // Update data arrays with validation
        if (typeof data.time === 'number') {
            timeData.push(data.time);
        }

        if (typeof data.state.x === 'number' && typeof data.state.y === 'number') {
            xData.push(data.state.x);
            yData.push(data.state.y);
        }

        if (typeof data.state.heading === 'number') {
            headingData.push(data.state.heading * 180 / Math.PI); // Convert to degrees
        }

        if (typeof data.state.surge === 'number' && typeof data.state.sway === 'number') {
            surgeData.push(data.state.surge);
            swayData.push(data.state.sway);
        }

        if (typeof data.state.yaw_rate === 'number') {
            yawRateData.push(data.state.yaw_rate);
        }

        if (typeof data.control.thrust === 'number' && typeof data.control.moment === 'number') {
            thrustData.push(data.control.thrust);
            momentData.push(data.control.moment);
        }
        
        // Update trajectory data with validation
        if (typeof data.state.x === 'number' && typeof data.state.y === 'number') {
            usvTrajectoryData.push({x: data.state.x, y: data.state.y});
        }
        if (typeof data.reference.x === 'number' && typeof data.reference.y === 'number') {
            referenceTrajectoryData.push({x: data.reference.x, y: data.reference.y});
        }
        
        // Limit data points to prevent performance issues
        if (timeData.length > MAX_DATA_POINTS) {
            timeData.shift();
            xData.shift();
            yData.shift();
            headingData.shift();
            surgeData.shift();
            swayData.shift();
            yawRateData.shift();
            thrustData.shift();
            momentData.shift();
        }
        
        // Limit trajectory data points
        if (usvTrajectoryData.length > MAX_DATA_POINTS) {
            usvTrajectoryData.shift();
            referenceTrajectoryData.shift();
        }
        
        // Force update all charts to ensure they refresh
        if (positionChart) {
            positionChart.update('none'); // Use 'none' animation mode for better performance
        }
        if (velocityChart) {
            velocityChart.update('none');
        }
        if (controlChart) {
            controlChart.update('none');
        }
        if (trajectoryChart) {
            trajectoryChart.update('none');
        }

        // Log success with minimal information
        console.log("Charts updated successfully - data points:", timeData.length);
    } catch (error) {
        console.error('Error in updateCharts:', error);
    }
}

/**
 * Reset all charts
 */
function resetCharts() {
    // Clear data arrays
    timeData.length = 0;
    xData.length = 0;
    yData.length = 0;
    headingData.length = 0;
    surgeData.length = 0;
    swayData.length = 0;
    yawRateData.length = 0;
    thrustData.length = 0;
    momentData.length = 0;
    usvTrajectoryData.length = 0;
    referenceTrajectoryData.length = 0;
    
    // Update charts if they exist
    if (positionChart) positionChart.update();
    if (velocityChart) velocityChart.update();
    if (controlChart) controlChart.update();
    if (trajectoryChart) trajectoryChart.update();
    
    // Debug to console
    console.log("Charts updated with new data:", {
        time: timeData.length > 0 ? timeData[timeData.length-1] : "no data",
        position: {
            x: xData.length > 0 ? xData[xData.length-1] : "no data",
            y: yData.length > 0 ? yData[yData.length-1] : "no data"
        }
    });
} 