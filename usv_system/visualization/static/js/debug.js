/**
 * Debug utilities for Socket.IO communication
 */

// Initialize when DOM is fully loaded
document.addEventListener('DOMContentLoaded', function() {
    // Add debug info UI
    addDebugUI();
    
    // Debug wrapper for socket.emit
    if (window.socket) {
        const originalEmit = window.socket.emit;
        window.socket.emit = function(event, ...args) {
            console.log(`[DEBUG] Emitting event: ${event}`, args);
            logToDebugPanel(`Sent: ${event} - ${JSON.stringify(args)}`);
            return originalEmit.apply(this, [event, ...args]);
        };
        
        // Debug listeners for key events
        const eventsToDebug = [
            'connect', 'disconnect', 'connection_response', 
            'simulation_status', 'simulation_data', 'simulation_state',
            'obstacle_avoidance_status'
        ];
        
        eventsToDebug.forEach(eventName => {
            window.socket.on(eventName, function(data) {
                console.log(`[DEBUG] Received event: ${eventName}`, data);
                logToDebugPanel(`Received: ${eventName} - ${JSON.stringify(data)}`);
            });
        });
        
        console.log('[DEBUG] Socket.IO debugging enabled');
    } else {
        console.error('[DEBUG] Socket.IO not initialized yet');
        setTimeout(function() {
            if (window.socket) {
                console.log('[DEBUG] Socket.IO found on delayed check');
            } else {
                console.error('[DEBUG] Socket.IO not available');
            }
        }, 2000);
    }
    
    // Debug wrapper for the start button
    const startButton = document.getElementById('start-button');
    if (startButton) {
        const originalOnClick = startButton.onclick;
        startButton.onclick = function(event) {
            console.log('[DEBUG] Start button clicked');
            logToDebugPanel('Start button clicked');
            
            // Check if socket is available
            if (!window.socket) {
                console.error('[DEBUG] Socket.IO not available when start button clicked');
                logToDebugPanel('ERROR: Socket.IO not available');
                alert('Error: Socket.IO connection not established. Please refresh the page and try again.');
                return;
            }
            
            // Call original handler if exists
            if (typeof originalOnClick === 'function') {
                return originalOnClick.call(this, event);
            }
        };
        console.log('[DEBUG] Start button debugging enabled');
    } else {
        console.error('[DEBUG] Start button not found');
    }
});

/**
 * Add debug UI panel to the page
 */
function addDebugUI() {
    const debugPanel = document.createElement('div');
    debugPanel.id = 'debug-panel';
    debugPanel.style.cssText = `
        position: fixed;
        bottom: 10px;
        right: 10px;
        width: 400px;
        height: 300px;
        background-color: rgba(0, 0, 0, 0.8);
        color: #00ff00;
        border: 1px solid #00ff00;
        border-radius: 5px;
        font-family: monospace;
        font-size: 12px;
        padding: 10px;
        overflow-y: auto;
        z-index: 9999;
        display: none;
    `;
    
    const debugHeader = document.createElement('div');
    debugHeader.innerHTML = '<strong>Debug Console</strong> <button id="clear-debug" style="float: right;">Clear</button>';
    debugPanel.appendChild(debugHeader);
    
    const debugContent = document.createElement('div');
    debugContent.id = 'debug-content';
    debugContent.style.cssText = `
        margin-top: 10px;
        height: calc(100% - 30px);
        overflow-y: auto;
    `;
    debugPanel.appendChild(debugContent);
    
    document.body.appendChild(debugPanel);
    
    // Add toggle button
    const toggleButton = document.createElement('button');
    toggleButton.id = 'toggle-debug';
    toggleButton.innerHTML = 'Debug';
    toggleButton.style.cssText = `
        position: fixed;
        bottom: 10px;
        right: 10px;
        background-color: #00ff00;
        color: black;
        border: none;
        border-radius: 5px;
        padding: 5px 10px;
        z-index: 10000;
    `;
    document.body.appendChild(toggleButton);
    
    // Add event listeners
    toggleButton.addEventListener('click', function() {
        const panel = document.getElementById('debug-panel');
        if (panel.style.display === 'none') {
            panel.style.display = 'block';
            toggleButton.style.display = 'none';
        }
    });
    
    document.getElementById('clear-debug').addEventListener('click', function() {
        document.getElementById('debug-content').innerHTML = '';
    });
    
    document.getElementById('debug-panel').addEventListener('click', function(e) {
        if (e.target.id === 'debug-panel') {
            document.getElementById('debug-panel').style.display = 'none';
            document.getElementById('toggle-debug').style.display = 'block';
        }
    });
    
    // Initialize with a message
    logToDebugPanel('Debug console initialized');
}

/**
 * Log a message to the debug panel
 */
function logToDebugPanel(message) {
    const debugContent = document.getElementById('debug-content');
    if (debugContent) {
        const timestamp = new Date().toLocaleTimeString();
        const logEntry = document.createElement('div');
        logEntry.innerHTML = `[${timestamp}] ${message}`;
        debugContent.appendChild(logEntry);
        debugContent.scrollTop = debugContent.scrollHeight;
    }
} 