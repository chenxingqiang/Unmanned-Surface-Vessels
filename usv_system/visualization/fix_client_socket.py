#!/usr/bin/env python3
"""
Script to inspect and fix client-side WebSocket connections in the USV visualization.
This checks and potentially fixes static JavaScript files to ensure proper socket connections.
"""

import os
import sys
import re
import shutil
import json

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
JS_DIR = os.path.join(STATIC_DIR, 'js')
BACKUP_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'backups')

def ensure_backup_dir():
    """Ensure the backup directory exists."""
    if not os.path.exists(BACKUP_DIR):
        os.makedirs(BACKUP_DIR)
        print(f"Created backup directory: {BACKUP_DIR}")

def backup_file(filepath):
    """Create a backup of a file before modifying it."""
    ensure_backup_dir()
    basename = os.path.basename(filepath)
    backup_path = os.path.join(BACKUP_DIR, f"{basename}.bak")
    
    # Check if backup already exists
    if os.path.exists(backup_path):
        print(f"Backup already exists: {backup_path}")
        return
    
    # Create backup
    shutil.copy2(filepath, backup_path)
    print(f"Created backup: {backup_path}")

def find_socket_related_files():
    """Find JavaScript files that likely contain socket.io related code."""
    socket_files = []
    
    if not os.path.exists(JS_DIR):
        print(f"JavaScript directory not found: {JS_DIR}")
        return []
    
    for filename in os.listdir(JS_DIR):
        if not filename.endswith('.js'):
            continue
        
        filepath = os.path.join(JS_DIR, filename)
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
            
            # Look for socket.io related terms
            if any(term in content for term in ['socket', 'socketio', 'Socket.IO', 'emit(', 'on(']):
                socket_files.append((filepath, content))
                print(f"Found socket-related file: {filename}")
    
    return socket_files

def analyze_socket_config(file_content):
    """Analyze socket configuration in a JavaScript file."""
    issues = []
    
    # Check for disconnection timeout settings
    if 'reconnection' in file_content and 'false' in file_content.lower():
        issues.append("Socket reconnection might be disabled")
    
    # Check for connection URL
    url_pattern = r'(https?://[^"\']+)'
    urls = re.findall(url_pattern, file_content)
    if urls:
        for url in urls:
            if 'localhost' in url or '127.0.0.1' in url:
                print(f"Found localhost URL: {url}")
    else:
        issues.append("No explicit connection URLs found")
    
    # Check for error handlers
    if 'connect_error' not in file_content:
        issues.append("No connect_error handler found")
    
    # Check for missing event listeners
    key_events = ['simulation_data', 'simulation_status']
    for event in key_events:
        if event not in file_content:
            issues.append(f"Missing event handler for '{event}'")
    
    return issues

def fix_socket_reconnection(filepath, content):
    """Fix socket reconnection settings in a JavaScript file."""
    # Patterns to look for
    reconnection_disabled_pattern = r'reconnection\s*:\s*false'
    connect_timeout_pattern = r'timeout\s*:\s*\d+'
    
    # Fixed versions
    reconnection_fixed = 'reconnection: true, reconnectionAttempts: 10, reconnectionDelay: 1000'
    connect_timeout_fixed = 'timeout: 10000'  # 10 seconds
    
    # Apply fixes
    new_content = content
    if re.search(reconnection_disabled_pattern, content):
        new_content = re.sub(reconnection_disabled_pattern, reconnection_fixed, new_content)
        print("Fixed reconnection settings")
    
    if re.search(connect_timeout_pattern, content):
        new_content = re.sub(connect_timeout_pattern, connect_timeout_fixed, new_content)
        print("Fixed connection timeout")
    
    # Only write if changes were made
    if new_content != content:
        backup_file(filepath)
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(new_content)
        print(f"Updated file: {filepath}")
        return True
    
    return False

def add_error_handlers(filepath, content):
    """Add error handlers if missing."""
    # Look for socket initialization
    socket_init_pattern = r'(const|let|var)\s+socket\s*=\s*io\('
    
    # Check if error handlers exist
    has_connect_error = 'connect_error' in content
    has_disconnect = 'disconnect' in content
    
    if not has_connect_error or not has_disconnect:
        # Find where to insert error handlers
        match = re.search(socket_init_pattern, content)
        if match:
            # Find the end of this statement
            start_pos = match.start()
            next_semicolon = content.find(';', start_pos)
            if next_semicolon > -1:
                # Insert handlers after socket initialization
                handlers = "\n\n// Error handlers added by fix_client_socket.py\n"
                if not has_connect_error:
                    handlers += "socket.on('connect_error', (error) => {\n"
                    handlers += "    console.error('Connection error:', error);\n"
                    handlers += "    alert('Connection error. Please check server status.');\n"
                    handlers += "});\n"
                
                if not has_disconnect:
                    handlers += "socket.on('disconnect', () => {\n"
                    handlers += "    console.warn('Disconnected from server');\n"
                    handlers += "});\n"
                
                new_content = content[:next_semicolon+1] + handlers + content[next_semicolon+1:]
                
                backup_file(filepath)
                with open(filepath, 'w', encoding='utf-8') as f:
                    f.write(new_content)
                
                print(f"Added error handlers to {filepath}")
                return True
    
    return False

def create_test_html():
    """Create a simple HTML test file to verify socket connection."""
    html_content = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Socket.IO Connection Test</title>
    <script src="https://cdn.socket.io/4.4.1/socket.io.min.js"></script>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .log { background: #f5f5f5; border: 1px solid #ddd; padding: 10px; height: 300px; overflow-y: auto; }
        .success { color: green; }
        .error { color: red; }
        .info { color: blue; }
    </style>
</head>
<body>
    <h1>Socket.IO Connection Test</h1>
    <div>
        <label for="port">Port:</label>
        <input type="number" id="port" value="9000" min="1" max="65535">
        <button id="connect">Connect</button>
        <button id="disconnect" disabled>Disconnect</button>
    </div>
    <div>
        <button id="get-state" disabled>Get State</button>
        <button id="start-sim" disabled>Start Simulation</button>
        <button id="stop-sim" disabled>Stop Simulation</button>
    </div>
    <h2>Log:</h2>
    <div class="log" id="log"></div>

    <script>
        let socket = null;
        const log = document.getElementById('log');
        const connectBtn = document.getElementById('connect');
        const disconnectBtn = document.getElementById('disconnect');
        const getStateBtn = document.getElementById('get-state');
        const startSimBtn = document.getElementById('start-sim');
        const stopSimBtn = document.getElementById('stop-sim');
        const portInput = document.getElementById('port');

        function logMessage(message, type = 'info') {
            const entry = document.createElement('div');
            entry.className = type;
            entry.textContent = `${new Date().toLocaleTimeString()} - ${message}`;
            log.appendChild(entry);
            log.scrollTop = log.scrollHeight;
        }

        connectBtn.addEventListener('click', () => {
            const port = portInput.value;
            try {
                logMessage(`Connecting to http://localhost:${port}...`);
                
                socket = io(`http://localhost:${port}`, {
                    reconnection: true,
                    reconnectionAttempts: 5,
                    timeout: 10000
                });

                socket.on('connect', () => {
                    logMessage(`Connected to server with ID: ${socket.id}`, 'success');
                    connectBtn.disabled = true;
                    disconnectBtn.disabled = false;
                    getStateBtn.disabled = false;
                    startSimBtn.disabled = false;
                    stopSimBtn.disabled = false;
                });

                socket.on('connect_error', (error) => {
                    logMessage(`Connection error: ${error}`, 'error');
                });

                socket.on('disconnect', () => {
                    logMessage('Disconnected from server', 'info');
                    connectBtn.disabled = false;
                    disconnectBtn.disabled = true;
                    getStateBtn.disabled = true;
                    startSimBtn.disabled = true;
                    stopSimBtn.disabled = true;
                });

                // Simulation events
                socket.on('simulation_status', (data) => {
                    logMessage(`Simulation status: ${JSON.stringify(data)}`, 'info');
                });

                socket.on('simulation_state', (data) => {
                    logMessage(`Simulation state: ${JSON.stringify(data)}`, 'info');
                });

                socket.on('simulation_data', (data) => {
                    logMessage(`Received simulation data at time: ${data.time}`, 'success');
                    if (data.state) {
                        const pos = data.state;
                        logMessage(`Position: (${pos.x.toFixed(2)}, ${pos.y.toFixed(2)}), heading: ${pos.heading.toFixed(2)}`, 'info');
                    }
                });
            } catch (error) {
                logMessage(`Error creating socket: ${error}`, 'error');
            }
        });

        disconnectBtn.addEventListener('click', () => {
            if (socket) {
                socket.disconnect();
                logMessage('Manually disconnected from server');
            }
        });

        getStateBtn.addEventListener('click', () => {
            if (socket) {
                logMessage('Getting simulation state...');
                socket.emit('get_simulation_state');
            }
        });

        startSimBtn.addEventListener('click', () => {
            if (socket) {
                logMessage('Starting simulation...');
                socket.emit('start_simulation', {
                    scenario: 'obstacle',
                    controller: 'pid',
                    use_obstacle_avoidance: true
                });
            }
        });

        stopSimBtn.addEventListener('click', () => {
            if (socket) {
                logMessage('Stopping simulation...');
                socket.emit('stop_simulation');
            }
        });
    </script>
</body>
</html>
"""
    
    test_html_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'socket_test.html')
    with open(test_html_path, 'w', encoding='utf-8') as f:
        f.write(html_content)
    
    print(f"Created test HTML file: {test_html_path}")
    print(f"Open this file in your browser to test socket connections")
    return test_html_path

def main():
    print("USV Visualization Client Socket Inspector and Fixer")
    print("=" * 60)
    
    # Step 1: Find socket-related JavaScript files
    socket_files = find_socket_related_files()
    
    if not socket_files:
        print("No socket-related JavaScript files found.")
        choice = input("Create a socket test HTML file? (y/n): ").lower().strip()
        if choice == 'y':
            create_test_html()
        return
    
    # Step 2: Analyze each file
    fixes_needed = False
    for filepath, content in socket_files:
        print(f"\nAnalyzing {os.path.basename(filepath)}:")
        issues = analyze_socket_config(content)
        
        if issues:
            fixes_needed = True
            print("Issues found:")
            for issue in issues:
                print(f"  - {issue}")
        else:
            print("No issues detected in this file")
    
    # Step 3: Apply fixes if needed
    if fixes_needed:
        choice = input("\nFix detected issues? (y/n): ").lower().strip()
        if choice == 'y':
            fixed_any = False
            for filepath, content in socket_files:
                fixed_reconnection = fix_socket_reconnection(filepath, content)
                fixed_handlers = add_error_handlers(filepath, content)
                fixed_any = fixed_any or fixed_reconnection or fixed_handlers
            
            if fixed_any:
                print("\nApplied fixes to JavaScript files.")
                print("Please refresh your browser and try the simulation again.")
            else:
                print("\nNo changes were needed or could be applied automatically.")
    
    # Step 4: Create test HTML
    choice = input("\nCreate a socket test HTML file? (y/n): ").lower().strip()
    if choice == 'y':
        create_test_html()

if __name__ == "__main__":
    main() 