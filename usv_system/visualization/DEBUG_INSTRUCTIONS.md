# USV Simulation Debugging Instructions

If you're experiencing issues with the USV simulation visualization (such as the simulation not starting or the visualization not updating), follow these steps to diagnose and fix the problem.

## Quick Fix Instructions

1. Make sure you're running the latest version of the server:
   ```bash
   cd usv_system/visualization
   python app_with_obstacles.py --port 8080
   ```

2. In a separate terminal, you can test if the simulation is working correctly:
   ```bash
   cd usv_system/visualization
   python simple_socket_test.py 8080
   ```

3. Use the debug interface to test the socket connection:
   - Open `debug.html` in your browser
   - Enter port 8080 and click "Connect"
   - Once connected, click "Start Simulation"
   - Check if you see simulation data in the log

## Troubleshooting Steps

If you're still having issues:

1. **Check server status**:
   ```bash
   lsof -i :8080
   ```
   This should show if the server is running on the specified port.

2. **Fix client-side socket connections**:
   ```bash
   python fix_client_socket.py
   ```
   This will examine and fix any issues in the client-side JavaScript files.

3. **Try with different browser**:
   The visualization works best with Chrome or Firefox. If using Safari, you might encounter WebSocket compatibility issues.

4. **Check for WebSocket errors in the browser console**:
   - Open browser developer tools (F12 in most browsers)
   - Look for errors in the Console tab
   - WebSocket connection issues will be displayed there

## Common Issues and Solutions

1. **No real-time updates after clicking Start**:
   - The simulation might be starting but not sending data
   - Use the debug.html interface to verify socket communication
   - Check if "simulation_data" events are being received

2. **Server starts but client can't connect**:
   - Port might be blocked by firewall
   - Try a different port (e.g., 8090, 3000, 5000)
   - Ensure no other applications are using the same port

3. **No obstacle avoidance behavior**:
   - Make sure obstacle avoidance is enabled
   - Check if obstacles are correctly defined in the configuration
   - Verify if obstacle data is included in the simulation_data events

4. **Simulation is too slow or fast**:
   - Speed can be adjusted in the UI with the slider
   - Server-side real_time_factor can be modified in simulation_worker()

## For Developers

If you need to modify the visualization system:

1. The main server code is in `app_with_obstacles.py`
2. Frontend JavaScript is in `static/js/app.js` and `static/js/visualization.js` 
3. Socket event handling occurs in both files
4. Simulation data flow: simulation_worker() → socketio.emit() → socket.on() in browser

Remember to restart the server after making any server-side code changes. 