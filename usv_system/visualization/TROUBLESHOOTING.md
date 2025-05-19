# USV Visualization Troubleshooting Guide

This guide helps diagnose and fix issues with the USV visualization system.

## Common Issues

### 1. Visualization Not Working / No Simulation Movement

If the simulation interface loads but there's no movement in the 3D view or no data appears in the data panels:

1. **Check if the server is running**
   ```bash
   lsof -i :9000  # Replace 9000 with your port number
   ```

2. **Run the diagnostic script**
   ```bash
   ./check_simulation.py
   ```
   This will tell you if data is being sent properly from the server.

3. **Try to fix the visualization**
   ```bash
   ./fix_visualization.py
   ```
   This script applies various fixes to resolve common visualization issues.

4. **Try specific fixes**
   ```bash
   # Restart the simulation
   ./fix_visualization.py --fix restart
   
   # Toggle obstacle avoidance
   ./fix_visualization.py --fix toggle
   
   # Force client reconnection
   ./fix_visualization.py --fix reconnect
   ```

5. **Refresh your browser** after applying fixes.

### 2. Server Won't Start on Desired Port

If you're having trouble starting the server on a specific port:

1. **Check if the port is in use**
   ```bash
   lsof -i :9000  # Replace 9000 with your port number
   ```

2. **Kill any process using the port**
   ```bash
   kill -9 <PID>  # Replace <PID> with the process ID
   ```

3. **Use the port-specific start script**
   ```bash
   ./run_simulator.sh --port 9000
   ```

### 3. Manual Restart of the Simulation

If you need to manually restart the simulation:

```bash
./restart_simulation.py
```

## Utility Scripts

| Script | Description |
|--------|-------------|
| `run_simulator.sh` | Main script to start the visualization server with a browser interface |
| `start_simulation_only.py` | Starts the simulation on an already running server |
| `restart_simulation.py` | Stops any running simulation and starts a new one |
| `check_simulation.py` | Diagnoses if simulation data is being properly sent |
| `fix_visualization.py` | Applies various fixes to resolve visualization issues |

## Advanced Troubleshooting

If the above solutions don't work:

1. **Check server logs**
   Look for error messages in the terminal where the server is running.

2. **Check browser console**
   Open your browser's developer tools (F12) and look for errors in the Console tab.

3. **Verify WebSocket connection**
   In the browser's Network tab, look for WebSocket connections to confirm they're properly established.

4. **Try a different browser**
   Some browsers handle WebSocket connections better than others.

5. **Restart the server completely**
   Stop the server process and start it again with:
   ```bash
   ./run_simulator.sh --port 9000
   ```

## Configuration Issues

If you suspect there might be issues with the configuration:

1. Check that the obstacle avoidance system is properly configured in the YAML config file
2. Verify that the port settings are consistent across server startup and client connection
3. Make sure the simulation parameters (time step, etc.) are appropriate for your scenario 