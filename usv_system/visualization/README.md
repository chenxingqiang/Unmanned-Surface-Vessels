# USV Visualization Frontend

A web-based frontend for real-time visualization of the USV navigation and control system.

## Features

- Real-time 3D visualization of USV movement using Three.js
- Real-time data plots of state variables, control signals, and trajectories using Chart.js 
- Interactive controls for simulation parameters
- Multiple view modes (top-down, follow USV, free camera)
- Support for all simulation scenarios and controllers
- Real-time performance metrics display
- Automatic port selection if the default port is already in use

## Architecture

The visualization system uses a client-server architecture:

- Backend: Flask server with Flask-SocketIO for real-time data streaming
- Frontend: HTML/CSS/JavaScript with Bootstrap, Three.js, and Chart.js

```
+--------------------+      +--------------------+      +--------------------+
|                    |      |                    |      |                    |
|    USV Simulation  +----->+  Flask WebSocket   +----->+  Web Browser UI   |
|                    |      |      Server        |      |                    |
+--------------------+      +--------------------+      +--------------------+
```

## Installation

1. Install the required dependencies:

```bash
pip install -r visualization/requirements.txt
```

## Usage

### Using Launcher Scripts

For Unix/Linux/macOS users:
```bash
cd usv_system/visualization
./run_server.sh --port 8080  # Optionally specify port
```

For Windows or cross-platform use:
```bash
cd usv_system/visualization
python run_server.py --browser  # Opens browser automatically
```

Launcher script options:
- `--port PORT`: Specify custom port (default: 5540)
- `--debug`: Run in debug mode with console output
- `--browser` (Python script only): Automatically open browser
- `--help`: Show all available options

### Running Directly

1. Run the visualization server:

```bash
cd usv_system/visualization
python app.py
```

2. Open a web browser and navigate to:

```
http://localhost:5540
```

Note: If port 5540 is already in use, the server will automatically find an available port. Check the console output for the actual port being used.

## Port Configuration

The server port can be configured in multiple ways, in order of precedence:

1. Environment variable: Set `USV_VISUALIZATION_PORT` to your desired port number
   ```bash
   export USV_VISUALIZATION_PORT=8080
   python app.py
   ```

2. Configuration file: Add a `visualization` section with a `port` field in the config YAML file
   ```yaml
   visualization:
     port: 8080
   ```

3. Default port: If no configuration is provided, port 5540 will be used
   - If the configured port is unavailable, the server will automatically find the next available port

## Testing

To test the automatic port selection feature:

```bash
python test_port.py
```

This will run tests to verify:
1. The server can automatically find an available port if the default port is in use
2. The server correctly uses the port specified in the environment variable

## UI Components

### Simulation Controls
- Scenario selection dropdown
- Controller selection dropdown
- Simulation speed slider
- Start/Pause/Stop buttons

### 3D Visualization
- Interactive 3D view of the USV, environment, and obstacles
- Multiple view modes (top-down, follow, free camera)
- Trajectory visualization

### Data Plots
- Position data (X, Y, Heading)
- Velocity data (Surge, Sway, Yaw Rate)
- Control signals (Thrust, Moment)
- Trajectory plot (X-Y plane)

### Status Information
- Simulation progress
- Performance metrics (path error, heading error)
- Simulation time

## Keyboard Shortcuts

- Space: Pause/Resume simulation
- Escape: Stop simulation
- 1: Top-down view
- 2: Follow USV view
- 3: Free camera view

## Browser Compatibility

The visualization frontend has been tested with:
- Google Chrome 90+
- Mozilla Firefox 88+
- Microsoft Edge 90+

## Extending the Visualization

To add new visualization features:

1. Add new charts or 3D elements in the respective JavaScript files
2. Update the data streaming in app.py to include the new data
3. Add new UI elements to index.html as needed 