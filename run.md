# USV Simulation System - Run Guide

This guide explains how to run the Unmanned Surface Vessel (USV) Simulation and Visualization system.

## Prerequisites
- **Python 3.8+** (recommended: Python 3.10)
- **pip** (Python package manager)
- **Git** (for code management)
- **Google Chrome** or another modern web browser

## Setup
1. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```
2. (Optional) If you want to use a virtual environment:
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

## Running the Simulation System

There are multiple ways to run the USV simulation system:

### Method 1: Using run_usv_visualization.sh (Recommended)

```bash
./run_usv_visualization.sh [options]
```

**Options:**
- `-p, --port PORT` - Specify the port number (default: 8080)
- `-d, --debug` - Run in debug mode
- `-o, --no-obstacles` - Disable obstacle avoidance
- `-b, --no-browser` - Don't open browser automatically
- `-h, --help` - Display help message

**Example:**
```bash
./run_usv_visualization.sh --port 9000 --debug
```

### Method 2: Using run_usv_visualization.py

```bash
python run_usv_visualization.py [options]
```

**Options:**
- `-p, --port PORT` - Port to run the server on (default: 8080)
- `-d, --debug` - Run in debug mode
- `-o, --no-obstacles` - Disable obstacle avoidance
- `-b, --no-browser` - Don't open browser automatically
- `-s, --scenario SCENARIO` - Scenario to initialize (default: "obstacle")

### Method 3: Using run_usv.sh

```bash
./run_usv.sh [options]
```

**Options:**
- `-m, --mode MODE` - Server mode: standard or obstacles (default: obstacles)
- `-p, --port PORT` - Port to run the server on (default: 9000)
- `-d, --debug` - Run in debug mode
- `-b, --browser` - Open web browser automatically
- `-h, --help` - Show help message

### Method 4: Using run_usv.py

```bash
python run_usv.py [options]
```

**Options:**
- `--mode, -m` - Server mode: standard or obstacles (default: obstacles)
- `--port, -p` - Port to run the server on (default: 9000)
- `--debug, -d` - Run in debug mode
- `--browser, -b` - Open web browser automatically

## Accessing the Web Interface

After starting the server with any of the methods above:

1. Open your browser and go to: http://localhost:PORT
   - Default is http://localhost:8080 for run_usv_visualization scripts
   - Default is http://localhost:9000 for run_usv scripts

2. You should see the USV simulation dashboard and visualization.

## Stopping the Simulation

### Method 1: Using Keyboard Interrupt
- Press `Ctrl+C` in the terminal to stop the server.

### Method 2: Using reset_usv.sh Script

To completely reset the USV simulation environment:

```bash
./reset_usv.sh
```

This script will:
- Kill any processes using ports 8080, 8081, 8082
- Stop any running USV server processes
- Verify if all processes have been terminated

## Troubleshooting

- **Port already in use:**
  - Use the `reset_usv.sh` script to clear any processes using the ports
  - Or specify a different port using the `--port` option
  
- **Browser does not open automatically:**
  - Open your browser manually and go to http://localhost:PORT
  
- **Dependency issues:**
  - Make sure you installed all requirements with `pip install -r requirements.txt`
  
- **Permission denied:**
  - If you get a permission error running the scripts, make them executable:
    ```bash
    chmod +x run_usv_visualization.sh reset_usv.sh run_usv.sh
    ```

- **Server crashes or unexpected behavior:**
  - Check log files in the `usv_system/visualization/` directory
  - Run in debug mode using the `--debug` option for more verbose output

## Server Modes

- **Obstacles Mode**: Includes obstacle avoidance simulation (default)
- **Standard Mode**: Basic visualization without obstacle avoidance

## Log Files

- Server logs are saved to `usv_system/visualization/server.log`

## Advanced Configuration

For advanced configuration options, see the individual script files or use the `--help` option.