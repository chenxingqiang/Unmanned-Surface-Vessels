# USV Visualization System - Run Guide

This guide explains how to run the Unmanned Surface Vessel (USV) Visualization system.

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
   source venv/bin/activate
   pip install -r requirements.txt
   ```

## Running the Visualization Server

1. **Start the server:**
   ```bash
   ./run_usv_visualization.sh
   ```
   - This script will:
     - Set up the simulation environment
     - Start the visualization server (default port: 8080)
     - Open your default browser to the visualization page

2. **Access the Web Interface:**
   - Open your browser and go to: [http://localhost:8080](http://localhost:8080)
   - You should see the USV simulation dashboard and charts.

## Stopping the Server
- Press `Ctrl+C` in the terminal to stop the server.

## Troubleshooting
- **Port already in use:**
  - If you see an error about port 8080 being in use, either stop the other process or edit the script to use a different port.
- **Browser does not open automatically:**
  - Open your browser manually and go to [http://localhost:8080](http://localhost:8080)
- **Dependency issues:**
  - Make sure you installed all requirements with `pip install -r requirements.txt`.
- **Permission denied:**
  - If you get a permission error running the script, make it executable:
    ```bash
    chmod +x run_usv_visualization.sh
    ```

## Log Files
- Server logs are saved to `usv_system/visualization/server.log`.

## More
- For advanced usage or debugging, see the `README.md` and `server.log`. 