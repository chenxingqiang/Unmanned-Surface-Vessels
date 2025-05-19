
📘 Project Proposal: Development of Intelligent Navigation and Control System for Unmanned Surface Vessels (USVs)

⸻

1. Background and Significance

Unmanned Surface Vessels (USVs), as high-autonomy watercraft, have gained increasing attention in recent years due to their broad applications in oceanographic research, environmental monitoring, maritime rescue, and military patrols. With the rapid advancement of artificial intelligence, sensor fusion, and autonomous control, USVs have demonstrated exceptional potential in replacing manned operations in hazardous or repetitive environments.

Internationally, regions such as Europe, Japan, South Korea, and the United States are heavily investing in USV research and development. The IMO has initiated frameworks for Maritime Autonomous Surface Ships (MASS), emphasizing safety and regulatory integration. Domestically, initiatives such as “Made in China 2025” and the CCS “Intelligent Ship Guidelines” have positioned smart ships as a key development area.

Despite the progress, challenges remain in ensuring autonomous navigation under complex maritime environments (e.g., narrow channels, high-traffic waters). Developing an intelligent navigation and control system is essential for enhancing USV safety, decision-making, and environmental adaptability.

⸻

2. Objectives
	•	Design and develop a modular, intelligent navigation and control system for USVs.
	•	Achieve real-time autonomous navigation, path planning, and obstacle avoidance in complex waters.
	•	Enhance system reliability, scalability, and compatibility with various sensors and hardware platforms.
	•	Integrate the system for both simulation and real-world field deployment.

⸻

3. Research Content and Technical Roadmap

3.1 Core Modules

Module	Description
System Architecture	Hardware-software co-design including sensors, actuators, onboard computing, and communication modules.
Navigation and Path Planning	Global path planning (e.g., A*, Dijkstra) and local reactive planning (e.g., DWA, RRT*) with geofencing and safe-zone heuristics.
Obstacle Avoidance	Real-time perception-based avoidance using LiDAR, stereo vision, and YOLO-based object detection.
Motion Control	Underactuated vessel dynamics modeling, PID/LQR/MPC-based tracking controller.
Simulation and Validation	ROS2 + Gazebo or Unity simulation environment, followed by waterborne field testing.

3.2 Research Methods
	•	Literature Review: Analyze domestic and international progress in intelligent USV systems.
	•	Mathematical Modeling: Establish motion/dynamic models for the vessel.
	•	Algorithm Design: Propose and validate navigation, control, and decision-making algorithms.
	•	Simulation: Use Gazebo or MATLAB/Simulink for verification.
	•	Experimental Validation: Deploy system on prototype vessel in lake/harbor scenarios.

⸻

4. Technical Architecture Diagram

+------------------------------------------------+
|            Shore-Based Monitoring Center       |
| Task Assignment | Remote Telemetry | GUI       |
+--------------------▲---------------------------+
                     | Wi-Fi/5G/LoRa
+--------------------▼---------------------------+
|   Onboard USV Navigation and Control System    |
|  - Sensor Fusion (GPS+IMU+LiDAR+Camera)        |
|  - Path Planning & Decision Layer              |
|  - Motion Controller (PID/LQR/MPC)             |
+--------------------▲---------------------------+
                     | CAN/ROS Middleware
+--------------------▼---------------------------+
|     Actuation Layer (ESC, Motor, Rudder)       |
+------------------------------------------------+


⸻

5. Schedule (18 Weeks)

Phase	Duration	Task
P1	Weeks 1–2	Foreign literature review and translation
P2	Weeks 3–5	System design and hardware selection
P3	Weeks 6–9	Algorithm development: navigation & control
P4	Weeks 10–13	User interface and telemetry module
P5	Weeks 14–16	System integration and field testing
P6	Weeks 17–18	Report writing and thesis completion


⸻

6. Expected Outcomes
	•	A functional prototype of an intelligent USV navigation and control system.
	•	Simulation test results and real-world navigation trial videos.
	•	One SCI/EI-indexed publication or national patent.
	•	Open-source codebase for academic and industrial extension.

⸻

7. References (selected English)
	1.	Liu W., Wang X., Hu Y., et al. An improved center difference Kalman filtering method for navigation and positioning of unmanned surface vessels, Ocean Engineering, 2025.
	2.	Yang X., Song Y., He L., et al. USV-YOLO: An Algorithm for Detecting Floating Objects on the Surface of an Environmentally Friendly Unmanned Vessel, IJCS, 2025.
	3.	Han X., Yuan Y., Zhong J., et al. Water Segmentation for USV Navigation Based on Multi-Scale Feature Fusion, Applied Sciences, 2025.
	4.	Zhang C., Zhu J., Ye W., et al. Real-time detection of organophosphorus pesticides using a boat-borne Raman spectrometer, Analyst, 2025.
	5.	Yang X., Song Y., He L., et al. CFP-PSPNet: a lightweight unmanned vessel water segmentation algorithm, JRTIP, 2025.
