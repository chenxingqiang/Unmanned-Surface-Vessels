/**
 * 3D Visualization using Three.js
 */

// Global variables for Three.js
let scene, camera, renderer, controls;
let usv, usvRef, waterPlane, axesHelper;
let followMode = false;
let topViewMode = false;
let trajectory = [];
let referenceTrajectory = [];
let staticObstacles = [];  // Array to store static obstacle objects
let dynamicObstacles = []; // Array to store dynamic obstacle objects

// USV model dimensions
const USV_LENGTH = 2;
const USV_WIDTH = 1;
const USV_HEIGHT = 0.5;

// Debug mode
const DEBUG = true;

/**
 * Logging function with debug toggle
 */
function log(message, obj = null) {
    const timestamp = new Date().toLocaleTimeString();
    if (DEBUG) {
        if (obj !== null) {
            console.log(`[Visualization ${timestamp}] ${message}`, obj);
        } else {
            console.log(`[Visualization ${timestamp}] ${message}`);
        }
    }
}

/**
 * Initialize the 3D visualization
 */
function initVisualization() {
    log("Initializing ThreeJS visualization");
    const container = document.getElementById('visualization-container');
    if (!container) {
        console.error("Visualization container not found in the DOM");
        return;
    }
    
    try {
        // Create scene
        scene = new THREE.Scene();
        scene.background = new THREE.Color(0xd4f1f9); // Light blue background
        
        // Create camera
        const width = container.clientWidth;
        const height = container.clientHeight;
        camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
        camera.position.set(0, 80, 0);
        camera.lookAt(0, 0, 0);
        
        // Create renderer
        renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(width, height);
        renderer.shadowMap.enabled = true;
        container.appendChild(renderer.domElement);
        
        // Add orbit controls
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.25;
        
        // Add ambient light
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        scene.add(ambientLight);
        
        // Add directional light
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(0, 100, 0);
        directionalLight.castShadow = true;
        scene.add(directionalLight);
        
        // Add water plane
        const waterGeometry = new THREE.PlaneGeometry(200, 200);
        const waterMaterial = new THREE.MeshStandardMaterial({
            color: 0x0077be,
            transparent: true,
            opacity: 0.7,
            metalness: 0.1,
            roughness: 0.8
        });
        waterPlane = new THREE.Mesh(waterGeometry, waterMaterial);
        waterPlane.rotation.x = -Math.PI / 2;
        waterPlane.receiveShadow = true;
        scene.add(waterPlane);
        
        // Add grid
        const gridHelper = new THREE.GridHelper(200, 40, 0x555555, 0x777777);
        scene.add(gridHelper);
        
        // Add coordinate axes
        axesHelper = new THREE.AxesHelper(5);
        axesHelper.position.set(-95, 0.1, -95);
        scene.add(axesHelper);
        
        // Create USV
        createUSV();
        
        // Create reference trajectory line
        createTrajectoryLine(0x0088ff, 2, true);  // Blue dashed line for reference
        
        // Create USV trajectory line
        createTrajectoryLine(0x00ff00, 2, false); // Green solid line for actual path
        
        // Handle window resize
        window.addEventListener('resize', onWindowResize);
        
        // Start animation loop
        animate();
        
        log("ThreeJS initialization completed successfully");
    } catch (error) {
        console.error("Failed to initialize ThreeJS visualization:", error);
    }
}

/**
 * Create the USV model
 */
function createUSV() {
    try {
        // Create a simple USV model (boat-like shape)
        const usvGroup = new THREE.Group();
        
        // Hull
        const hullGeometry = new THREE.BoxGeometry(2, 0.5, 1);
        const hullMaterial = new THREE.MeshStandardMaterial({ color: 0x22aa22 });
        const hull = new THREE.Mesh(hullGeometry, hullMaterial);
        hull.position.y = 0.25;
        hull.castShadow = true;
        usvGroup.add(hull);
        
        // Cabin/Bridge
        const cabinGeometry = new THREE.BoxGeometry(0.8, 0.4, 0.7);
        const cabinMaterial = new THREE.MeshStandardMaterial({ color: 0x333333 });
        const cabin = new THREE.Mesh(cabinGeometry, cabinMaterial);
        cabin.position.set(-0.2, 0.7, 0);
        cabin.castShadow = true;
        usvGroup.add(cabin);
        
        // Bow (front pointer)
        const bowGeometry = new THREE.ConeGeometry(0.5, 1, 4);
        const bowMaterial = new THREE.MeshStandardMaterial({ color: 0x22aa22 });
        const bow = new THREE.Mesh(bowGeometry, bowMaterial);
        bow.position.set(1.5, 0.25, 0);
        bow.rotation.z = -Math.PI / 2;
        bow.castShadow = true;
        usvGroup.add(bow);
        
        // Add direction arrow
        const arrowGeometry = new THREE.ConeGeometry(0.2, 0.8, 8);
        const arrowMaterial = new THREE.MeshStandardMaterial({ color: 0xff0000 });
        const arrow = new THREE.Mesh(arrowGeometry, arrowMaterial);
        arrow.position.set(1.6, 1.0, 0);
        arrow.rotation.z = -Math.PI / 2;
        usvGroup.add(arrow);
        
        // Add usv to scene
        scene.add(usvGroup);
        usv = usvGroup;
        
        // Create a "reference" USV (semi-transparent version)
        usvRef = usvGroup.clone();
        const refMaterial = new THREE.MeshStandardMaterial({ 
            color: 0x2222ff, 
            transparent: true, 
            opacity: 0.5 
        });
        usvRef.traverse(function(child) {
            if (child instanceof THREE.Mesh) {
                child.material = refMaterial;
            }
        });
        scene.add(usvRef);
        usvRef.visible = false;
        
        log("USV models created successfully");
    } catch (error) {
        console.error("Failed to create USV models:", error);
    }
}

/**
 * Handle window resize
 */
function onWindowResize() {
    try {
        const container = document.getElementById('visualization-container');
        if (!container) return;
        
        const width = container.clientWidth;
        const height = container.clientHeight;
        
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
        renderer.setSize(width, height);
        
        log(`Resized renderer to ${width}x${height}`);
    } catch (error) {
        console.error("Error handling window resize:", error);
    }
}

/**
 * Animation loop
 */
function animate() {
    requestAnimationFrame(animate);
    if (controls) controls.update();
    render();
}

/**
 * Update the visualization with new simulation data
 */
function updateVisualization(data) {
    // Check if visualization components are initialized
    if (!scene || !usv || !renderer || !camera) {
        console.warn("Scene, USV, renderer or camera not initialized in visualization");
        return;
    }
    
    try {
        // Debug log only basic info to avoid console flooding
        console.log(`Updating visualization - Position: (${data.state.x.toFixed(2)}, ${data.state.y.toFixed(2)})`);
        
        // Update USV position and orientation
        if (data.state) {
            const x = data.state.x;
            const y = data.state.y;
            const heading = data.state.heading;
            
            // Position the USV
            usv.position.set(x, 0.5, y);
            usv.rotation.y = heading;
            
            // Add point to trajectory
            trajectory.push(new THREE.Vector3(x, 0.1, y));
            
            // Update trajectory line
            updateTrajectoryLine(trajectory, 0x00ff00);
        }
        
        // Update reference position
        if (data.reference) {
            const refX = data.reference.x;
            const refY = data.reference.y;
            
            // Update reference USV if available
            if (usvRef) {
                usvRef.visible = true;
                usvRef.position.set(refX, 0.5, refY);
                usvRef.rotation.y = data.reference.heading || 0;
            }
            
            // Add point to reference trajectory
            referenceTrajectory.push(new THREE.Vector3(refX, 0.1, refY));
            
            // Update reference line
            updateTrajectoryLine(referenceTrajectory, 0x0088ff, true);
        }
        
        // Update obstacle visualization if available
        if (data.obstacles) {
            updateObstacles(data.obstacles);
        }
        
        // Update camera position based on view mode
        updateCamera();
        
        // Force a render update
        renderer.render(scene, camera);
        
        // Log success with minimal info
        console.log("Visualization updated successfully");
    } catch (error) {
        console.error("Error updating visualization:", error);
    }
}

/**
 * Update a trajectory line with new points
 */
function updateTrajectoryLine(pointsArray, colorHex, isDashed = false) {
    try {
        if (!scene) {
            console.error("Scene not initialized, can't update trajectory line");
            return;
        }
        
        // Find the line in the scene
        const line = scene.children.find(obj => 
            obj instanceof THREE.Line && 
            obj.material && 
            obj.material.color && 
            obj.material.color.getHex() === colorHex
        );
        
        if (line) {
            // Create a new points array with at least 2 points
            let validPoints;
            
            if (pointsArray.length > 1) {
                validPoints = pointsArray;
                log(`Updating line with ${pointsArray.length} points`);
            } else if (pointsArray.length === 1) {
                // Create a minimal line if we only have one point
                validPoints = [
                    pointsArray[0], 
                    new THREE.Vector3(pointsArray[0].x + 0.01, pointsArray[0].y, pointsArray[0].z + 0.01)
                ];
                log("Only one point available, creating minimal line");
            } else {
                // Default to origin if no points
                validPoints = [
                    new THREE.Vector3(0, 0.1, 0),
                    new THREE.Vector3(0, 0.1, 0.1)
                ];
                log("No points available, creating default line at origin");
            }
            
            // Update the line geometry
            if (line.geometry) {
                line.geometry.dispose();
            }
            
            line.geometry = new THREE.BufferGeometry().setFromPoints(validPoints);
            
            // If using a dashed material, we need to update the line distances
            if (line.material instanceof THREE.LineDashedMaterial) {
                line.computeLineDistances();
            }
        } else {
            log(`Line with color 0x${colorHex.toString(16)} not found in scene, creating new line`);
            // Create a new line if not found
            createTrajectoryLine(colorHex, 2, isDashed);
        }
    } catch (error) {
        console.error("Error updating trajectory line:", error);
    }
}

/**
 * Update the obstacles in the scene
 */
function updateObstacles(obstacleData) {
    try {
        // Handle static obstacles
        if (obstacleData.static) {
            log(`Updating ${obstacleData.static.length} static obstacles`);
            
            // First, remove all existing static obstacles
            staticObstacles.forEach(obstacle => {
                scene.remove(obstacle);
            });
            staticObstacles = [];
            
            // Add new static obstacles
            obstacleData.static.forEach(obstacleInfo => {
                const geometry = new THREE.CylinderGeometry(
                    obstacleInfo.radius, 
                    obstacleInfo.radius, 
                    2, // height 
                    16 // radial segments
                );
                const material = new THREE.MeshStandardMaterial({
                    color: 0xff4500, // Orange-red
                    roughness: 0.7
                });
                const obstacle = new THREE.Mesh(geometry, material);
                obstacle.position.set(obstacleInfo.x, 1, obstacleInfo.y);
                obstacle.castShadow = true;
                
                scene.add(obstacle);
                staticObstacles.push(obstacle);
            });
        }
        
        // Handle dynamic obstacles
        if (obstacleData.dynamic) {
            log(`Updating ${obstacleData.dynamic.length} dynamic obstacles`);
            
            // First, remove all existing dynamic obstacles
            dynamicObstacles.forEach(obstacle => {
                scene.remove(obstacle);
            });
            dynamicObstacles = [];
            
            // Add new dynamic obstacles
            obstacleData.dynamic.forEach(obstacleInfo => {
                // Create a group for the obstacle and its direction indicator
                const obstacleGroup = new THREE.Group();
                
                // Create the obstacle cylinder
                const geometry = new THREE.CylinderGeometry(
                    obstacleInfo.radius, 
                    obstacleInfo.radius, 
                    2, // height 
                    16 // radial segments
                );
                const material = new THREE.MeshStandardMaterial({
                    color: 0x1e90ff, // Dodger blue
                    roughness: 0.7
                });
                const obstacle = new THREE.Mesh(geometry, material);
                obstacle.castShadow = true;
                obstacleGroup.add(obstacle);
                
                // Add direction arrow to show heading
                const arrowGeometry = new THREE.ConeGeometry(
                    obstacleInfo.radius * 0.3, // base radius
                    obstacleInfo.radius * 1.5, // height
                    8 // radial segments
                );
                const arrowMaterial = new THREE.MeshStandardMaterial({
                    color: 0xffff00, // Yellow
                });
                const arrow = new THREE.Mesh(arrowGeometry, arrowMaterial);
                arrow.position.set(obstacleInfo.radius * 0.8, 0, 0);
                arrow.rotation.z = -Math.PI / 2;
                obstacleGroup.add(arrow);
                
                // Position and rotate the obstacle group
                obstacleGroup.position.set(obstacleInfo.x, 1, obstacleInfo.y);
                obstacleGroup.rotation.y = -obstacleInfo.heading + Math.PI/2;
                
                scene.add(obstacleGroup);
                dynamicObstacles.push(obstacleGroup);
            });
        }
    } catch (error) {
        console.error("Error updating obstacles:", error);
    }
}

/**
 * Set the view mode
 */
function setViewMode(mode) {
    try {
        log(`Setting view mode to: ${mode}`);
        
        switch(mode) {
            case 'follow':
                followMode = true;
                topViewMode = false;
                break;
            case 'top':
                followMode = false;
                topViewMode = true;
                break;
            case 'free':
                followMode = false;
                topViewMode = false;
                break;
        }
        
        // Update camera immediately
        updateCamera();
    } catch (error) {
        console.error("Error setting view mode:", error);
    }
}

/**
 * Reset the visualization
 */
function resetVisualization() {
    try {
        // Check if Three.js has been initialized
        if (!scene) {
            console.error("Cannot reset visualization: scene not initialized");
            return;
        }
        
        log("Resetting visualization");
        
        // Reset trajectories
        trajectory = [];
        referenceTrajectory = [];
        
        // Reset USV position
        if (usv) {
            usv.position.set(0, 0.5, 0);
            usv.rotation.y = 0;
        }
        
        if (usvRef) {
            usvRef.visible = false;
        }
        
        // Clear trajectory lines
        const trajectoryLine = scene.children.find(obj => obj instanceof THREE.Line && obj.material.color.getHex() === 0x00ff00);
        if (trajectoryLine) {
            trajectoryLine.geometry.dispose();
            // Create a minimal geometry with two points to avoid errors
            const points = [
                new THREE.Vector3(0, 0.1, 0),
                new THREE.Vector3(0, 0.1, 0.1)
            ];
            trajectoryLine.geometry = new THREE.BufferGeometry().setFromPoints(points);
            if (trajectoryLine.material instanceof THREE.LineDashedMaterial) {
                trajectoryLine.computeLineDistances();
            }
        }
        
        const referenceLine = scene.children.find(obj => obj instanceof THREE.Line && obj.material.color.getHex() === 0x0088ff);
        if (referenceLine) {
            referenceLine.geometry.dispose();
            // Create a minimal geometry with two points to avoid errors
            const points = [
                new THREE.Vector3(0, 0.1, 0),
                new THREE.Vector3(0, 0.1, 0.1)
            ];
            referenceLine.geometry = new THREE.BufferGeometry().setFromPoints(points);
            if (referenceLine.material instanceof THREE.LineDashedMaterial) {
                referenceLine.computeLineDistances();
            }
        }
        
        // Reset obstacles
        clearObstacles();
        
        // Render the scene
        render();
        
        log("Visualization reset completed");
    } catch (error) {
        console.error("Error resetting visualization:", error);
    }
}

// Create a line for the USV trajectory
function createTrajectoryLine(color, linewidth = 2, dashed = false) {
    try {
        // Initialize with a single point to avoid errors
        const points = [
            new THREE.Vector3(0, 0.1, 0),
            new THREE.Vector3(0, 0.1, 0.1)  // Add a second point to ensure the line has at least one segment
        ];
        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        let material;
        
        if (dashed) {
            // Use LineDashedMaterial for dashed lines
            material = new THREE.LineDashedMaterial({
                color: color,
                linewidth: linewidth,
                dashSize: 3,
                gapSize: 1
            });
        } else {
            // Use LineBasicMaterial for solid lines
            material = new THREE.LineBasicMaterial({
                color: color,
                linewidth: linewidth
            });
        }
        
        const line = new THREE.Line(geometry, material);
        
        // Compute line distances for dashed lines
        if (dashed) {
            line.computeLineDistances();
        }
        
        // Add to scene
        scene.add(line);
        
        log(`Created trajectory line with color 0x${color.toString(16)}, dashed: ${dashed}`);
        
        return line;
    } catch (error) {
        console.error("Error creating trajectory line:", error);
        return null;
    }
}

/**
 * Clear all obstacles from the scene
 */
function clearObstacles() {
    try {
        // Clear static obstacles
        if (staticObstacles && staticObstacles.length > 0) {
            log(`Clearing ${staticObstacles.length} static obstacles`);
            staticObstacles.forEach(obstacle => {
                scene.remove(obstacle);
            });
            staticObstacles = [];
        }
        
        // Clear dynamic obstacles
        if (dynamicObstacles && dynamicObstacles.length > 0) {
            log(`Clearing ${dynamicObstacles.length} dynamic obstacles`);
            dynamicObstacles.forEach(obstacle => {
                scene.remove(obstacle);
            });
            dynamicObstacles = [];
        }
    } catch (error) {
        console.error("Error clearing obstacles:", error);
    }
}

/**
 * Update camera position based on view mode
 */
function updateCamera() {
    try {
        if (!usv || !camera) return;
        
        const x = usv.position.x;
        const y = usv.position.z; // In Three.js, Y is up, Z is forward
        const heading = usv.rotation.y;
        
        if (followMode) {
            // Follow mode - position camera behind USV
            const distance = 15;
            const height = 5;
            
            // Position camera behind USV based on heading
            const cameraX = x - distance * Math.sin(heading);
            const cameraZ = y - distance * Math.cos(heading);
            
            camera.position.set(cameraX, height, cameraZ);
            camera.lookAt(x, 0.5, y);
            if (controls) controls.target.set(x, 0.5, y);
            
            log(`Updated camera to follow mode at position (${cameraX}, ${height}, ${cameraZ})`);
        } 
        else if (topViewMode) {
            // Top view mode - position camera directly above USV
            camera.position.set(x, 50, y);
            camera.lookAt(x, 0, y);
            if (controls) controls.target.set(x, 0, y);
            
            log(`Updated camera to top view mode at position (${x}, 50, ${y})`);
        }
    } catch (error) {
        console.error("Error updating camera:", error);
    }
}

/**
 * Render the scene
 */
function render() {
    if (renderer && scene && camera) {
        try {
            renderer.render(scene, camera);
        } catch (error) {
            console.error("Error rendering scene:", error);
        }
    }
}

// Log when visualization.js is loaded
log("Visualization module loaded"); 