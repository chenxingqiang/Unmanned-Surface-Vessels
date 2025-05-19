"""
Obstacle detection module for USV navigation.

This module provides classes and functions for detecting obstacles 
using various sensor modalities (LIDAR, camera, radar, etc.) and 
processing sensor data to create a representation of the environment.
"""

import numpy as np
from typing import List, Tuple, Dict, Optional
import matplotlib.pyplot as plt
from scipy.spatial import KDTree


class ObstacleDetector:
    """Base class for obstacle detection."""
    
    def __init__(self, detection_range: float = 100.0, safety_margin: float = 1.0):
        """
        Initialize the obstacle detector.

        Args:
            detection_range: Maximum detection range in meters
            safety_margin: Safety margin to add to detected obstacle sizes
        """
        self.detection_range = detection_range
        self.safety_margin = safety_margin
        self.detected_obstacles = []  # List of [x, y, radius] obstacles
    
    def detect(self, sensor_data: np.ndarray, sensor_position: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        """
        Detect obstacles from sensor data.

        Args:
            sensor_data: Raw sensor data
            sensor_position: Sensor position and orientation (x, y, heading)

        Returns:
            List of detected obstacles as (x, y, radius) tuples
        """
        # This is an abstract method to be implemented by subclasses
        raise NotImplementedError("Subclasses must implement this method.")
    
    def visualize(self, ax=None, show: bool = True):
        """
        Visualize the detected obstacles.

        Args:
            ax: Matplotlib axis
            show: Whether to show the plot
        """
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 8))
        
        for obs in self.detected_obstacles:
            circle = plt.Circle((obs[0], obs[1]), obs[2], color='red', alpha=0.5)
            ax.add_patch(circle)
            
        ax.set_aspect('equal')
        ax.grid(True)
        
        if show:
            plt.show()
            
        return ax


class LidarDetector(ObstacleDetector):
    """Obstacle detector using LIDAR sensor data."""
    
    def __init__(self, 
                 detection_range: float = 100.0, 
                 safety_margin: float = 1.0,
                 angle_increment: float = 0.01,  # radians
                 cluster_threshold: float = 0.5,  # meters
                 min_cluster_size: int = 5):
        """
        Initialize the LIDAR obstacle detector.

        Args:
            detection_range: Maximum detection range in meters
            safety_margin: Safety margin to add to detected obstacle sizes
            angle_increment: Angular resolution of the LIDAR
            cluster_threshold: Distance threshold for clustering points
            min_cluster_size: Minimum number of points to form a cluster
        """
        super().__init__(detection_range, safety_margin)
        self.angle_increment = angle_increment
        self.cluster_threshold = cluster_threshold
        self.min_cluster_size = min_cluster_size
    
    def detect(self, sensor_data: np.ndarray, sensor_position: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        """
        Detect obstacles from LIDAR data.

        Args:
            sensor_data: LIDAR range data [ranges]
            sensor_position: Sensor position and orientation (x, y, heading)

        Returns:
            List of detected obstacles as (x, y, radius) tuples
        """
        # Extract sensor position and orientation
        x_s, y_s, heading = sensor_position
        
        # Convert LIDAR data to Cartesian coordinates
        n_points = len(sensor_data)
        angles = np.linspace(-np.pi, np.pi, n_points, endpoint=False)
        
        # Filter out invalid readings
        valid_indices = sensor_data < self.detection_range
        valid_ranges = sensor_data[valid_indices]
        valid_angles = angles[valid_indices]
        
        if len(valid_ranges) == 0:
            self.detected_obstacles = []
            return []
        
        # Convert to Cartesian coordinates in sensor frame
        x_points = valid_ranges * np.cos(valid_angles)
        y_points = valid_ranges * np.sin(valid_angles)
        
        # Transform to global frame
        cos_h, sin_h = np.cos(heading), np.sin(heading)
        x_global = x_s + x_points * cos_h - y_points * sin_h
        y_global = y_s + x_points * sin_h + y_points * cos_h
        
        # Combine into point cloud
        points = np.column_stack((x_global, y_global))
        
        # Cluster points to identify obstacles
        clusters = self._cluster_points(points)
        
        # Extract obstacle information
        obstacles = []
        for cluster in clusters:
            if len(cluster) >= self.min_cluster_size:
                # Calculate centroid
                centroid = np.mean(cluster, axis=0)
                
                # Calculate radius as max distance from centroid to any point
                distances = np.linalg.norm(cluster - centroid, axis=1)
                radius = np.max(distances) + self.safety_margin
                
                obstacles.append((centroid[0], centroid[1], radius))
        
        self.detected_obstacles = obstacles
        return obstacles
    
    def _cluster_points(self, points: np.ndarray) -> List[np.ndarray]:
        """
        Cluster point cloud using a simple distance-based approach.

        Args:
            points: Point cloud as Nx2 array of (x, y) points

        Returns:
            List of clusters, where each cluster is an array of points
        """
        if len(points) == 0:
            return []
            
        # Sort points by x-coordinate for more efficient clustering
        sorted_indices = np.argsort(points[:, 0])
        sorted_points = points[sorted_indices]
        
        clusters = []
        current_cluster = [sorted_points[0]]
        
        for i in range(1, len(sorted_points)):
            # Check if this point is close to any point in current cluster
            point = sorted_points[i]
            
            # Check distance to closest point in current cluster
            min_dist = min(np.linalg.norm(point - cluster_point) for cluster_point in current_cluster)
            
            if min_dist <= self.cluster_threshold:
                # Add to current cluster
                current_cluster.append(point)
            else:
                # Start new cluster
                if len(current_cluster) >= self.min_cluster_size:
                    clusters.append(np.array(current_cluster))
                current_cluster = [point]
        
        # Add last cluster if it's large enough
        if len(current_cluster) >= self.min_cluster_size:
            clusters.append(np.array(current_cluster))
            
        return clusters


class RadarDetector(ObstacleDetector):
    """Obstacle detector using radar sensor data."""
    
    def __init__(self, 
                 detection_range: float = 150.0, 
                 safety_margin: float = 2.0,
                 angle_resolution: float = 2.0,  # degrees
                 min_detection_size: float = 5.0,  # minimum target size in m²
                 noise_threshold: float = 0.2):  # radar return energy threshold
        """
        Initialize the radar obstacle detector.

        Args:
            detection_range: Maximum detection range in meters
            safety_margin: Safety margin to add to detected obstacle sizes
            angle_resolution: Angular resolution of the radar in degrees
            min_detection_size: Minimum target size to be detected in m²
            noise_threshold: Threshold for radar return energy
        """
        super().__init__(detection_range, safety_margin)
        self.angle_resolution = np.radians(angle_resolution)
        self.min_detection_size = min_detection_size
        self.noise_threshold = noise_threshold
    
    def detect(self, sensor_data: Dict, sensor_position: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        """
        Detect obstacles from radar data.

        Args:
            sensor_data: Radar data dictionary containing 'ranges', 'intensities', and 'angles'
            sensor_position: Sensor position and orientation (x, y, heading)

        Returns:
            List of detected obstacles as (x, y, radius) tuples
        """
        # Extract sensor position and orientation
        x_s, y_s, heading = sensor_position
        
        # Extract radar data
        ranges = sensor_data.get('ranges', np.array([]))
        intensities = sensor_data.get('intensities', np.array([]))
        angles = sensor_data.get('angles', np.array([]))
        
        # Filter out weak returns and objects beyond range
        valid_indices = (intensities > self.noise_threshold) & (ranges < self.detection_range)
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        valid_intensities = intensities[valid_indices]
        
        if len(valid_ranges) == 0:
            self.detected_obstacles = []
            return []
        
        # Convert to Cartesian coordinates in sensor frame
        x_points = valid_ranges * np.cos(valid_angles)
        y_points = valid_ranges * np.sin(valid_angles)
        
        # Transform to global frame
        cos_h, sin_h = np.cos(heading), np.sin(heading)
        x_global = x_s + x_points * cos_h - y_points * sin_h
        y_global = y_s + x_points * sin_h + y_points * cos_h
        
        # Combine into point cloud with intensities
        points = np.column_stack((x_global, y_global))
        
        # Use advanced clustering for radar data (e.g., DBSCAN)
        # For simplicity, we'll use a similar approach to LIDAR with intensity weighting
        clusters = self._cluster_points(points, valid_intensities)
        
        # Extract obstacle information with intensity-based weighting
        obstacles = []
        for cluster, intensities in clusters:
            # Calculate centroid (weighted by intensity)
            total_weight = np.sum(intensities)
            centroid = np.sum(cluster * intensities[:, np.newaxis], axis=0) / total_weight
            
            # Calculate radius based on both geometric extent and intensity
            # Higher intensity targets are typically larger and more important
            distances = np.linalg.norm(cluster - centroid, axis=1)
            geometry_radius = np.max(distances)
            intensity_factor = np.mean(intensities) / self.noise_threshold
            
            # Combine geometric and intensity factors
            radius = geometry_radius * np.sqrt(intensity_factor) + self.safety_margin
            
            obstacles.append((centroid[0], centroid[1], radius))
        
        self.detected_obstacles = obstacles
        return obstacles
    
    def _cluster_points(self, points: np.ndarray, intensities: np.ndarray) -> List[Tuple[np.ndarray, np.ndarray]]:
        """
        Cluster radar points using intensity-weighted clustering.

        Args:
            points: Point cloud as Nx2 array of (x, y) points
            intensities: Array of radar return intensities

        Returns:
            List of (cluster_points, cluster_intensities) tuples
        """
        if len(points) == 0:
            return []
            
        # Use KD-Tree for efficient nearest neighbor search
        tree = KDTree(points)
        
        # Cluster with custom distance threshold based on range
        # Radar resolution decreases with range
        clusters = []
        processed = set()
        
        for i in range(len(points)):
            if i in processed:
                continue
                
            # Find neighbors within threshold
            # Scale threshold with range (further points have higher uncertainty)
            range_i = np.linalg.norm(points[i])
            threshold = self.cluster_threshold * (1.0 + 0.02 * range_i)  # 2% increase per meter
            
            indices = tree.query_ball_point(points[i], threshold)
            
            if len(indices) > self.min_cluster_size:
                # Mark as processed
                processed.update(indices)
                
                # Create cluster
                cluster_points = points[indices]
                cluster_intensities = intensities[indices]
                
                clusters.append((cluster_points, cluster_intensities))
                
        return clusters


class VisionDetector(ObstacleDetector):
    """
    Obstacle detector using camera vision data with basic computer vision techniques.
    
    Note: This is a simplified version. A real implementation would use techniques like 
    segmentation, optical flow, and/or deep learning-based object detection.
    """
    
    def __init__(self, 
                 detection_range: float = 80.0, 
                 safety_margin: float = 1.5,
                 camera_fov: float = 60.0,  # degrees
                 camera_params: Dict = None):
        """
        Initialize the vision-based obstacle detector.

        Args:
            detection_range: Maximum detection range in meters
            safety_margin: Safety margin to add to detected obstacle sizes
            camera_fov: Camera field of view in degrees
            camera_params: Camera parameters dictionary
        """
        super().__init__(detection_range, safety_margin)
        self.camera_fov = np.radians(camera_fov)
        
        # Default camera parameters (intrinsics)
        default_params = {
            'focal_length': 500.0,  # pixels
            'image_width': 640,     # pixels
            'image_height': 480,    # pixels
            'cx': 320.0,            # principal point x (pixels)
            'cy': 240.0,            # principal point y (pixels)
            'dist_per_pixel': 0.1   # approximate distance per pixel at 1m (for size estimation)
        }
        
        self.camera_params = default_params
        if camera_params:
            self.camera_params.update(camera_params)
    
    def detect(self, sensor_data: Dict, sensor_position: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        """
        Detect obstacles from camera data.

        Args:
            sensor_data: Camera data dictionary containing:
                - 'image': RGB or grayscale image as numpy array
                - 'water_mask': Optional binary mask of water region
                - 'object_detections': Optional list of bounding boxes [x, y, w, h, confidence, class]
            sensor_position: Sensor position and orientation (x, y, heading)

        Returns:
            List of detected obstacles as (x, y, radius) tuples
        """
        # Extract sensor position and orientation
        x_s, y_s, heading = sensor_position
        
        # Extract image data
        image = sensor_data.get('image', None)
        water_mask = sensor_data.get('water_mask', None)
        object_detections = sensor_data.get('object_detections', None)
        
        if image is None:
            print("No image data provided")
            return []
        
        # Process based on available data
        if object_detections:
            # Process provided object detections (e.g., from YOLO)
            obstacles = self._process_object_detections(object_detections, sensor_position)
        else:
            # Perform simple image processing if no detections provided
            obstacles = self._process_image(image, water_mask, sensor_position)
        
        self.detected_obstacles = obstacles
        return obstacles
    
    def _process_object_detections(self, 
                                  detections: List, 
                                  sensor_position: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        """
        Process object detections from an object detector.

        Args:
            detections: List of detection bounding boxes [x, y, w, h, confidence, class]
            sensor_position: Sensor position and orientation

        Returns:
            List of obstacles as (x, y, radius) tuples
        """
        x_s, y_s, heading = sensor_position
        obstacles = []
        
        for det in detections:
            x, y, w, h, confidence, obj_class = det
            
            # Skip low confidence detections
            if confidence < 0.5:
                continue
                
            # Skip non-obstacle classes (could filter based on class)
            if obj_class not in [0, 2, 3, 5, 7]:  # Example: assume these are obstacle classes
                continue
                
            # Estimate distance based on bounding box size
            # This is a simplified approach; real systems would use more sophisticated methods
            size = max(w, h)  # pixels
            distance_estimate = self.camera_params['focal_length'] / (size * self.camera_params['dist_per_pixel'])
            
            # Skip if beyond detection range
            if distance_estimate > self.detection_range:
                continue
                
            # Calculate angle from center of image
            dx_pixels = x + w/2 - self.camera_params['cx']
            angle = np.arctan2(dx_pixels, self.camera_params['focal_length'])
            
            # Global position
            angle_global = heading + angle
            x_global = x_s + distance_estimate * np.cos(angle_global)
            y_global = y_s + distance_estimate * np.sin(angle_global)
            
            # Estimate radius based on object size
            radius = max(w, h) * distance_estimate * self.camera_params['dist_per_pixel'] / 2.0 + self.safety_margin
            
            obstacles.append((x_global, y_global, radius))
            
        return obstacles
    
    def _process_image(self, 
                      image: np.ndarray, 
                      water_mask: Optional[np.ndarray], 
                      sensor_position: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        """
        Process raw image data to detect obstacles.
        
        This is a placeholder for a real computer vision pipeline.

        Args:
            image: RGB or grayscale image
            water_mask: Binary mask of water region
            sensor_position: Sensor position and orientation

        Returns:
            List of obstacles as (x, y, radius) tuples
        """
        # This would be a complex computer vision pipeline in a real system
        # Here we'll return a dummy obstacle for illustration
        
        x_s, y_s, heading = sensor_position
        
        # Dummy obstacle 20m ahead
        x_obs = x_s + 20.0 * np.cos(heading)
        y_obs = y_s + 20.0 * np.sin(heading)
        
        return [(x_obs, y_obs, 2.0 + self.safety_margin)]


class SensorFusion:
    """
    Fuse obstacle detections from multiple sensors.
    """
    
    def __init__(self, sensor_weights: Dict[str, float] = None):
        """
        Initialize sensor fusion.

        Args:
            sensor_weights: Dictionary mapping sensor types to weights
        """
        # Default weights
        default_weights = {
            'lidar': 1.0,
            'radar': 0.7,
            'vision': 0.5
        }
        
        self.weights = default_weights
        if sensor_weights:
            self.weights.update(sensor_weights)
            
        self.fused_obstacles = []
    
    def fuse_detections(self, 
                       sensor_detections: Dict[str, List[Tuple[float, float, float]]], 
                       cluster_threshold: float = 2.0) -> List[Tuple[float, float, float]]:
        """
        Fuse obstacle detections from different sensors.

        Args:
            sensor_detections: Dictionary mapping sensor types to obstacle lists
            cluster_threshold: Distance threshold for clustering obstacles

        Returns:
            List of fused obstacles as (x, y, radius) tuples
        """
        # Collect all obstacles with their weights
        all_obstacles = []
        
        for sensor_type, obstacles in sensor_detections.items():
            weight = self.weights.get(sensor_type, 0.5)
            
            for obs in obstacles:
                all_obstacles.append((obs[0], obs[1], obs[2], weight, sensor_type))
        
        # No obstacles detected
        if not all_obstacles:
            self.fused_obstacles = []
            return []
            
        # Cluster obstacles
        clusters = self._cluster_obstacles(all_obstacles, cluster_threshold)
        
        # Fuse each cluster
        fused_obstacles = []
        
        for cluster in clusters:
            if not cluster:
                continue
                
            # Extract coordinates and weights
            coords = np.array([(obs[0], obs[1]) for obs in cluster])
            radii = np.array([obs[2] for obs in cluster])
            weights = np.array([obs[3] for obs in cluster])
            
            # Normalize weights
            weights_sum = np.sum(weights)
            if weights_sum > 0:
                norm_weights = weights / weights_sum
            else:
                norm_weights = np.ones_like(weights) / len(weights)
            
            # Weighted centroid
            centroid = np.sum(coords * norm_weights[:, np.newaxis], axis=0)
            
            # Take maximum radius with safety margin
            radius = np.max(radii)
            
            fused_obstacles.append((centroid[0], centroid[1], radius))
        
        self.fused_obstacles = fused_obstacles
        return fused_obstacles
    
    def _cluster_obstacles(self, 
                          obstacles: List[Tuple[float, float, float, float, str]], 
                          threshold: float) -> List[List]:
        """
        Cluster obstacles based on distance.

        Args:
            obstacles: List of (x, y, radius, weight, sensor_type) tuples
            threshold: Distance threshold for clustering

        Returns:
            List of clusters, where each cluster is a list of obstacles
        """
        if not obstacles:
            return []
            
        # Extract coordinates for clustering
        coords = np.array([(obs[0], obs[1]) for obs in obstacles])
        
        # Simple clustering algorithm
        clusters = []
        remaining = set(range(len(obstacles)))
        
        while remaining:
            # Start a new cluster with first remaining obstacle
            current = remaining.pop()
            current_cluster = [obstacles[current]]
            current_coords = coords[current]
            
            # Find all obstacles close to this one
            to_check = list(remaining)
            for i in to_check:
                dist = np.linalg.norm(coords[i] - current_coords)
                
                if dist <= threshold + obstacles[current][2] + obstacles[i][2]:
                    current_cluster.append(obstacles[i])
                    remaining.remove(i)
            
            clusters.append(current_cluster)
            
        return clusters
    
    def visualize(self, ax=None, show: bool = True):
        """
        Visualize the fused obstacles.

        Args:
            ax: Matplotlib axis
            show: Whether to show the plot
        """
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 8))
        
        for obs in self.fused_obstacles:
            circle = plt.Circle((obs[0], obs[1]), obs[2], color='blue', alpha=0.5)
            ax.add_patch(circle)
            
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_title('Fused Obstacle Detections')
        
        if show:
            plt.show()
            
        return ax
