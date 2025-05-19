"""
Global path planning module for USV navigation.

This module provides algorithms for global path planning,
including A* and Dijkstra algorithms, along with utilities for path representation
and manipulation.
"""

import numpy as np
import heapq
from typing import Tuple, List, Dict, Set, Optional
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, LineString


class Node:
    """Class representing a node in the path planning graph."""

    def __init__(self, x: float, y: float, cost: float = 0, heuristic: float = 0):
        """
        Initialize a node.

        Args:
            x: x-coordinate
            y: y-coordinate
            cost: Cost to reach this node from start
            heuristic: Heuristic cost estimate to goal
        """
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.parent = None

    def __lt__(self, other):
        """
        Comparison operator for priority queue.

        Args:
            other: Another node to compare with

        Returns:
            True if this node has a lower f-value than other
        """
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

    def __eq__(self, other):
        """
        Equality operator.

        Args:
            other: Another node to compare with

        Returns:
            True if coordinates are the same
        """
        if isinstance(other, Node):
            return self.x == other.x and self.y == other.y
        return False

    def __hash__(self):
        """
        Hash function for using Node in sets and dictionaries.

        Returns:
            Hash value based on coordinates
        """
        return hash((self.x, self.y))


class ObstacleMap:
    """Class representing a map with obstacles for path planning."""

    def __init__(self, width: float, height: float):
        """
        Initialize an obstacle map.

        Args:
            width: Width of the map
            height: Height of the map
        """
        self.width = width
        self.height = height
        self.obstacles = []  # List of polygons

    def add_polygon_obstacle(self, vertices: List[Tuple[float, float]]):
        """
        Add a polygon obstacle to the map.

        Args:
            vertices: List of (x, y) coordinates forming the polygon
        """
        self.obstacles.append(Polygon(vertices))

    def add_circular_obstacle(self, center: Tuple[float, float], radius: float, num_points: int = 20):
        """
        Add a circular obstacle to the map.

        Args:
            center: (x, y) coordinates of the center
            radius: Radius of the circle
            num_points: Number of points to approximate the circle
        """
        # Generate points around the circle
        theta = np.linspace(0, 2*np.pi, num_points, endpoint=False)
        x = center[0] + radius * np.cos(theta)
        y = center[1] + radius * np.sin(theta)
        vertices = list(zip(x, y))
        self.add_polygon_obstacle(vertices)

    def is_point_in_obstacle(self, point: Tuple[float, float]) -> bool:
        """
        Check if a point is inside any obstacle.

        Args:
            point: (x, y) coordinates to check

        Returns:
            True if the point is inside any obstacle
        """
        p = Point(point)
        for obstacle in self.obstacles:
            if p.within(obstacle) or p.touches(obstacle):
                return True
        return False

    def is_line_crossing_obstacle(self, start: Tuple[float, float], end: Tuple[float, float]) -> bool:
        """
        Check if a line segment crosses any obstacle.

        Args:
            start: (x, y) start coordinates
            end: (x, y) end coordinates

        Returns:
            True if the line crosses any obstacle
        """
        line = LineString([start, end])
        for obstacle in self.obstacles:
            if line.intersects(obstacle):
                return True
        return False

    def is_point_within_bounds(self, point: Tuple[float, float]) -> bool:
        """
        Check if a point is within the map bounds.

        Args:
            point: (x, y) coordinates to check

        Returns:
            True if the point is within bounds
        """
        x, y = point
        return 0 <= x < self.width and 0 <= y < self.height

    def plot(self, ax=None, show=True):
        """
        Plot the obstacle map.

        Args:
            ax: Matplotlib axis
            show: Whether to show the plot
        """
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 8))

        # Plot map boundaries
        ax.set_xlim(0, self.width)
        ax.set_ylim(0, self.height)

        # Plot obstacles
        for obstacle in self.obstacles:
            x, y = obstacle.exterior.xy
            ax.fill(x, y, alpha=0.5, fc='red', ec='black')

        ax.set_aspect('equal')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Map with Obstacles')

        if show:
            plt.show()

        return ax


class AStarPlanner:
    """A* path planning algorithm implementation."""

    def __init__(self, obstacle_map: ObstacleMap, resolution: float = 1.0, safety_distance: float = 0.5):
        """
        Initialize the A* planner.

        Args:
            obstacle_map: ObstacleMap object
            resolution: Grid resolution for discretization
            safety_distance: Safety distance from obstacles
        """
        self.obstacle_map = obstacle_map
        self.resolution = resolution
        self.safety_distance = safety_distance

        # Calculate grid dimensions
        self.x_dim = int(obstacle_map.width / resolution)
        self.y_dim = int(obstacle_map.height / resolution)

    def _continuous_to_grid(self, point: Tuple[float, float]) -> Tuple[int, int]:
        """
        Convert continuous coordinates to grid indices.

        Args:
            point: (x, y) continuous coordinates

        Returns:
            (i, j) grid indices
        """
        x, y = point
        i = min(max(int(x / self.resolution), 0), self.x_dim - 1)
        j = min(max(int(y / self.resolution), 0), self.y_dim - 1)
        return (i, j)

    def _grid_to_continuous(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        """
        Convert grid indices to continuous coordinates.

        Args:
            cell: (i, j) grid indices

        Returns:
            (x, y) continuous coordinates (cell center)
        """
        i, j = cell
        x = (i + 0.5) * self.resolution
        y = (j + 0.5) * self.resolution
        return (x, y)

    def _get_neighbors(self, node: Node) -> List[Tuple[float, float]]:
        """
        Get neighboring positions for a node.

        Args:
            node: Current node

        Returns:
            List of (x, y) coordinates of neighbors
        """
        # 8-connected grid
        directions = [
            (1, 0),   # right
            (1, 1),   # right-up
            (0, 1),   # up
            (-1, 1),  # left-up
            (-1, 0),  # left
            (-1, -1), # left-down
            (0, -1),  # down
            (1, -1)   # right-down
        ]

        neighbors = []
        i, j = self._continuous_to_grid((node.x, node.y))

        for di, dj in directions:
            ni, nj = i + di, j + dj

            # Check if within bounds
            if 0 <= ni < self.x_dim and 0 <= nj < self.y_dim:
                x, y = self._grid_to_continuous((ni, nj))

                # Check if not in obstacle
                if not self._is_in_obstacle((x, y)):
                    neighbors.append((x, y))

        return neighbors

    def _is_in_obstacle(self, point: Tuple[float, float]) -> bool:
        """
        Check if a point is in an obstacle considering safety distance.

        Args:
            point: (x, y) coordinates

        Returns:
            True if the point is in an obstacle with safety distance
        """
        # Check if point is directly in an obstacle
        if self.obstacle_map.is_point_in_obstacle(point):
            return True

        # Check safety distance
        if self.safety_distance > 0:
            x, y = point
            safety_point = Point(x, y).buffer(self.safety_distance)

            for obstacle in self.obstacle_map.obstacles:
                if safety_point.intersects(obstacle):
                    return True

        return False

    def _heuristic(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """
        Calculate heuristic cost between two points (Euclidean distance).

        Args:
            a: First point (x, y)
            b: Second point (x, y)

        Returns:
            Euclidean distance between a and b
        """
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def plan(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Plan a path from start to goal using A*.

        Args:
            start: (x, y) start position
            goal: (x, y) goal position

        Returns:
            List of (x, y) waypoints forming the path
        """
        # Check if start or goal are in obstacles
        if self._is_in_obstacle(start):
            print("Start position is inside an obstacle or too close!")
            return []

        if self._is_in_obstacle(goal):
            print("Goal position is inside an obstacle or too close!")
            return []

        # Initialize start and goal nodes
        start_node = Node(start[0], start[1], 0.0, self._heuristic(start, goal))
        goal_node = Node(goal[0], goal[1])

        # Initialize open and closed sets
        open_set = []
        heapq.heappush(open_set, start_node)
        open_dict = {(start_node.x, start_node.y): start_node}
        closed_set = set()

        # Main loop
        while open_set:
            # Get node with lowest f-value
            current = heapq.heappop(open_set)
            current_key = (current.x, current.y)
            if current_key in open_dict:
                del open_dict[current_key]

            # Check if goal is reached
            if self._heuristic((current.x, current.y), goal) < self.resolution:
                path = self._reconstruct_path(current)
                return path

            # Add current to closed set
            closed_set.add(current_key)

            # Check all neighbors
            for neighbor_pos in self._get_neighbors(current):
                if neighbor_pos in closed_set:
                    continue

                # Calculate g-value for this neighbor
                tentative_g = current.cost + self._heuristic((current.x, current.y), neighbor_pos)

                neighbor = None
                if neighbor_pos in open_dict:
                    neighbor = open_dict[neighbor_pos]
                    if tentative_g >= neighbor.cost:
                        continue

                # Create new node or update existing one
                if not neighbor:
                    neighbor = Node(neighbor_pos[0], neighbor_pos[1])

                neighbor.cost = tentative_g
                neighbor.heuristic = self._heuristic(neighbor_pos, goal)
                neighbor.parent = current

                # Add to open set
                if neighbor_pos not in open_dict:
                    heapq.heappush(open_set, neighbor)
                    open_dict[neighbor_pos] = neighbor

        # No path found
        print("No path found!")
        return []

    def _reconstruct_path(self, node: Node) -> List[Tuple[float, float]]:
        """
        Reconstruct the path from start to goal.

        Args:
            node: End node

        Returns:
            List of (x, y) waypoints forming the path
        """
        path = []
        current = node

        while current is not None:
            path.append((current.x, current.y))
            current = current.parent

        # Reverse path to get start-to-goal order
        return list(reversed(path))

    def smooth_path(self, path: List[Tuple[float, float]], weight_data: float = 0.5, weight_smooth: float = 0.1, iterations: int = 100) -> List[Tuple[float, float]]:
        """
        Smooth the path using gradient descent.

        Args:
            path: Original path as a list of (x, y) waypoints
            weight_data: Weight for original data
            weight_smooth: Weight for smoothing
            iterations: Number of smoothing iterations

        Returns:
            Smoothed path as a list of (x, y) waypoints
        """
        if len(path) <= 2:
            return path

        # Make a copy of the path
        smoothed = np.array(path, dtype=float)
        path_array = np.array(path, dtype=float)

        for _ in range(iterations):
            for i in range(1, len(path) - 1):
                for j in range(2):  # x and y coordinates
                    smoothed[i][j] += weight_data * (path_array[i][j] - smoothed[i][j])
                    smoothed[i][j] += weight_smooth * (smoothed[i-1][j] + smoothed[i+1][j] - 2 * smoothed[i][j])

                # Check if the new point is in an obstacle
                if self._is_in_obstacle((smoothed[i][0], smoothed[i][1])):
                    smoothed[i] = path_array[i]  # Revert to original point

        # Check for line-of-sight between consecutive points
        final_path = [tuple(smoothed[0])]

        for i in range(1, len(smoothed)):
            if not self.obstacle_map.is_line_crossing_obstacle(final_path[-1], tuple(smoothed[i])):
                final_path.append(tuple(smoothed[i]))
            else:
                # If line crosses obstacle, add the original path point
                final_path.append(path[i])

        return final_path

    def plot_path(self, path: List[Tuple[float, float]], start: Tuple[float, float], goal: Tuple[float, float], ax=None, show=True):
        """
        Plot the planned path.

        Args:
            path: Path as a list of (x, y) waypoints
            start: (x, y) start position
            goal: (x, y) goal position
            ax: Matplotlib axis
            show: Whether to show the plot
        """
        if ax is None:
            ax = self.obstacle_map.plot(show=False)

        # Plot path
        if path:
            path_array = np.array(path)
            ax.plot(path_array[:,0], path_array[:,1], 'b-', linewidth=2, label='Path')

        # Plot start and goal
        ax.plot(start[0], start[1], 'go', markersize=10, label='Start')
        ax.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')

        ax.legend()

        if show:
            plt.show()

        return ax


class DijkstraPlanner(AStarPlanner):
    """
    Dijkstra's algorithm implementation for path planning.
    This is essentially A* with a zero heuristic.
    """

    def _heuristic(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """
        Zero heuristic function for Dijkstra's algorithm.

        Args:
            a: First point (x, y)
            b: Second point (x, y)

        Returns:
            Always returns 0
        """
        return 0.0


class WaypointFollower:
    """
    Class for waypoint following and path tracking.
    """

    def __init__(self, lookahead_distance: float = 3.0):
        """
        Initialize the waypoint follower.

        Args:
            lookahead_distance: Lookahead distance for pure pursuit
        """
        self.lookahead_distance = lookahead_distance
        self.waypoints = []
        self.current_waypoint_idx = 0

    def set_waypoints(self, waypoints: List[Tuple[float, float]]):
        """
        Set the waypoints to follow.

        Args:
            waypoints: List of (x, y) waypoints
        """
        self.waypoints = waypoints
        self.current_waypoint_idx = 0

    def get_target_point(self, current_pos: Tuple[float, float]) -> Tuple[float, float]:
        """
        Get the target point to follow using pure pursuit algorithm.

        Args:
            current_pos: Current USV position (x, y)

        Returns:
            Target point (x, y) to follow
        """
        if not self.waypoints:
            return current_pos

        # If we're at the last waypoint, return it
        if self.current_waypoint_idx >= len(self.waypoints) - 1:
            return self.waypoints[-1]

        # Find the first waypoint that is at least lookahead_distance away
        while self.current_waypoint_idx < len(self.waypoints) - 1:
            dist = np.sqrt((current_pos[0] - self.waypoints[self.current_waypoint_idx + 1][0])**2 +
                          (current_pos[1] - self.waypoints[self.current_waypoint_idx + 1][1])**2)

            if dist > self.lookahead_distance:
                break

            self.current_waypoint_idx += 1

        # Calculate target point on the line segment
        start = np.array(self.waypoints[self.current_waypoint_idx])
        end = np.array(self.waypoints[self.current_waypoint_idx + 1])
        vec = end - start
        vec_norm = np.linalg.norm(vec)

        if vec_norm < 1e-6:
            return self.waypoints[self.current_waypoint_idx + 1]

        vec_unit = vec / vec_norm

        # Project current position onto the line segment
        pos = np.array(current_pos)
        start_to_pos = pos - start
        projection_dist = np.dot(start_to_pos, vec_unit)

        # Calculate the point along the line segment
        projection_point = start + projection_dist * vec_unit

        # Calculate the lookahead point
        target_point = projection_point + self.lookahead_distance * vec_unit

        # Check if target point is beyond the end waypoint
        if np.linalg.norm(target_point - start) > vec_norm:
            target_point = end

        return (target_point[0], target_point[1])

    def get_desired_heading(self, current_pos: Tuple[float, float], current_heading: float) -> float:
        """
        Calculate the desired heading to follow the path.

        Args:
            current_pos: Current USV position (x, y)
            current_heading: Current USV heading in radians

        Returns:
            Desired heading in radians
        """
        target = self.get_target_point(current_pos)

        # Calculate angle to target
        dx = target[0] - current_pos[0]
        dy = target[1] - current_pos[1]

        target_heading = np.arctan2(dy, dx)

        return target_heading

    def is_path_complete(self, current_pos: Tuple[float, float], threshold: float = 1.0) -> bool:
        """
        Check if the path following is complete.

        Args:
            current_pos: Current USV position (x, y)
            threshold: Distance threshold to consider goal reached

        Returns:
            True if the path is complete
        """
        if not self.waypoints:
            return True

        final_waypoint = self.waypoints[-1]
        dist_to_goal = np.sqrt((current_pos[0] - final_waypoint[0])**2 +
                              (current_pos[1] - final_waypoint[1])**2)

        return dist_to_goal < threshold
