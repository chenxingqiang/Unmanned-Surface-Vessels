"""
Local path planning module for USV navigation.

This module provides algorithms for local path planning,
including Dynamic Window Approach (DWA) and RRT* algorithms.
These are used for reactive obstacle avoidance and local trajectory generation.
"""

import numpy as np
import random
import matplotlib.pyplot as plt
from typing import List, Tuple, Dict, Optional, Set
from scipy.spatial import KDTree

from usv_system.models.usv_model import USVKinematicModel


class DynamicWindowPlanner:
    """
    Dynamic Window Approach (DWA) planner for local obstacle avoidance.
    """

    def __init__(self,
                 model_params: Dict = None,
                 cost_params: Dict = None,
                 window_params: Dict = None,
                 dt: float = 0.1,
                 predict_time: float = 3.0):
        """
        Initialize the DWA planner.

        Args:
            model_params: USV model parameters
            cost_params: Cost function weights
            window_params: Dynamic window parameters
            dt: Time step for simulation
            predict_time: Time horizon for trajectory prediction
        """
        # Default model parameters
        default_model = {
            'max_speed': 2.0,         # m/s
            'min_speed': 0.0,         # m/s
            'max_yawrate': 40.0 * np.pi / 180.0,  # rad/s
            'max_accel': 0.5,         # m/s^2
            'max_dyawrate': 40.0 * np.pi / 180.0,  # rad/s^2
            'robot_radius': 1.0,      # m
        }

        self.model = default_model
        if model_params:
            self.model.update(model_params)

        # Default cost function weights
        default_cost = {
            'heading': 1.0,   # Heading cost weight
            'dist': 0.5,      # Obstacle distance cost weight
            'velocity': 0.2,  # Velocity cost weight
        }

        self.cost = default_cost
        if cost_params:
            self.cost.update(cost_params)

        # Default window parameters
        default_window = {
            'v_reso': 0.1,    # m/s
            'yawrate_reso': 0.1 * np.pi / 180.0  # rad/s
        }

        self.window = default_window
        if window_params:
            self.window.update(window_params)

        self.dt = dt
        self.predict_time = predict_time

    def _calculate_dynamic_window(self,
                                 x: np.ndarray,
                                 config: Dict) -> Tuple[float, float, float, float]:
        """
        Calculate the dynamic window based on current state and constraints.

        Args:
            x: Current state vector [x, y, yaw, v, yawrate]
            config: Configuration parameters

        Returns:
            v_min, v_max, yawrate_min, yawrate_max
        """
        # Window from robot specification
        Vs = [config['min_speed'], config['max_speed'],
              -config['max_yawrate'], config['max_yawrate']]

        # Dynamic window from motion model
        Vd = [x[3] - config['max_accel'] * self.dt,
              x[3] + config['max_accel'] * self.dt,
              x[4] - config['max_dyawrate'] * self.dt,
              x[4] + config['max_dyawrate'] * self.dt]

        # Final dynamic window
        vmin = max(Vs[0], Vd[0])
        vmax = min(Vs[1], Vd[1])
        yawrate_min = max(Vs[2], Vd[2])
        yawrate_max = min(Vs[3], Vd[3])

        return vmin, vmax, yawrate_min, yawrate_max

    def _predict_trajectory(self,
                           x_init: np.ndarray,
                           v: float,
                           y: float,
                           predict_time: float) -> np.ndarray:
        """
        Predict the trajectory for given velocity and yaw rate.

        Args:
            x_init: Initial state vector [x, y, yaw, v, yawrate]
            v: Velocity
            y: Yaw rate
            predict_time: Prediction time horizon

        Returns:
            Predicted trajectory as a list of states
        """
        x = np.array(x_init)
        trajectory = np.array([x])
        time = 0

        while time <= predict_time:
            x = self._motion(x, [v, y], self.dt)
            trajectory = np.vstack((trajectory, x))
            time += self.dt

        return trajectory

    def _motion(self, x: np.ndarray, u: List[float], dt: float) -> np.ndarray:
        """
        Motion model for state update.

        Args:
            x: Current state vector [x, y, yaw, v, yawrate]
            u: Control inputs [v, yawrate]
            dt: Time step

        Returns:
            Updated state vector
        """
        # Simple kinematic model
        x[2] += u[1] * dt  # yaw
        x[0] += u[0] * np.cos(x[2]) * dt  # x
        x[1] += u[0] * np.sin(x[2]) * dt  # y
        x[3] = u[0]  # v
        x[4] = u[1]  # yawrate

        return x

    def _calc_to_goal_cost(self, trajectory: np.ndarray, goal: Tuple[float, float]) -> float:
        """
        Calculate the cost based on distance to goal.

        Args:
            trajectory: Predicted trajectory
            goal: Goal position (x, y)

        Returns:
            Cost value (lower is better)
        """
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        goal_dist = np.sqrt(dx**2 + dy**2)

        # Angle cost
        goal_dir = np.arctan2(dy, dx)
        final_heading = trajectory[-1, 2]
        angle_diff = abs(goal_dir - final_heading) % (2 * np.pi)
        angle_cost = min(angle_diff, 2 * np.pi - angle_diff) / np.pi

        cost = self.cost['heading'] * angle_cost + goal_dist

        return cost

    def _calc_obstacle_cost(self,
                           trajectory: np.ndarray,
                           obstacles: List[Tuple[float, float, float]],
                           config: Dict) -> float:
        """
        Calculate the cost based on distance to obstacles.

        Args:
            trajectory: Predicted trajectory
            obstacles: List of obstacles [x, y, radius]
            config: Configuration parameters

        Returns:
            Cost value (lower is better)
        """
        if not obstacles:
            return 0.0

        min_dist = float('inf')

        for i in range(len(trajectory)):
            for obs in obstacles:
                ox, oy, or_ = obs
                dx = trajectory[i, 0] - ox
                dy = trajectory[i, 1] - oy
                dist = np.sqrt(dx**2 + dy**2) - or_ - config['robot_radius']

                if dist <= 0:  # Collision
                    return float('inf')

                min_dist = min(min_dist, dist)

        # Cost is inversely proportional to minimum distance to obstacles
        if min_dist == float('inf'):
            return 0.0

        return self.cost['dist'] * (1.0 / min_dist)

    def _calc_velocity_cost(self, trajectory: np.ndarray, config: Dict) -> float:
        """
        Calculate the cost based on velocity preference.

        Args:
            trajectory: Predicted trajectory
            config: Configuration parameters

        Returns:
            Cost value (lower is better)
        """
        # Prefer higher velocities (within limits)
        max_cost = config['max_speed'] - config['min_speed']

        if max_cost == 0:
            return 0.0

        cost = (config['max_speed'] - trajectory[-1, 3]) / max_cost

        return self.cost['velocity'] * cost

    def plan(self,
            x: np.ndarray,
            goal: Tuple[float, float],
            obstacles: List[Tuple[float, float, float]]) -> Tuple[float, float, np.ndarray]:
        """
        Plan local trajectory using DWA.

        Args:
            x: Current state vector [x, y, yaw, v, yawrate]
            goal: Goal position (x, y)
            obstacles: List of obstacles [x, y, radius]

        Returns:
            Tuple of (v, yawrate, trajectory)
        """
        # Calculate dynamic window
        v_min, v_max, y_min, y_max = self._calculate_dynamic_window(x, self.model)

        # Discretize window
        vs = np.arange(v_min, v_max + self.window['v_reso'], self.window['v_reso'])
        yawrates = np.arange(y_min, y_max + self.window['yawrate_reso'], self.window['yawrate_reso'])

        # Evaluate all trajectories
        best_cost = float('inf')
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        for v in vs:
            for y in yawrates:
                # Predict trajectory
                trajectory = self._predict_trajectory(x, v, y, self.predict_time)

                # Calculate costs
                to_goal_cost = self._calc_to_goal_cost(trajectory, goal)
                obstacle_cost = self._calc_obstacle_cost(trajectory, obstacles, self.model)
                velocity_cost = self._calc_velocity_cost(trajectory, self.model)

                # Total cost
                total_cost = to_goal_cost + obstacle_cost + velocity_cost

                # Find minimum cost
                if total_cost < best_cost:
                    best_cost = total_cost
                    best_u = [v, y]
                    best_trajectory = trajectory

        return best_u[0], best_u[1], best_trajectory

    def plot_trajectory(self, trajectory: np.ndarray, obstacles: List[Tuple[float, float, float]], goal: Tuple[float, float], title: str = 'DWA Trajectory'):
        """
        Plot the planned trajectory.

        Args:
            trajectory: Predicted trajectory
            obstacles: List of obstacles [x, y, radius]
            goal: Goal position (x, y)
            title: Plot title
        """
        plt.figure(figsize=(10, 8))

        # Plot obstacles
        for obs in obstacles:
            circle = plt.Circle((obs[0], obs[1]), obs[2], color='r', alpha=0.7)
            plt.gca().add_patch(circle)

        # Plot trajectory
        plt.plot(trajectory[:, 0], trajectory[:, 1], 'b-', label='Trajectory')

        # Plot start and goal
        plt.plot(trajectory[0, 0], trajectory[0, 1], 'go', markersize=10, label='Start')
        plt.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')

        plt.axis('equal')
        plt.grid(True)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title(title)
        plt.legend()
        plt.show()


class RRTStarPlanner:
    """
    RRT* (Rapidly-exploring Random Tree Star) path planning algorithm implementation.
    """

    class Node:
        """Node class for RRT*."""

        def __init__(self, x: float, y: float):
            """
            Initialize a node.

            Args:
                x: x-coordinate
                y: y-coordinate
            """
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None
            self.cost = 0.0

    def __init__(self,
                 start: Tuple[float, float],
                 goal: Tuple[float, float],
                 obstacle_list: List[Tuple[float, float, float]],
                 rand_area: Tuple[float, float, float, float],
                 expand_dis: float = 3.0,
                 goal_sample_rate: int = 10,
                 max_iter: int = 100,
                 robot_radius: float = 1.0,
                 connect_circle_dist: float = 50.0):
        """
        Initialize RRT* planner.

        Args:
            start: Start position (x, y)
            goal: Goal position (x, y)
            obstacle_list: List of obstacles [x, y, radius]
            rand_area: Random sampling area [min_x, max_x, min_y, max_y]
            expand_dis: Distance to expand tree
            goal_sample_rate: Goal sampling rate (0-100)
            max_iter: Maximum number of iterations
            robot_radius: Robot radius for collision checking
            connect_circle_dist: Distance for finding nearby nodes
        """
        self.start = self.Node(start[0], start[1])
        self.goal = self.Node(goal[0], goal[1])
        self.obstacle_list = obstacle_list
        self.min_rand_x, self.max_rand_x, self.min_rand_y, self.max_rand_y = rand_area
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.robot_radius = robot_radius
        self.connect_circle_dist = connect_circle_dist
        self.node_list = [self.start]

    def planning(self, animation: bool = True) -> Tuple[List[float], List[float]]:
        """
        Run the RRT* planning algorithm.

        Args:
            animation: Whether to show animation

        Returns:
            Lists of x and y coordinates forming the path
        """
        for i in range(self.max_iter):
            # Sample a random node
            rnd_node = self._get_random_node()

            # Find nearest node
            nearest_ind = self._get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            # Steer towards the random node
            new_node = self._steer(nearest_node, rnd_node, self.expand_dis)

            # Check if collision free
            if self._is_collision_free(new_node):
                near_indices = self._find_near_nodes(new_node)
                new_node = self._choose_parent(new_node, near_indices)

                if new_node:
                    # Add node to tree
                    self.node_list.append(new_node)
                    # Rewire tree
                    self._rewire(new_node, near_indices)

            if animation and i % 5 == 0:
                self._draw_graph(rnd_node)

            # Check if we can connect to goal
            if i % 5 == 0:
                # Find closest node to goal
                dist_to_goal_list = [
                    self._calc_dist_to_goal(node.x, node.y) for node in self.node_list
                ]
                goal_inds = [
                    dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.expand_dis
                ]

                if goal_inds:
                    for goal_ind in goal_inds:
                        # Check path to goal for collision
                        final_node = self._steer(self.node_list[goal_ind], self.goal, self.expand_dis)
                        if self._is_collision_free(final_node):
                            # Found path to goal
                            print("Goal found!")
                            return self._generate_final_course(len(self.node_list) - 1)

        print(f"Reached max iterations ({self.max_iter}). Path not found.")

        # Return the path to the closest node to goal
        dist_to_goal_list = [self._calc_dist_to_goal(node.x, node.y) for node in self.node_list]
        min_ind = dist_to_goal_list.index(min(dist_to_goal_list))

        return self._generate_final_course(min_ind)

    def _calc_dist_to_goal(self, x: float, y: float) -> float:
        """
        Calculate distance to goal.

        Args:
            x: x-coordinate
            y: y-coordinate

        Returns:
            Distance to goal
        """
        return np.sqrt((x - self.goal.x)**2 + (y - self.goal.y)**2)

    def _get_random_node(self) -> Node:
        """
        Sample a random node.

        Returns:
            Random node
        """
        if random.randint(0, 100) > self.goal_sample_rate:
            # Sample random point
            rnd = self.Node(
                random.uniform(self.min_rand_x, self.max_rand_x),
                random.uniform(self.min_rand_y, self.max_rand_y)
            )
        else:
            # Sample goal point
            rnd = self.Node(self.goal.x, self.goal.y)

        return rnd

    def _get_nearest_node_index(self, node_list: List[Node], rnd_node: Node) -> int:
        """
        Find the nearest node index in the tree.

        Args:
            node_list: List of nodes
            rnd_node: Random node

        Returns:
            Index of the nearest node
        """
        dlist = [
            (node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 for node in node_list
        ]
        minind = dlist.index(min(dlist))

        return minind

    def _steer(self, from_node: Node, to_node: Node, extend_length: float) -> Node:
        """
        Steer from one node towards another.

        Args:
            from_node: Starting node
            to_node: Target node
            extend_length: Maximum extension length

        Returns:
            New node
        """
        new_node = self.Node(from_node.x, from_node.y)
        d = np.sqrt((to_node.x - from_node.x)**2 + (to_node.y - from_node.y)**2)

        if d > extend_length:
            # Limit distance
            theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
            new_node.x = from_node.x + extend_length * np.cos(theta)
            new_node.y = from_node.y + extend_length * np.sin(theta)
        else:
            # Reach the point
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node
        new_node.cost = from_node.cost + d

        return new_node

    def _is_collision_free(self, node: Node) -> bool:
        """
        Check if the path to this node is collision free.

        Args:
            node: Node to check

        Returns:
            True if collision free
        """
        if node.parent is None:
            return True

        for obs in self.obstacle_list:
            # Check collision with path
            ox, oy, or_ = obs

            # Check collision between two line segments
            dx = node.x - node.parent.x
            dy = node.y - node.parent.y
            step_size = self.robot_radius / 2.0

            d = np.sqrt(dx**2 + dy**2)

            if d <= step_size:
                # Direct check for short segments
                if np.sqrt((node.x - ox)**2 + (node.y - oy)**2) <= or_ + self.robot_radius:
                    return False
            else:
                # Check along the path
                steps = int(np.ceil(d / step_size))
                for i in range(steps + 1):
                    u = i / steps
                    x = node.parent.x + u * dx
                    y = node.parent.y + u * dy

                    if np.sqrt((x - ox)**2 + (y - oy)**2) <= or_ + self.robot_radius:
                        return False

        return True

    def _find_near_nodes(self, new_node: Node) -> List[int]:
        """
        Find nearby nodes for rewiring.

        Args:
            new_node: New node

        Returns:
            List of nearby node indices
        """
        n_nodes = len(self.node_list) + 1
        r = self.connect_circle_dist * np.sqrt(np.log(n_nodes) / n_nodes)
        r = min(r, self.expand_dis * 5.0)

        dist_list = [
            (node.x - new_node.x)**2 + (node.y - new_node.y)**2 for node in self.node_list
        ]
        near_inds = [i for i, dist in enumerate(dist_list) if dist <= r**2]

        return near_inds

    def _choose_parent(self, new_node: Node, near_inds: List[int]) -> Optional[Node]:
        """
        Choose parent node for new node from nearby nodes.

        Args:
            new_node: New node
            near_inds: List of nearby node indices

        Returns:
            New node with updated parent or None if no valid parent found
        """
        if not near_inds:
            return None

        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self._steer(near_node, new_node, self.expand_dis)

            if t_node and self._is_collision_free(t_node):
                costs.append(near_node.cost + np.sqrt(
                    (t_node.x - near_node.x)**2 + (t_node.y - near_node.y)**2
                ))
            else:
                costs.append(float('inf'))

        min_cost = min(costs)

        if min_cost == float('inf'):
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self._steer(self.node_list[min_ind], new_node, self.expand_dis)
        new_node.cost = min_cost

        return new_node

    def _rewire(self, new_node: Node, near_inds: List[int]):
        """
        Rewire the tree.

        Args:
            new_node: New node
            near_inds: List of nearby node indices
        """
        for i in near_inds:
            near_node = self.node_list[i]

            # Calculate edge cost
            edge_cost = np.sqrt((near_node.x - new_node.x)**2 + (near_node.y - new_node.y)**2)

            # Calculate new cost
            new_cost = new_node.cost + edge_cost

            # Check if rewiring would reduce cost
            if near_node.cost > new_cost:
                t_node = self._steer(new_node, near_node, self.expand_dis)

                if t_node and self._is_collision_free(t_node):
                    # Update parent and cost
                    near_node.parent = new_node
                    near_node.cost = new_cost

    def _generate_final_course(self, goal_ind: int) -> Tuple[List[float], List[float]]:
        """
        Generate the final path.

        Args:
            goal_ind: Index of the goal node

        Returns:
            Lists of x and y coordinates forming the path
        """
        path_x = [self.goal.x]
        path_y = [self.goal.y]
        node = self.node_list[goal_ind]

        while node.parent is not None:
            path_x.append(node.x)
            path_y.append(node.y)
            node = node.parent

        path_x.append(node.x)
        path_y.append(node.y)

        # Reverse to get start-to-goal order
        return list(reversed(path_x)), list(reversed(path_y))

    def _draw_graph(self, rnd: Optional[Node] = None):
        """
        Draw the RRT* graph for visualization.

        Args:
            rnd: Random node being considered
        """
        plt.clf()

        # Plot obstacles
        for obs in self.obstacle_list:
            circle = plt.Circle((obs[0], obs[1]), obs[2], color='k')
            plt.gca().add_patch(circle)

        # Plot start and goal
        plt.plot(self.start.x, self.start.y, 'go', markersize=10, label='Start')
        plt.plot(self.goal.x, self.goal.y, 'ro', markersize=10, label='Goal')

        # Plot RRT* tree
        for node in self.node_list:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'g-')

        # Plot random node if provided
        if rnd:
            plt.plot(rnd.x, rnd.y, 'kx')

        plt.axis('equal')
        plt.grid(True)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('RRT* Path Planning')
        plt.pause(0.01)

    def get_path_as_waypoints(self) -> List[Tuple[float, float]]:
        """
        Convert the path to a list of waypoints.

        Returns:
            List of (x, y) waypoints
        """
        x_path, y_path = self.planning(animation=False)
        return list(zip(x_path, y_path))

    def plot_path(self, path_x: List[float], path_y: List[float]):
        """
        Plot the final path.

        Args:
            path_x: List of x coordinates
            path_y: List of y coordinates
        """
        plt.figure(figsize=(10, 8))

        # Plot obstacles
        for obs in self.obstacle_list:
            circle = plt.Circle((obs[0], obs[1]), obs[2], color='k')
            plt.gca().add_patch(circle)

        # Plot RRT* tree
        for node in self.node_list:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'g-', alpha=0.3)

        # Plot final path
        plt.plot(path_x, path_y, 'r-', linewidth=2, label='Final Path')

        # Plot start and goal
        plt.plot(self.start.x, self.start.y, 'go', markersize=10, label='Start')
        plt.plot(self.goal.x, self.goal.y, 'ro', markersize=10, label='Goal')

        plt.axis('equal')
        plt.grid(True)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('RRT* Path Planning')
        plt.legend()
        plt.show()


class ReactivePlanner:
    """
    Reactive planner for obstacle avoidance, using hybrid DWA and potential field approaches.
    """

    def __init__(self,
                dwa_planner: DynamicWindowPlanner,
                safety_radius: float = 5.0,
                force_scale: float = 1.0):
        """
        Initialize the reactive planner.

        Args:
            dwa_planner: DynamicWindowPlanner instance
            safety_radius: Safety radius for emergency response
            force_scale: Scaling factor for potential field forces
        """
        self.dwa = dwa_planner
        self.safety_radius = safety_radius
        self.force_scale = force_scale
        self.emergency_mode = False

    def _calculate_potential_field(self,
                                  current_pos: Tuple[float, float],
                                  obstacles: List[Tuple[float, float, float]],
                                  goal: Tuple[float, float]) -> Tuple[float, float]:
        """
        Calculate potential field forces for emergency avoidance.

        Args:
            current_pos: Current USV position (x, y)
            obstacles: List of obstacles [x, y, radius]
            goal: Goal position (x, y)

        Returns:
            (force_x, force_y) forces from potential field
        """
        # Parameters
        attractive_gain = 1.0
        repulsive_gain = 100.0

        # Attractive force (goal)
        dx = goal[0] - current_pos[0]
        dy = goal[1] - current_pos[1]
        dist_to_goal = np.sqrt(dx**2 + dy**2)

        # Scale attractive force based on distance
        if dist_to_goal > 5.0:
            fx_att = attractive_gain * dx / dist_to_goal
            fy_att = attractive_gain * dy / dist_to_goal
        else:
            fx_att = attractive_gain * dx
            fy_att = attractive_gain * dy

        # Repulsive forces (obstacles)
        fx_rep = 0.0
        fy_rep = 0.0

        for obs in obstacles:
            ox, oy, or_ = obs
            dx = current_pos[0] - ox
            dy = current_pos[1] - oy
            dist = np.sqrt(dx**2 + dy**2) - or_

            # If within influence range
            if dist < self.safety_radius:
                # Stronger response as distance decreases
                if dist > 0:
                    factor = repulsive_gain * (1.0 / dist - 1.0 / self.safety_radius) / (dist**3)
                    fx_rep += factor * dx
                    fy_rep += factor * dy
                else:
                    # Very close to obstacle, strong repulsion
                    angle = np.arctan2(dy, dx)
                    fx_rep += repulsive_gain * np.cos(angle) * 10
                    fy_rep += repulsive_gain * np.sin(angle) * 10

        # Combine forces
        fx = fx_att + fx_rep
        fy = fy_att + fy_rep

        # Scale forces
        magnitude = np.sqrt(fx**2 + fy**2)
        if magnitude > 0:
            fx = self.force_scale * fx / magnitude
            fy = self.force_scale * fy / magnitude

        return fx, fy

    def _check_emergency(self,
                        current_pos: Tuple[float, float],
                        obstacles: List[Tuple[float, float, float]]) -> bool:
        """
        Check if emergency avoidance is needed.

        Args:
            current_pos: Current USV position (x, y)
            obstacles: List of obstacles [x, y, radius]

        Returns:
            True if emergency avoidance is needed
        """
        for obs in obstacles:
            ox, oy, or_ = obs
            dx = current_pos[0] - ox
            dy = current_pos[1] - oy
            dist = np.sqrt(dx**2 + dy**2) - or_

            if dist < self.safety_radius:
                return True

        return False

    def plan(self,
            current_state: np.ndarray,
            goal: Tuple[float, float],
            obstacles: List[Tuple[float, float, float]]) -> Tuple[float, float, np.ndarray]:
        """
        Plan local trajectory using reactive approach.

        Args:
            current_state: Current state [x, y, yaw, v, yawrate]
            goal: Goal position (x, y)
            obstacles: List of obstacles [x, y, radius]

        Returns:
            Tuple of (v, yawrate, trajectory)
        """
        current_pos = (current_state[0], current_state[1])

        # Check if emergency avoidance is needed
        self.emergency_mode = self._check_emergency(current_pos, obstacles)

        if self.emergency_mode:
            # Calculate potential field forces for emergency
            fx, fy = self._calculate_potential_field(current_pos, obstacles, goal)

            # Convert forces to heading
            desired_heading = np.arctan2(fy, fx)

            # Calculate heading error
            current_heading = current_state[2]
            heading_error = desired_heading - current_heading
            heading_error = ((heading_error + np.pi) % (2 * np.pi)) - np.pi

            # Set control values (emergency mode uses different control strategy)
            v = 0.5  # Lower speed in emergency
            yawrate = 5.0 * heading_error  # Proportional control

            # Limit yaw rate
            yawrate = max(min(yawrate, 0.5), -0.5)

            # Create simple trajectory prediction
            trajectory = self._predict_simple_trajectory(current_state, v, yawrate)

            return v, yawrate, trajectory
        else:
            # Use DWA for normal operation
            return self.dwa.plan(current_state, goal, obstacles)

    def _predict_simple_trajectory(self,
                                 x_init: np.ndarray,
                                 v: float,
                                 yawrate: float) -> np.ndarray:
        """
        Predict a simple trajectory for emergency avoidance.

        Args:
            x_init: Initial state vector [x, y, yaw, v, yawrate]
            v: Velocity
            yawrate: Yaw rate

        Returns:
            Predicted trajectory as a list of states
        """
        x = np.array(x_init)
        trajectory = np.array([x])
        time = 0

        while time <= 2.0:  # Short prediction time for emergency
            x = np.array(x)  # Ensure x is a numpy array
            x[2] += yawrate * self.dwa.dt  # yaw
            x[0] += v * np.cos(x[2]) * self.dwa.dt  # x
            x[1] += v * np.sin(x[2]) * self.dwa.dt  # y
            x[3] = v  # v
            x[4] = yawrate  # yawrate

            trajectory = np.vstack((trajectory, x))
            time += self.dwa.dt

        return trajectory
