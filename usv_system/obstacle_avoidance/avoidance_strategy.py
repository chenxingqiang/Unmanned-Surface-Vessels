"""
Obstacle avoidance strategy module for USV navigation.

This module provides strategies for avoiding detected obstacles
during USV navigation, following COLREGs rules when applicable.
"""

import numpy as np
from typing import List, Tuple, Dict, Optional, Callable
import matplotlib.pyplot as plt

from usv_system.models.usv_model import USVDynamicModel
from usv_system.navigation.local_planner import DynamicWindowPlanner


class COLREGRules:
    """
    Implementation of COLREGs (International Regulations for Preventing Collisions at Sea).
    
    This class implements simplified versions of the relevant COLREGs rules:
    - Rule 13: Overtaking
    - Rule 14: Head-on situation
    - Rule 15: Crossing situation
    """
    
    # Encounter type constants
    HEAD_ON = 0
    CROSSING_FROM_RIGHT = 1
    CROSSING_FROM_LEFT = 2
    OVERTAKING = 3
    BEING_OVERTAKEN = 4
    NO_COLLISION_RISK = 5
    
    def __init__(self, 
                 collision_detection_range: float = 200.0,
                 collision_risk_threshold: float = 0.7,
                 tcpa_threshold: float = 120.0,  # seconds
                 dcpa_threshold: float = 20.0,   # meters
                 encounter_angle_tolerance: float = 20.0):  # degrees
        """
        Initialize the COLREGs rules.

        Args:
            collision_detection_range: Range for collision detection in meters
            collision_risk_threshold: Threshold for collision risk [0,1]
            tcpa_threshold: Time to closest point of approach threshold
            dcpa_threshold: Distance at closest point of approach threshold
            encounter_angle_tolerance: Angle tolerance for encounter type classification
        """
        self.collision_detection_range = collision_detection_range
        self.collision_risk_threshold = collision_risk_threshold
        self.tcpa_threshold = tcpa_threshold
        self.dcpa_threshold = dcpa_threshold
        self.angle_tolerance = np.radians(encounter_angle_tolerance)
        
    def get_encounter_type(self, 
                          own_pos: Tuple[float, float],
                          own_heading: float,
                          own_speed: float,
                          target_pos: Tuple[float, float],
                          target_heading: float,
                          target_speed: float) -> int:
        """
        Determine the type of encounter according to COLREGs.

        Args:
            own_pos: USV position (x, y)
            own_heading: USV heading in radians
            own_speed: USV speed in m/s
            target_pos: Target vessel position (x, y)
            target_heading: Target vessel heading in radians
            target_speed: Target vessel speed in m/s

        Returns:
            Encounter type as integer constant
        """
        # Calculate relative bearing to target
        dx = target_pos[0] - own_pos[0]
        dy = target_pos[1] - own_pos[1]
        distance = np.sqrt(dx**2 + dy**2)
        
        # If distance is too large, no collision risk
        if distance > self.collision_detection_range:
            return self.NO_COLLISION_RISK
            
        # Bearing to target (from USV to target)
        bearing_to_target = np.arctan2(dy, dx)
        # Relative bearing (angle between heading and target)
        rel_bearing = bearing_to_target - own_heading
        rel_bearing = ((rel_bearing + np.pi) % (2 * np.pi)) - np.pi  # Normalize to [-pi, pi]
        
        # Target's bearing to own vessel
        bearing_to_own = np.arctan2(-dy, -dx)
        target_rel_bearing = bearing_to_own - target_heading
        target_rel_bearing = ((target_rel_bearing + np.pi) % (2 * np.pi)) - np.pi
        
        # Check TCPA (Time to Closest Point of Approach) and DCPA (Distance at CPA)
        tcpa, dcpa = self._calculate_cpa(
            own_pos, own_heading, own_speed, target_pos, target_heading, target_speed
        )
        
        # If no immediate collision risk
        if dcpa > self.dcpa_threshold or tcpa > self.tcpa_threshold or tcpa < 0:
            return self.NO_COLLISION_RISK
            
        # Check collision scenarios based on relative bearings
        
        # 1. Head-on (Rule 14): Relative bearing within 10° of bow
        if abs(rel_bearing) < self.angle_tolerance:
            # Must be reciprocal courses (approaching head-on)
            # Target should also see us within 10° of its bow
            if abs(target_rel_bearing) < self.angle_tolerance:
                return self.HEAD_ON
                
        # 2. Crossing (Rules 15 & 16)
        if 0 < rel_bearing < np.pi/2 + self.angle_tolerance:
            # Target coming from right
            return self.CROSSING_FROM_RIGHT
        
        if -np.pi/2 - self.angle_tolerance < rel_bearing < 0:
            # Target coming from left
            return self.CROSSING_FROM_LEFT
            
        # 3. Overtaking (Rule 13): Target is more than 22.5° abaft the beam
        if abs(rel_bearing) > 3*np.pi/4 - self.angle_tolerance:
            if own_speed > target_speed:
                return self.OVERTAKING
            else:
                return self.BEING_OVERTAKEN
                
        # Default: No specific rule applies
        return self.NO_COLLISION_RISK
    
    def _calculate_cpa(self, 
                      own_pos: Tuple[float, float],
                      own_heading: float,
                      own_speed: float,
                      target_pos: Tuple[float, float],
                      target_heading: float,
                      target_speed: float) -> Tuple[float, float]:
        """
        Calculate Time and Distance at Closest Point of Approach (CPA).

        Args:
            own_pos: USV position (x, y)
            own_heading: USV heading in radians
            own_speed: USV speed in m/s
            target_pos: Target vessel position (x, y)
            target_heading: Target vessel heading in radians
            target_speed: Target vessel speed in m/s

        Returns:
            Tuple of (TCPA, DCPA)
        """
        # Convert positions to numpy arrays
        own_pos = np.array(own_pos)
        target_pos = np.array(target_pos)
        
        # Calculate velocity vectors
        own_vel = own_speed * np.array([np.cos(own_heading), np.sin(own_heading)])
        target_vel = target_speed * np.array([np.cos(target_heading), np.sin(target_heading)])
        
        # Relative velocity vector
        rel_vel = target_vel - own_vel
        rel_vel_squared = np.dot(rel_vel, rel_vel)
        
        # If relative velocity is zero, vessels are moving in parallel
        if rel_vel_squared < 1e-6:
            # DCPA is current distance, TCPA is infinity or 0 depending on interpretation
            return 0.0, np.linalg.norm(target_pos - own_pos)
        
        # Relative position vector
        rel_pos = target_pos - own_pos
        
        # Calculate time to CPA
        tcpa = -np.dot(rel_pos, rel_vel) / rel_vel_squared
        
        # Calculate position at CPA
        own_pos_cpa = own_pos + tcpa * own_vel
        target_pos_cpa = target_pos + tcpa * target_vel
        
        # Calculate distance at CPA
        dcpa = np.linalg.norm(target_pos_cpa - own_pos_cpa)
        
        return tcpa, dcpa
    
    def get_recommended_action(self, encounter_type: int, is_give_way: bool = True) -> int:
        """
        Get the recommended action based on encounter type and vessel role.

        Args:
            encounter_type: Encounter type (from get_encounter_type)
            is_give_way: Whether this vessel is the give-way vessel

        Returns:
            Recommended action (e.g., MAINTAIN, TURN_RIGHT, TURN_LEFT)
        """
        if encounter_type == self.NO_COLLISION_RISK:
            return self.MAINTAIN
            
        if encounter_type == self.HEAD_ON:
            # Rule 14: Both vessels should alter course to starboard
            return self.TURN_RIGHT
            
        if encounter_type == self.CROSSING_FROM_RIGHT:
            # Rule 15: We're the give-way vessel, we should give way
            return self.TURN_RIGHT
            
        if encounter_type == self.CROSSING_FROM_LEFT:
            if is_give_way:
                # If for some reason we're still the give-way vessel
                return self.TURN_RIGHT
            else:
                # Rule 17: Stand-on vessel should maintain course and speed
                return self.MAINTAIN
                
        if encounter_type == self.OVERTAKING:
            # Rule 13: Overtaking vessel should keep out of the way
            return self.TURN_RIGHT if is_give_way else self.MAINTAIN
            
        if encounter_type == self.BEING_OVERTAKEN:
            # Rule 13: Vessel being overtaken should maintain course and speed
            return self.MAINTAIN
            
        # Default
        return self.MAINTAIN
    
    # Action constants
    MAINTAIN = 0
    TURN_RIGHT = 1
    TURN_LEFT = 2


class ObstacleAvoidanceStrategy:
    """Base class for obstacle avoidance strategies."""
    
    def __init__(self, safety_distance: float = 10.0):
        """
        Initialize the avoidance strategy.

        Args:
            safety_distance: Minimum safety distance to maintain from obstacles
        """
        self.safety_distance = safety_distance
    
    def avoid_obstacles(self, 
                        current_pos: Tuple[float, float],
                        current_heading: float,
                        current_speed: float,
                        target_heading: float,
                        target_speed: float,
                        obstacles: List[Tuple[float, float, float]]) -> Tuple[float, float]:
        """
        Compute avoidance heading and speed.

        Args:
            current_pos: Current position (x, y)
            current_heading: Current heading in radians
            current_speed: Current speed in m/s
            target_heading: Desired heading in radians
            target_speed: Desired speed in m/s
            obstacles: List of obstacles (x, y, radius)

        Returns:
            New heading and speed after obstacle avoidance
        """
        # Abstract method to be implemented by subclasses
        raise NotImplementedError("Subclasses must implement this method.")


class APFAvoidanceStrategy(ObstacleAvoidanceStrategy):
    """Artificial Potential Field (APF) based obstacle avoidance."""
    
    def __init__(self, 
                 safety_distance: float = 10.0,
                 attractive_gain: float = 1.0,
                 repulsive_gain: float = 100.0,
                 influence_distance: float = 20.0,
                 max_heading_change: float = np.pi/4,  # 45 degrees
                 max_speed_reduction: float = 0.5):
        """
        Initialize the APF avoidance strategy.

        Args:
            safety_distance: Minimum safety distance from obstacles
            attractive_gain: Gain for attractive force (goal)
            repulsive_gain: Gain for repulsive force (obstacles)
            influence_distance: Distance of obstacle influence
            max_heading_change: Maximum heading change per step
            max_speed_reduction: Maximum speed reduction factor
        """
        super().__init__(safety_distance)
        self.attractive_gain = attractive_gain
        self.repulsive_gain = repulsive_gain
        self.influence_distance = influence_distance
        self.max_heading_change = max_heading_change
        self.max_speed_reduction = max_speed_reduction
    
    def avoid_obstacles(self, 
                        current_pos: Tuple[float, float],
                        current_heading: float,
                        current_speed: float,
                        target_heading: float,
                        target_speed: float,
                        obstacles: List[Tuple[float, float, float]]) -> Tuple[float, float]:
        """
        Compute avoidance heading and speed using APF.

        Args:
            current_pos: Current position (x, y)
            current_heading: Current heading in radians
            current_speed: Current speed in m/s
            target_heading: Desired heading in radians
            target_speed: Desired speed in m/s
            obstacles: List of obstacles (x, y, radius)

        Returns:
            New heading and speed after obstacle avoidance
        """
        if not obstacles:
            return target_heading, target_speed
            
        # Calculate goal direction (attractive force)
        fx_att = self.attractive_gain * np.cos(target_heading)
        fy_att = self.attractive_gain * np.sin(target_heading)
        
        # Calculate obstacle repulsive forces
        fx_rep, fy_rep, min_dist_factor = self._calculate_repulsive_forces(current_pos, obstacles)
        
        # Combine forces
        fx_total = fx_att + fx_rep
        fy_total = fy_att + fy_rep
        
        # Calculate resultant heading
        new_heading = np.arctan2(fy_total, fx_total)
        
        # Limit heading change
        heading_diff = new_heading - current_heading
        heading_diff = ((heading_diff + np.pi) % (2 * np.pi)) - np.pi  # Normalize to [-pi, pi]
        
        if abs(heading_diff) > self.max_heading_change:
            # Limit the heading change
            new_heading = current_heading + self.max_heading_change * np.sign(heading_diff)
            
        # Calculate speed reduction based on obstacle proximity
        speed_factor = 1.0 - self.max_speed_reduction * (1.0 - min_dist_factor)
        speed_factor = max(0.3, min(1.0, speed_factor))  # Limit speed reduction
        
        new_speed = target_speed * speed_factor
        
        return new_heading, new_speed
    
    def _calculate_repulsive_forces(self, 
                                   current_pos: Tuple[float, float],
                                   obstacles: List[Tuple[float, float, float]]) -> Tuple[float, float, float]:
        """
        Calculate repulsive forces from obstacles.

        Args:
            current_pos: Current position (x, y)
            obstacles: List of obstacles (x, y, radius)

        Returns:
            x-component, y-component of repulsive force, minimum distance factor
        """
        fx_rep = 0.0
        fy_rep = 0.0
        min_dist_factor = 1.0  # 1.0 means far away, 0.0 means touching
        
        for obs in obstacles:
            ox, oy, radius = obs
            
            # Distance to obstacle center minus radius
            dx = current_pos[0] - ox
            dy = current_pos[1] - oy
            distance = np.sqrt(dx**2 + dy**2) - radius
            
            # Normalize direction
            if distance > 0:
                nx = dx / (distance + radius)
                ny = dy / (distance + radius)
            else:
                # Already inside obstacle - move directly away
                if dx == 0 and dy == 0:
                    # Directly at center, pick arbitrary direction
                    nx, ny = 1.0, 0.0
                else:
                    norm = np.sqrt(dx**2 + dy**2)
                    nx = dx / norm
                    ny = dy / norm
            
            # Repulsive force inversely proportional to distance
            if distance <= self.influence_distance:
                # Calculate influence factor
                if distance <= self.safety_distance:
                    # Very strong repulsion inside safety distance
                    factor = self.repulsive_gain
                    min_dist_factor = min(min_dist_factor, 0.0)
                else:
                    # Gradually reducing repulsion
                    dist_ratio = (self.influence_distance - distance) / (self.influence_distance - self.safety_distance)
                    factor = self.repulsive_gain * dist_ratio
                    min_dist_factor = min(min_dist_factor, (distance - self.safety_distance) / 
                                        (self.influence_distance - self.safety_distance))
                
                fx_rep += factor * nx
                fy_rep += factor * ny
        
        return fx_rep, fy_rep, min_dist_factor


class COLREGAvoidanceStrategy(APFAvoidanceStrategy):
    """COLREG-compliant obstacle avoidance strategy."""
    
    def __init__(self, 
                 safety_distance: float = 12.0,
                 attractive_gain: float = 1.0,
                 repulsive_gain: float = 120.0,
                 influence_distance: float = 25.0,
                 tcpa_threshold: float = 120.0,
                 dcpa_threshold: float = 20.0):
        """
        Initialize the COLREG-compliant avoidance strategy.

        Args:
            safety_distance: Minimum safety distance from obstacles
            attractive_gain: Gain for attractive force (goal)
            repulsive_gain: Gain for repulsive force (obstacles)
            influence_distance: Distance of obstacle influence
            tcpa_threshold: Time to closest point of approach threshold
            dcpa_threshold: Distance at closest point of approach threshold
        """
        super().__init__(
            safety_distance=safety_distance,
            attractive_gain=attractive_gain,
            repulsive_gain=repulsive_gain,
            influence_distance=influence_distance
        )
        
        self.colreg = COLREGRules(
            tcpa_threshold=tcpa_threshold,
            dcpa_threshold=dcpa_threshold
        )
        
        # Override default max heading change for more drastic avoidance
        self.max_heading_change = np.pi/3  # 60 degrees
    
    def avoid_obstacles(self, 
                        current_pos: Tuple[float, float],
                        current_heading: float,
                        current_speed: float,
                        target_heading: float,
                        target_speed: float,
                        obstacles: List[Tuple[float, float, float]],
                        dynamic_obstacles: Optional[List[Dict]] = None) -> Tuple[float, float]:
        """
        Compute COLREG-compliant avoidance heading and speed.

        Args:
            current_pos: Current position (x, y)
            current_heading: Current heading in radians
            current_speed: Current speed in m/s
            target_heading: Desired heading in radians
            target_speed: Desired speed in m/s
            obstacles: List of static obstacles (x, y, radius)
            dynamic_obstacles: List of dynamic obstacles with position, heading, speed

        Returns:
            New heading and speed after obstacle avoidance
        """
        # Handle static obstacles using APF
        new_heading, new_speed = super().avoid_obstacles(
            current_pos, current_heading, current_speed, 
            target_heading, target_speed, obstacles
        )
        
        # Handle dynamic obstacles with COLREG rules
        if dynamic_obstacles:
            for obs in dynamic_obstacles:
                # Get encounter type
                encounter_type = self.colreg.get_encounter_type(
                    current_pos, current_heading, current_speed,
                    obs['position'], obs['heading'], obs['speed']
                )
                
                if encounter_type != self.colreg.NO_COLLISION_RISK:
                    # Get recommended action
                    action = self.colreg.get_recommended_action(encounter_type)
                    
                    # Apply the recommended action
                    if action == self.colreg.TURN_RIGHT:
                        # Turn right by 30 degrees
                        colreg_heading = current_heading - np.pi/6
                    elif action == self.colreg.TURN_LEFT:
                        # Turn left by 30 degrees
                        colreg_heading = current_heading + np.pi/6
                    else:  # MAINTAIN
                        colreg_heading = current_heading
                    
                    # Blend COLREG heading with APF heading using a weight
                    # The weight could be based on collision risk
                    weight = 0.7  # Prioritize COLREG rules
                    new_heading = self._blend_headings(colreg_heading, new_heading, weight)
                    
                    # Adjust speed for collision avoidance
                    if encounter_type == self.colreg.CROSSING_FROM_RIGHT:
                        # Slow down when giving way
                        new_speed = new_speed * 0.7
        
        return new_heading, new_speed
    
    def _blend_headings(self, heading1: float, heading2: float, weight: float) -> float:
        """
        Blend two headings with weighting.

        Args:
            heading1: First heading in radians
            heading2: Second heading in radians
            weight: Weight for first heading [0,1]

        Returns:
            Blended heading in radians
        """
        # Convert to unit vectors
        x1, y1 = np.cos(heading1), np.sin(heading1)
        x2, y2 = np.cos(heading2), np.sin(heading2)
        
        # Weighted blend
        x = weight * x1 + (1 - weight) * x2
        y = weight * y1 + (1 - weight) * y2
        
        # Convert back to heading
        blended_heading = np.arctan2(y, x)
        
        return blended_heading


class HybridAvoidanceStrategy:
    """
    Hybrid obstacle avoidance strategy that combines multiple approaches.
    """
    
    def __init__(self, 
                 dwa_planner: DynamicWindowPlanner,
                 apf_strategy: APFAvoidanceStrategy,
                 colreg_strategy: COLREGAvoidanceStrategy,
                 usv_model: USVDynamicModel):
        """
        Initialize the hybrid avoidance strategy.

        Args:
            dwa_planner: Dynamic Window Approach planner
            apf_strategy: Artificial Potential Field strategy
            colreg_strategy: COLREG-compliant strategy
            usv_model: USV dynamic model
        """
        self.dwa = dwa_planner
        self.apf = apf_strategy
        self.colreg = colreg_strategy
        self.usv_model = usv_model
        
        # Strategy selection weights (to be adapted based on conditions)
        self.strategy_weights = {
            'dwa': 0.4,
            'apf': 0.3,
            'colreg': 0.3
        }
    
    def set_strategy_weights(self, weights: Dict[str, float]):
        """
        Set the strategy selection weights.

        Args:
            weights: Dictionary of strategy weights
        """
        # Normalize weights
        total = sum(weights.values())
        if total > 0:
            self.strategy_weights = {k: v / total for k, v in weights.items()}
        else:
            self.strategy_weights = weights
    
    def avoid_obstacles(self, 
                        current_state: np.ndarray,
                        target_heading: float,
                        target_speed: float,
                        static_obstacles: List[Tuple[float, float, float]],
                        dynamic_obstacles: Optional[List[Dict]] = None) -> Tuple[float, float]:
        """
        Compute avoidance heading and speed using hybrid approach.

        Args:
            current_state: Current state vector [x, y, heading, u, v, r]
            target_heading: Desired heading in radians
            target_speed: Desired speed in m/s
            static_obstacles: List of static obstacles (x, y, radius)
            dynamic_obstacles: List of dynamic obstacles with position, heading, speed

        Returns:
            New heading and speed after obstacle avoidance
        """
        # Extract current position, heading, and speed
        current_pos = (current_state[0], current_state[1])
        current_heading = current_state[2]
        current_speed = current_state[3]  # Assuming index 3 is surge velocity
        
        # Prepare state vector for DWA
        dwa_state = np.array([current_state[0], current_state[1], current_state[2], 
                               current_speed, current_state[4]])
        
        # Set goal for DWA (based on target heading and some distance ahead)
        goal_dist = 50.0  # meters
        goal_x = current_pos[0] + goal_dist * np.cos(target_heading)
        goal_y = current_pos[1] + goal_dist * np.sin(target_heading)
        goal = (goal_x, goal_y)
        
        # Get avoidance control from DWA
        v_dwa, yawrate_dwa, _ = self.dwa.plan(dwa_state, goal, static_obstacles)
        
        # Convert DWA output to heading and speed
        # Note: DWA provides velocity and yaw rate, need to convert to heading
        heading_dwa = current_heading + yawrate_dwa * 0.5  # Simplification for small dt
        heading_dwa = ((heading_dwa + np.pi) % (2 * np.pi)) - np.pi  # Normalize
        
        # Get avoidance control from APF
        heading_apf, speed_apf = self.apf.avoid_obstacles(
            current_pos, current_heading, current_speed, 
            target_heading, target_speed, static_obstacles
        )
        
        # Get avoidance control from COLREG
        heading_colreg, speed_colreg = self.colreg.avoid_obstacles(
            current_pos, current_heading, current_speed, 
            target_heading, target_speed, static_obstacles, dynamic_obstacles
        )
        
        # Combine strategies with weights
        heading_components = [
            (heading_dwa, self.strategy_weights['dwa']),
            (heading_apf, self.strategy_weights['apf']),
            (heading_colreg, self.strategy_weights['colreg'])
        ]
        
        speed_components = [
            (v_dwa, self.strategy_weights['dwa']),
            (speed_apf, self.strategy_weights['apf']),
            (speed_colreg, self.strategy_weights['colreg'])
        ]
        
        # Calculate weighted heading using vector addition
        x_component = sum(np.cos(h) * w for h, w in heading_components)
        y_component = sum(np.sin(h) * w for h, w in heading_components)
        final_heading = np.arctan2(y_component, x_component)
        
        # Calculate weighted speed
        final_speed = sum(s * w for s, w in speed_components)
        
        return final_heading, final_speed
    
    def get_avoidance_controls(self, 
                             current_state: np.ndarray,
                             target_heading: float,
                             target_speed: float,
                             static_obstacles: List[Tuple[float, float, float]],
                             dynamic_obstacles: Optional[List[Dict]] = None) -> Tuple[float, float]:
        """
        Get the avoidance controls (thrust and moment) for the USV.

        Args:
            current_state: Current state vector [x, y, heading, u, v, r]
            target_heading: Desired heading in radians
            target_speed: Desired speed in m/s
            static_obstacles: List of static obstacles (x, y, radius)
            dynamic_obstacles: List of dynamic obstacles with position, heading, speed

        Returns:
            Tuple of (thrust, moment) controls
        """
        # Get avoidance heading and speed
        avoidance_heading, avoidance_speed = self.avoid_obstacles(
            current_state, target_heading, target_speed,
            static_obstacles, dynamic_obstacles
        )
        
        # Calculate heading error
        heading_error = avoidance_heading - current_state[2]
        heading_error = ((heading_error + np.pi) % (2 * np.pi)) - np.pi  # Normalize to [-pi, pi]
        
        # Calculate speed error
        speed_error = avoidance_speed - current_state[3]
        
        # Simple P controller for demonstration
        # In a real system, this would use the full control system
        K_thrust = 10.0
        K_moment = 20.0
        
        thrust = K_thrust * speed_error
        moment = K_moment * heading_error
        
        # Limit controls
        max_thrust = 100.0
        max_moment = 50.0
        
        thrust = np.clip(thrust, -max_thrust, max_thrust)
        moment = np.clip(moment, -max_moment, max_moment)
        
        return thrust, moment


class ObstacleAvoidanceSystem:
    """
    System for obstacle detection and avoidance.
    
    This class integrates various obstacle detection sensors and
    avoidance strategies to provide a complete collision avoidance system.
    """
    
    def __init__(self, 
                 avoidance_strategy: HybridAvoidanceStrategy,
                 detection_range: float = 100.0,
                 visualization_enabled: bool = False):
        """
        Initialize the obstacle avoidance system.
        
        Args:
            avoidance_strategy: Strategy to avoid obstacles
            detection_range: Maximum detection range [m]
            visualization_enabled: Whether to generate visualization data
        """
        self.avoidance_strategy = avoidance_strategy
        self.detection_range = detection_range
        self.visualization_enabled = visualization_enabled
        
        # Initialize obstacle lists
        self.static_obstacles = []
        self.dynamic_obstacles = []
        self.detected_obstacles = []
        self.detected_dynamic_obstacles = []
        
        # Adding initial dummy obstacles for visualization testing
        if visualization_enabled:
            self._add_test_obstacles()
            
    def _add_test_obstacles(self):
        """Add test obstacles for visualization testing"""
        # Add some static obstacles
        self.static_obstacles = [
            (20.0, 20.0, 5.0),  # x, y, radius
            (40.0, -30.0, 8.0),
            (-10.0, 50.0, 3.0),
            (-40.0, -20.0, 6.0)
        ]
        
        # Add some dynamic obstacles
        self.dynamic_obstacles = [
            {
                'position': (30.0, 0.0),
                'heading': 0.0,
                'speed': 2.0,
                'radius': 4.0
            },
            {
                'position': (-20.0, 40.0),
                'heading': 3.14/4,  # 45 degrees
                'speed': 1.5,
                'radius': 3.0
            }
        ]
        
        # Update detected obstacles too
        self.detected_obstacles = self.static_obstacles
        self.detected_dynamic_obstacles = self.dynamic_obstacles
        
    def update_obstacles(self, 
                        static_obstacles: List[Tuple[float, float, float]],
                        dynamic_obstacles: Optional[List[Dict]] = None):
        """
        Update the system's knowledge of obstacles.
        
        Args:
            static_obstacles: List of static obstacles (x, y, radius)
            dynamic_obstacles: List of dynamic obstacles (Dict with position, heading, speed, radius)
        """
        # Only update if non-empty lists are provided
        if static_obstacles:
            self.static_obstacles = static_obstacles
            self.detected_obstacles = self.static_obstacles
        
        if dynamic_obstacles:
            self.dynamic_obstacles = dynamic_obstacles
            self.detected_dynamic_obstacles = self.dynamic_obstacles
        
        # If both lists are empty and we're in visualization mode, add test obstacles
        if (not static_obstacles and not dynamic_obstacles and 
            not self.static_obstacles and not self.dynamic_obstacles and
            self.visualization_enabled):
            self._add_test_obstacles()
    
    def compute_avoidance_controls(self, 
                                  current_state: np.ndarray,
                                  target_heading: float,
                                  target_speed: float) -> Tuple[float, float]:
        """
        Compute avoidance controls based on current state and targets.
        
        Args:
            current_state: Current USV state [x, y, heading, u, v, r]
            target_heading: Desired heading in radians
            target_speed: Desired speed in m/s
            
        Returns:
            Tuple of (thrust, moment) controls
        """
        try:
            # Filter obstacles by distance for efficiency
            current_pos = (current_state[0], current_state[1])
            
            # Update dynamic obstacles positions before computing controls
            for obs in self.dynamic_obstacles:
                if not isinstance(obs, dict) or 'position' not in obs:
                    continue
                
                heading = obs.get('heading', 0.0)
                speed = obs.get('speed', 0.0)
                
                # Current position
                x, y = obs['position']
                
                # Ensure obstacles stay in the visualization area
                # If they go too far, bring them back by reversing direction
                if abs(x) > 100 or abs(y) > 100:
                    # Reverse direction
                    heading = heading + np.pi
                    heading = ((heading + np.pi) % (2 * np.pi)) - np.pi  # Normalize to [-pi, pi]
                    obs['heading'] = heading
            
            # Filter static obstacles
            nearby_obstacles = []
            for obs in self.static_obstacles:
                if not isinstance(obs, (list, tuple)) or len(obs) < 3:
                    continue
                    
                ox, oy = obs[0], obs[1]
                radius = obs[2] if len(obs) > 2 else 3.0
                dist = np.sqrt((current_pos[0] - ox)**2 + (current_pos[1] - oy)**2) - radius
                
                # Calculate distance for visualization
                obstacle_with_dist = list(obs)
                obstacle_with_dist.append(dist)  # Add distance info for visualization
                
                if dist < self.detection_range:
                    nearby_obstacles.append(obs)
            
            # Filter dynamic obstacles
            nearby_dynamic_obstacles = []
            for obs in self.dynamic_obstacles:
                if not isinstance(obs, dict):
                    continue
                    
                pos = obs.get('position', obs.get('initial_position'))
                if not isinstance(pos, (list, tuple)) or len(pos) < 2:
                    continue
                    
                dist = np.sqrt((current_pos[0] - pos[0])**2 + (current_pos[1] - pos[1])**2)
                
                # Add distance info for visualization
                obs_with_dist = obs.copy()
                obs_with_dist['distance'] = dist
                
                if dist < self.detection_range:
                    nearby_dynamic_obstacles.append(obs_with_dist)
            
            # For visualization testing, add some randomness to make the simulation interesting
            if self.visualization_enabled:
                # Add slight randomness to target heading for more dynamic visualization
                noise_heading = target_heading + (np.random.random() - 0.5) * 0.1  # ±0.05 rad noise
                noise_speed = target_speed * (1.0 + (np.random.random() - 0.5) * 0.2)  # ±10% noise
                
                # Compute controls with noise
                thrust, moment = self.avoidance_strategy.get_avoidance_controls(
                    current_state, 
                    noise_heading, 
                    noise_speed,
                    nearby_obstacles,
                    nearby_dynamic_obstacles
                )
            else:
                # Normal operation without added randomness
                thrust, moment = self.avoidance_strategy.get_avoidance_controls(
                    current_state, 
                    target_heading, 
                    target_speed,
                    nearby_obstacles,
                    nearby_dynamic_obstacles
                )
            
            # For demonstration, make sure we have non-zero values to show movement
            if abs(thrust) < 5.0:
                thrust = 10.0  # Ensure minimum thrust for movement
            
            return thrust, moment
            
        except Exception as e:
            import traceback
            print(f"Error in compute_avoidance_controls: {e}")
            traceback.print_exc()
            # Return default controls in case of error
            return 10.0, 0.0  # Ensure non-zero thrust for movement
    
    def get_visualization_data(self):
        """
        Get obstacle data for visualization.
        
        Returns:
            Dictionary with static and dynamic obstacle data
        """
        try:
            # Format obstacle data for visualization
            static_vis_data = []
            for obs in self.detected_obstacles:
                if not isinstance(obs, (list, tuple)) or len(obs) < 2:
                    continue
                    
                # Add distance data if available
                distance = None
                if len(obs) > 3:
                    distance = obs[3]
                    
                static_vis_data.append({
                    'x': float(obs[0]),
                    'y': float(obs[1]),
                    'radius': float(obs[2]) if len(obs) > 2 else 3.0,
                    'distance': float(distance) if distance is not None else None
                })
                
            dynamic_vis_data = []
            for obs in self.detected_dynamic_obstacles:
                if not isinstance(obs, dict):
                    continue
                    
                # Get position (either current_position or initial_position)
                pos = obs.get('position', obs.get('initial_position'))
                if not isinstance(pos, (list, tuple)) or len(pos) < 2:
                    continue
                    
                distance = obs.get('distance')
                    
                dynamic_vis_data.append({
                    'x': float(pos[0]),
                    'y': float(pos[1]),
                    'radius': float(obs.get('radius', 3.0)),
                    'heading': float(obs.get('heading', 0.0)),
                    'speed': float(obs.get('speed', 0.0)),
                    'distance': float(distance) if distance is not None else None
                })
                
            # Add visualization metadata for UI elements
            collision_detection = {
                'nearby_obstacles': 0,
                'collision_detected': False
            }
            
            # Check for nearby obstacles and collisions
            for obs in static_vis_data + dynamic_vis_data:
                if obs.get('distance') is not None:
                    if obs['distance'] < 10.0:
                        collision_detection['nearby_obstacles'] += 1
                    if obs['distance'] < 3.0:
                        collision_detection['collision_detected'] = True
                
            return {
                'static': static_vis_data,
                'dynamic': dynamic_vis_data,
                'visualization': {
                    'collision_detection': collision_detection
                }
            }
        except Exception as e:
            print(f"Error in get_visualization_data: {e}")
            # Return empty data in case of error
            return {'static': [], 'dynamic': []}
