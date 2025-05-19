"""
PID controller implementation for USV control.

This module provides a PID controller class for heading and speed control
of an Unmanned Surface Vessel.
"""

import numpy as np
from typing import Tuple, Dict, List, Optional


class PIDController:
    """
    PID controller implementation for USV control.
    """

    def __init__(self,
                 kp: float = 1.0,
                 ki: float = 0.0,
                 kd: float = 0.0,
                 windup_limit: float = 10.0,
                 output_limits: Tuple[float, float] = (-float('inf'), float('inf')),
                 dt: float = 0.1):
        """
        Initialize the PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            windup_limit: Anti-windup limit for integral term
            output_limits: Tuple of (min, max) output limits
            dt: Time step
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.windup_limit = windup_limit
        self.output_limits = output_limits
        self.dt = dt

        # Internal state
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = 0.0

    def reset(self):
        """Reset the controller internal state."""
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = 0.0

    def compute(self, setpoint: float, measurement: float, dt: Optional[float] = None) -> float:
        """
        Compute the control output based on the setpoint and measurement.

        Args:
            setpoint: Desired value
            measurement: Current measured value
            dt: Time step (optional, uses default if not provided)

        Returns:
            Control output
        """
        if dt is None:
            dt = self.dt

        # Calculate error
        error = setpoint - measurement

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.windup_limit, self.windup_limit)
        i_term = self.ki * self.integral

        # Derivative term (with setpoint derivative term removed to avoid derivative kick)
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative

        # Calculate output
        output = p_term + i_term + d_term

        # Apply output limits
        output = np.clip(output, self.output_limits[0], self.output_limits[1])

        # Update internal state
        self.prev_error = error

        return output


class HeadingController:
    """
    PID controller for USV heading control.
    """

    def __init__(self,
                 kp: float = 1.0,
                 ki: float = 0.0,
                 kd: float = 0.0,
                 max_yaw_rate: float = 1.0,
                 dt: float = 0.1):
        """
        Initialize the heading controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            max_yaw_rate: Maximum yaw rate in rad/s
            dt: Time step
        """
        self.controller = PIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            windup_limit=max_yaw_rate * 5,  # Arbitrary scaling
            output_limits=(-max_yaw_rate, max_yaw_rate),
            dt=dt
        )

    def compute(self, desired_heading: float, current_heading: float) -> float:
        """
        Compute the yaw rate command based on desired and current heading.

        Args:
            desired_heading: Desired heading in radians
            current_heading: Current heading in radians

        Returns:
            Yaw rate command in rad/s
        """
        # Calculate heading error accounting for angle wrapping
        heading_error = desired_heading - current_heading
        heading_error = ((heading_error + np.pi) % (2 * np.pi)) - np.pi

        # Use PID to compute yaw rate
        yaw_rate = self.controller.compute(0, -heading_error)

        return yaw_rate

    def reset(self):
        """Reset the controller internal state."""
        self.controller.reset()


class SpeedController:
    """
    PID controller for USV speed control.
    """

    def __init__(self,
                 kp: float = 1.0,
                 ki: float = 0.0,
                 kd: float = 0.0,
                 max_thrust: float = 100.0,
                 dt: float = 0.1):
        """
        Initialize the speed controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            max_thrust: Maximum thrust force in Newtons
            dt: Time step
        """
        self.controller = PIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            windup_limit=max_thrust * 0.5,  # Arbitrary scaling
            output_limits=(-max_thrust, max_thrust),
            dt=dt
        )

    def compute(self, desired_speed: float, current_speed: float) -> float:
        """
        Compute the thrust force command based on desired and current speed.

        Args:
            desired_speed: Desired speed in m/s
            current_speed: Current speed in m/s

        Returns:
            Thrust force command in Newtons
        """
        # Use PID to compute thrust force
        thrust = self.controller.compute(desired_speed, current_speed)

        return thrust

    def reset(self):
        """Reset the controller internal state."""
        self.controller.reset()


class USVController:
    """
    Combined PID controller for both heading and speed control of a USV.
    """

    def __init__(self,
                 heading_gains: Dict[str, float] = None,
                 speed_gains: Dict[str, float] = None,
                 max_yaw_rate: float = 0.5,
                 max_thrust: float = 100.0,
                 dt: float = 0.1):
        """
        Initialize the USV controller.

        Args:
            heading_gains: Dictionary of heading controller gains (kp, ki, kd)
            speed_gains: Dictionary of speed controller gains (kp, ki, kd)
            max_yaw_rate: Maximum yaw rate in rad/s
            max_thrust: Maximum thrust force in Newtons
            dt: Time step
        """
        # Default heading gains
        default_heading_gains = {'kp': 1.0, 'ki': 0.1, 'kd': 0.3}
        if heading_gains:
            default_heading_gains.update(heading_gains)

        # Default speed gains
        default_speed_gains = {'kp': 10.0, 'ki': 5.0, 'kd': 1.0}
        if speed_gains:
            default_speed_gains.update(speed_gains)

        # Create controllers
        self.heading_controller = HeadingController(
            kp=default_heading_gains['kp'],
            ki=default_heading_gains['ki'],
            kd=default_heading_gains['kd'],
            max_yaw_rate=max_yaw_rate,
            dt=dt
        )

        self.speed_controller = SpeedController(
            kp=default_speed_gains['kp'],
            ki=default_speed_gains['ki'],
            kd=default_speed_gains['kd'],
            max_thrust=max_thrust,
            dt=dt
        )

    def compute_control_inputs(self,
                              desired_heading: float,
                              desired_speed: float,
                              current_heading: float,
                              current_speed: float) -> Tuple[float, float]:
        """
        Compute control inputs for heading and speed control.

        Args:
            desired_heading: Desired heading in radians
            desired_speed: Desired speed in m/s
            current_heading: Current heading in radians
            current_speed: Current speed in m/s

        Returns:
            Tuple of (thrust, yaw_moment)
        """
        # Compute yaw rate command
        yaw_rate = self.heading_controller.compute(desired_heading, current_heading)

        # Compute thrust command
        thrust = self.speed_controller.compute(desired_speed, current_speed)

        # For now, we directly use the yaw rate as moment command
        # In a real system, this would be converted to rudder or thruster commands
        yaw_moment = yaw_rate * 10.0  # Scale to appropriate range

        return thrust, yaw_moment

    def reset(self):
        """Reset all controllers' internal state."""
        self.heading_controller.reset()
        self.speed_controller.reset()


class PositionController:
    """
    PID controller for USV position control.
    """

    def __init__(self,
                 kp: float = 1.0,
                 max_speed: float = 5.0,
                 dt: float = 0.1):
        """
        Initialize the position controller.

        Args:
            kp: Proportional gain
            max_speed: Maximum allowed speed in m/s
            dt: Time step
        """
        self.kp = kp
        self.max_speed = max_speed
        self.dt = dt

    def compute(self, desired_position: Tuple[float, float],
               current_position: Tuple[float, float],
               current_heading: float) -> Tuple[float, float]:
        """
        Compute the desired heading and speed based on position error.

        Args:
            desired_position: Desired position (x, y)
            current_position: Current position (x, y)
            current_heading: Current heading in radians

        Returns:
            Tuple of (desired_heading, desired_speed)
        """
        # Calculate position error vector
        error_x = desired_position[0] - current_position[0]
        error_y = desired_position[1] - current_position[1]

        # Calculate distance to target
        distance = np.sqrt(error_x**2 + error_y**2)

        # Calculate desired heading
        desired_heading = np.arctan2(error_y, error_x)

        # Calculate desired speed proportional to distance (with limit)
        desired_speed = np.clip(self.kp * distance, 0, self.max_speed)

        # If close to the target, slow down
        if distance < 1.0:
            desired_speed *= distance

        return desired_heading, desired_speed


class USVPIDController:
    """
    PID-based controller for USV that combines position, heading, and speed control.
    """

    def __init__(self,
                 Kp_heading: float = 5.0,
                 Ki_heading: float = 0.1,
                 Kd_heading: float = 1.0,
                 Kp_speed: float = 10.0,
                 Ki_speed: float = 0.5,
                 Kd_speed: float = 1.0,
                 Kp_position: float = 1.0,
                 max_yaw_rate: float = 0.5,
                 max_thrust: float = 100.0,
                 max_speed: float = 5.0,
                 dt: float = 0.1):
        """
        Initialize the USV PID controller.

        Args:
            Kp_heading: Proportional gain for heading control
            Ki_heading: Integral gain for heading control
            Kd_heading: Derivative gain for heading control
            Kp_speed: Proportional gain for speed control
            Ki_speed: Integral gain for speed control
            Kd_speed: Derivative gain for speed control
            Kp_position: Proportional gain for position control
            max_yaw_rate: Maximum yaw rate in rad/s
            max_thrust: Maximum thrust force in Newtons
            max_speed: Maximum speed in m/s
            dt: Time step
        """
        # Create heading and speed controller
        heading_gains = {'kp': Kp_heading, 'ki': Ki_heading, 'kd': Kd_heading}
        speed_gains = {'kp': Kp_speed, 'ki': Ki_speed, 'kd': Kd_speed}

        self.usv_controller = USVController(
            heading_gains=heading_gains,
            speed_gains=speed_gains,
            max_yaw_rate=max_yaw_rate,
            max_thrust=max_thrust,
            dt=dt
        )

        # Create position controller
        self.position_controller = PositionController(
            kp=Kp_position,
            max_speed=max_speed,
            dt=dt
        )

        # Store parameters
        self.dt = dt

    def compute_control_inputs(self,
                              desired_position: Tuple[float, float],
                              desired_heading: float,
                              desired_speed: float,
                              current_state: np.ndarray) -> Tuple[float, float]:
        """
        Compute control inputs for the USV using cascaded PID control.

        Args:
            desired_position: Desired position (x, y)
            desired_heading: Desired heading in radians
            desired_speed: Desired surge speed in m/s
            current_state: Current state vector [x, y, psi, u, v, r]

        Returns:
            Control input tuple (thrust, moment)
        """
        # Extract current state
        current_position = (current_state[0], current_state[1])
        current_heading = current_state[2]
        current_speed = current_state[3]  # Surge velocity

        # Determine if we're in positioning mode or heading/speed mode
        if desired_speed > 0.05:
            # We're in heading/speed tracking mode (desired speed is non-zero)
            # Use the provided desired heading and speed
            heading_cmd = desired_heading
            speed_cmd = desired_speed
        else:
            # We're in position control mode (desired speed is effectively zero)
            # Calculate desired heading and speed based on position error
            heading_cmd, speed_cmd = self.position_controller.compute(
                desired_position, current_position, current_heading
            )

        # Compute final control inputs
        thrust, moment = self.usv_controller.compute_control_inputs(
            heading_cmd, speed_cmd, current_heading, current_speed
        )

        return thrust, moment

    def reset(self):
        """Reset controllers' internal state."""
        self.usv_controller.reset()
