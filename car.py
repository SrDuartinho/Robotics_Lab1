import numpy as np
import math
from constants import *

class Car:
    """
    Kinematic bicycle model for a car-like robot.
    
    State variables (WCS):
        x, y: Position of the rear axle center (pixels)
        theta: Heading angle (radians)
        
    Configuration (Car frame):
        phi: Steering angle (radians, relative to car body)
        L: Wheelbase (pixels)
    """
    
    def __init__(self, x, y, theta, phi=0.0, wheelbase_px=None, 
                 max_steer=None, steer_rate=None):
        
        # State (WCS)
        self.x = x
        self.y = y
        self.theta = theta
        
        # Configuration (Car frame)
        self.phi = phi
        
        # Physical Properties
        self.L = wheelbase_px if wheelbase_px is not None else CAR_WHEELBASE_PX
        self.max_steer = max_steer if max_steer is not None else math.radians(MAX_STEER_ANGLE_RAD)
        self.steer_rate = steer_rate if steer_rate is not None else math.radians(STEER_RATE_RPS)

        # Telemetry/Debugging
        self._last_V = 0.0
        self._last_omega_s = 0.0
    
    def update(self, V, omega_s, dt):
        """
        Update car state using kinematic bicycle model.
        
        Args:
            V: Linear velocity in car frame (pixels/sec)
            omega_s: Steering rate (radians/sec)
            dt: Time step (seconds)
        """
        self._last_V = V
        self._last_omega_s = omega_s

        # 1. Kinematic Bicycle Model (Rear Axle Reference)
        x_dot     = np.cos(self.theta) * np.cos(self.phi) * V
        y_dot     = np.sin(self.theta) * np.cos(self.phi) * V
        theta_dot = (np.sin(self.phi) / self.L) * V
        phi_dot   = omega_s
        
        # 2. Integration
        self.x     += x_dot * dt
        self.y     += y_dot * dt
        self.theta += theta_dot * dt
        self.phi   += phi_dot * dt

        # Limit Steering angle
        self.phi = max(-self.max_steer, min(self.phi, self.max_steer))
        
        # Normalize theta to [-π, π]
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
    
    def get_state(self):
        """Returns full state vector: x, y, theta, phi, L."""
        return self.x, self.y, self.theta, self.phi, self.L
    
    def get_velocity(self):
        """Returns last commanded velocity (for HUD/telemetry)."""
        return self._last_V
        
    def reset(self, x=None, y=None, theta=None):
        """Reset car to a new state."""
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if theta is not None:
            self.theta = theta
        self.phi = 0.0
        self._last_V = 0.0
        self._last_omega_s = 0.0
