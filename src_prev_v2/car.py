import numpy as np
import math

class Car:
    def __init__(self, x=200, y=500, theta=-np.pi/2, phi=0.0, L=60):
        """
        theta = initial heading ( -pi/2 means pointing upward in pygame )
        L = pixel wheelbase for pygame visualization
        """
        self.x = x
        self.y = y
        self.theta = theta   # orientation
        self.phi = phi       # steering angle
        self.v = 0.0         # forward velocity (pixels/sec)
        self.w = 0.0         # angular velocity
        self.L = L           # wheelbase in pixels

    def step(self, V, omega_s, dt):
        # Kinematic model from your slides
        x_dot     = np.cos(self.theta) * np.cos(self.phi) * V
        y_dot     = np.sin(self.theta) * np.cos(self.phi) * V
        theta_dot = (np.sin(self.phi) / self.L) * V
        phi_dot   = omega_s
        # ---- LIMIT STEERING ANGLE HERE ----
        max_angle = math.radians(80)     # 80 degrees in radians
        self.phi = max(-max_angle, min(self.phi, max_angle))

        self.x     += x_dot * dt
        self.y     += y_dot * dt
        self.theta += theta_dot * dt
        self.phi   += phi_dot * dt

    def get_state(self):
        return self.x, self.y, self.theta, self.phi, self.L

    def get_front_center(self):
        """
        Returns location of the point at the front bumper.
        """
        fx = self.x + np.cos(self.theta) * self.L
        fy = self.y + np.sin(self.theta) * self.L
        return fx, fy