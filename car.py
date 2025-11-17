# car.py
import pygame
import math
import numpy as np
from constants import (COLORS, MAX_SPEED_PPF, REVERSE_SPEED_PPF,
                       MAX_STEER_ANGLE_DEG, STEER_RATE_DPF, CAR_WHEELBASE_PX)

class Car:
    """
    Represents the car agent, simplified as a "red dot".
    - Stores its own pose (x, y, angle) in the WCS.
    - Implements a simple DEBUG movement model (non-kinematic).
    """
    def __init__(self, x, y, angle=0.0):
        # --- WCS Pose (The car's ground truth) ---
        self.x = x
        self.y = y
        self.angle = angle  # 0=Up (North), 90=Right (East)

        # --- WCS Velocities (For Debug Movement) ---
        self.x_vel = 0.0
        self.y_vel = 0.0
        
        # --- Kinematic Controls (Kept for separate testing) ---
        self.steer_angle = 0.0  # Current steering angle (phi)

        self.sensors = []
        
        # Simple radius for the "red dot"
        self.radius = 8

    def add_sensor(self, sensor):
        """Adds a sensor to the car's sensor list."""
        self.sensors.append(sensor)

    def update(self, keys, dt):
        """
        Updates the car's state:
        1. Handle user input to set WCS velocities and steering angle.
        2. Apply the simple debug movement.
        """
        self._handle_debug_input(keys, dt)
        self._apply_debug_movement()
        
        # --- SENSORS DEACTIVATED ---
        # for sensor in self.sensors:
        #     sensor.update(self.x, self.y, self.angle)

    def _handle_debug_input(self, keys, dt):
        """
        Reads keyboard input.
        - 8/2/4/6 (and numpad) control simple WCS movement.
        - Arrow keys (LEFT/RIGHT) control the car's angle (for testing).
        """
        
        # --- WCS Movement (Debug) ---
        self.x_vel = 0
        self.y_vel = 0
        
        if keys[pygame.K_UP] or keys[pygame.K_8] or keys[pygame.K_KP8]:
            self.y_vel = -MAX_SPEED_PPF # WCS Up
        elif keys[pygame.K_DOWN] or keys[pygame.K_2] or keys[pygame.K_KP2]:
            self.y_vel = MAX_SPEED_PPF  # WCS Down
            
        if keys[pygame.K_4] or keys[pygame.K_KP4]:
            self.x_vel = -MAX_SPEED_PPF # WCS Left
        elif keys[pygame.K_6] or keys[pygame.K_KP6]:
            self.x_vel = MAX_SPEED_PPF  # WCS Right

        # --- Angle Control (Separate) ---
        # Use arrow keys to test changing the angle
        if keys[pygame.K_LEFT]:
            self.angle = (self.angle + STEER_RATE_DPF * 2) % 360
        elif keys[pygame.K_RIGHT]:
            self.angle = (self.angle - STEER_RATE_DPF * 2) % 360

    def _apply_debug_movement(self):
        """Applies the simple X/Y WCS movement."""
        self.x += self.x_vel
        self.y += self.y_vel
        
        # Note: self.angle does NOT affect movement in this debug mode.

    # --- KINEMATIC MODEL (DEACTIVATED) ---
    # def _apply_kinematics(self, dt):
    #     """
    #     Updates self.x, self.y, and self.angle based on the car kinematic model
    #     from lecture_2.pdf, slide 18.
    #     """
    #     if self.speed == 0:
    #         return # No movement
    # 
    #     steer_rad = math.radians(self.steer_angle)
    #     angle_rad = math.radians(self.angle)
    # 
    #     theta_dot = (self.speed * math.tan(-steer_rad)) / CAR_WHEELBASE_PX
    #     x_dot = self.speed * math.sin(angle_rad)
    #     y_dot = -self.speed * math.cos(angle_rad) # Negative b/c y-axis is down
    #     
    #     self.x += x_dot
    #     self.y += y_dot
    #     self.angle = (self.angle + math.degrees(theta_dot)) % 360

    def draw(self, screen, camera):
        """Draws the car (red dot) and its sensors."""
        
        # 1. Get screen position from camera
        screen_pos = camera.world_to_screen_scalar((self.x, self.y))
        
        # 2. Draw the car's "red dot"
        pygame.draw.circle(screen, COLORS['car_red'], screen_pos, self.radius)
        
        # --- SENSORS DEACTIVATED ---
        # for sensor in self.sensors:
        #     sensor.draw(screen, camera)