# sensor.py
import pygame
import math
from constants import COLORS, SENSOR_BASE_RADIUS_PX

class Sensor:
    """
    Represents a sensor as a line.
    - Defined by its pose (x, y, angle) in the CCS (local to the car).
    - Calculates its own pose in the WCS every frame (Sensor -> Car -> WCS).
    """
    def __init__(self, x_local, y_local, angle_local=0, range_px=100):
        # --- CCS Pose (Definition) ---
        # This is the sensor's mounting position relative to the car's center
        self.x_local = x_local
        self.y_local = y_local
        self.angle_local = angle_local
        self.range_px = range_px
        
        # --- WCS Pose (Calculated) ---
        self.x_world = 0
        self.y_world = 0
        self.angle_world = 0
        self.beam_end_wcs = (0, 0) # The WCS end-point of the sensor beam
    
    def update(self, car_x_wcs, car_y_wcs, car_angle_wcs):
        """
        Calculates the sensor's WCS pose based on the car's WCS pose.
        This is the T_sensor_to_car * T_car_to_wcs transformation.
        """
        
        # 1. Calculate the sensor's global WCS angle
        self.angle_world = (car_angle_wcs + self.angle_local) % 360
        
        # 2. Calculate the sensor's base WCS position
        #    (Rotate local offset and add to car's WCS pos)
        rad_car = math.radians(car_angle_wcs)
        sin_a = math.sin(rad_car)
        cos_a = math.cos(rad_car)
        
        # Rotate the local (x,y) offset by the car's angle
        x_offset = self.x_local * cos_a - self.y_local * sin_a
        y_offset = self.x_local * sin_a + self.y_local * cos_a
        
        # Add the rotated offset to the car's WCS position
        self.x_world = car_x_wcs + x_offset
        self.y_world = car_y_wcs - y_offset # Pygame y-axis is inverted

        # 3. Calculate beam end point in WCS
        rad_world = math.radians(self.angle_world)
        self.beam_end_wcs = (
            self.x_world + self.range_px * math.sin(rad_world),
            self.y_world - self.range_px * math.cos(rad_world) # Pygame y-axis
        )
        
    def get_reading(self, environment):
        """
        Performs raycasting against the environment's WCS obstacles.
        (To be implemented later)
        """
        pass

    def draw(self, screen, camera):
        """Draws the sensor beam in the WCS, viewed by the camera."""
        
        # 1. Convert the beam's WCS start/end points to screen points
        start_pos = camera.world_to_screen_scalar((self.x_world, self.y_world))
        end_pos = camera.world_to_screen_scalar(self.beam_end_wcs)
        
        # 2. Draw the beam (the "line")
        pygame.draw.line(screen, COLORS['sensor_beam'], start_pos, end_pos, 1)
        
        # 3. Draw the sensor's base (a small dot where it connects to the car)
        pygame.draw.circle(screen, COLORS['sensor_base'], start_pos, SENSOR_BASE_RADIUS_PX)