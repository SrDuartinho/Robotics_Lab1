import pygame
import numpy as np
from constants import SCREEN_WIDTH_PX, SCREEN_HEIGHT_PX, DEFAULT_ZOOM, ZOOM_SPEED

class Viewport:
    """
    Manages the viewport (pan and zoom).
    Translates WCS (World) coordinates to Screen (Pixel) coordinates.
    """
    def __init__(self, x, y, zoom=DEFAULT_ZOOM):
        # WCS point the camera is looking at
        self.x = x
        self.y = y
        self.zoom = zoom
        
    @property
    def screen_center(self):
        return np.array([SCREEN_WIDTH_PX / 2, SCREEN_HEIGHT_PX / 2])
        
    def update(self, car):
        """Follows the car's WCS position."""
        self.x = car.x
        self.y = car.y
        
        # --- Handle Zoom Input (Optional) ---
        keys = pygame.key.get_pressed()
        if keys[pygame.K_EQUALS] or keys[pygame.K_PLUS]:
             self.zoom = min(self.zoom + ZOOM_SPEED, 3.0) # Cap zoom in
        if keys[pygame.K_MINUS]:
             self.zoom = max(self.zoom - ZOOM_SPEED, 0.1) # Cap zoom out


    def world_to_screen_points(self, world_points_array):
        """
        Converts a numpy array of WCS points to screen points.
        """
        if world_points_array.size == 0:
            return np.array([])
            
        # 1. Translate points relative to camera's WCS position
        relative_points = world_points_array - np.array([self.x, self.y])
        
        # 2. Scale by zoom
        zoomed_points = relative_points * self.zoom
        
        # 3. Translate to screen center
        screen_points = zoomed_points + self.screen_center
        
        return screen_points.astype(int)

    def world_to_screen_scalar(self, world_pos_tuple):
        """Helper function to convert a single (x, y) WCS tuple."""
        # Convert tuple to numpy array for the function
        world_pos_array = np.array([world_pos_tuple])
        screen_point = self.world_to_screen_points(world_pos_array)
        return tuple(screen_point[0])

    def world_to_screen_poly(self, poly_points):
        """Alias for world_to_screen_points, for clarity."""
        return self.world_to_screen_points(poly_points)