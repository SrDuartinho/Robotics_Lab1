# camera.py
import numpy as np
from constants import SCREEN_WIDTH_PX, SCREEN_HEIGHT_PX

class Camera:
    """
    Manages the viewport (pan and zoom).
    Translates WCS (World) coordinates to Screen (Pixel) coordinates.
    """
    def __init__(self, x, y, zoom=1.0):
        # WCS point the camera is looking at
        self.x = x
        self.y = y
        self.zoom = zoom
        
        self.screen_center = np.array([SCREEN_WIDTH_PX / 2, SCREEN_HEIGHT_PX / 2])
        
    def update(self, car):
        """Follows the car's WCS position."""
        self.x = car.x
        self.y = car.y
        # (Add zoom input handling here if desired)

    def world_to_screen_points(self, world_points_array):
        """
        Converts a numpy array of WCS points to screen points.
        """
        if world_points_array.size == 0:
            return np.array([])
            
        # 1. Translate points relative to camera
        relative_points = world_points_array - np.array([self.x, self.y])
        
        # 2. Scale by zoom
        zoomed_points = relative_points * self.zoom
        
        # 3. Translate to screen center
        screen_points = zoomed_points + self.screen_center
        
        return screen_points.astype(int)

    def world_to_screen_scalar(self, world_pos_tuple):
        """Helper function to convert a single (x, y) WCS tuple."""
        screen_point = self.world_to_screen_points(np.array([world_pos_tuple]))
        return tuple(screen_point[0])