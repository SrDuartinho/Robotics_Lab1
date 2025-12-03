import pygame
import numpy as np
from constants import *

class Renderer:
    def __init__(self, screen, viewport):
        self.screen = screen
        self.viewport = viewport
        
    def render_environment(self, env):
        """
        Draws the environment.
        Works for ANY environment class that has 'road_poly' and 'lines' attributes.
        """
        # 1. Draw Background (Grass)
        self.screen.fill(COLORS['bg_gray'])
        
        # 2. Draw Road Asphalt
        # We use hasattr so this doesn't crash if you create a test environment without a road
        if hasattr(env, 'road_poly'):
            road_screen_poly = self.viewport.world_to_screen_poly(env.road_poly)
            
            # Pygame requires at least 3 points to draw a polygon
            if len(road_screen_poly) >= 3:
                pygame.draw.polygon(self.screen, COLORS['road_gray'], road_screen_poly)
            
        # 3. Draw Lane Lines
        if hasattr(env, 'lines'):
            for line_poly in env.lines:
                line_screen_poly = self.viewport.world_to_screen_poly(line_poly)
                
                # Only draw if the line is actually on screen (has points)
                if len(line_screen_poly) >= 3:
                    pygame.draw.polygon(self.screen, COLORS['white_line'], line_screen_poly)

    def render_car(self, car):
        """
        Draws the car using polygon transformation.
        """
        # Unpack state. 
        # Note: Ensure your Car.get_state() actually returns these 5 values!
        # If L (wheelbase) is a constant in your car class, you might need to adjust this.
        x, y, theta, phi, L = car.get_state()
        
        # --- 1. DRAW CHASSIS ---
        rear_overhang = (CAR_LENGTH_PX - CAR_WHEELBASE_PX) / 2
        
        # Corners in Car Local Coordinates [x_forward, y_left]
        chassis_local = np.array([
            [CAR_LENGTH_PX - rear_overhang,  CAR_WIDTH_PX / 2], # Front Left
            [CAR_LENGTH_PX - rear_overhang, -CAR_WIDTH_PX / 2], # Front Right
            [-rear_overhang,                -CAR_WIDTH_PX / 2], # Rear Right
            [-rear_overhang,                 CAR_WIDTH_PX / 2], # Rear Left
        ])
        
        # Transform chassis points to Screen Coordinates
        chassis_screen = self._transform_local_to_screen(chassis_local, x, y, theta)
        
        if len(chassis_screen) >= 3:
            # Draw filled body
            pygame.draw.polygon(self.screen, COLORS['car_body'], chassis_screen)
            # Draw outline (Dark Red)
            pygame.draw.polygon(self.screen, (50, 0, 0), chassis_screen, 2)

        # --- 2. DRAW WHEELS ---
        wheel_len = CAR_LENGTH_PX * 0.2
        wheel_wid = CAR_WIDTH_PX * 0.25
        
        # Rear Wheels (Fixed)
        self._draw_wheel(x, y, theta, 0, 0,  CAR_WIDTH_PX/2, wheel_len, wheel_wid)
        self._draw_wheel(x, y, theta, 0, 0, -CAR_WIDTH_PX/2, wheel_len, wheel_wid)
        
        # Front Wheels (Steered by phi)
        self._draw_wheel(x, y, theta, phi, L,  CAR_WIDTH_PX/2, wheel_len, wheel_wid)
        self._draw_wheel(x, y, theta, phi, L, -CAR_WIDTH_PX/2, wheel_len, wheel_wid)

    def _draw_wheel(self, car_x, car_y, car_theta, steer_angle, x_offset, y_offset, length, width):
        """Helper to draw a single wheel."""
        wheel_local = np.array([
            [ length/2,  width/2],
            [ length/2, -width/2],
            [-length/2, -width/2],
            [-length/2,  width/2],
        ])
        
        # 1. Rotate wheel by steering angle (phi)
        c_steer, s_steer = np.cos(steer_angle), np.sin(steer_angle)
        R_steer = np.array([[c_steer, -s_steer], [s_steer, c_steer]])
        wheel_rotated = wheel_local @ R_steer.T
        
        # 2. Translate wheel to its position on the car
        wheel_on_car = wheel_rotated + np.array([x_offset, y_offset])
        
        # 3. Transform to World and then Screen
        wheel_screen = self._transform_local_to_screen(wheel_on_car, car_x, car_y, car_theta)
        
        if len(wheel_screen) >= 3:
            pygame.draw.polygon(self.screen, (0, 0, 0), wheel_screen)

    def _transform_local_to_screen(self, points_local, world_x, world_y, world_theta):
        """
        Rotates local points by theta, translates to world_x/y, 
        then converts to screen coordinates.
        """
        # 1. Rotate by Car Heading (Theta)
        c, s = np.cos(world_theta), np.sin(world_theta)
        R = np.array([[c, -s], [s, c]])
        
        points_rotated = points_local @ R.T
        
        # 2. Translate to World Position
        points_world = points_rotated + np.array([world_x, world_y])
        
        # 3. Convert to Screen
        return self.viewport.world_to_screen_poly(points_world)