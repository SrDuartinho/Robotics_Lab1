# environment.py
import pygame
import numpy as np
import random
from scipy.interpolate import splprep, splev
import pygame.gfxdraw

# Import all constants from our settings file
from constants import (PPM, SCREEN_WIDTH_PX, SCREEN_HEIGHT_PX,
                       LANE_WIDTH_PX, LANE_COUNT, WHITE_LINE_WIDTH_PX,
                       STRAIGHT_ENV_LENGTH_M, RANDOM_ENV_POINTS,
                       RANDOM_ENV_MIN_RAD, RANDOM_ENV_MAX_RAD,
                       MINIMAP_SCALE, MINIMAP_POS, MINIMAP_WIDTH, MINIMAP_HEIGHT,
                       COLORS)

# Set a seed for reproducible random tracks
random.seed(42)

# --- Base Class ---

class BaseEnvironment:
    """
    Abstract base class for all environments.
    - Defines all geometry (road, lines) in WCS polygons.
    - Implements a common draw method.
    - Implements a common minimap renderer.
    """
    def __init__(self):
        # WCS polygons
        self.obstacles = []
        self.road_poly = np.array([])
        self.left_line_poly = np.array([])
        self.right_line_poly = np.array([])
        self.centerline = np.array([])
        
        # WCS coordinate that the track is centered on
        self.track_center_wcs = np.array([0.0, 0.0])
        
        self.start_angle = 0.0
        self.minimap_surface = pygame.Surface((MINIMAP_WIDTH, MINIMAP_HEIGHT))
        self.minimap_center_px = np.array([MINIMAP_WIDTH / 2, MINIMAP_HEIGHT / 2])

    def _transform_to_minimap(self, wcs_points_array):
        """Helper function to transform WCS points to minimap coordinates."""
        # 1. Translate points relative to the track's WCS center
        relative_points = wcs_points_array - self.track_center_wcs
        # 2. Scale
        scaled_points = relative_points * MINIMAP_SCALE
        # 3. Translate to the minimap's pixel center
        minimap_points = scaled_points + self.minimap_center_px
        return minimap_points.astype(int)

    def draw(self, screen, camera):
        """
        Draws the environment relative to the camera.
        This method is shared by all children classes.
        """
        
        # 1. Fill background
        screen.fill(COLORS['grass_green'])
        
        # 2. Transform WCS polygons to Screen coordinates
        screen_road = camera.world_to_screen_points(self.road_poly)
        screen_left = camera.world_to_screen_points(self.left_line_poly)
        screen_right = camera.world_to_screen_points(self.right_line_poly)
        
        # 3. Draw polygons
        pygame.gfxdraw.filled_polygon(screen, screen_road, COLORS['bg_gray'])
        pygame.gfxdraw.filled_polygon(screen, screen_left, COLORS['white_line'])
        pygame.gfxdraw.filled_polygon(screen, screen_right, COLORS['white_line'])

        # 4. Draw minimap
        minimap_with_car = self.minimap_surface.copy()
        
        # --- MINIMAP FIX ---
        # Apply the same WCS -> Minimap transformation to the car's position
        car_pos_wcs = np.array([[camera.x, camera.y]])
        minimap_car_pos = self._transform_to_minimap(car_pos_wcs)[0]
        # --- END FIX ---
        
        pygame.draw.circle(minimap_with_car, COLORS['car_red'], minimap_car_pos, 4)
        pygame.draw.rect(minimap_with_car, COLORS['minimap_border'], minimap_with_car.get_rect(), 2)
        screen.blit(minimap_with_car, MINIMAP_POS)


    def get_obstacles(self):
        """Returns a list of obstacle geometries for sensor collision."""
        return self.obstacles

    def _create_minimap(self):
        """
        Renders the static track geometry (self.road_poly, etc.)
        to a separate minimap surface using the correct transformation.
        """
        minimap_surf = pygame.Surface((MINIMAP_WIDTH, MINIMAP_HEIGHT))
        minimap_surf.fill(COLORS['grass_green'])
        
        # --- MINIMAP FIX ---
        # Transform polygons to be centered on the minimap
        minimap_road = self._transform_to_minimap(self.road_poly)
        minimap_left = self._transform_to_minimap(self.left_line_poly)
        minimap_right = self._transform_to_minimap(self.right_line_poly)
        # --- END FIX ---

        pygame.gfxdraw.filled_polygon(minimap_surf, minimap_road, COLORS['bg_gray'])
        pygame.gfxdraw.filled_polygon(minimap_surf, minimap_left, COLORS['white_line'])
        pygame.gfxdraw.filled_polygon(minimap_surf, minimap_right, COLORS['white_line'])
        
        return minimap_surf

# --- Straight Environment ---

class StraightEnvironment(BaseEnvironment):
    """
    Generates a simple, finite-length straight road.
    All geometry is defined in WCS as polygons.
    """
    def __init__(self, length_m=STRAIGHT_ENV_LENGTH_M):
        super().__init__()
        print(f"Generating straight road: {length_m}m long.")
        
        length_px = length_m * PPM
        
        # Center the road at the top-middle of the WCS
        road_width_px = LANE_WIDTH_PX * LANE_COUNT
        road_left_x = (SCREEN_WIDTH_PX / 2) - (road_width_px / 2)
        road_right_x = (SCREEN_WIDTH_PX / 2) + (road_width_px / 2)
        center_x = road_left_x + road_width_px / 2
        
        # --- MINIMAP FIX ---
        # Define the WCS center of the track
        self.track_center_wcs = np.array([center_x, -length_px / 2])
        # --- END FIX ---

        # Define WCS polygon for the road (Y=0 is start, Y=-Length is end)
        self.road_poly = np.array([
            (road_left_x, 0),
            (road_right_x, 0),
            (road_right_x, -length_px),
            (road_left_x, -length_px)
        ], dtype=float)
        
        # Define WCS polygons for the lines
        line_w = WHITE_LINE_WIDTH_PX
        self.left_line_poly = np.array([
            (road_left_x - line_w, 0),
            (road_left_x, 0),
            (road_left_x, -length_px),
            (road_left_x - line_w, -length_px)
        ], dtype=float)
        
        self.right_line_poly = np.array([
            (road_right_x, 0),
            (road_right_x + line_w, 0),
            (road_right_x + line_w, -length_px),
            (road_right_x, -length_px)
        ], dtype=float)
        
        # Set obstacles for sensors
        self.obstacles = [self.left_line_poly, self.right_line_poly]
        
        # Define centerline and start angle
        self.centerline = np.array([
            [center_x, 0],
            [center_x, -length_px]
        ])
        self.start_angle = 0.0 # 0 degrees = pointing "up" (negative Y)
        
        # Create the minimap (after polys are generated)
        self.minimap_surface = self._create_minimap()


# --- Random Closed Environment ---

class RandomClosedEnvironment(BaseEnvironment):
    """
    Generates a random, closed-loop track with spline interpolation.
    All track geometry is stored in WCS coordinates.
    """
    def __init__(self, num_points=RANDOM_ENV_POINTS, 
                 min_radius=RANDOM_ENV_MIN_RAD, 
                 max_radius=RANDOM_ENV_MAX_RAD):
        super().__init__()
        print("Generating random racetrack...")

        # --- MINIMAP FIX ---
        # Define the WCS center point around which the track is generated
        self.track_center_wcs = np.array([SCREEN_WIDTH_PX / 2, SCREEN_HEIGHT_PX / 2])
        # --- END FIX ---
        
        # Generate and store WCS polygons
        (self.centerline, 
         self.road_poly, 
         self.left_line_poly, 
         self.right_line_poly) = self._generate_track_geometry(
             num_points, min_radius, max_radius
         )
         
        # Define the WCS obstacles for sensors
        self.obstacles = [self.left_line_poly, self.right_line_poly]
        
        # Calculate start angle from the first segment
        self.start_angle = self._get_start_angle()
        
        # Pre-render the static minimap
        self.minimap_surface = self._create_minimap()
        print("Track generation complete.")

    def _generate_track_geometry(self, num_points, min_r, max_r):
        """Uses scipy splines to generate WCS polygons for the track."""
        
        # 1. Generate random control points around the track center
        center_x, center_y = self.track_center_wcs
        
        angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        radii = np.random.uniform(min_r, max_r, num_points)
        
        x_points = center_x + radii * np.cos(angles)
        y_points = center_y + radii * np.sin(angles)
        
        # Close the loop
        control_points = np.array([x_points, y_points]).T
        control_points = np.append(control_points, [control_points[0]], axis=0)
        
        # 2. Interpolate with B-spline
        tck, u = splprep([control_points[:, 0], control_points[:, 1]], s=0, per=True, k=3)
        
        num_smooth_points = int(num_points * 100)
        u_new = np.linspace(0, 1, num_smooth_points)
        center_x, center_y = splev(u_new, tck)
        centerline = np.array([center_x, center_y]).T
        
        # 3. Calculate normals to create road width
        dx, dy = splev(u_new, tck, der=1)
        norm = np.sqrt(dx**2 + dy**2)
        norm[norm == 0] = 1 # Avoid division by zero
        
        nx = -dy / norm # "Left" normal vector
        ny = dx / norm
        normals = np.array([nx, ny]).T
        
        # 4. Define all 4 edges of the road/lines
        half_lane = (LANE_WIDTH_PX * LANE_COUNT) / 2.0
        half_line = WHITE_LINE_WIDTH_PX / 2.0
        
        edge_left_outer = centerline + normals * (half_lane + half_line)
        edge_left_inner = centerline + normals * (half_lane - half_line)
        edge_right_inner = centerline - normals * (half_lane - half_line)
        edge_right_outer = centerline - normals * (half_lane + half_line)
        
        # 5. Create final polygons
        road_poly = np.concatenate([edge_left_inner, edge_right_inner[::-1]])
        left_line_poly = np.concatenate([edge_left_outer, edge_left_inner[::-1]])
        right_line_poly = np.concatenate([edge_right_outer, edge_right_inner[::-1]])
        
        return centerline, road_poly, left_line_poly, right_line_poly
        
    def _get_start_angle(self):
        """Calculates the initial angle of the car."""
        p1 = self.centerline[0]
        p2 = self.centerline[1]
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        
        math_angle_rad = np.arctan2(dy, dx)
        pygame_angle_deg = -np.degrees(math_angle_rad) + 90
        return pygame_angle_deg % 360