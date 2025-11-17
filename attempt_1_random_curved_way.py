import numpy as np
import pygame
import random
import sys
from scipy.interpolate import splprep, splev
import pygame.gfxdraw # For smooth, filled polygons

random.seed(42)  # For reproducibility

# --- Constants ---
SCREEN_WIDTH = 1000
SCREEN_HEIGHT = 750
FPS = 60
LANE_WIDTH = 80.0       # Distance BETWEEN the lines
WHITE_LINE_WIDTH = 3.0 # <--- NEW: How thick the lines are

# --- Minimap Constants ---
MINIMAP_WIDTH = 250
MINIMAP_HEIGHT = int(MINIMAP_WIDTH * (SCREEN_HEIGHT / SCREEN_WIDTH))
MINIMAP_SCALE = MINIMAP_WIDTH / SCREEN_WIDTH
MINIMAP_POS = (10, 10)

# Colors
COLORS = {
    'bg_gray': (100, 100, 100),
    'white_line': (255, 255, 255),
    'car_red': (255, 0, 0),
    'car_heading': (255, 255, 0),
    'grass_green': (50, 150, 50),
    'minimap_border': (0, 0, 0)
}

# --- Pygame Setup ---
pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Robotics Lab 1 - LTA Simulation (Thick Lines)")
clock = pygame.time.Clock()


# -----------------------------
# Generate Random Closed-Loop Track
# -----------------------------
def generate_random_track(
    num_control_points=12, 
    min_radius=200, 
    max_radius=350, 
    num_smooth_points=1000
):
    """
    Generates polygon data for a smooth, closed-loop racetrack
    with thick, drawable lines.
    """
    center_x, center_y = SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2
    control_points = []
    
    angles = np.linspace(0, 2 * np.pi, num_control_points, endpoint=False)
    for angle in angles:
        radius = random.uniform(min_radius, max_radius)
        angle_jitter = random.uniform(-0.2, 0.2)
        x = center_x + radius * np.cos(angle + angle_jitter)
        y = center_y + radius * np.sin(angle + angle_jitter)
        control_points.append((x, y))

    control_points.append(control_points[0])
    x_points, y_points = zip(*control_points)

    tck, u = splprep([x_points, y_points], s=0, per=True, k=3)
    u_new = np.linspace(0, 1, num_smooth_points)
    
    # Get smooth centerline points
    center_x, center_y = splev(u_new, tck)
    centerline = np.array([center_x, center_y]).T
    
    # Get smooth normals
    dx, dy = splev(u_new, tck, der=1)
    norm = np.sqrt(dx**2 + dy**2)
    norm[norm == 0] = 1
    nx, ny = -dy / norm, dx / norm
    normals = np.array([nx, ny]).T
    
    # --- Define the edges of all polygons ---
    half_lane = LANE_WIDTH / 2
    half_line = WHITE_LINE_WIDTH / 2
    
    # Calculate all 4 edges of the road/lines
    edge_left_outer = centerline + normals * (half_lane + half_line)
    edge_left_inner = centerline + normals * (half_lane - half_line)
    edge_right_inner = centerline - normals * (half_lane - half_line)
    edge_right_outer = centerline - normals * (half_lane + half_line)
    
    # --- Create the final polygon arrays ---
    
    # The gray road surface (between the inner edges of the lines)
    road_polygon = np.concatenate([edge_left_inner, edge_right_inner[::-1]])
    
    # The white left line polygon
    left_line_polygon = np.concatenate([edge_left_outer, edge_left_inner[::-1]])
    
    # The white right line polygon
    right_line_polygon = np.concatenate([edge_right_outer, edge_right_inner[::-1]])

    # Return the centerline (for the car) and the 3 draw-able polygons
    return centerline, road_polygon, left_line_polygon, right_line_polygon

# --- Main Simulation Function ---
def main():
    
    # --- 1. Generate the Environment (Polygons) ---
    print("Generating random racetrack...")
    centerline, road_polygon, left_line_poly, right_line_poly = generate_random_track()
    
    # --- 2. Create Minimap-Scaled Track (Do this ONCE) ---
    minimap_track_surface = pygame.Surface((MINIMAP_WIDTH, MINIMAP_HEIGHT))
    minimap_track_surface.fill(COLORS['grass_green'])
    
    # Scale and draw the polygons onto the minimap
    minimap_road = (road_polygon * MINIMAP_SCALE).astype(int).tolist()
    minimap_left = (left_line_poly * MINIMAP_SCALE).astype(int).tolist()
    minimap_right = (right_line_poly * MINIMAP_SCALE).astype(int).tolist()

    pygame.gfxdraw.filled_polygon(minimap_track_surface, minimap_road, COLORS['bg_gray'])
    pygame.gfxdraw.filled_polygon(minimap_track_surface, minimap_left, COLORS['white_line'])
    pygame.gfxdraw.filled_polygon(minimap_track_surface, minimap_right, COLORS['white_line'])
    
    print("Track generation complete.")

    # --- 3. Car State ---
    start_pos_index = 0
    car_world_pos = np.array(centerline[start_pos_index], dtype=float)
    
    dx = centerline[start_pos_index + 1][0] - centerline[start_pos_index][0]
    dy = centerline[start_pos_index + 1][1] - centerline[start_pos_index][1]
    car_world_heading = np.arctan2(dy, dx)
    
    screen_car_pos = np.array([SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2], dtype=float)

    move_speed = 1.0
    turn_speed = 0.05
    
    zoom_level = 3.0
    zoom_speed = 0.05
    
    # --- 4. Main Game Loop ---
    running = True
    while running:
        
        # --- 5. Event Handling ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # --- 6. Handle Input (Updates car's WORLD state) ---
        keys = pygame.key.get_pressed()
        
        if keys[pygame.K_LEFT]:
            car_world_heading -= turn_speed
        if keys[pygame.K_RIGHT]:
            car_world_heading += turn_speed
            
        if keys[pygame.K_UP]:
            car_world_pos += np.array([np.cos(car_world_heading), np.sin(car_world_heading)]) * move_speed
        if keys[pygame.K_DOWN]:
            car_world_pos -= np.array([np.cos(car_world_heading), np.sin(car_world_heading)]) * move_speed

        if keys[pygame.K_EQUALS] or keys[pygame.K_PLUS]:
            zoom_level += zoom_speed
        if keys[pygame.K_MINUS]:
            zoom_level -= zoom_speed
            
        zoom_level = np.clip(zoom_level, 0.2, 5.0)

        # --- 7. Main View Rendering (Zoom + Pan Camera) ---
        
        screen.fill(COLORS['grass_green'])
        
        # --- 7a. Transform all polygons (Fast, vectorized) ---
        # Formula: screen_point = (world_point - car_world_pos) * zoom_level + screen_car_pos
        
        screen_road_points = ((road_polygon - car_world_pos) * zoom_level + screen_car_pos).astype(int).tolist()
        screen_left_line_points = ((left_line_poly - car_world_pos) * zoom_level + screen_car_pos).astype(int).tolist()
        screen_right_line_points = ((right_line_poly - car_world_pos) * zoom_level + screen_car_pos).astype(int).tolist()
        
        # --- 7b. Draw the polygons (Anti-aliased) ---
        # We draw the road first, then the lines on top.
        pygame.gfxdraw.filled_polygon(screen, screen_road_points, COLORS['bg_gray'])
        pygame.gfxdraw.filled_polygon(screen, screen_left_line_points, COLORS['white_line'])
        pygame.gfxdraw.filled_polygon(screen, screen_right_line_points, COLORS['white_line'])
        
        # --- 7c. Draw the Car ---
        car_center_pos = (int(screen_car_pos[0]), int(screen_car_pos[1]))
        pygame.draw.circle(screen, COLORS['car_red'], car_center_pos, 8) 
        
        heading_vec = np.array([np.cos(car_world_heading), np.sin(car_world_heading)])
        line_end_pos = screen_car_pos + heading_vec * 20
        
        pygame.draw.line(screen, COLORS['car_heading'], 
                         car_center_pos, 
                         (int(line_end_pos[0]), int(line_end_pos[1])), 3)
        
        # --- 8. Minimap Rendering (Global) ---
        minimap_with_car = minimap_track_surface.copy()
        
        minimap_car_x = int(car_world_pos[0] * MINIMAP_SCALE)
        minimap_car_y = int(car_world_pos[1] * MINIMAP_SCALE)
        
        pygame.draw.circle(minimap_with_car, COLORS['car_red'], (minimap_car_x, minimap_car_y), 4)
        pygame.draw.rect(minimap_with_car, COLORS['minimap_border'], minimap_with_car.get_rect(), 2)
        screen.blit(minimap_with_car, MINIMAP_POS)
        
        # --- 9. Update the Display ---
        pygame.display.flip()
        
        # --- 10. Control Frame Rate ---
        clock.tick(FPS)

    # --- 11. Quit ---
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()