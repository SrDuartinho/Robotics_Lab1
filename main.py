# main.py
import pygame
import sys
from constants import (SCREEN_WIDTH_PX, SCREEN_HEIGHT_PX, FPS, WINDOW_TITLE, DT,
                       CAR_LENGTH_PX, CAR_WIDTH_PX, SENSOR_RANGE_FRONT_PX,
                       SENSOR_RANGE_SIDE_PX, FONT_NAME, FONT_SIZE, COLORS, WHITE) # Added font and color imports
from enviroment import RandomClosedEnvironment, StraightEnvironment
from car import Car
# from sensor import Sensor # <-- DEACTIVATED
from camera import Camera

def draw_info(screen, font, car):
    """Draws debug text with car's WCS state."""
    
    # Create text lines
    info_lines = [
        f"--- Car WCS State (Debug) ---",
        f"Pos (X, Y): ({car.x:.1f}, {car.y:.1f})",
        f"Vel (X, Y): ({car.x_vel:.1f}, {car.y_vel:.1f})",
        f"Angle (deg): {car.angle:.1f}",
    ]
    
    # Render and blit each line
    for i, line in enumerate(info_lines):
        text_surface = font.render(line, True, WHITE)
        # Position the text block in the top right, away from minimap
        screen.blit(text_surface, (SCREEN_WIDTH_PX - 300, 10 + i * (FONT_SIZE + 5)))

def run_simulation():
    """Initializes all components and runs the main simulation loop."""
    
    # --- 1. Initialization ---
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH_PX, SCREEN_HEIGHT_PX))
    pygame.display.set_caption(WINDOW_TITLE)
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(FONT_NAME, FONT_SIZE) # <-- Added font init

    # --- 2. Create World and Agent ---
    env = RandomClosedEnvironment()
    
    # Get start pose from the environment
    start_pos = env.centerline[0]
    start_angle = env.start_angle
    
    car = Car(x=start_pos[0], y=start_pos[1], angle=start_angle)
    
    # --- 3. Create Sensors (DEACTIVATED) ---
    # ... (sensors remain commented out) ...

    # --- 4. Create Camera ---
    camera = Camera(car.x, car.y)

    # --- 5. Main Simulation Loop ---
    running = True
    while running:
        # --- Event Handling ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # --- Read Inputs ---
        keys = pygame.key.get_pressed()
        
        # --- Update ---
        car.update(keys, DT) # Update car using kinematics
        camera.update(car)   # Camera follows car
        
        # --- Draw ---
        env.draw(screen, camera)
        car.draw(screen, camera)
        
        # --- Draw Info Text ---
        draw_info(screen, font, car)

        # --- Finalize Frame ---
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    run_simulation()