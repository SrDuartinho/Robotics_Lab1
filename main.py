import pygame
import math
import numpy as np
from car import Car
from sensors import ForwardSensor
from lane import Lane
from viewport import Viewport
from renderer import Renderer
from constants import *
from enviroment import StraightEnvironment
import sys

def handle_sensors(screen,car, lane_model, left_sensor, right_sensor):
    for sensor in [left_sensor, right_sensor]:
        for start, end in sensor.get_rays(car):
            pygame.draw.line(screen, (0, 0, 255), start, end, 1)

            hit = lane_model.intersect_ray(start, end)
            if hit is not None:
                pygame.draw.circle(screen, (255, 255, 0), (int(hit[0]), int(hit[1])), 4)
                print(f"{sensor.name} sensor hit at {hit}")
                
def handle_input():
    """
    Process keyboard input and return control commands.
    
    Returns:
        V: Linear velocity (pixels/sec)
        omega_s: Steering rate (radians/sec)
    """
    keys = pygame.key.get_pressed()
    
    # Compute velocity command
    V = MAX_SPEED_PPS
    # if keys[pygame.K_UP] or keys[pygame.K_w]:
    #     V = MAX_SPEED_PPS
    # elif keys[pygame.K_DOWN] or keys[pygame.K_s]:
    #     V = -MAX_SPEED_PPS * REVERSE_SPEED_FACTOR
    
    # Compute steering rate command
    omega_s = 0.0
    if keys[pygame.K_LEFT] or keys[pygame.K_a]:
        omega_s = -STEER_RATE_RPS  # Turn left (increase phi)
    elif keys[pygame.K_RIGHT] or keys[pygame.K_d]:
        omega_s = STEER_RATE_RPS  # Turn right (decrease phi)
    
    return V, omega_s


def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH_PX, SCREEN_HEIGHT_PX))
    pygame.display.set_caption(WINDOW_TITLE)
    clock = pygame.time.Clock()
    
    # --- 1. SETUP ENVIRONMENT ---
    env = StraightEnvironment()
    
    lane_model = Lane(env.get_roadx(), LANE_WIDTH_PX, SCREEN_HEIGHT_PX)
    
    # --- 2. SETUP CAR (Using Env Spawn Point) ---
    start_x, start_y, start_theta = env.get_start_pose()
    
    car = Car(
        x=start_x,
        y=start_y,
        theta=start_theta,
        wheelbase_px=CAR_WHEELBASE_PX,
        max_steer=MAX_STEER_ANGLE_RAD,
        steer_rate=STEER_RATE_RPS
    )
    # --- 3. SETUP SENSORS ---
    left_sensor = ForwardSensor(
        ray_angles=[-math.radians(40), -math.radians(20)],
        ray_length=300
    )
    left_sensor.name = "left"
    right_sensor = ForwardSensor(
        ray_angles=[math.radians(20), math.radians(40)],
        ray_length=300
    )
    right_sensor.name = "right"
    
    # --- 4. SETUP VIEW ---
    viewport = Viewport(car.x, car.y)
    renderer = Renderer(screen, viewport)
    
    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0
        
        # Event Handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE: running = False
                if event.key == pygame.K_r: 
                    # Reset to Env Spawn
                    sx, sy, st = env.get_start_pose()
                    car.reset(sx, sy, st)

        # Logic
        v_cmd, s_cmd = handle_input()
        car.update(v_cmd, s_cmd, dt)
        viewport.update(car)
        
        # Rendering
        renderer.render_environment(env)
        renderer.render_car(car)
        handle_sensors(screen,car, lane_model, left_sensor, right_sensor)
        
        draw_hud(screen, car, clock)
        
        pygame.display.flip()
        
    pygame.quit()
    sys.exit()


def draw_hud(screen, car, clock):
    """Draw heads-up display with car telemetry."""
    font = pygame.font.SysFont(FONT_NAME, FONT_SIZE)
    
    # Get telemetry
    x, y, theta, phi, _ = car.get_state()
    V = car.get_velocity()
    fps = clock.get_fps()
    
    # Format text
    lines = [
        f"FPS: {fps:.1f}",
        f"Pos: ({x:.0f}, {y:.0f})",
        f"Heading: {np.degrees(theta):.1f}°",
        f"Steering: {np.degrees(phi):.1f}°",
        f"Speed: {V:.1f} px/s ({V/PPM:.1f} m/s)"
    ]
    
    # Render text
    y_offset = 10
    for line in lines:
        text_surface = font.render(line, True, WHITE)
        screen.blit(text_surface, (10, y_offset))
        y_offset += 30


if __name__ == "__main__":
    main()