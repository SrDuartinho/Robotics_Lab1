import pygame
import math
import numpy as np
from car import Car
from sensors import ForwardSensor
from viewport import Viewport
from plot import LiveSensorPlot
from renderer import Renderer
from constants import *
from enviroment import *
import sys
def handle_sensors(screen, car, env, left_sensor, right_sensor, viewport):

    ray_distances = []   # list of (distance, relative_angle)

    for sensor in [left_sensor, right_sensor]:
        rays = sensor.get_rays(car)

        for idx, (start, end) in enumerate(rays):
            hit = env.intersect_ray(start, end)

            # draw the rays
            screen_start = viewport.world_to_screen_scalar(start)
            screen_end = viewport.world_to_screen_scalar(end)
            pygame.draw.line(screen, (0, 0, 255), screen_start, screen_end, 1)

            # get the sensor's own relative angle
            relative_angle = sensor.ray_angles[idx]

            if hit is not None:
                hit_screen = viewport.world_to_screen_scalar(hit)
                pygame.draw.circle(
                    screen, (255, 255, 0),
                    (int(hit_screen[0]), int(hit_screen[1])), 4
                )

                dist = np.linalg.norm(np.array(hit) - np.array(start))

                # store both the distance and the angle of the ray
                ray_distances.append((dist, relative_angle))

            else:
                ray_distances.append((None, relative_angle))

    # filter valid rays
    valid = [(d, ang) for (d, ang) in ray_distances if d is not None]
    if not valid:
        return None

    # find ray with minimum raw distance
    raw_min_dist, ang = min(valid, key=lambda v: v[0])
    print (raw_min_dist)
    # perpendicular (lane-normal) distance
    perpendicular_distance = raw_min_dist * math.cos(ang)

    return perpendicular_distance



def handle_input(car):
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
    else:
        if car.phi < 0.0:
            omega_s = STEER_RATE_RPS
        if car.phi > 0.0:
            omega_s = -STEER_RATE_RPS
    return V, omega_s


def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH_PX, SCREEN_HEIGHT_PX))
    pygame.display.set_caption(WINDOW_TITLE)
    clock = pygame.time.Clock()
    plotter = LiveSensorPlot()
    
    # --- 1. SETUP ENVIRONMENT ---
    env = CurvedEnvironment()
    
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
        ray_length=500
    )
    left_sensor.name = "left"
    
    right_sensor = ForwardSensor(
        ray_angles=[math.radians(20), math.radians(40)],
        ray_length=500
    )
    right_sensor.name = "right"
    
    # --- 4. SETUP VIEW ---
    viewport = Viewport(car.x, car.y)
    renderer = Renderer(screen, viewport)
    
    # --- 5. MAIN LOOP ---
    running = True
    paused = False
    while running:
        dt = clock.tick(FPS) / 1000.0
        
        # Event Handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT: 
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE: 
                    running = False
                if event.key == pygame.K_r: 
                    # Reset to Env Spawn
                    sx, sy, st = env.get_start_pose()
                    car.reset(sx, sy, st)
                if event.key == pygame.K_SPACE:
                    paused = not paused

        if paused:
            continue
        
        # Logic
        v_cmd, s_cmd = handle_input(car)
        car.update(v_cmd, s_cmd, dt)
        viewport.update(car)
        
        # Rendering
        renderer.render_environment(env)
        renderer.render_car(car)

        # Draw sensors on top (convert world -> screen with viewport)
        min_distance = handle_sensors(screen, car, env, left_sensor, right_sensor, viewport)
        plotter.update(min_distance)
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
        text_surface = font.render(line, True, COLORS['text'])
        screen.blit(text_surface, (10, y_offset))
        y_offset += 30


if __name__ == "__main__":
    main()