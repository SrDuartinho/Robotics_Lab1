# main.py
import numpy
import pygame
import sys
import math
from constants import (LANE_WIDTH_PX, LANE_COUNT, 
                       SENSOR_RANGE_FRONT_PX, 
                       SENSOR_RANGE_SIDE_PX, 
                       SENSOR_RANGE_DIAG_PX)  # Import constants

from car import Car
from sensor import ForwardSensor
from lane import Lane

pygame.init()

# Screen settings
WIDTH, HEIGHT = 400, 600
ROAD_WIDTH = LANE_WIDTH_PX
ROAD_X = (WIDTH - ROAD_WIDTH) // 2  # Center the road

# Colors
WHITE = (255, 255, 255)
GRAY = (64, 64, 64)
GREEN = (34, 177, 76)


screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# Car + sensor + lane model
car = Car()

left_sensor = ForwardSensor(
    ray_angles=[-math.radians(40), -math.radians(20)],
    ray_length=SENSOR_RANGE_FRONT_PX
)
left_sensor.name = "left"
right_sensor = ForwardSensor(
    ray_angles=[math.radians(20), math.radians(40)],
    ray_length=SENSOR_RANGE_FRONT_PX
)
right_sensor.name = "right"
lane_model = Lane(ROAD_X, ROAD_WIDTH, HEIGHT)

# Simulation settings
V = 4        # pixel forward speed
dt = 0.1     # simulation timestep

# Distance tracking
distance = 0
line_offset = 0
SCROLL_SPEED = 4

def draw_road():
    global line_offset
    
    screen.fill(GREEN)
    pygame.draw.rect(screen, GRAY, (ROAD_X, 0, ROAD_WIDTH, HEIGHT))
    
    # Road outline (continuous white lines)
    pygame.draw.line(screen, WHITE, (ROAD_X, 0), (ROAD_X, HEIGHT), 4)
    pygame.draw.line(screen, WHITE, (ROAD_X + ROAD_WIDTH, 0), (ROAD_X + ROAD_WIDTH, HEIGHT), 4)
    
    # Animated lane lines
    line_offset += SCROLL_SPEED
    if line_offset >= 40:
        line_offset = 0
    
    for i in range(1, LANE_COUNT):
        x = ROAD_X + i * (ROAD_WIDTH / LANE_COUNT)
        for y in range(-40 + line_offset, HEIGHT, 40):
            pygame.draw.line(screen, WHITE, (x, y), (x, y + 20), 4)

def draw_car():
    x, y, theta, phi, L = car.get_state()
    steer_dir = theta + phi
    
    # Car body dimensions
    car_width = 40  # pixels
    car_length = 80  # length of car
    
    front_x = x + math.cos(theta) * L
    front_y = y + math.sin(theta) * L
    
    mid_x = (x + front_x) / 2
    mid_y = (y + front_y) / 2
    
    steering_angle_x = front_x + math.cos(steer_dir) * L / 2
    steering_angle_y = front_y + math.sin(steer_dir) * L / 2
    
    # ---- Draw car body as a rotated rectangle ----
    car_surf = pygame.Surface((car_length, car_width), pygame.SRCALPHA)  # transparent surface
    car_surf.fill((66, 99, 145))
    
    # Rotate surface
    rotated_car = pygame.transform.rotate(car_surf, -math.degrees(theta))
    
    # Position the rotated rectangle so center matches car's (x, y)
    rect = rotated_car.get_rect(center=(mid_x, mid_y))
    screen.blit(rotated_car, rect.topleft)
    
    pygame.draw.circle(screen, (255, 0, 0), (int(x), int(y)), 8)
    pygame.draw.line(screen, (255, 0, 0), (x, y), (front_x, front_y), 2)
    pygame.draw.circle(screen, (255, 0, 0), (front_x, front_y), 8)
    pygame.draw.line(screen, (255, 0, 0), (front_x, front_y), (steering_angle_x, steering_angle_y), 2)

def main():
    running = True
    steering_speed = 0.1  # how fast steering angle changes

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # --- Keyboard input for steering ---
        keys = pygame.key.get_pressed()
        omega_s = 0

        if keys[pygame.K_LEFT]:
            omega_s = -steering_speed
        if keys[pygame.K_RIGHT]:
            omega_s = +steering_speed

        # --- Kinematic update ---
        car.step(V, omega_s, dt)

        # --- Draw everything ---
        draw_road()
        draw_car()

        # --- Draw sensor rays and intersections ---
        for sensor in [left_sensor, right_sensor]:
            for start, end in sensor.get_rays(car):
                pygame.draw.line(screen, (0, 0, 255), start, end, 1)

                hit = lane_model.intersect_ray(start, end)
                if hit is not None:
                    pygame.draw.circle(screen, (255, 255, 0), (int(hit[0]), int(hit[1])), 4)
                    print(f"{sensor.name} sensor hit at {hit}")

        pygame.display.update()
        clock.tick(60)

if __name__ == "__main__":
    main()