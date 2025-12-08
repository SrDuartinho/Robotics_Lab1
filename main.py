import pygame
import math
import numpy as np
import csv
from datetime import datetime
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
    sensor_points = []   # list of (x, y) of hit points

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
                ray_distances.append((dist, relative_angle, hit))
                sensor_points.append ((np.array(hit)))
            else:
                ray_distances.append((None, relative_angle, None))
                sensor_points.append (None)
    
    # filter valid rays
    valid = [(d, ang, p) for (d, ang, p) in ray_distances if d is not None]
    if not valid:
        return None

    # find ray with minimum raw distance
    raw_min_dist, ang, min_hit = min(valid, key=lambda v: v[0])
    
    side = 'left' if ang < 0 else 'right'
    
    # Primary match
    p1 = None
    p2 = None

    for i in range(len(sensor_points)):
        if sensor_points[i] is not None and np.array_equal(sensor_points[i], min_hit):

            # try neighbor
            if i+1 < len(sensor_points) and sensor_points[i+1] is not None:
                if sensor_points[1] is None:
                    p1 = sensor_points[2]
                    p2 = sensor_points[3]
                else:
                    p1 = sensor_points[i]
                    p2 = sensor_points[i+1]

            # fallback: right side hits available
            elif sensor_points[2] is not None and sensor_points[3] is not None:
                p1 = sensor_points[2]
                p2 = sensor_points[3]

            # fallback: left side hits available
            elif sensor_points[0] is not None and sensor_points[1] is not None:
                p1 = sensor_points[0]
                p2 = sensor_points[1]
                
            break

    # final guard
    if p1 is None or p2 is None:
        print("No valid p1/p2 found")
        return None

    # compute the angle
    v = p2 - p1
    road_angle = np.arctan2(v[0], v[1])
    
    _, _, theta, _, _ = car.get_state()

    perpendicular_distance = raw_min_dist * abs(math.cos((theta+road_angle) + ang))
    
    e_x = raw_min_dist * abs(math.sin((theta+road_angle) + ang))


    if np.linalg.norm(v) < 1e-8:
        # degenerate — don't draw
        return perpendicular_distance, e_x, theta, side

    # tangent angle relative to X-axis (standard math convention)
    tangent_angle = math.atan2(v[1], v[0])

    # normal is tangent rotated by +90 degrees
    normal_angle = tangent_angle + math.pi / 2.0

    # unit normal vector
    lane_normal = np.array([math.cos(normal_angle), math.sin(normal_angle)], dtype=float)

    # If you want the normal to point to the car side (match your 'ang' sign),
    # flip it when necessary (ang < 0 means left-side sensor in your code).
    # You might need to invert the sign depending on your coordinate convention.
   
    lane_normal = -lane_normal
    lane_normal = lane_normal / np.linalg.norm(lane_normal)
    
    #print(ang, lane_normal)

    # compute endpoint in world coordinates
    end_world = start + perpendicular_distance * lane_normal

    # draw line in screen coords
    screen_start = viewport.world_to_screen_scalar(tuple(start))
    end_pos_screen = viewport.world_to_screen_scalar(tuple(end_world))
    pygame.draw.line(screen, (255, 0, 0), screen_start, end_pos_screen, 2)

    return perpendicular_distance,e_x, tangent_angle, side


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

def normalize_angle(angle):
    """Wraps angle to [-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def align_road_angle(car_theta, road_angle):
    """
    Flips the road_angle 180 degrees if it points backwards relative to the car.
    This fixes the issue where the Right Sensor vector points in reverse.
    """
    diff = normalize_angle(road_angle - car_theta)
    # If the difference is huge (> 90 deg), the vector is pointing backwards.
    if abs(diff) > (np.pi / 2):
        return normalize_angle(road_angle + np.pi)
    return road_angle


def is_moving_towards_wall(car, road_angle, side):
    """
    Checks if the car heading is pointing towards the detected wall.
    Uses aligned angles to ensure robust detection for both Left and Right sensors.
    """
    _, _, theta, _, _ = car.get_state()
    
    # 1. Fix the direction ambiguity (Crucial for Right Sensor)
    forward_road_angle = align_road_angle(theta, road_angle)
    
    # 2. Calculate Heading Error (Road - Car)
    # Positive Error: Car is pointing Left relative to road.
    # Negative Error: Car is pointing Right relative to road.
    heading_error = normalize_angle(forward_road_angle - theta)
    
    threshold = 0.01 # Sensitivity buffer
    
    if side == 'left':
        # Wall is on Left. Danger if car points Left (Positive Error).
        return heading_error > threshold
        
    elif side == 'right':
        # Wall is on Right. Danger if car points Right (Negative Error).
        return heading_error < -threshold
        
    return False

def car_robot_control(car, dist, e_x, road_angle, side):
    """
    Task 5: LTA Controller implementation.
    
    Control Law (Lecture 4, Slide 15): omega = Ks * e_theta + Kl * e_y
    
    We combine:
    1. Heading Error (e_theta): Align car with road tangent.
    2. Lateral Error (e_y): "Push" the car away if it is too close to the wall.
    """
    # --- TUNING PARAMETERS ---
    # Ks: Heading Gain. Controls how fast we align to the road.
    Ks = 100 
    
    # Kl: Lateral Repulsion Gain. Controls how hard we "bounce" off the wall.
    # Higher = stronger push when close to the line.
    Kl = 0.3
    
    Kv = 0.05                       # slowdown gain per pixel of risk
    
    # Speed tuning
    V_base = MAX_SPEED_PPS * 0.95  # cruise speed when LTA active
    V_min = MAX_SPEED_PPS * 0.3    # floor speed under high risk
    
    
    # --- 1. CALCULATE HEADING ERROR (e_theta) ---
    _, _, theta, _, _ = car.get_state()
    
    # Align road angle to fix sensor direction ambiguity
    forward_road_angle = align_road_angle(theta, road_angle)
    e_theta = normalize_angle(forward_road_angle - theta)
    
    # --- 2. CALCULATE LATERAL ERROR (e_y / Repulsion) ---
    # How deep are we in the danger zone?
    # If dist is 0 (touching wall), push is Max. If dist is Threshold, push is 0.
    push_magnitude = max(0, LTA_THRESHOLD - dist)
    
    # --- 3. COMBINE TERMS ---
    # Base correction from heading
    omega_s = Ks * e_theta
    
    # Add repulsion based on side
    if side == 'left':
        # Left Wall: We want to steer Right (+omega) to move away
        omega_s += Kl * push_magnitude
    elif side == 'right':
        # Right Wall: We want to steer Left (-omega) to move away
        omega_s -= Kl * push_magnitude
        
    # --- 4. OUTPUT ---
    # Clamp to physical limits
    omega_s = max(-STEER_RATE_RPS, min(omega_s, STEER_RATE_RPS))
    
    # ------------------------------------------------------------
    # --- 4. RESPONSIVE SPEED CONTROL UNDER LTA ------------------
    # ------------------------------------------------------------
    # Risk grows as we get closer to the wall; slow down accordingly.
    risk = max(0.0, LTA_THRESHOLD - dist)
    V = V_base - Kv * risk * e_x

    # Clamp to avoid stopping completely and to keep within limits
    V = max(V_min, min(V, MAX_SPEED_PPS))
    print(V)
    return V, omega_s


def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH_PX, SCREEN_HEIGHT_PX))
    pygame.display.set_caption(WINDOW_TITLE)
    clock = pygame.time.Clock()
    #plotter = LiveSensorPlot()
    
    # --- CSV LOGGING SETUP ---
    log_filename = f"lta_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    csv_file = open(log_filename, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    # Write header
    csv_writer.writerow(['time_s', 'lateral_dist_px', 'lateral_dist_m', 'in_lane', 'speed_pps', 'speed_mps', 'steering_rad', 'heading_rad', 'lta_active'])
    csv_file.flush()
    
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
        ray_angles=[-math.radians(60), -math.radians(20)],
        ray_length=500
    )
    left_sensor.name = "left"
    
    right_sensor = ForwardSensor(
        ray_angles=[math.radians(20), math.radians(60)],
        ray_length=500
    )
    right_sensor.name = "right"
    
    # --- 4. SETUP VIEW ---
    viewport = Viewport(car.x, car.y)
    renderer = Renderer(screen, viewport)
    
    # --- 5. MAIN LOOP ---
    running = True
    paused = False
    frame_count = 0
    test_start_time = None
    
    while running:
        dt = clock.tick(FPS) / 1000.0
        
        # Start timer on first frame
        if test_start_time is None:
            test_start_time = 0.0
        else:
            test_start_time += dt
        
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
        
        # Rendering
        renderer.render_environment(env)
        renderer.render_car(car)
        
        sensor_result = handle_sensors(screen, car, env, left_sensor, right_sensor, viewport)
        
        # Logic
        v_cmd, s_cmd = handle_input(car)
        LTA_activate = False
        
        if sensor_result is not None:
            lateral_dist, e_x, road_angle, side = sensor_result

            # A. Check Danger Zone (engage as soon as we hit threshold)
            in_danger_zone = abs(lateral_dist) <= LTA_THRESHOLD
            
            # B. (Optional) heading check; keep for logging/future tuning
            moving_to_danger = is_moving_towards_wall(car, road_angle, side)
            
            # D. Final Decision
            # Engage LTA immediately upon entering the zone, regardless of heading
            if in_danger_zone and moving_to_danger:
                LTA_activate = True
                v_cmd, s_cmd = car_robot_control(car, lateral_dist, e_x, road_angle, side)
        
        # Physics Update
        car.update(v_cmd, s_cmd, dt)
        viewport.update(car)
        
        # --- LOG METRICS TO CSV ---
        if sensor_result is not None:
            lateral_dist, e_x, road_angle, side = sensor_result
            # Check if car is within safe lane bounds (assume LANE_WIDTH_PX / 2 from center)
            in_lane = abs(lateral_dist) > (CAR_WIDTH_PX/2)
        else:
            lateral_dist = None
            in_lane = None
        
        x, y, theta, phi, _ = car.get_state()
        speed_pps = car.get_velocity()
        
        csv_writer.writerow([
            test_start_time,
            lateral_dist,
            lateral_dist / PPM if lateral_dist is not None else None,
            in_lane,
            speed_pps,
            speed_pps / PPM,
            phi,
            theta,
            LTA_activate
        ])
        csv_file.flush()
        frame_count += 1
        
        #print("car speed: ", v_cmd)
        #print(e_x)
        
        # Visualization
        plot_dist = sensor_result[0] if sensor_result else None
        #plotter.update(plot_dist)
        draw_hud(screen, car, clock)
        
        # TODO: Remove this at the end or move it to draw_hub function
        # Visual Warning for LTA
        if LTA_activate:
            font = pygame.font.SysFont("Arial", 26, bold=True)
            text_str = f"!!! LTA ACTIVE ({side.upper()}) !!!"
            text_surface = font.render(text_str, True, (255, 50, 50))
            rect = text_surface.get_rect(center=(SCREEN_WIDTH_PX // 2, 50))
            screen.blit(text_surface, rect)
        
        pygame.display.flip()
        
    pygame.quit()
    csv_file.close()
    print(f"\nTest complete. Logged {frame_count} frames to: {log_filename}")
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