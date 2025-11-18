import pygame
import numpy as np

# --- 1. REAL-WORLD UNITS (METERS) ---
PPM = 50  # Pixels Per Meter

# Car dimensions
CAR_LENGTH_M = 4.5    # meters
CAR_WIDTH_M = 1.8     # meters
CAR_WHEELBASE_M = 2.8  # meters

# Environment dimensions
LANE_WIDTH_M = 3.7     # meters
LANE_COUNT = 3
WHITE_LINE_WIDTH_M = 0.1  # meters

# Car physics
MAX_SPEED_MPS = 50.0  # Meters per Second
REVERSE_SPEED_FACTOR = 0.5  # As a factor of max speed
MAX_STEER_ANGLE_DEG = 30.0  # Max angle of the front wheels
STEER_RATE_DPS = 180.0  # Degrees per Second

# Sensor properties
SENSOR_RANGE_FRONT_M = 5.0  # 5 meters
SENSOR_RANGE_SIDE_M = 3.0   # 3 meters
SENSOR_RANGE_DIAG_M = 4.0   # 4 meters

# --- 2. SIMULATION & SCREEN UNITS (PIXELS) ---
SCREEN_WIDTH_PX = 1280
SCREEN_HEIGHT_PX = 720
FPS = 60
DT = 1.0 / FPS  # Delta Time (time per frame)

WINDOW_TITLE = "Robotics Lab 1: LTA"

# --- 3. DERIVED PIXEL VALUES ---
CAR_LENGTH_PX = CAR_LENGTH_M * PPM
CAR_WIDTH_PX = CAR_WIDTH_M * PPM
CAR_WHEELBASE_PX = CAR_WHEELBASE_M * PPM
LANE_WIDTH_PX = LANE_WIDTH_M * PPM
WHITE_LINE_WIDTH_PX = WHITE_LINE_WIDTH_M * PPM

MAX_SPEED_PPS = MAX_SPEED_MPS * PPM  # Pixels per Second
MAX_SPEED_PPF = MAX_SPEED_PPS * DT   # Pixels per Frame
REVERSE_SPEED_PPF = MAX_SPEED_PPF * REVERSE_SPEED_FACTOR

STEER_RATE_DPF = STEER_RATE_DPS * DT  # Degrees per Frame

SENSOR_RANGE_FRONT_PX = SENSOR_RANGE_FRONT_M * PPM
SENSOR_RANGE_SIDE_PX = SENSOR_RANGE_SIDE_M * PPM
SENSOR_RANGE_DIAG_PX = SENSOR_RANGE_DIAG_M * PPM

# --- 4. ENVIRONMENT GENERATION ---
STRAIGHT_ENV_LENGTH_M = 10  # Length of the straight road
RANDOM_ENV_POINTS = 7        # Control points for spline
RANDOM_ENV_MIN_RAD = 3000    # Min radius for spline points
RANDOM_ENV_MAX_RAD = 3100    # Max radius for spline points

# --- 5. CAMERA & MINIMAP ---
DEFAULT_ZOOM = 1.00
ZOOM_SPEED = 0.05

MINIMAP_WIDTH = 250
MINIMAP_HEIGHT = int(MINIMAP_WIDTH * (SCREEN_HEIGHT_PX / SCREEN_WIDTH_PX))
MINIMAP_SCALE = MINIMAP_WIDTH / (RANDOM_ENV_MAX_RAD * 2 * 1.5)

# --- 6. AESTHETICS ---
FONT_NAME = 'Arial'
FONT_SIZE = 24

SENSOR_BASE_RADIUS_PX = 4

COLORS = {
    'bg_gray': (100, 100, 100),
    'white_line': (255, 255, 255),
    'car_red': (255, 0, 0),
    'car_heading': (255, 255, 0),
    'grass_green': (50, 150, 50),
    'sensor_beam': (0, 255, 0),
    'sensor_base': (255, 255, 0),
    'minimap_border': (0, 0, 0)
}

# Legacy Colors
WHITE = (255, 255, 255)
GRAY = (100, 100, 100)
GREEN = (50, 150, 50)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
BLACK = (0, 0, 0)