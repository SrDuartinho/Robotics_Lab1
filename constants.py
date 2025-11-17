# constants.py
import pygame
import numpy as np

# --- 1. REAL-WORLD UNITS (METERS) ---
# This is where you define your simulation in metric units.
# We assume 1 meter in the real world = PPM pixels on the screen.
PPM = 50  # Pixels Per Meter (This is your main scale factor)

# Car dimensions
CAR_LENGTH_M = 4.5    # meters
CAR_WIDTH_M = 1.8     # meters
CAR_WHEELBASE_M = 2.8 # L, for the kinematic model

# Environment dimensions
LANE_WIDTH_M = 3.7
LANE_COUNT = 3
WHITE_LINE_WIDTH_M = 0.1

# Car physics
MAX_SPEED_MPS = 50.0  # Meters per Second
REVERSE_SPEED_FACTOR = 0.5 # As a factor of max speed
MAX_STEER_ANGLE_DEG = 30.0 # Max angle of the front wheels
STEER_RATE_DPS = 180.0   # Degrees per Second (how fast you can turn the wheel)

# Sensor properties
SENSOR_RANGE_FRONT_M = 5.0 # 5 meters
SENSOR_RANGE_SIDE_M = 3.0  # 3 meters
SENSOR_RANGE_DIAG_M = 4.0  # 4 meters


# --- 2. SIMULATION & SCREEN UNITS (PIXELS) ---
SCREEN_WIDTH_PX = 1280
SCREEN_HEIGHT_PX = 720
FPS = 60
DT = 1.0 / FPS  # Delta Time (time per frame)

WINDOW_TITLE = "Robotics Lab 1: LTA"

# --- 3. DERIVED PIXEL VALUES ---
# Convert all real-world meters to pixels
CAR_LENGTH_PX = CAR_LENGTH_M * PPM
CAR_WIDTH_PX = CAR_WIDTH_M * PPM
CAR_WHEELBASE_PX = CAR_WHEELBASE_M * PPM
LANE_WIDTH_PX = LANE_WIDTH_M * PPM
WHITE_LINE_WIDTH_PX = WHITE_LINE_WIDTH_M * PPM

# Convert speeds (meters/sec) to (pixels/frame)
MAX_SPEED_PPS = MAX_SPEED_MPS * PPM  # Pixels per Second
MAX_SPEED_PPF = MAX_SPEED_PPS * DT   # Pixels per Frame (this is what the car update uses)
REVERSE_SPEED_PPF = MAX_SPEED_PPF * REVERSE_SPEED_FACTOR

# Convert turn rates (deg/sec) to (deg/frame)
STEER_RATE_DPF = STEER_RATE_DPS * DT # Degrees per Frame

# Convert sensor ranges (meters) to (pixels)
SENSOR_RANGE_FRONT_PX = SENSOR_RANGE_FRONT_M * PPM
SENSOR_RANGE_SIDE_PX = SENSOR_RANGE_SIDE_M * PPM
SENSOR_RANGE_DIAG_PX = SENSOR_RANGE_DIAG_M * PPM


# --- 4. ENVIRONMENT GENERATION ---
STRAIGHT_ENV_LENGTH_M = 10 # Length of the straight road
RANDOM_ENV_POINTS = 7       # Control points for spline
RANDOM_ENV_MIN_RAD = 3000     # Min radius for spline points
RANDOM_ENV_MAX_RAD = 3100     # Max radius for spline points


# --- 5. CAMERA & MINIMAP ---
# Camera
DEFAULT_ZOOM = 1.00
ZOOM_SPEED = 0.05

# Minimap
MINIMAP_WIDTH = 250
MINIMAP_HEIGHT = int(MINIMAP_WIDTH * (SCREEN_HEIGHT_PX / SCREEN_WIDTH_PX))
MINIMAP_SCALE = MINIMAP_WIDTH / (RANDOM_ENV_MAX_RAD * 2 * 1.5) # Fit the largest possible track
MINIMAP_POS = (10, 10)


# --- 6. AESTHETICS ---
# Font
FONT_NAME = 'Arial'
FONT_SIZE = 24

# Sensor drawing
SENSOR_BASE_RADIUS_PX = 4

# Colors
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

# Legacy Colors (for convenience)
WHITE = (255, 255, 255)
GRAY = (100, 100, 100)
GREEN = (50, 150, 50)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
BLACK = (0, 0, 0)