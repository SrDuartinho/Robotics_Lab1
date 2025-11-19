import numpy as np
import math

# --- 1. SIMULATION CONFIGURATION ---
SCREEN_WIDTH_PX = 1280
SCREEN_HEIGHT_PX = 720
FPS = 60
DT = 1.0 / FPS             # Time per frame (seconds)
WINDOW_TITLE = "Robotics Lab 1: Kinematic Bicycle Model"

# --- 2. SCALE (The Bridge between Worlds) ---
PPM = 50  # Pixels Per Meter (50 px = 1 m)

# --- 3. REAL-WORLD CONSTANTS (METERS & SECONDS) ---
# Physical Dimensions
CAR_LENGTH_M = 3
CAR_WIDTH_M = 1.5
CAR_WHEELBASE_M = 2      # Distance between front and rear axles
LANE_WIDTH_M = 4
WHITE_LINE_WIDTH_M = 0.15

# Motion Limits
MAX_SPEED_MPS = 10.0
REVERSE_SPEED_FACTOR = 0.5 # Reverse is half max speed
MAX_STEER_ANGLE_DEG = 35.0
STEER_RATE_DPS = 30.0     # Degrees per second

# Sensors
SENSOR_RANGE_FRONT_M = 10.0
SENSOR_RANGE_SIDE_M = 5.0
SENSOR_RANGE_DIAG_M = 7.0

# --- 4. INITIALIZATION (Start State) ---
# We define these here so main.py doesn't need magic numbers
CAR_INITIAL_X_PX = SCREEN_WIDTH_PX // 2
CAR_INITIAL_Y_PX = SCREEN_HEIGHT_PX // 2
CAR_INITIAL_THETA = -np.pi / 2  # Pointing UP in Pygame

# --- 5. DERIVED PIXEL CONSTANTS (Calculated automatically) ---
# Dimensions
CAR_LENGTH_PX = CAR_LENGTH_M * PPM
CAR_WIDTH_PX = CAR_WIDTH_M * PPM
CAR_WHEELBASE_PX = CAR_WHEELBASE_M * PPM
LANE_WIDTH_PX = LANE_WIDTH_M * PPM
WHITE_LINE_WIDTH_PX = WHITE_LINE_WIDTH_M * PPM

# Overhang for rendering (Center the wheelbase visually)
REAR_OVERHANG_PX = (CAR_LENGTH_PX - CAR_WHEELBASE_PX) / 2

# Motion
MAX_SPEED_PPS = MAX_SPEED_MPS * PPM          # Pixels/Second
MAX_STEER_ANGLE_RAD = math.radians(MAX_STEER_ANGLE_DEG)
STEER_RATE_RPS = math.radians(STEER_RATE_DPS)

# Sensors
SENSOR_RANGE_FRONT_PX = SENSOR_RANGE_FRONT_M * PPM
SENSOR_RANGE_SIDE_PX = SENSOR_RANGE_SIDE_M * PPM
SENSOR_RANGE_DIAG_PX = SENSOR_RANGE_DIAG_M * PPM

# --- 6. CAMERA & VISUALS ---
DEFAULT_ZOOM = 1.0
ZOOM_SPEED = 0.05

FONT_NAME = 'Arial'
FONT_SIZE = 20

# --- ENVIRONMENT CONSTANTS ---
STRAIGHT_ROAD_LENGTH_M = 500.0 # 100 Meters long
LANE_COUNT = 3

# Derived Environment Pixels
STRAIGHT_ROAD_LENGTH_PX = STRAIGHT_ROAD_LENGTH_M * PPM
ROAD_WIDTH_PX = LANE_WIDTH_PX * LANE_COUNT

# Colors (R, G, B)
COLORS = {
    'bg_gray': (50, 50, 50),
    'car_body': (200, 50, 50),     # Red
    'car_outline': (100, 0, 0),    # Dark Red
    'wheels': (20, 20, 20),        # Black
    'text': (255, 255, 255),
    'white_line': (255, 255, 255),
    'sensor_beam': (0, 255, 0),
    'debug': (255, 255, 0),
    'road_gray': (80, 80, 80),
    'grass_green': (34, 139, 34),
}

# Legacy color support (if other files need them directly)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)