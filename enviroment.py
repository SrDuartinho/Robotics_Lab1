import numpy as np
from constants import *

class StraightEnvironment:
    def __init__(self):
        # 1. Define Road Dimensions
        self.width = ROAD_WIDTH_PX
        self.length = STRAIGHT_ROAD_LENGTH_PX
        
        # 2. Define Start Position (Bottom Center of the road)
        # We place the road starting at (0,0) and going upwards (Negative Y)
        # This aligns with the car's initial Heading of -pi/2 (Up)
        self.start_x = 0
        self.start_y = 0
        self.start_theta = -np.pi / 2

        # 3. Generate Geometry (Polygons)
        # The road is a rectangle centered on X=0, extending from Y=0 to Y=-Length
        half_w = self.width / 2
        
        # Main Asphalt Polygon (Counter-Clockwise)
        self.road_poly = np.array([
            [-half_w, 0],           # Bottom Left
            [half_w,  0],           # Bottom Right
            [half_w, -self.length], # Top Right
            [-half_w, -self.length] # Top Left
        ])
        
        # 4. Generate Lane Lines (Visuals)
        self.lines = []
        line_width = WHITE_LINE_WIDTH_PX
        
        # --- CONSTANTS FOR DASHED LINES ---
        DASH_LENGTH = LANE_WIDTH_PX / 3  # Length of the dash segment
        GAP_LENGTH = LANE_WIDTH_PX / 3   # Length of the gap
        SEGMENT_TOTAL = DASH_LENGTH + GAP_LENGTH
        
        # Create border lines (SOLID)
        self.lines.append(self._create_vertical_rect_solid(-half_w, line_width)) # Left Edge
        self.lines.append(self._create_vertical_rect_solid(half_w, line_width))  # Right Edge
        
        # Create lane dividers (DASHED)
        for i in range(1, LANE_COUNT):
            # Calculate x-offset for lane dividers
            lane_x = -half_w + (i * LANE_WIDTH_PX)
            
            # Generate dashed segments along the full length
            y_start = 0
            while y_start > -self.length:
                y_end = y_start - DASH_LENGTH
                
                # Ensure the segment doesn't go past the road top
                if y_end < -self.length:
                    y_end = -self.length
                    
                # Add the dash segment
                self.lines.append(self._create_vertical_rect_segment(
                    x_center=lane_x, 
                    width=line_width / 2, # Thinner line for divider
                    y_start=y_start, 
                    y_end=y_end
                ))
                
                # Move to the start of the next dash segment (including the gap)
                y_start -= SEGMENT_TOTAL

    def _create_vertical_rect_solid(self, x_center, width):
        """Helper to create a solid vertical line polygon (used for borders)"""
        hw = width / 2
        return np.array([
            [x_center - hw, 0],
            [x_center + hw, 0],
            [x_center + hw, -self.length],
            [x_center - hw, -self.length]
        ])

    def _create_vertical_rect_segment(self, x_center, width, y_start, y_end):
        """Helper to create one segment of a vertical line polygon (used for dashes)"""
        hw = width / 2
        return np.array([
            [x_center - hw, y_start],
            [x_center + hw, y_start],
            [x_center + hw, y_end],
            [x_center - hw, y_end]
        ])

    def get_start_pose(self):
        """Returns the ideal spawn point for the car."""
        # Spawn in the center lane, slightly up from the bottom
        return self.start_x, self.start_y - 100, self.start_theta
    
    def get_roadx (self):
        cont_lines_x = self.width / 2
        dash_lines_x = -cont_lines_x + LANE_WIDTH_PX
        
        return cont_lines_x, dash_lines_x
    
     