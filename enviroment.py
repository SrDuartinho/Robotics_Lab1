import numpy as np
import math
from constants import *

class Environment:
    """Base class for environments to handle common intersection logic."""
    def intersect_ray(self, ray_start, ray_end):
        """
        Finds the closest intersection between a sensor ray and ANY road line.
        ray_start, ray_end: Tuples or arrays (x, y) in WCS.
        """
        closest_hit = None
        min_dist = float('inf')
        
        # Check against every segment of every line
        # self.boundary_lines is a list of lists: [ [((x1,y1),(x2,y2)), ...], [...] ]
        for line_segments in self.boundary_lines:
            for seg_start, seg_end in line_segments:
                hit = self._get_line_intersection(ray_start, ray_end, seg_start, seg_end)
                
                if hit:
                    dist = np.hypot(ray_start[0] - hit[0], ray_start[1] - hit[1])
                    if dist < min_dist:
                        min_dist = dist
                        closest_hit = hit
                        
        return closest_hit

    def _get_line_intersection(self, p0, p1, p2, p3):
        """
        Math helper: Intersection of two line segments (Ray vs Road Segment).
        p0-p1 is the Ray. p2-p3 is the Road Segment.
        Returns (x, y) or None.
        """
        s1_x = p1[0] - p0[0]
        s1_y = p1[1] - p0[1]
        s2_x = p3[0] - p2[0]
        s2_y = p3[1] - p2[1]

        denom = (-s2_x * s1_y + s1_x * s2_y)
        if denom == 0: return None  # Parallel

        s = (-s1_y * (p0[0] - p2[0]) + s1_x * (p0[1] - p2[1])) / denom
        t = ( s2_x * (p0[1] - p2[1]) - s2_y * (p0[0] - p2[0])) / denom

        if 0 <= s <= 1 and 0 <= t <= 1:
            return (p0[0] + (t * s1_x), p0[1] + (t * s1_y))
        return None


class StraightEnvironment(Environment):
    def __init__(self):
        # 1. Define Dimensions
        self.width = ROAD_WIDTH_PX
        self.length = STRAIGHT_ROAD_LENGTH_PX
        
        self.start_x = 0
        self.start_y = 0
        self.start_theta = -np.pi / 2

        # 2. Generate Visual Geometry (Polygons)
        half_w = self.width / 2
        self.road_poly = np.array([
            [-half_w, 0], [half_w, 0],
            [half_w, -self.length], [-half_w, -self.length]
        ])
        
        # 3. Generate Lane Boundaries (Mathematical & Visual)
        # We store boundaries as lists of segments: [((x1,y1), (x2,y2))]
        # There are 3 lanes, so 4 Lines: Left Border, Div 1, Div 2, Right Border
        self.boundary_lines = [] 
        self.lines = [] # For Renderer (Polygons)
        
        line_width = WHITE_LINE_WIDTH_PX
        
        # Calculate X coordinates for the 4 lines
        # Left Edge, Divider 1, Divider 2, Right Edge
        xs = [
            -half_w,
            -half_w + LANE_WIDTH_PX,
            -half_w + (2 * LANE_WIDTH_PX),
            half_w
        ]
        
        # Create Lines
        for i, x in enumerate(xs):
            # A. Mathematical Representation (For Sensors)
            # Straight line from Y=0 to Y=-Length
            segment = ((x, 0), (x, -self.length))
            self.boundary_lines.append([segment])
            
            # B. Visual Representation (For Renderer)
            is_dashed = (i == 1 or i == 2) # Inner lines are dashed
            
            if is_dashed:
                self._add_dashed_visuals(x, line_width/2)
            else:
                self.lines.append(self._create_vertical_rect_solid(x, line_width))

    def _add_dashed_visuals(self, x, width):
        """Generates dashed visual polygons."""
        DASH = LANE_WIDTH_PX / 3
        GAP = LANE_WIDTH_PX / 3
        y = 0
        while y > -self.length:
            y_end = max(y - DASH, -self.length)
            self.lines.append(self._create_vertical_rect_segment(x, width, y, y_end))
            y -= (DASH + GAP)

    def _create_vertical_rect_solid(self, x_center, width):
        hw = width / 2
        return np.array([
            [x_center - hw, 0], [x_center + hw, 0],
            [x_center + hw, -self.length], [x_center - hw, -self.length]
        ])

    def _create_vertical_rect_segment(self, x_center, width, y_start, y_end):
        hw = width / 2
        return np.array([
            [x_center - hw, y_start], [x_center + hw, y_start],
            [x_center + hw, y_end], [x_center - hw, y_end]
        ])

    def get_start_pose(self):
        return self.start_x, self.start_y - 100, self.start_theta


class CurvedEnvironment(Environment):
    def __init__(self):
        # 1. Configuration
        self.width = ROAD_WIDTH_PX
        self.straight_len = STRAIGHT_ROAD_LENGTH_PX / 5
        self.turn_radius = 200
        self.turn_angle_deg = 30
        self.exit_len = STRAIGHT_ROAD_LENGTH_PX / 5
        self.resolution = 30

        self.start_x = 0
        self.start_y = 0
        self.start_theta = -np.pi / 2

        self.half_w = self.width / 2
        self.center_x = self.turn_radius
        self.center_y = -self.straight_len
        
        self.angle_start = np.pi 
        self.angle_end = np.pi + np.radians(self.turn_angle_deg)

        # 2. Geometry & Boundaries
        self.road_poly = self._generate_road_polygon()
        self.lines = []           # Visual Polygons
        self.boundary_lines = []  # Mathematical Segments for Sensors

        # Calculate the 4 lines (Left to Right)
        offsets = [
            -self.half_w,               # Left Border
            -self.half_w + LANE_WIDTH_PX,   # Div 1
            -self.half_w + (2*LANE_WIDTH_PX), # Div 2
            self.half_w                 # Right Border
        ]

        line_width = WHITE_LINE_WIDTH_PX
        
        for i, offset in enumerate(offsets):
            is_dashed = (i == 1 or i == 2)
            w = line_width/2 if is_dashed else line_width
            
            # Generate BOTH visuals and math segments
            self._create_combined_line(offset, w, is_dashed)

    def _generate_road_polygon(self):
        # (Same logic as previous answer, omitted for brevity but required)
        # Using simplified version here for context
        outer_line = [[-self.half_w, 0], [-self.half_w, -self.straight_len]]
        inner_line = [[self.half_w, -self.straight_len], [self.half_w, 0]]
        
        r_outer = self.turn_radius + self.half_w
        curve_outer = self._get_arc_points(r_outer, self.angle_start, self.angle_end)
        
        r_inner = self.turn_radius - self.half_w
        curve_inner = self._get_arc_points(r_inner, self.angle_start, self.angle_end)

        dx = -np.sin(self.angle_end)
        dy = np.cos(self.angle_end)
        direction = np.array([dx, dy])
        
        end_outer = curve_outer[-1] + (direction * self.exit_len)
        end_inner = curve_inner[-1] + (direction * self.exit_len)

        points = outer_line + curve_outer.tolist() + [end_outer.tolist(), end_inner.tolist()] + curve_inner[::-1].tolist() + inner_line
        return np.array(points)

    def _create_combined_line(self, offset_x, width, dashed):
        """Generates visual polys AND Mathematical segments."""
        visual_polys = []
        math_segments = [] # List of ((x1,y1), (x2,y2))
        hw = width / 2
        
        # --- 1. ENTRY STRAIGHT ---
        p_start = (offset_x, 0)
        p_end = (offset_x, -self.straight_len)
        math_segments.append((p_start, p_end))
        
        if dashed:
            DASH = LANE_WIDTH_PX / 3
            GAP = LANE_WIDTH_PX / 3
            y = 0
            while y > -self.straight_len:
                y_end = max(y - DASH, -self.straight_len)
                visual_polys.append(np.array([
                    [offset_x - hw, y], [offset_x + hw, y],
                    [offset_x + hw, y_end], [offset_x - hw, y_end]
                ]))
                y -= (DASH + GAP)
        else:
            visual_polys.append(np.array([
                [offset_x - hw, 0], [offset_x + hw, 0],
                [offset_x + hw, -self.straight_len], [offset_x - hw, -self.straight_len]
            ]))

        # --- 2. CURVE SECTION ---
        radius = self.turn_radius + offset_x
        curve_points = self._get_arc_points(radius, self.angle_start, self.angle_end)
        
        # Convert curve points to segments for Math Model
        for k in range(len(curve_points) - 1):
            math_segments.append((tuple(curve_points[k]), tuple(curve_points[k+1])))

        # Visuals for curve
        if dashed:
            # (Simplified dash logic for brevity)
            p_in = self._get_arc_points(radius - hw, self.angle_start, self.angle_end)
            p_out = self._get_arc_points(radius + hw, self.angle_start, self.angle_end)
            visual_polys.append(np.vstack((p_out, p_in[::-1])))
        else:
            p_in = self._get_arc_points(radius - hw, self.angle_start, self.angle_end)
            p_out = self._get_arc_points(radius + hw, self.angle_start, self.angle_end)
            visual_polys.append(np.vstack((p_out, p_in[::-1])))

        # --- 3. EXIT STRAIGHT ---
        dx = -np.sin(self.angle_end)
        dy = np.cos(self.angle_end)
        direction = np.array([dx, dy])
        
        # Start point is the last point of the curve
        start_pt = curve_points[-1]
        end_pt = start_pt + (direction * self.exit_len)
        
        math_segments.append((tuple(start_pt), tuple(end_pt)))
        
        # Visuals for exit
        ortho = np.array([-dy, dx]) * hw
        p1 = start_pt + ortho
        p2 = start_pt - ortho
        p3 = end_pt - ortho
        p4 = end_pt + ortho
        visual_polys.append(np.array([p1, p2, p3, p4]))

        # Store data
        self.boundary_lines.append(math_segments)
        self.lines.extend(visual_polys)

    def _get_arc_points(self, r, start, end, steps=None):
        if steps is None: steps = self.resolution
        t = np.linspace(start, end, steps)
        x = self.center_x + r * np.cos(t)
        y = self.center_y + r * np.sin(t)
        return np.column_stack((x, y))

    def get_start_pose(self):
        return self.start_x, self.start_y - 100, self.start_theta