# lane.py
import pygame
import numpy as np

class Lane:
    def __init__(self, road_tuple, road_width, height):
        cont_linex, disc_linex = road_tuple
        self.left_x = cont_linex
        self.right_x = -cont_linex
        self.left_disc_x = disc_linex
        self.right_disc_x = -disc_linex
        self.height  = height

    def intersect_ray(self, ray_start, ray_end):
        """
        Compute intersection of ray with vertical lane lines.
        Lane borders are x = left_x and x = right_x.
        Returns closest intersection or None.
        """
        px, py = ray_start
        ex, ey = ray_end
        dx = ex - px
        dy = ey - py

        intersections = []

        # intersection with left boundary x = left_x
        if dx != 0:
            t = (self.left_x - px) / dx
            if 0 < t < 1:
                y = py + t * dy
                intersections.append((self.left_x, y))

        # intersection with right boundary x = right_x
        if dx != 0:
            t = (self.right_x - px) / dx
            if 0 < t < 1:
                y = py + t * dy
                intersections.append((self.right_x, y))
                
        # intersection with left non continuous boundary x = left_disc_x
        if dx != 0:
            t = (self.left_disc_x - px) / dx
            if 0 < t < 1:
                y = py + t * dy
                intersections.append((self.left_disc_x, y))

        # intersection with right non continuous boundary x = right_disc_x
        if dx != 0:
            t = (self.right_disc_x - px) / dx
            if 0 < t < 1:
                y = py + t * dy
                intersections.append((self.right_disc_x, y))

        if not intersections:
            return None

        # return closest intersection
        dists = [np.hypot(px-x, py-y) for x,y in intersections]
        return intersections[np.argmin(dists)]