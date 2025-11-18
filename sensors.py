# sensors.py
import numpy as np

class ForwardSensor:
    def __init__(self, ray_angles=[-0.15, 0.15], ray_length=500):
        """
        ray_angles: list of angles relative to car heading
        ray_length: pixel length of each ray
        """
        self.ray_angles = ray_angles
        self.ray_length = ray_length

    def get_rays(self, car):
        cx, cy, theta, _, _= car.get_state()
        fx, fy = car.get_front_center()

        rays = []
        for ang in self.ray_angles:
            direction = theta + ang
            dx = np.cos(direction)
            dy = np.sin(direction)
            end_x = fx + dx * self.ray_length
            end_y = fy + dy * self.ray_length
            rays.append(((fx, fy), (end_x, end_y)))

        return rays
