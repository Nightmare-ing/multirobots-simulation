import numpy as np


class Camera:
    def __init__(self, field_angle=np.pi / 2, visible_radius=5.0, visible_range=np.array([0.25, 2.0]),
                 rotate_speed=0.1):
        self.field_angle = field_angle
        self.visible_radius = visible_radius
        self.visible_range = visible_range
        self.rotate_speed = rotate_speed


