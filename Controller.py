import matplotlib.pyplot as plt
import numpy as np
import itertools


class CentralController:
    def update_pos_and_draw(self, fig, data):
        for robot in self.robots:
            dt, speed = next(data)
            robot.update(dt, speed)
            robot.draw(fig)

    def __init__(self, robots):
        self.central_point = (0, 0)
        self.radius = 3.0
        self.robots = robots
        self.rotate_speed = 3.0

    def speed_gen(self):
        for _ in itertools.count():
            speeds = []
            dt = 0.0001
            for robot in self.robots:
                cur_angle = np.arctan2(robot.posture[1] - self.central_point[1],
                                       robot.posture[0] - self.central_point[0])
                speed = (-self.rotate_speed * self.radius * np.sin(cur_angle),
                         self.rotate_speed * self.radius * np.cos(cur_angle),
                         0.0)
                speeds.append(speed)
            yield dt, speeds
