import matplotlib.pyplot as plt
import numpy as np


class CentralController:
    def __init__(self, robots, fig):
        self.central_point = (0, 0)
        self.radius = 3.0
        self.robots = robots
        self.fig = fig

    def run(self):
        total_time = 100.0
        dt = 0.01
        t = 0.0
        rotate_speed = 3
        while t < total_time:
            for robot in self.robots:
                cur_angle = np.arctan2(robot.posture[1] - self.central_point[1],
                                       robot.posture[0] - self.central_point[0])
                speed = (-rotate_speed * self.radius * np.sin(cur_angle),
                         rotate_speed * self.radius * np.cos(cur_angle),
                         0.0)
                robot.update(dt, speed)
                robot.draw(self.fig)
            plt.draw()
            plt.pause(0.1)
            plt.cla()
            t += dt


