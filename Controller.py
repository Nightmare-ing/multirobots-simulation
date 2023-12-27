import matplotlib.patches as patches
import numpy as np
import itertools


class Controller:
    __gravity_velocity = 9.81

    def __init__(self):
        self.radius = 5.0
        self.central_point = (0.0, 0.0)
        self.w_r = 1.0

    @property
    def _speed_range(self):
        return np.array([0.0, 2 * self.w_r * self.radius])

    @property
    def _angular_vel_range(self):
        return np.array([-10 * self.w_r, 10 * self.w_r])

    @property
    def _acceleration_range(self):
        return np.array([-self.__gravity_velocity, self.__gravity_velocity])

    def speed_round(self, speeds_xy):
        """
        round transferred in speeds to valid range
        :param speeds_xy: speeds to be round
        :return: rounded speeds
        """
        if np.linalg.norm(speeds_xy) < self._speed_range[1]:
            return speeds_xy
        else:
            return speeds_xy / np.linalg.norm(speeds_xy) * self._speed_range[1]

    def angular_vel_round(self, angular_vel):
        """
        round transferred in angular velocity to valid range
        :param angular_vel: angular velocity to be round
        :return: rounded angular velocity
        """
        if angular_vel > self._angular_vel_range[1]:
            return self._angular_vel_range[1]
        elif angular_vel < self._angular_vel_range[0]:
            return self._angular_vel_range[0]
        else:
            return angular_vel


class CentralController(Controller):
    def update_pos_and_draw(self, fig, data):
        for robot in self.robots:
            dt, speed = next(data)
            robot.update(dt, speed)
            robot.draw(fig)

    def __init__(self, robots):
        super().__init__()
        self.robots = robots

    @property
    def desired_trace(self):
        return patches.Circle(self.central_point, self.radius, color='cyan', fill=False)

    def speed_gen(self):
        for _ in itertools.count():
            speeds = []
            dt = 0.005
            for robot in self.robots:
                cur_angle = np.arctan2(robot.posture[1] - self.central_point[1],
                                       robot.posture[0] - self.central_point[0])
                speed = (-self.w_r * self.radius * np.sin(cur_angle),
                         self.w_r * self.radius * np.cos(cur_angle),
                         0.0)
                speeds.append(speed)
            yield dt, speeds

