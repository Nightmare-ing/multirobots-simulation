import math
from typing import List, Any

import matplotlib.patches as patches
import numpy as np
import itertools
import pdb


class Controller:
    __gravity_velocity = 9.81

    def __init__(self, robots):
        self.radius = 5.0
        self.central_point = (0.0, 0.0)
        self.w_r = 1.0
        self.robots = robots
        self.matrix = np.zeros((len(self.robots), len(self.robots)))

    @property
    def _speed_range(self):
        return np.array([0.0, 2 * self.w_r * self.radius])

    @property
    def _angular_vel_range(self):
        return np.array([-10 * self.w_r, 10 * self.w_r])

    @property
    def _acceleration_range(self):
        return np.array([-self.__gravity_velocity, self.__gravity_velocity])

    @property
    def desired_trace(self):
        return None

    @property
    def l_matrix(self):
        a_matrix = np.zeros((len(self.robots), len(self.robots)))
        d_matrix = np.zeros((len(self.robots), len(self.robots)))
        for (i, j), _ in np.ndenumerate(self.matrix):
            if self.robots[i].inspect(self.robots[j]):
                a_matrix[i, j] = np.linalg.norm(self.robots[i].posture[:2] - self.robots[j].posture[:2])

        for i in range(len(self.robots)):
            d_matrix[i, i] = a_matrix[i, :].sum()

        self.matrix = a_matrix - d_matrix
        return self.matrix

    def speed_gen(self):
        yield 0, [(0, 0, 0)] * len(self.robots)

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
    __k = 100.0  # adjust param for controlling speed_along_radius

    def __init__(self, robots):
        super().__init__(robots)

    @property
    def desired_trace(self):
        return patches.Circle(self.central_point, self.radius, color='cyan', fill=False)

    def speed_gen(self):
        for _ in itertools.count():
            speeds = []
            dt = 0.005
            for robot in self.robots:
                cur_pos_angle = np.arctan2(robot.posture[1] - self.central_point[1],
                                           robot.posture[0] - self.central_point[0])
                speed_along_trace = self.w_r * self.radius
                distance = np.linalg.norm(robot.posture[:2] - self.central_point)
                speed_along_radius = np.sign(distance - self.radius) * speed_along_trace / self.__k
                speed = (-speed_along_trace * np.sin(cur_pos_angle) - speed_along_radius * np.cos(cur_pos_angle),
                         speed_along_trace * np.cos(cur_pos_angle) - speed_along_radius * np.sin(cur_pos_angle),
                         3.0)
                speeds.append(speed)
            yield dt, speeds


class DecentralizedController(Controller):
    __gama = 0.1
    __kp = 0.1
    __ki = 0.1
    __k1 = 0.1
    __k2 = 0.001
    __k3 = 0.1
    __sigma = 1
    __k = 10

    def __init__(self, robots):
        super().__init__(robots)
        self.z = np.ones((len(self.robots), 2))
        self.w = np.ones((len(self.robots), 2))
        self.v_tilde2 = np.ones(len(self.robots))
        self.lambda2 = np.ones(len(self.robots))

    @property
    def alpha(self):
        return np.array([self.v_tilde2, self.v_tilde2 ** 2]).transpose()

    @property
    def desired_trace(self):
        return patches.Circle(self.central_point, self.radius, color='cyan', fill=False)

    def update_z(self):
        """
        compute and update the average of eigen vector estimation of robot i
        """
        for index, robot in enumerate(self.robots):
            visible_robots_index = [visible_robot.robot_id for visible_robot in
                                    robot.get_visible_robots(self.robots)]
            sum_zi_minus_zj = (self.z[index] - self.z[visible_robots_index]).sum(axis=0)
            sum_wi_minus_wj = (self.w[index] - self.w[visible_robots_index]).sum(axis=0)
            dw_i = -self.__ki * sum_zi_minus_zj
            dz_i = (self.__gama * (self.alpha[index] - self.z[index]) - self.__kp * sum_zi_minus_zj +
                    self.__ki * sum_wi_minus_wj)
            self.w[index] += dw_i
            self.z[index] += dz_i

    def update_v_tilde2(self):
        """
        compute and update the eigen vector(self.v_tilde2) estimation of robot i
        """
        for index, robot in enumerate(self.robots):
            visible_robots_index = [visible_robot.robot_id for visible_robot in
                                    robot.get_visible_robots(self.robots)]
            sum_aij_times_v2i_minus_v2j = ((self.l_matrix[index, visible_robots_index] *
                                            (self.v_tilde2[index] - self.v_tilde2[visible_robots_index]))
                                           .sum(axis=0))
            dv_tilde2_i = (-self.__k1 * self.z[index, 0] - self.__k2 * sum_aij_times_v2i_minus_v2j - self.__k3 *
                           (self.z[index, 1] - 1) * self.v_tilde2[index])
            self.v_tilde2[index] += dv_tilde2_i

        self.update_z()

    @property
    def lambda2_tilde(self):
        return self.__k3 / self.__k2 * (1 - self.z[:, 0])

    def speed_gen(self):
        for _ in itertools.count():
            speeds = []
            dt = 0.005
            for index, robot in enumerate(self.robots):
                cur_pos_angle = np.arctan2(robot.posture[1] - self.central_point[1],
                                       robot.posture[0] - self.central_point[0])
                visible_robots = robot.get_visible_robots(self.robots)
                visible_robots_index: list[int] = [visible_robot.robot_id for visible_robot in
                                                   visible_robots]
                distance = np.linalg.norm(robot.posture[:2] - self.central_point)
                if visible_robots_index:
                    pj_vec = np.array([robot.posture[:2] for robot in visible_robots])
                    w_i = ((-self.l_matrix[index, visible_robots_index]
                            * (self.v_tilde2[index] - self.v_tilde2[visible_robots_index])) ** 2 *
                           np.linalg.norm(self.robots[index].posture[:2] - pj_vec) / self.__sigma ** 2).sum()
                    speed_along_trace = self.speed_round(w_i * self.radius)
                    rotate_speed = self.angular_vel_round(w_i / 400)
                    self.update_v_tilde2()
                else:
                    speed_along_trace = self._speed_range[1]
                    rotate_speed = self._angular_vel_range[1] / 10
                speed_along_radius = np.sign(distance - self.radius) * speed_along_trace / self.__k
                speed = (-speed_along_trace * np.sin(cur_pos_angle) - speed_along_radius * np.cos(cur_pos_angle),
                         speed_along_trace * np.cos(cur_pos_angle) - speed_along_radius * np.sin(cur_pos_angle),
                         rotate_speed)
                speeds.append(speed)
            # pdb.set_trace()
            yield dt, speeds
