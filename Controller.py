import math
from typing import List, Any

import matplotlib.patches as patches
import numpy as np
import itertools
import pdb


class CircularTraceController:
    __gravity_velocity = 9.81
    __k = 100.0  # adjust param for controlling speed_along_radius

    def __init__(self, robots):
        self.radius = 5.0  # radius of the desired trace
        self.central_point = (0.0, 0.0)  # central point coordinates of the desired trace
        self.w_r = 1.0  # required robots rotating speed along the trace
        self.robots = robots  # robots to control
        self.matrix = np.zeros((len(self.robots), len(self.robots)))  # for storing Laplacian matrix

    @property
    def _speed_range(self):
        """
        Define available norm of speed range
        :return: available norm of speed range, np.array([a, b])
        """
        return np.array([0.0, 2 * self.w_r * self.radius])

    @property
    def _angular_vel_range(self):
        """
        Define available angular velocity range here
        :return: available angular velocity range, np.array([a, b])
        """
        return np.array([-10 * self.w_r, 10 * self.w_r])

    @property
    def _acceleration_range(self):
        """
        Define available acceleration range here
        :return: available acceleration range, np.array([a, b])
        """
        return np.array([-self.__gravity_velocity, self.__gravity_velocity])

    @property
    def desired_trace_artist(self):
        """
        Desired trace Artist for drawing
        :return:
        """
        return None

    @property
    def l_matrix(self):
        """
        Dynamically compute Laplacian matrix
        :return: Laplacian matrix
        """
        a_matrix = np.zeros((len(self.robots), len(self.robots)))
        d_matrix = np.zeros((len(self.robots), len(self.robots)))
        for (i, j), _ in np.ndenumerate(self.matrix):
            if self.robots[i].inspect(self.robots[j]):
                a_matrix[i, j] = np.linalg.norm(self.robots[i].posture[:2] - self.robots[j].posture[:2])

        for i in range(len(self.robots)):
            d_matrix[i, i] = a_matrix[i, :].sum()

        self.matrix = a_matrix - d_matrix
        return self.matrix

    def speed_adjust(self, robot, speed_along_trace, angle):
        """
        Adjust speed_xy to control robot to move along the desired trace without offset.
        You can modify this to use your own control algorithm to move along the desired trace.
        :param angle: cur pos angle in the trace
        :param robot: robot to compute
        :param speed_along_trace: speed along trace
        :return: speed along radius
        """
        distance = np.linalg.norm(robot.posture[:2] - self.central_point)
        speed_along_radius = np.sign(distance - self.radius) * speed_along_trace / self.__k
        speed_xy = (-speed_along_trace * np.sin(angle) - speed_along_radius * np.cos(angle),
                    speed_along_trace * np.cos(angle) - speed_along_radius * np.sin(angle))
        return speed_xy

    def speed_gen(self):
        """
        Main control algorithm here, just override this
        :return: dt(time increment step), speeds for all robots
        """
        yield 0, [(0, 0, 0)] * len(self.robots)

    def speed_round(self, speeds_xy):
        """
        Round transferred in speeds to valid range
        :param speeds_xy: speeds (x, y) or speed_norm to be round
        :return: rounded speeds
        """
        if np.linalg.norm(speeds_xy) < self._speed_range[1]:
            return speeds_xy
        else:
            return speeds_xy / np.linalg.norm(speeds_xy) * self._speed_range[1]

    def angular_vel_round(self, angular_vel):
        """
        Round transferred in angular velocity to valid range
        :param angular_vel: angular velocity to be round
        :return: rounded angular velocity
        """
        if angular_vel > self._angular_vel_range[1]:
            return self._angular_vel_range[1]
        elif angular_vel < self._angular_vel_range[0]:
            return self._angular_vel_range[0]
        else:
            return angular_vel


class CentralController(CircularTraceController):
    def __init__(self, robots):
        super().__init__(robots)

    @property
    def desired_trace_artist(self):
        return patches.Circle(self.central_point, self.radius, color='cyan', fill=False)

    def speed_gen(self):
        for _ in itertools.count():
            speeds = []
            dt = 0.005
            for robot in self.robots:
                cur_pos_angle = np.arctan2(robot.posture[1] - self.central_point[1],
                                           robot.posture[0] - self.central_point[0])
                speed_along_trace = self.w_r * self.radius
                speed = self.speed_adjust(robot, speed_along_trace, cur_pos_angle) + (self._angular_vel_range[1],)
                speeds.append(speed)
            yield dt, speeds


class DecentralizedController(CircularTraceController):
    __gama = 0.1  # param for computing dz_i
    __kp = 0.1  # param for computing dz_i
    __ki = 0.1  # param for computing dw_i
    __k1 = 0.1  # param for computing dv_tilde2_i
    __k2 = 0.001  # param for computing dv_tilde2_i
    __k3 = 0.1  # param for computing dv_tilde2_i
    __sigma = 1  # param for computing d_tilde_lambda / d position

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
    def desired_trace_artist(self):
        return patches.Circle(self.central_point, self.radius, color='cyan', fill=False)

    def update_z(self):
        """
        Compute and update the average of eigen vector estimation of robot i
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
        Compute and update the eigen vector(self.v_tilde2) estimation of robot i
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
                if visible_robots_index:
                    pj_vec = np.array([robot.posture[:2] for robot in visible_robots])
                    w_i = ((-self.l_matrix[index, visible_robots_index]
                            * (self.v_tilde2[index] - self.v_tilde2[visible_robots_index])) ** 2 *
                           np.linalg.norm(self.robots[index].posture[:2] - pj_vec) / self.__sigma ** 2).sum()
                    speed_along_trace = self.speed_round(w_i * self.radius)
                    rotate_speed = self.angular_vel_round(w_i / 800)
                    self.update_v_tilde2()
                else:
                    speed_along_trace = self._speed_range[1]
                    rotate_speed = self._angular_vel_range[1] / 5
                speed = self.speed_adjust(robot, speed_along_trace, cur_pos_angle) + (rotate_speed,)
                speeds.append(speed)
            # pdb.set_trace()
            yield dt, speeds
