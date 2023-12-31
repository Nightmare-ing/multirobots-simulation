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

    def update_l_matrix(self):
        """
        Dynamically compute Laplacian matrix
        :return: Laplacian matrix
        """
        a_matrix = np.zeros((len(self.robots), len(self.robots)))
        d_matrix = np.zeros((len(self.robots), len(self.robots)))
        for (i, j), _ in np.ndenumerate(self.matrix):
            if self.robots[i].inspect(self.robots[j]):
                a_matrix[i, j] = np.exp(-np.linalg.norm(self.robots[i].posture[:2] - self.robots[j].posture[:2]))

        for i in range(len(self.robots)):
            d_matrix[i, i] = a_matrix[i, :].sum()

        self.matrix = a_matrix - d_matrix

    def speed_adjust(self, robot, speed_along_trace):
        """
        Adjust speed_xy to control robot to move along the desired trace without offset.
        You can modify this to use your own control algorithm to move along the desired trace.
        :param robot: robot to compute
        :param speed_along_trace: speed along trace
        :return: speed along radius
        """
        cur_pos_angle = np.arctan2(robot.posture[1] - self.central_point[1],
                                   robot.posture[0] - self.central_point[0])
        distance = np.linalg.norm(robot.posture[:2] - self.central_point)
        speed_along_radius = np.sign(distance - self.radius) * speed_along_trace / self.__k
        speed_xy = (-speed_along_trace * np.sin(cur_pos_angle) - speed_along_radius * np.cos(cur_pos_angle),
                    speed_along_trace * np.cos(cur_pos_angle) - speed_along_radius * np.sin(cur_pos_angle))
        return speed_xy

    def adjust_ave(self, speeds):
        """
        Adjust all speeds so that the average angular velocity along the trace is w_r,
        the elem of speeds is [(speed_x, speed_y, rotate_speed), ...]
        :param speeds: speeds to adjust
        :return: adjusted speeds
        """
        speeds_np = np.array(speeds)
        speed_norm = np.linalg.norm(speeds_np[:, :2], axis=1)
        cur_ave = speed_norm.mean()
        speed_norm_adjusted = speed_norm - cur_ave + self.w_r * self.radius
        for index, (speed_x, speed_y, rotate_speed) in enumerate(speeds):
            angle = np.arctan2(speed_y, speed_x)
            speed_x_adjusted = speed_norm_adjusted[index] * np.cos(angle)
            speed_y_adjusted = speed_norm_adjusted[index] * np.sin(angle)
            speeds[index] = (speed_x_adjusted, speed_y_adjusted, rotate_speed)

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
                speed = self.speed_adjust(robot, speed_along_trace) + (self._angular_vel_range[1],)
                speeds.append(speed)
            yield dt, speeds


class DecentralizedController(CircularTraceController):
    __gama = 0.1  # param for computing dz_i
    __kp = 0.1  # param for computing dz_i
    __ki = 0.1  # param for computing dw_i
    __k1 = 0.1  # param for computing dv_tilde2_i
    __k2 = 0.001  # param for computing dv_tilde2_i
    __k3 = 0.1  # param for computing dv_tilde2_i
    __sigma = 1.0  # param for computing d_tilde_lambda / d position
    __k = 1.0  # param for adjusting rotating speed

    def __init__(self, robots):
        super().__init__(robots)
        self.z_mat = np.ones((len(self.robots), 2))  # for computing z vector by integrating
        self.w_mat = np.ones((len(self.robots), 2))  # for computing w vector by integrating
        self.v_tilde2_vec = np.ones(len(self.robots))  # for computing v_tilde2 vector by integrating

    @property
    def alpha(self):
        """
        Compute alpha vector, which is input for computing dz_i.
        As a vector for parallel computing.
        :return: [v_tilde2, v_tilde2 ** 2]
        """
        return np.array([self.v_tilde2_vec, self.v_tilde2_vec ** 2]).transpose()

    @property
    def desired_trace_artist(self):
        """
        Desired trace Artist for drawing
        :return: desired circular trace
        """
        return patches.Circle(self.central_point, self.radius, color='cyan', fill=False)

    def update_z(self):
        """
        Compute and update z vector, namely the Ave({v_tilde2, v_tilde2 ** 2})
        """
        for index, robot in enumerate(self.robots):
            visible_robots_index = [visible_robot.robot_id for visible_robot in
                                    robot.get_visible_robots(self.robots)]
            sum_zi_minus_zj = (self.z_mat[index] - self.z_mat[visible_robots_index]).sum(axis=0)
            sum_wi_minus_wj = (self.w_mat[index] - self.w_mat[visible_robots_index]).sum(axis=0)
            dw_i = -self.__ki * sum_zi_minus_zj
            dz_i = (self.__gama * (self.alpha[index] - self.z_mat[index]) - self.__kp * sum_zi_minus_zj +
                    self.__ki * sum_wi_minus_wj)
            self.w_mat[index] += dw_i
            self.z_mat[index] += dz_i

    def update_v_tilde(self):
        """
        Compute and update the eigen vector(self.v_tilde2) estimation of robot i
        """
        for index, robot in enumerate(self.robots):
            visible_robots_index = [visible_robot.robot_id for visible_robot in
                                    robot.get_visible_robots(self.robots)]
            sum_aij_times_v2i_minus_v2j = ((self.matrix[index, visible_robots_index] *
                                            (self.v_tilde2_vec[index] - self.v_tilde2_vec[visible_robots_index]))
                                           .sum(axis=0))
            dv_tilde2_i = (-self.__k1 * self.z_mat[index, 0] - self.__k2 * sum_aij_times_v2i_minus_v2j - self.__k3 *
                           (self.z_mat[index, 1] - 1) * self.v_tilde2_vec[index])
            self.v_tilde2_vec[index] += dv_tilde2_i
            self.update_z()

    @property
    def lambda2_tilde(self):
        """
        Get lambda2_tilde, namely estimation of the second smallest eigen value of Laplacian matrix
        :return: lambda2_tilde
        """
        return self.__k3 / self.__k2 * (1 - self.z_mat[:, 0])

    def speed_gen(self):
        for _ in itertools.count():
            dt = 0.005
            control_speeds = self.speeds_to_maintain_connection()
            self.adjust_ave(control_speeds)
            self.update_v_tilde()
            self.update_l_matrix()
            print(self.lambda2_tilde)
            yield dt, control_speeds

    def speeds_to_maintain_connection(self):
        speeds = []  # [(speed_along_trace, rotate_speed), ...]
        for index, robot in enumerate(self.robots):
            visible_robots_index: list[int] = [visible_robot.robot_id for visible_robot in
                                               robot.get_visible_robots(self.robots)]
            if visible_robots_index:
                pj_vec = np.array([self.robots[i].posture[:2] for i in visible_robots_index])
                u = ((-self.matrix[index, visible_robots_index]
                      * (self.v_tilde2_vec[index] - self.v_tilde2_vec[visible_robots_index])) ** 2 *
                     np.linalg.norm(self.robots[index].posture[:2] - pj_vec) / self.__sigma ** 2).sum()
                speed_along_trace = self.speed_round(u)
                rotate_speed = self.angular_vel_round(u / self.radius) / self.__k
            else:
                speed_along_trace = self._speed_range[1]
                rotate_speed = self._angular_vel_range[1]
            speed = self.speed_adjust(robot, speed_along_trace) + (rotate_speed,)
            speeds.append(speed)
        return speeds

            self.adjust_ave(speeds)
            self.update_v_tilde()
            self.update_l_matrix()
            print(self.lambda2_tilde)
            yield dt, speeds
