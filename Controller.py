import matplotlib.patches as patches
import numpy as np
import itertools


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
        for i, j in self.matrix.shape:
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
                cur_angle = np.arctan2(robot.posture[1] - self.central_point[1],
                                       robot.posture[0] - self.central_point[0])
                speed = (-self.w_r * self.radius * np.sin(cur_angle),
                         self.w_r * self.radius * np.cos(cur_angle),
                         0.0)
                speeds.append(speed)
            yield dt, speeds


class DecentralizedController(Controller):
    __gama = 0.1
    __kp = 0.1
    __ki = 0.1

    def __init__(self, robots):
        super().__init__(robots)
        self.z = np.zeros((2, len(self.robots)))
        self.w = np.zeros((2, len(self.robots)))
        self.v2 = np.zeros(len(self.robots))
        self.lambda2 = np.zeros(len(self.robots))

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

    def update_z(self, alpha):
        """
        compute the average of eigen vector estimation of robot i
        :param alpha: input 1*2-dim vector, representing [Ave{{v2}}, Ave{{v2^2}}]
        :return: the average of eigen vector estimation of robot i
        """
        for index, robot in enumerate(self.robots):
            visible_robots_index = [visible_robot.robot_id for visible_robot in
                                    robot.get_visible_robots(self.robots)]
            sum_zi_minus_zj = (self.z[index] - self.z[visible_robots_index]).sum(axis=0)
            sum_wi_minus_wj = (self.w[index] - self.w[visible_robots_index]).sum(axis=0)
            dw_i = -self.__ki * sum_zi_minus_zj
            dz_i = self.__gama * (alpha - self.z[index]) - sum_zi_minus_zj + self.__ki * sum_wi_minus_wj
            self.w[index] += dw_i
            self.z[index] += dz_i

