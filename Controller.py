import matplotlib.patches as patches
import numpy as np
import itertools


class Controller:
    def __init__(self, robots):
        self.robots = robots  # robots to control

    @property
    def _speed_range(self):
        """
        Define available norm of speed range
        :return: available norm of speed range, np.array([a, b])
        """
        return None

    @property
    def _angular_vel_range(self):
        """
        Define available angular velocity range here
        :return: available angular velocity range, np.array([a, b])
        """
        return None

    @property
    def _acceleration_range(self):
        """
        Define available acceleration range here
        :return: available acceleration range, np.array([a, b])
        """
        return None

    @property
    def desired_trace_artist(self):
        """
        Desired trace Artist for drawing
        :return:
        """
        return None

    def speed_gen(self):
        """
        Main control algorithm here, just override this
        :return: dt(time increment step), speeds for all robots
        """
        yield 0, [(0, 0, 0)] * len(self.robots)

    def speed_round(self, speeds_xy):
        """
        Round transferred in speeds to valid range
        :param speeds_xy: speeds (x, y) to be round
        :return: rounded speeds
        """
        return None

    def angular_vel_round(self, angular_vel):
        """
        Round transferred in angular velocity to valid range
        :param angular_vel: angular velocity to be round
        :return: rounded angular velocity
        """
        return None

    def acceleration_round(self, acceleration_xy):
        """
        Round transferred in acceleration to valid range
        :param acceleration_xy: acceleration (x, y) to be round
        :return: rounded acceleration
        """
        return None


class CircularTraceController(Controller):
    __gravity_velocity = 9.81
    k = 100.0  # adjust param for controlling speed_along_radius

    def __init__(self, robots):
        super().__init__(robots)
        self.radius = 5.0  # radius of the desired trace
        self.central_point = (0.0, 0.0)  # central point coordinates of the desired trace
        self.w_r = 1.0  # required robots rotating speed along the trace

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
    def desired_trace_artist(self):
        return patches.Circle(self.central_point, self.radius, color='cyan', fill=False)

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
        speed_along_radius = np.sign(distance - self.radius) * speed_along_trace / self.k
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

    def speed_round(self, speeds_xy):
        if np.linalg.norm(speeds_xy) < self._speed_range[1]:
            return tuple(speeds_xy)
        else:
            angle = np.arctan2(speeds_xy[1], speeds_xy[0])
            return tuple(np.array([np.cos(angle), np.sin(angle)]) * self._speed_range[1])

    def angular_vel_round(self, angular_vel):
        if angular_vel > self._angular_vel_range[1]:
            return self._angular_vel_range[1]
        elif angular_vel < self._angular_vel_range[0]:
            return self._angular_vel_range[0]
        else:
            return angular_vel

    def acceleration_round(self, acceleration_xy):
        if np.linalg.norm(acceleration_xy) < self._acceleration_range[1]:
            return tuple(acceleration_xy)
        else:
            angle = np.arctan2(acceleration_xy[1], acceleration_xy[0])
            return tuple(np.array([np.cos(angle), np.sin(angle)]) * self._acceleration_range[1])


class CentralController(CircularTraceController):
    def __init__(self, robots):
        super().__init__(robots)

    def speed_gen(self):
        for _ in itertools.count():
            speeds = []
            dt = 0.005
            for robot in self.robots:
                speed_along_trace = self.w_r * self.radius
                speed = self.speed_adjust(robot, speed_along_trace) + (self._angular_vel_range[1],)
                speeds.append(speed)
            yield dt, speeds


class DecentralizedController(CircularTraceController):
    gama = 0.1  # param for computing dz_i
    kp = 0.1  # param for computing dz_i
    ki = 0.1  # param for computing dw_i
    k1 = 0.1  # param for computing dv_tilde2_i
    k2 = 0.001  # param for computing dv_tilde2_i
    k3 = 0.1  # param for computing dv_tilde2_i
    sigma = 1.0  # param for computing d_tilde_lambda / d position
    k = 1.0  # param for adjusting rotating speed

    def __init__(self, robots):
        super().__init__(robots)
        self.matrix = np.zeros((len(self.robots), len(self.robots)))  # for storing Laplacian matrix
        self.z_mat = np.ones((len(self.robots), 2))  # for computing z vector by integrating
        self.w_mat = np.ones((len(self.robots), 2))  # for computing w vector by integrating
        self.v_tilde2_vec = np.ones(len(self.robots))  # for computing v_tilde2 vector by integrating

    def update_l_matrix(self):
        """
        Dynamically compute Laplacian matrix
        :return: Laplacian matrix
        """
        a_matrix = np.zeros((len(self.robots), len(self.robots)))
        d_matrix = np.zeros((len(self.robots), len(self.robots)))
        for (i, j), _ in np.ndenumerate(self.matrix):
            if self.robots[i].inspect(self.robots[j]):
                # the farther distance, the smaller weight
                a_matrix[i, j] = np.exp(-np.linalg.norm(self.robots[i].posture[:2] - self.robots[j].posture[:2]))

        for i in range(len(self.robots)):
            d_matrix[i, i] = a_matrix[i, :].sum()

        self.matrix = a_matrix - d_matrix

    @property
    def alpha(self):
        """
        Compute alpha vector, which is input for computing dz_i.
        As a vector for parallel computing.
        :return: [v_tilde2, v_tilde2 ** 2]
        """
        return np.array([self.v_tilde2_vec, self.v_tilde2_vec ** 2]).transpose()

    def update_z(self):
        """
        Compute and update z vector, namely the Ave({v_tilde2, v_tilde2 ** 2})
        """
        for index, robot in enumerate(self.robots):
            _, visible_robots_index = robot.get_visible_robots(self.robots)
            sum_zi_minus_zj = (self.z_mat[index] - self.z_mat[visible_robots_index]).sum(axis=0)
            sum_wi_minus_wj = (self.w_mat[index] - self.w_mat[visible_robots_index]).sum(axis=0)
            dw_i = -self.ki * sum_zi_minus_zj
            dz_i = (self.gama * (self.alpha[index] - self.z_mat[index]) - self.kp * sum_zi_minus_zj +
                    self.ki * sum_wi_minus_wj)
            self.w_mat[index] += dw_i
            self.z_mat[index] += dz_i

    def update_v_tilde(self):
        """
        Compute and update the eigen vector(self.v_tilde2) estimation of robot i
        """
        for index, robot in enumerate(self.robots):
            _, visible_robots_index = robot.get_visible_robots(self.robots)
            sum_aij_times_v2i_minus_v2j = ((self.matrix[index, visible_robots_index] *
                                            (self.v_tilde2_vec[index] - self.v_tilde2_vec[visible_robots_index]))
                                           .sum(axis=0))
            dv_tilde2_i = (-self.k1 * self.z_mat[index, 0] - self.k2 * sum_aij_times_v2i_minus_v2j - self.k3 *
                           (self.z_mat[index, 1] - 1) * self.v_tilde2_vec[index])
            self.v_tilde2_vec[index] += dv_tilde2_i
            self.update_z()

    @property
    def lambda2_tilde(self):
        """
        Get lambda2_tilde, namely estimation of the second smallest eigen value of Laplacian matrix
        :return: lambda2_tilde
        """
        return self.k3 / self.k2 * (1 - self.z_mat[:, 0])

    def speed_gen(self):
        for _ in itertools.count():
            dt = 0.005
            control_speeds = self.speeds_to_maintain_connection()
            self.adjust_ave(control_speeds)
            self.update_v_tilde()
            self.update_l_matrix()
            # print(f"Estimation of lambda_tilde of decentralized controller: {self.lambda2_tilde}")
            yield dt, control_speeds

    def speeds_to_maintain_connection(self):
        """
        Compute the speeds according to the algorithm described in paper
        :return: control speeds, [(speed_x, speed_y, rotate_speed), ...]
        """
        speeds = []
        for index, robot in enumerate(self.robots):
            _, visible_robots_index = robot.get_visible_robots(self.robots)
            if visible_robots_index:
                pj_vec = np.array([self.robots[i].posture[:2] for i in visible_robots_index])
                u = ((-self.matrix[index, visible_robots_index]
                      * (self.v_tilde2_vec[index] - self.v_tilde2_vec[visible_robots_index])) ** 2 *
                     np.linalg.norm(self.robots[index].posture[:2] - pj_vec) / self.sigma ** 2).sum()
                speed_along_trace = u
                rotate_speed = self.angular_vel_round(u / self.radius) / self.k
            else:
                speed_along_trace = self._speed_range[1]
                rotate_speed = self._angular_vel_range[1]
            speed = self.speed_round(self.speed_adjust(robot, speed_along_trace)) + (rotate_speed,)
            speeds.append(speed)
        return speeds


class DoubleIntegralController(DecentralizedController):
    __k_acc = 0.95  # param for adjusting gain of acceleration

    def __init__(self, robots):
        super().__init__(robots)

    def speed_gen(self):
        for _ in itertools.count():
            dt = 0.005
            control_speeds = self.speeds_to_maintain_connection()
            cur_speeds = [robot.speed for robot in self.robots]
            self.adjust_ave(control_speeds)
            acceleration = [(speed - cur_speed) * self.__k_acc for speed, cur_speed in zip(control_speeds, cur_speeds)]
            acceleration = [self.acceleration_round(acc[:2]) + (acc[2],) for acc in acceleration]
            self.update_v_tilde()
            self.update_l_matrix()
            # print(f"Estimation of lambda_tilde of double integral Controller: {self.lambda2_tilde}")
            yield dt, acceleration


class EclipseTraceController(DecentralizedController):
    """
    The EclipseTraceController based on CircularTraceController, just apply affine transformation on the
    circular trace in CircularTraceController
    """
    def __init__(self, robots):
        super().__init__(robots)
        self.a_radius = 5.0
        self.b_radius = 2.0
        self.radius = self.a_radius

    @property
    def desired_trace_artist(self):
        return patches.Ellipse(self.central_point, self.a_radius * 2, self.b_radius * 2, color='cyan', fill=False)

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
        eclipse_radius = self.a_radius * self.b_radius / np.sqrt(
            (self.a_radius * np.sin(cur_pos_angle)) ** 2 + (self.b_radius * np.cos(cur_pos_angle)) ** 2)
        speed_along_radius = np.sign(distance - eclipse_radius) * speed_along_trace / self.k
        speed_xy = (-speed_along_trace * np.sin(cur_pos_angle) - speed_along_radius * np.cos(cur_pos_angle),
                    self.b_radius / self.a_radius * (
                            speed_along_trace * np.cos(cur_pos_angle) - speed_along_radius * np.sin(cur_pos_angle)))
        return speed_xy
