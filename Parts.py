import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np


class Camera:
    def __init__(self, field_angle=np.pi / 2, visible_radius=6.0, visible_range=np.array([0.25, 2.0]),
                 rotate_speed=0.1):
        self.field_angle = field_angle
        self.visible_radius = visible_radius
        self.visible_range = visible_range
        self.rotate_speed = rotate_speed


class Robot:
    # to count how many robots have been created
    robot_num = 0

    @staticmethod
    def initialize_robots(num_robots, posture, speed):
        robots = [Robot(initial_posture=posture[i], speed=speed[i]) for i in range(num_robots)]
        return robots

    def __init__(self, initial_posture=(0, 0, 0), speed=(0, 0, 0), camera=Camera(), camera_angle=0.0):
        # give each robot a unique id for plt legend
        self.robot_id = Robot.robot_num
        Robot.robot_num += 1
        self.posture = np.array(initial_posture)
        self.speed = np.array(speed)
        self.camera = camera
        self.camera_angle = camera_angle

        # some elems for drawing
        self.color = np.random.uniform(0, 0.6, 3)
        self.point = patches.Circle((0, 0), 0.2, color=self.color)
        self.entire_visible_region = patches.Wedge((0, 0),
                                                   float(self.camera.visible_radius * self.camera.visible_range[1]),
                                                   0, 0,
                                                   edgecolor=self.color, linestyle='--',
                                                   facecolor=self.color, alpha=0.3)
        self.invisible_region = patches.Wedge((0, 0),
                                              float(self.camera.visible_radius * self.camera.visible_range[0]),
                                              0, 0,
                                              facecolor=plt.gcf().get_facecolor())

    def update(self, dt, speed=(0, 0, 0)):
        self.posture = self.posture + (self.speed * dt)
        self.speed = np.array(speed)

    @property
    def robot_point(self):
        self.point.center = (float(self.posture[0]), float(self.posture[1]))
        return self.point

    @property
    def visible_region(self):
        theta1 = math.degrees(self.posture[2] + self.camera_angle - self.camera.field_angle / 2)
        theta2 = math.degrees(self.posture[2] + self.camera_angle + self.camera.field_angle / 2)
        position = (float(self.posture[0]), float(self.posture[1]))
        self.entire_visible_region.set_center(position)
        self.entire_visible_region.set_theta1(theta1)
        self.entire_visible_region.set_theta2(theta2)
        self.invisible_region.set_center(position)
        self.invisible_region.set_theta1(theta1)
        self.invisible_region.set_theta2(theta2)
        return self.entire_visible_region, self.invisible_region

    def inspect(self, robot):
        relative_pos_vec = self.posture[:2] - robot.posture[:2]
        dist = np.linalg.norm(relative_pos_vec)
        angle = np.arctan2(relative_pos_vec[0], relative_pos_vec[1])
        visible_radius_range = self.camera.visible_radius * self.camera.visible_range
        visible_angle_range = [self.posture[2] + self.camera_angle - self.camera.field_angle / 2,
                               self.posture[2] + self.camera_angle + self.camera.field_angle / 2]
        dis_valid = (dist > visible_radius_range[0]) & (dist < visible_radius_range[1])
        angle_valid = (angle < visible_angle_range[1]) & (angle > visible_angle_range[0])
        return dis_valid & angle_valid

    def get_visible_robots(self, robots):
        """
        get the robots that self can see
        :param robots: robot inspecting
        :return: robots that self can see
        """
        visible_robots = []
        for other_robot in robots:
            if other_robot is not robots and self.inspect(other_robot):
                visible_robots.append(other_robot)
        return visible_robots

    def __str__(self):
        return f"Robot {self.robot_id}- Position: {self.posture}"
