import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np


class Camera:
    def __init__(self, field_angle=np.pi / 2, visible_radius=4.0, visible_range=np.array([0.25, 2.0]),
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
        self.point_artist = patches.Circle((0, 0), 0.2, color=self.color)
        self.entire_visible_region_artist = patches.Wedge((0, 0),
                                                          float(self.camera.visible_radius * self.camera.visible_range[1]),
                                                          0, 0,
                                                          edgecolor=self.color, linestyle='--',
                                                          facecolor=self.color, alpha=0.3)
        self.invisible_region_artist = patches.Wedge((0, 0),
                                                     float(self.camera.visible_radius * self.camera.visible_range[0]),
                                                     0, 0,
                                                     facecolor=plt.gcf().get_facecolor())

    def update(self, dt, speed=(0, 0, 0)):
        self.posture = self.posture + (self.speed * dt)
        self.speed = np.array(speed)

    @property
    def robot_point_artist(self):
        self.point_artist.center = (float(self.posture[0]), float(self.posture[1]))
        return self.point_artist

    @property
    def visible_region_artist(self):
        theta1 = math.degrees(self.posture[2] + self.camera_angle - self.camera.field_angle / 2)
        theta2 = math.degrees(self.posture[2] + self.camera_angle + self.camera.field_angle / 2)
        position = (float(self.posture[0]), float(self.posture[1]))
        self.entire_visible_region_artist.set_center(position)
        self.entire_visible_region_artist.set_theta1(theta1)
        self.entire_visible_region_artist.set_theta2(theta2)
        self.invisible_region_artist.set_center(position)
        self.invisible_region_artist.set_theta1(theta1)
        self.invisible_region_artist.set_theta2(theta2)
        return self.entire_visible_region_artist, self.invisible_region_artist

    def inspect(self, robot):
        transformed_point = tuple[float, float](self.visible_region_artist[0].get_transform().transform(robot.posture[:2]))
        inside_entire_region = self.visible_region_artist[0].contains_point(transformed_point)
        transformed_point = tuple[float, float](self.visible_region_artist[1].get_transform().transform(robot.posture[:2]))
        inside_invisible_region = self.visible_region_artist[1].contains_point(transformed_point)
        return inside_entire_region and not inside_invisible_region

    def get_visible_robots(self, robots):
        """
        get the robots that self can see
        :param robots: robot inspecting
        :return: robots that self can see
        """
        visible_robots = [other_robot for other_robot in robots
                          if other_robot is not self and self.inspect(other_robot)]
        return visible_robots

    def __str__(self):
        return f"Robot {self.robot_id}- Position: {self.posture}"
