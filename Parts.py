import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from Camera import Camera


class Robot:
    """
    Single integral robot model, can communicate with all other robots in its visible region
    """
    # to count how many robots have been created for single simulation
    robot_num = 0

    @staticmethod
    def initialize_group(num_robots, posture, speeds):
        """
        Initialize a group of robots,
        recommended to use this method to initialize robots
        :param num_robots:
        :param posture:
        :param speeds:
        :return:
        """
        Robot.robot_num = 0
        robots = [Robot(initial_posture=posture[i], speeds=speeds[i]) for i in range(num_robots)]
        return robots

    @staticmethod
    def clear_robot_num():
        """
        clear the robot_num,
        should be called when creating a new group of robots
        """
        Robot.robot_num = 0

    def __init__(self, initial_posture=(0, 0, 0), speeds=(0, 0, 0), camera=Camera(), camera_angle=0.0):
        """
        Initialize a robot,
        not recommended to use this method to initialize robots, if you really want to use this method,
        please call Robot.clear_robot_num() before creating a new group of robots
        :param initial_posture: (cor_x, cor_y, theta)
        :param speeds: (speed_x, speed_y, rotate_speed)
        :param camera: customize camera parameters here
        :param camera_angle: the fixed angle of camera relative to robot
        """
        # give each robot a unique id for plt legend
        self.robot_id = Robot.robot_num
        Robot.robot_num += 1
        self.posture = np.array(initial_posture)
        self.speed = np.array(speeds)
        self.camera = camera
        self.camera_angle = camera_angle

        # some elems for drawing
        self.color = np.random.uniform(0, 0.6, 3)
        self.point_artist = patches.Circle((0, 0), 0.2, color=self.color)
        self.entire_visible_region_artist = patches.Wedge((0, 0),
                                                          float(self.camera.visible_radius * self.camera.visible_range[
                                                              1]),
                                                          0, 0,
                                                          edgecolor=self.color, linestyle='--',
                                                          facecolor=self.color, alpha=0.3)
        self.invisible_region_artist = patches.Wedge((0, 0),
                                                     float(self.camera.visible_radius * self.camera.visible_range[0]),
                                                     0, 0,
                                                     facecolor=plt.gcf().get_facecolor())

    def update(self, dt, speeds=(0, 0, 0)):
        """
        update robot position
        :param dt: increment step of time
        :param speeds: (speed_x, speed_y, rotate_speed)
        """
        self.posture = self.posture + (self.speed * dt)
        self.speed = np.array(speeds)

    @property
    def robot_point_artist(self):
        """
        get the artist of robot point, update before been accessed
        :return: point_artist (patches.Circle object)
        """
        self.point_artist.center = (float(self.posture[0]), float(self.posture[1]))
        return self.point_artist

    @property
    def visible_region_artist(self):
        """
        get the artist of visible region, update before been accessed
        :return: entire_visible_region, invisible_region (patches.Wedge object)
        """
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
        """
        check if robot is in self's visible region
        :param robot: robot to be inspected
        :return: True if robot is in self's visible region, False otherwise
        """
        transformed_point = tuple[float, float](
            self.visible_region_artist[0].get_transform().transform(robot.posture[:2]))
        inside_entire_region = self.visible_region_artist[0].contains_point(transformed_point)
        transformed_point = tuple[float, float](
            self.visible_region_artist[1].get_transform().transform(robot.posture[:2]))
        inside_invisible_region = self.visible_region_artist[1].contains_point(transformed_point)
        return inside_entire_region and not inside_invisible_region

    def get_visible_robots(self, robots):
        """
        get the robots that self can see
        :param robots: robots to be inspected
        :return: list of robots that self can see, and list of their index in robots group
        """
        visible_robots = []
        visible_robots_index = []
        for other_robot in robots:
            if other_robot is not self and self.inspect(other_robot):
                visible_robots.append(other_robot)
                visible_robots_index.append(other_robot.robot_id)
        return visible_robots, visible_robots_index

    def __str__(self):
        return f"Robot {self.robot_id}- Position: {self.posture}"


class SeeFarthestOneRobot(Robot):
    """
    Single integral robot that can only communicate with the farthest robot in its visible region
    """
    def __init__(self, initial_posture=(0, 0, 0), speeds=(0, 0, 0), camera=Camera(), camera_angle=0.0):
        super().__init__(initial_posture, speeds, camera, camera_angle)

    def get_visible_robots(self, robots):
        """
        get the robots that self can see
        :param robots: robot inspecting
        :return: robots that self can see
        """
        visible_robots = [other_robot for other_robot in robots
                          if other_robot is not self and self.inspect(other_robot)]
        closest_robot = max(visible_robots, key=lambda robot: np.linalg.norm(robot.posture[:2] - self.posture[:2]))
        return closest_robot, [closest_robot.robot_id]


class DoubleIntegralRobot(SeeFarthestOneRobot):
    """
    Double integral robot model, can only communicate with the farthest robot in its visible region
    """
    def __init__(self, initial_posture=(0, 0, 0), speeds=(0, 0, 0), camera=Camera(), camera_angle=0.0):
        super().__init__(initial_posture, speeds, camera, camera_angle)

    def update(self, dt, acceleration=(0, 0, 0)):
        self.speed = self.speed + (acceleration * dt)
        self.posture = self.posture + (self.speed * dt)
