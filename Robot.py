import matplotlib.pyplot as plt
import numpy as np


class Camera:
    def __init__(self, field_angle=np.pi / 2, visible_radius=2.0, visible_range=np.array([0.25, 2.0]),
                 rotate_speed=0.1):
        self.field_angle = field_angle
        self.visible_radius = visible_radius
        self.visible_range = visible_range
        self.rotate_speed = rotate_speed


class Robot:
    # to count how many robots have been created
    robot_num = 0

    def __init__(self, initial_posture=(0, 0, 0), speed=(0, 0), camera=Camera(), camera_angle=0.0):
        # give each robot a unique id for plt legend
        self.robot_id = Robot.robot_num
        Robot.robot_num += 1
        self.posture = np.array(initial_posture)
        self.speed = np.array(speed)
        self.camera = camera
        self.camera_angle = camera_angle
        self.color = np.random.uniform(0.3, 1, 3)

    def update(self, dt, speed=(0, 0)):
        self.posture = self.posture + self.speed * dt
        self.speed = np.array(speed)

    def draw(self, fig):
        # generate random colors for each robot, the color should not be too light to see

        # draw robot point
        fig.plot(self.posture[0], self.posture[1], 'o', color=self.color, markersize=8,
                 label=f'Robot {self.robot_id}')

        # draw sector part of fov
        fov_radius = self.camera.visible_radius
        fov_theta = np.linspace(self.posture[2] + self.camera_angle - self.camera.field_angle / 2,
                                self.posture[2] + self.camera_angle + self.camera.field_angle / 2,
                                100)
        fov_x_start = self.posture[0] + fov_radius * self.camera.visible_range[0] * np.cos(fov_theta)
        fov_y_start = self.posture[1] + fov_radius * self.camera.visible_range[0] * np.sin(fov_theta)
        fov_x_end = self.posture[0] + fov_radius * self.camera.visible_range[1] * np.cos(fov_theta)
        fov_y_end = self.posture[1] + fov_radius * self.camera.visible_range[1] * np.sin(fov_theta)
        # add shadow of fov, alpha=0.1 means 10% transparent
        fig.fill_between(fov_x_end, fov_y_end, color=self.color, alpha=0.3)
        # draw fov edge
        fig.plot(fov_x_end, fov_y_end, color=self.color, linestyle='--')
        fig.fill([fov_x_start[0], fov_x_end[0], fov_x_end[-1], fov_x_start[-1]],
                 [fov_y_start[0], fov_y_end[0], fov_y_end[-1], fov_y_start[-1]],
                 color=self.color, alpha=0.3)
        fig.fill_between(fov_x_start, fov_y_start, color=plt.gcf().get_facecolor())
        fig.plot(fov_x_start, fov_y_start, color=self.color, linestyle='--')

    def __str__(self):
        return f"Robot {self.robot_id}- Position: {self.posture}"
