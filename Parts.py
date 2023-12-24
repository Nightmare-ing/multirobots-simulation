import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
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
        self.color = np.random.uniform(0.3, 1, 3)

    def update(self, dt, speed=(0, 0, 0)):
        self.posture = self.posture + (self.speed * dt)
        self.speed = np.array(speed)

    def draw(self, fig):
        # draw robot point
        fig.plot(self.posture[0], self.posture[1], 'o', color=self.color, markersize=8,
                 label=f'Robot {self.robot_id}')

        # draw fov
        fig.add_patch(
            patches.Wedge(
                (float(self.posture[0]), float(self.posture[1])),
                float(self.camera.visible_radius * self.camera.visible_range[1]),
                math.degrees(self.posture[2] + self.camera_angle - self.camera.field_angle / 2),
                math.degrees(self.posture[2] + self.camera_angle + self.camera.field_angle / 2),
                color=self.color, alpha=0.3
            )
        )

        # erase invisible part of fov
        fig.add_patch(
            patches.Wedge(
                (float(self.posture[0]), float(self.posture[1])),
                float(self.camera.visible_radius * self.camera.visible_range[0]),
                math.degrees(self.posture[2] + self.camera_angle - self.camera.field_angle / 2),
                math.degrees(self.posture[2] + self.camera_angle + self.camera.field_angle / 2),
                color=plt.gcf().get_facecolor()
            )
        )

        # draw fov outer edge
        fig.add_patch(
            patches.Arc(
                (float(self.posture[0]), float(self.posture[1])),
                float(2 * self.camera.visible_radius * self.camera.visible_range[1]),
                float(2 * self.camera.visible_radius * self.camera.visible_range[1]),
                theta1=math.degrees(self.posture[2] + self.camera_angle - self.camera.field_angle / 2),
                theta2=math.degrees(self.posture[2] + self.camera_angle + self.camera.field_angle / 2),
                color=self.color, linestyle='--'
            )
        )

        # draw fov inner edge
        fig.add_patch(
            patches.Arc(
                (float(self.posture[0]), float(self.posture[1])),
                float(2 * self.camera.visible_radius * self.camera.visible_range[0]),
                float(2 * self.camera.visible_radius * self.camera.visible_range[0]),
                theta1=math.degrees(self.posture[2] + self.camera_angle - self.camera.field_angle / 2),
                theta2=math.degrees(self.posture[2] + self.camera_angle + self.camera.field_angle / 2),
                color=self.color, linestyle='--'
            )
        )

    def __str__(self):
        return f"Robot {self.robot_id}- Position: {self.posture}"
