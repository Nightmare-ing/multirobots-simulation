import matplotlib.pyplot as plt


class Simulation:
    def __init__(self, ax, robots, controller):
        self.ax = ax
        self.robots = robots
        self.controller = controller

    def initialize_scene(self):
        self.ax.set_xlim(-4, 8)
        self.ax.set_ylim(-8, 8)
        self.ax.set_aspect('equal')

        # plot robot and visible region
        for robot in self.robots:
            self.ax.add_patch(robot.robot_point)
            for region in robot.visible_region:
                self.ax.add_patch(region)

        return [robot.robot_point for robot in self.robots] + \
            [region for robot in self.robots for region in robot.visible_region]

    def update_scene(self):
        for robot in self.robots:
            robot.update(next(self.controller.speed_gen(robot)))


