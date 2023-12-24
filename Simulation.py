import matplotlib.pyplot as plt
import matplotlib.animation as animation


class Simulation:
    def __init__(self, fig, ax, robots, controller):
        self.fig = fig
        self.ax = ax
        self.robots = robots
        self.controller = controller

    def initialize_scene(self):
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_aspect('equal')

        # plot robot and visible region
        for robot in self.robots:
            self.ax.add_patch(robot.robot_point)
            for region in robot.visible_region:
                self.ax.add_patch(region)

        return [robot.robot_point for robot in self.robots] + \
            [region for robot in self.robots for region in robot.visible_region]

    def update_scene(self, data):
        # update robot position with controller output
        dt, speeds = data
        for i in range(len(self.robots)):
            self.robots[i].update(dt, speeds[i])

        return [robot.robot_point for robot in self.robots] + \
            [region for robot in self.robots for region in robot.visible_region]

    def show_animation(self):
        ani = animation.FuncAnimation(self.fig, self.update_scene, self.controller.speed_gen,
                                      init_func=self.initialize_scene, interval=0.01, blit=True)
        plt.show()


