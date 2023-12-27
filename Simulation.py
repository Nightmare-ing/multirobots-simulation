import matplotlib.pyplot as plt
import matplotlib.animation as animation


class Simulation:
    def __init__(self, fig, ax, robots, controller):
        self.fig = fig
        self.ax = ax
        self.robots = robots
        self.controller = controller
        self.ani = None

    def initialize_scene(self):
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_aspect('equal')

        # plot desired trace
        self.ax.add_patch(self.controller.desired_trace)

        # plot robot and visible region
        for robot in self.robots:
            for region in robot.visible_region:
                self.ax.add_patch(region)
            self.ax.add_patch(robot.robot_point)

        return [region for robot in self.robots for region in robot.visible_region] + \
            [robot.robot_point for robot in self.robots]

    def update_scene(self, data):
        # update robot position with controller output
        dt, speeds = data
        for i in range(len(self.robots)):
            self.robots[i].update(dt, speeds[i])

        return [region for robot in self.robots for region in robot.visible_region] + \
            [robot.robot_point for robot in self.robots]

    def show_animation(self):
        self.ani = animation.FuncAnimation(self.fig, self.update_scene, self.controller.speed_gen,
                                           init_func=self.initialize_scene, interval=100, blit=True,
                                           cache_frame_data=False, save_count=100)
        plt.show()

    def save_animation(self, filename, fps=60):
        self.ani.save(filename, fps=fps, dpi=200)
