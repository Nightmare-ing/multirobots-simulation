import matplotlib.pyplot as plt
import matplotlib.patches as patches
from Parts import Robot
from Controller import CentralController
from Simulation import Simulation


def main():
    robots = Robot.initialize_robots(4,
                                     posture=[(3, 0, 0), (0, 3, 0), (-3, 0, 0), (0, -3, 0)],
                                     speed=[(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)])
    fig, ax = plt.subplots()
    desired_trace = patches.Circle((0, 0), 3, color='black', fill=False)
    ax.add_patch(desired_trace)
    controller = CentralController(robots)
    simulation_scene = Simulation(fig, ax, robots, controller)
    simulation_scene.show_animation()


if __name__ == "__main__":
    main()
