import math

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from Parts import Robot
from Controller import CentralController, DecentralizedController
from Simulation import Simulation


def main():
    robots = Robot.initialize_robots(4,
                                     posture=[(5, 0, math.radians(135)), (0, 5, math.radians(225)),
                                              (-5, 0, math.radians(-45)), (0, -5, math.radians(45))],
                                     speed=[(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)])
    fig, ax = plt.subplots()
    # controller = DecentralizedController(robots)
    controller = CentralController(robots)
    simulation_scene = Simulation(fig, ax, robots, controller)
    simulation_scene.show_animation()
    # simulation_scene.save_animation("test.mp4")


if __name__ == "__main__":
    main()
