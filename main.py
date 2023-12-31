import math

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from Parts import Robot, OnlySeeFarthestOneRobot, DoubleIntegralOnlySeeFarthestOneRobot
from Controller import CentralController, DecentralizedController, DoubleIntegralController
from Simulation import Simulation


def main():
    # robots = Robot.initialize_robots(4,
    #                                  posture=[(5, 0, math.radians(135)), (0, 5, math.radians(225)),
    #                                           (-5, 0, math.radians(-45)), (0, -5, math.radians(45))],
    #                                  speed=[(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)])
    robots1 = OnlySeeFarthestOneRobot.initialize_robots(4,
                                                       posture=[(5, 0, math.radians(135)), (0, 5, math.radians(225)),
                                                        (-5, 0, math.radians(-45)), (0, -5, math.radians(45))],
                                                       speed=[(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)])
    robots = DoubleIntegralOnlySeeFarthestOneRobot.initialize_robots(4,
                                                                     posture=[(5, 0, math.radians(135)),
                                                                              (0, 5, math.radians(225)),
                                                                              (-5, 0, math.radians(-45)),
                                                                              (0, -5, math.radians(45))],
                                                                     speed=[(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)])
    fig, ax = plt.subplots()
    fig1, ax1 = plt.subplots()
    controller1 = DecentralizedController(robots1)
    simulation_scene1 = Simulation(fig1, ax1, robots1, controller1)
    simulation_scene1.show_animation()
    # controller = CentralController(robots)
    controller = DoubleIntegralController(robots)
    simulation_scene = Simulation(fig, ax, robots, controller)
    simulation_scene.show_animation()
    # simulation_scene.save_animation("test.mp4")
    plt.show()


if __name__ == "__main__":
    main()
