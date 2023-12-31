import math

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from Parts import Robot, OnlySeeFarthestOneRobot, DoubleIntegralOnlySeeFarthestOneRobot
from Controller import CentralController, DecentralizedController, DoubleIntegralController, EclipseTraceController
from Simulation import Simulation


def main():
    robots2 = DoubleIntegralOnlySeeFarthestOneRobot.initialize_robots(4,
                                                                      posture=[(5, 0, math.radians(135)),
                                                                               (0, 5, math.radians(225)),
                                                                               (-5, 0, math.radians(-45)),
                                                                               (0, -5, math.radians(45))],
                                                                      speed=[(0, 1, 0), (-1, 0, 0), (0, -1, 0),
                                                                             (1, 0, 0)])
    robots1 = OnlySeeFarthestOneRobot.initialize_robots(4,
                                                        posture=[
                                                            (5 * math.cos(math.pi / 6.0), 5 * math.sin(math.pi / 6.0),
                                                             math.radians(90)),
                                                            (5 * math.cos(math.pi / 3.0), 5 * math.sin(math.pi / 3.0),
                                                             math.radians(120)),
                                                            (5 * math.cos(math.pi / 2.0), 5 * math.sin(math.pi / 2.0),
                                                             math.radians(150)),
                                                            (5 * math.cos(math.pi / 1.5), 5 * math.sin(math.pi / 1.5),
                                                             math.radians(180))],
                                                        speed=[(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)])
    robots = OnlySeeFarthestOneRobot.initialize_robots(4,
                                                       posture=[(5, 0, math.radians(135)),
                                                                (0, 5, math.radians(225)),
                                                                (-5, 0, math.radians(-45)),
                                                                (0, -5, math.radians(45))],
                                                       speed=[(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)])
    fig, ax = plt.subplots()
    fig1, ax1 = plt.subplots()
    fig2, ax2 = plt.subplots()

    controller1 = DecentralizedController(robots1)
    simulation_scene1 = Simulation(fig1, ax1, robots1, controller1)
    simulation_scene1.show_animation()

    # controller = CentralController(robots)
    controller2 = DoubleIntegralController(robots2)
    simulation_scene2 = Simulation(fig2, ax2, robots2, controller2)
    simulation_scene2.show_animation()

    controller = EclipseTraceController(robots)
    simulation_scene = Simulation(fig, ax, robots, controller)
    simulation_scene.show_animation()
    # simulation_scene.save_animation("test.mp4")
    plt.show()


if __name__ == "__main__":
    main()
